#include "servo_bus.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace {
constexpr uint8_t INST_READ = 0x02;
constexpr uint8_t INST_WRITE = 0x03;

// Скорость шины в термиос-виде.
//
// cfsetospeed ждёт не числовую скорость, а её закодированную константу:
// с числом 1000000 вызов падает с EINVAL, скорость остаётся прежней, и
// порт молча работает на дефолтных 9600 бод. На 9600 бод сервоприводы не
// отвечают вовсе.
//
// Номер кодированной константы на разных libc разный, поэтому скорость
// проверяется на том, что выставило ядро, а не на том, что мы попросили.
constexpr speed_t kServoBusSpeed = B1000000;

// Трассировка обмена с железом. Включается переменной окружения
// SO101_TRACE=1 и нужна только при разборе сбоев связи: в обычном прогоне
// молчит.
bool trace_enabled() {
    static const bool enabled = std::getenv("SO101_TRACE") != nullptr;
    return enabled;
}

void trace_bytes(const char* label, const uint8_t* data, std::size_t size) {
    if (!trace_enabled()) return;
    std::cerr << "TRACE " << label << ":";
    for (std::size_t index = 0; index < size; ++index) {
        std::cerr << ' ' << std::hex << static_cast<int>(data[index]);
    }
    std::cerr << std::dec << '\n' << std::flush;
}

// Раскладка кадра STS3215:
//   [0..1]  заголовок 0xFF 0xFF
//   [2]     номер сервопривода
//   [3]     длина пакета
//   [4]     инструкция в запросе, код ошибки в ответе
//   [5..]   параметры: адрес регистра, либо значение
//   [последний] контрольная сумма
constexpr uint8_t kPacketHeader = 0xFF;
// Байт длины внутри кадра и поле «читаю столько-то байт» в запросе.
constexpr uint8_t kReadPacketLength = 0x04;
constexpr uint8_t kWritePacketLength = 0x05;
constexpr uint8_t kDataLength = 0x02;
constexpr uint8_t kReserved = 0x00;
constexpr std::size_t kServoIdIndex = 2;
constexpr std::size_t kLengthIndex = 3;
constexpr std::size_t kErrorCodeIndex = 4;
constexpr std::size_t kDataLowIndex = 5;
constexpr std::size_t kDataHighIndex = 6;

// Длина кадра в байтах: у STS3215 запрос чтения и ответ занимают по 8 байт,
// запись — 9.
constexpr std::size_t kReadPacketSize = 8;
constexpr std::size_t kWritePacketSize = 9;

// Сколько раз пробуем прочитать ответ, прежде чем признать мотор молчащим.
constexpr int kReadAttempts = 20;

// Сколько раз повторяем запрос целиком, прежде как признать, что мотор молчит.
constexpr int kRegisterReadAttempts = 2;
constexpr int kReadRetryDelayMicroseconds = 1000;

// Пауза между попытками, когда молчит вся шина: ждать тут есть чего, но и
// метаться по порту незачем.
constexpr int kBusRetryDelayMicroseconds = 1000000;

// Пауза между моторами при чтении границ всех шести: даёт шине time settle.
constexpr int kInterServoDelayMicroseconds = 1000;

// Пауза между записью скорости и записью целевой позиции: мотор должен успеть
// принять скорость до следующей команды.
constexpr int kVelocityToPositionDelayMicroseconds = 3000;

uint8_t checksum(const uint8_t* packet, int first, int last) {
    uint32_t sum = 0;
    for (int index = first; index <= last; ++index) {
        sum += packet[index];
    }
    return static_cast<uint8_t>(~sum);
}

// Контрольная сумма кадра: ~(сумма байт [2 .. total-2]), последним байтом
// кадра. Два байта заголовка и сама сумма в покрытие не входят.
void finish_checksum(uint8_t* packet, int total) {
    packet[total - 1] = checksum(packet, 2, total - 2);
}

bool write_packet(int fd, const uint8_t* packet, size_t packet_size) {
    size_t bytes_written = 0;
    while (bytes_written < packet_size) {
        ssize_t result = write(fd, packet + bytes_written,
                               packet_size - bytes_written);
        if (result > 0) {
            bytes_written += static_cast<size_t>(result);
        } else if (result < 0 && errno == EINTR) {
            continue;
        } else {
            return false;
        }
    }

    return tcdrain(fd) == 0;
}

// Итог одной попытки чтения регистра.
//
// «Не ответил» и «отвалилось устройство» различать не нужно: лечение у них
// одно и то же — переоткрыть порт. Различие живёт в reopen(): если узла
// порта нет, значит пропал адаптер, и об этом reopen() и сообщает.
enum class ReadResult { Ok, NoAnswer, ServoError };

ReadResult read_register_once(int fd, uint8_t servo_id, Register address,
                              uint16_t& value) {
    tcflush(fd, TCIOFLUSH);

    std::array<uint8_t, kReadPacketSize> request{
        kPacketHeader, kPacketHeader, servo_id, kReadPacketLength, INST_READ,
        static_cast<uint8_t>(address), kDataLength, kReserved
    };
    finish_checksum(request.data(), static_cast<int>(kReadPacketSize));

    if (!write_packet(fd, request.data(), request.size())) {
        trace_bytes("write failed", request.data(), request.size());
        return ReadResult::NoAnswer;
    }
    trace_bytes("request", request.data(), request.size());

    std::array<uint8_t, kReadPacketSize> response{};
    std::size_t total_bytes_read = 0;
    int attempts = 0;
    while (total_bytes_read < kReadPacketSize && attempts < kReadAttempts) {
        const ssize_t bytes_read = read(fd, response.data() + total_bytes_read,
                                        response.size() - total_bytes_read);

        if (bytes_read > 0) {
            total_bytes_read += static_cast<std::size_t>(bytes_read);
            continue;
        }
        if (bytes_read < 0) {
            const int error = errno;
            if (error == EINTR) {
                ++attempts;
                continue;
            }
            // Любая другая ошибка (EIO, ENODEV — устройство исчезло с шины)
            // означает, что работать с этим дескриптором больше нельзя.
            // Дальше работает вызывающий: он переоткроет порт.
            if (trace_enabled()) {
                std::cerr << "TRACE read error: errno=" << error << " ("
                          << std::strerror(error) << ")\n" << std::flush;
            }
            return ReadResult::NoAnswer;
        }
        // Ноль байт: устройство молчит. Может, это пропавший адаптер, а может
        // сервопривод. Разбираться будет reopen().
        usleep(kReadRetryDelayMicroseconds);
        ++attempts;
    }
    trace_bytes("response", response.data(), total_bytes_read);

    if (total_bytes_read < kReadPacketSize) {
        return ReadResult::NoAnswer;
    }
    if (response[0] != kPacketHeader || response[1] != kPacketHeader ||
        response[kServoIdIndex] != servo_id ||
        checksum(response.data(), 2, static_cast<int>(kReadPacketSize) - 2) !=
            response[kReadPacketSize - 1]) {
        return ReadResult::NoAnswer;
    }
    // Код ошибки в ответе означает, что параметры бессмысленны: мотора такого
    // диапазона не существует. Принимать их за границы нельзя — иначе в
    // motors уйдут углы, посчитанные от мусора.
    if (response[kErrorCodeIndex] != 0x00) {
        std::cerr << "Servo Joint " << static_cast<int>(servo_id)
                  << " reported error code: "
                  << static_cast<int>(response[kErrorCodeIndex]) << '\n';
        return ReadResult::ServoError;
    }

    value = static_cast<uint16_t>(response[kDataLowIndex]) |
            static_cast<uint16_t>(static_cast<uint16_t>(response[kDataHighIndex])
                                      << 8);
    return ReadResult::Ok;
}

// Проверяет, жива ли шина: читает регистр у соседнего мотора.
//
// Молчание одного мотора и молчание всей шины снаружи выглядят одинаково —
// молчат и те, и те. Но лечатся они по-разному: если молчит один мотор,
// ждать бесполезно, а если молчит вся шина, ждать есть чего. Отличаем одно
// от другого чтением у соседа.
bool bus_is_alive(SerialPort& port, uint8_t servo_id) {
	const uint8_t probe = (servo_id == 1) ? 2 : 1;
	uint16_t value = 0;
	return read_register_once(port.fd(), probe, Register::present_position,
	                          value) == ReadResult::Ok;
}

// Читает регистр, переживая и обрыв USB, и молчание сервопривода.
//
// Любая неудача лечится переоткрытием порта: адаптер QinHeng отвечает только
// на запрос, отправленный после свежего open. Так же он ведёт себя и сразу
// после подачи питания роботу, когда сервоприводы ещё не проснулись: первый
// open молчит, следующий работает. Повтор запроса на том же дескрипторе не
// помогает.
//
// Молчит один мотор или молчит вся шина — выясняем пробой соседа. Если
// сосед отвечает, то молчит именно этот мотор: ждать бессмысленно, и мы
// так и говорим. Если молчит и сосед, то дело в связи, и мы переоткрываем
// порт и ждём, пока шина оживёт.
bool read_register(SerialPort& port, uint8_t servo_id, Register address,
                   uint16_t& value) {
	const auto deadline = std::chrono::steady_clock::now() +
	                      std::chrono::seconds(port.device_wait_seconds());
	bool announced = false;

	for (int attempt = 0;; ++attempt) {
		const ReadResult result =
		    read_register_once(port.fd(), servo_id, address, value);
		if (result == ReadResult::Ok) {
			return true;
		}
		if (result == ReadResult::ServoError) {
			// Мотор ответил, но отказал: переподключение тут не поможет.
			return false;
		}

		const bool bus_alive = bus_is_alive(port, servo_id);
		if (attempt + 1 >= kRegisterReadAttempts && bus_alive) {
			std::cerr << "Servo Joint " << static_cast<int>(servo_id)
			          << " did not answer. Check the motor's power supply."
			          << '\n';
			return false;
		}

		if (std::chrono::steady_clock::now() >= deadline) {
			if (bus_alive) {
				std::cerr << "Servo Joint " << static_cast<int>(servo_id)
				          << " did not answer. Check the motor's power "
				             "supply."
				          << '\n';
			} else {
				std::cerr << "Servo bus " << port.path()
				          << " is not responding. Check the USB cable and "
				             "the adapter's port."
				          << '\n';
			}
			return false;
		}

		if (!bus_alive && !announced) {
			std::cerr << "Servo bus " << port.path()
			          << " is not responding, reconnecting..." << '\n';
			announced = true;
		}

		// Свежий open, а не повтор запроса на том же дескрипторе: именно он
		// возвращает связь с сервоприводами.
		if (!port.reopen()) {
			return false;
		}
		usleep(bus_alive ? kReopenDelayMicroseconds
		                 : kBusRetryDelayMicroseconds);
	}
}

bool write_register(SerialPort& port, uint8_t servo_id, Register address,
                    uint16_t value) {
    std::array<uint8_t, kWritePacketSize> packet{
        kPacketHeader, kPacketHeader, servo_id, kWritePacketLength, INST_WRITE,
        static_cast<uint8_t>(address), static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF), kReserved
    };
    finish_checksum(packet.data(), static_cast<int>(kWritePacketSize));

    return write_packet(port.fd(), packet.data(), packet.size());
}

bool write_byte_register(SerialPort& port, uint8_t servo_id, Register address,
                         uint8_t value) {
    std::array<uint8_t, kReadPacketSize> packet{
        kPacketHeader, kPacketHeader, servo_id, kReadPacketLength, INST_WRITE,
        static_cast<uint8_t>(address), value, kReserved
    };
    finish_checksum(packet.data(), static_cast<int>(kReadPacketSize));
    return write_packet(port.fd(), packet.data(), packet.size());
}
}  // namespace

SerialPort::SerialPort(const std::string& port, int device_wait_seconds)
    : port_(port), device_wait_seconds_(device_wait_seconds) {
	const OpenResult result = open_waiting(false);
	if (result == OpenResult::Opened) {
		return;
	}
	if (result == OpenResult::DeviceMissing) {
		std::cerr << "Servo bus " << port_
		          << " did not come back. Check the USB cable and the "
		             "adapter's port."
		          << '\n';
		throw ServoPortError("Servo bus " + port_ + " did not come back");
	}
	throw ServoPortError("Failed to open serial port " + port_ + ": " +
	                     std::strerror(last_error_));
}

SerialPort::OpenResult SerialPort::open_waiting(bool reconnecting) {
    if (trace_enabled()) {
        std::cerr << "TRACE open attempt " << port_ << '\n' << std::flush;
    }
    int fd = configure_serial_port(port_.c_str());

	if (fd >= 0) {
		fd_ = fd;
		return OpenResult::Opened;
	}

	// errno читается сразу и до построения строки: порядок вычисления
	// операндов operator+ не определён, а strerror отдаёт общий буфер.
	last_error_ = errno;
	if (last_error_ != ENOENT) {
		return OpenResult::Failed;
	}

	// Узла нет: адаптер либо не подключён, либо ещё перечисляется.
	// Перечисление занимает доли секунды, но обрыв может держаться дольше.
	if (reconnecting) {
		std::cerr << "Servo bus " << port_
		          << " disconnected, waiting for the adapter to reconnect..."
		          << '\n';
	} else {
		std::cerr << "Port " << port_
		          << " does not exist, waiting for the adapter to connect..."
		          << '\n';
	}

	const auto deadline = std::chrono::steady_clock::now() +
	                      std::chrono::seconds(device_wait_seconds_);
	while (std::chrono::steady_clock::now() < deadline) {
		usleep(kReopenDelayMicroseconds);
		fd = configure_serial_port(port_.c_str());
		if (fd >= 0) {
			fd_ = fd;
			return OpenResult::Opened;
		}
		last_error_ = errno;
	}
	return OpenResult::DeviceMissing;
}

bool SerialPort::reopen() {
	if (fd_ >= 0) {
		close(fd_);
		fd_ = -1;
	}

	const OpenResult result = open_waiting(true);
	if (result == OpenResult::Opened) {
		return true;
	}
	if (result == OpenResult::DeviceMissing) {
		std::cerr << "Servo bus " << port_
		          << " did not come back. Check the USB cable and the "
		             "adapter's port."
		          << '\n';
		return false;
	}
	std::cerr << "Failed to reopen the servo bus " << port_ << ": "
	          << std::strerror(last_error_) << '\n';
	return false;
}

SerialPort::~SerialPort() noexcept {
	if (fd_ >= 0) {
		close(fd_);
	}
}

SerialPort::SerialPort(SerialPort&& other) noexcept
    : port_(std::move(other.port_)), fd_(other.fd_),
      device_wait_seconds_(other.device_wait_seconds_),
      last_error_(other.last_error_) {
	// Путь переезжает вместе с дескриптором: перемещённый объект обязан
	// остаться пригодным для reopen(), иначе он попытался бы открыть пустую
	// строку. Окно ожидания переезжает по той же причине: перемещённый
	// объект ждёт столько же, сколько и исходный.
	other.fd_ = -1;
}

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
	if (this != &other) {
		if (fd_ >= 0) {
			close(fd_);
		}
		port_ = std::move(other.port_);
		fd_ = other.fd_;
		device_wait_seconds_ = other.device_wait_seconds_;
		last_error_ = other.last_error_;
		other.fd_ = -1;
	}
	return *this;
}

// Возвращает -1, оставляя errno, если порт недоступен: текст ошибки
// формирует SerialPort, чтобы не печатать одно и то же дважды.
int configure_serial_port(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        return -1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, kServoBusSpeed);
    cfsetispeed(&tty, kServoBusSpeed);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                     ICRNL | IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }

    // Скорость проверяем на том, что выставило ядро, а не на том, что мы
    // попросили. Молча сбитая скорость ломает всю связь целиком, и заметить
    // это можно только здесь: сервоприводы на дефолтных 9600 бод молчат так
    // же, как выключенные.
    termios applied{};
    if (tcgetattr(fd, &applied) != 0) {
        close(fd);
        return -1;
    }
    const speed_t actual_speed = cfgetispeed(&applied);
    if (actual_speed != kServoBusSpeed && actual_speed != kServoBusBaud) {
        std::cerr << "Servo bus " << port << " could not be set to "
                  << kServoBusBaud << " baud (got " << actual_speed << ")\n";
        close(fd);
        errno = EIO;
        return -1;
    }

    if (trace_enabled()) {
        std::cerr << "TRACE open " << port << " fd=" << fd
                  << " cflag=" << std::hex << tty.c_cflag
                  << " iflag=" << tty.c_iflag << " oflag=" << tty.c_oflag
                  << " lflag=" << tty.c_lflag << " VMIN="
                  << static_cast<int>(tty.c_cc[VMIN]) << " VTIME="
                  << static_cast<int>(tty.c_cc[VTIME]) << std::dec
                  << " ispeed=" << cfgetispeed(&applied)
                  << " ospeed=" << cfgetospeed(&applied) << '\n'
                  << std::flush;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

std::optional<ServoLimits> read_servo_limits(SerialPort& port,
                                              uint8_t servo_id) {
    ServoLimits limits{};
    if (!read_register(port, servo_id, Register::min_position_limit,
                       limits.min_position)) {
        std::cerr << "Joint " << static_cast<int>(servo_id)
                  << ": failed to read the min position register (0x09)\n";
        return std::nullopt;
    }
    if (!read_register(port, servo_id, Register::max_position_limit,
                       limits.max_position)) {
        std::cerr << "Joint " << static_cast<int>(servo_id)
                  << ": failed to read the max position register (0x0B)\n";
        return std::nullopt;
    }
    if (limits.max_position <= limits.min_position) {
        std::cerr << "Joint " << static_cast<int>(servo_id)
                  << ": register limits are empty (" << limits.min_position
                  << ".." << limits.max_position << ")\n";
        return std::nullopt;
    }
    return limits;
}

std::optional<std::array<ServoLimits, kServoCount>> read_all_servo_limits(
    SerialPort& port) {
    std::array<ServoLimits, kServoCount> limits{};
    for (std::size_t index = 0; index < kServoCount; ++index) {
        const uint8_t servo_id = static_cast<uint8_t>(index + 1);
        const std::optional<ServoLimits> motor_limits =
            read_servo_limits(port, servo_id);
        if (!motor_limits) {
            std::cerr << "Failed to read register limits from Joint "
                      << static_cast<int>(servo_id) << '\n';
            return std::nullopt;
        }
        limits[index] = *motor_limits;
        usleep(kInterServoDelayMicroseconds);
    }
    return limits;
}

std::optional<float> read_joint_angle(SerialPort& port, uint8_t servo_id,
                                      const ServoLimits& limits) {
    uint16_t present_position = 0;
    if (!read_register(port, servo_id, Register::present_position,
                       present_position)) {
        return std::nullopt;
    }
    if (present_position < limits.min_position ||
        present_position > limits.max_position) {
        std::cerr << "Joint " << static_cast<int>(servo_id)
                  << " position is outside servo register limits: "
                  << present_position << " (limits " << limits.min_position
                  << ".." << limits.max_position << ")\n";
        return std::nullopt;
    }

    return static_cast<float>(limits.position_to_degrees(present_position));
}

bool disable_servo_torque(SerialPort& port, uint8_t servo_id) {
    if (!write_byte_register(port, servo_id, Register::torque_enable, 0)) {
        std::cerr << "Failed to disable torque on Joint "
                  << static_cast<int>(servo_id) << '\n';
        return false;
    }

    std::cout << "Torque disabled on Joint "
              << static_cast<int>(servo_id) << '\n';
    return true;
}

bool write_joint_angle(SerialPort& port, uint8_t servo_id, float angle_degrees,
                       uint16_t speed, const ServoLimits& limits) {
    if (!std::isfinite(angle_degrees) || angle_degrees < 0.0f) return false;

    const float max_angle = limits.span_degrees();
    if (angle_degrees > max_angle) {
        std::cerr << "Angle " << angle_degrees << " is outside Joint "
                  << static_cast<int>(servo_id) << " range 0.." << max_angle
                  << " degrees\n";
        return false;
    }

    const uint16_t steps = limits.degrees_to_position(angle_degrees);
    if (!write_register(port, servo_id, Register::goal_velocity, speed)) {
        std::cerr << "Failed to transmit speed to Joint "
                  << static_cast<int>(servo_id) << '\n';
        return false;
    }
    usleep(kVelocityToPositionDelayMicroseconds);

    if (!write_register(port, servo_id, Register::goal_position, steps)) {
        std::cerr << "Failed to transmit packet to Joint "
                  << static_cast<int>(servo_id) << '\n';
        return false;
    }

    std::cout << "Joint " << static_cast<int>(servo_id)
              << " commanded to " << angle_degrees << " degrees ("
              << steps << " steps, speed " << speed << ").\n";
    return true;
}
