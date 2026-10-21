#include "servo_bus.hpp"

#include <array>
#include <cerrno>
#include <cmath>
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

// Сколько раз повторяем запрос целиком, прежде чем сдаться.
constexpr int kRegisterReadAttempts = 2;
constexpr int kReadRetryDelayMicroseconds = 1000;

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

// Итог одной попытки чтения регистра. Различать важно: «мотор не ответил» и
// «USB-устройство отвалилось» требуют разных действий, а сообщение об ошибке
// обязано их различать, иначе обрыв кабеля выглядит как поломка робота.
enum class ReadResult { Ok, NoAnswer, Disconnected, ServoError };

constexpr bool is_disconnect_error(int error) {
    // EIO и ENODEV — устройство исчезло с шины; ENXIO встречается на
    // отключённом устройстве, EPIPE — на оборванном конце. После
    // переподключения read() на живом tty возвращает именно их.
    return error == EIO || error == ENODEV || error == ENXIO ||
           error == EPIPE;
}

ReadResult read_register_once(int fd, uint8_t servo_id, Register address,
                              uint16_t& value) {
    tcflush(fd, TCIOFLUSH);

    std::array<uint8_t, kReadPacketSize> request{
        kPacketHeader, kPacketHeader, servo_id, kReadPacketLength, INST_READ,
        static_cast<uint8_t>(address), kDataLength, kReserved
    };
    finish_checksum(request.data(), static_cast<int>(kReadPacketSize));

    if (!write_packet(fd, request.data(), request.size())) {
        return ReadResult::Disconnected;
    }

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
            if (is_disconnect_error(error)) {
                return ReadResult::Disconnected;
            }
            if (error == EINTR) {
                ++attempts;
                continue;
            }
        }
        usleep(kReadRetryDelayMicroseconds);
        ++attempts;
    }

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

// Читает регистр, переживая обрыв USB: адаптер мог отвалиться от хаба, и тогда
// порт надо открыть заново, а мотора — спросить ещё раз.
bool read_register(SerialPort& port, uint8_t servo_id, Register address,
                   uint16_t& value) {
    for (int attempt = 0; attempt < kRegisterReadAttempts; ++attempt) {
        const ReadResult result =
            read_register_once(port.fd(), servo_id, address, value);
        if (result == ReadResult::Ok) {
            return true;
        }
        if (result == ReadResult::ServoError) {
            // Мотор ответил, но отказал: переподключение тут не поможет.
            return false;
        }
        if (result == ReadResult::Disconnected) {
            std::cerr << "Servo bus " << port.path()
                      << " disconnected, reconnecting...\n";
            if (!port.reopen()) {
                std::cerr << "Servo bus " << port.path()
                          << " did not come back. Check the USB cable and the "
                             "adapter's port.\n";
                return false;
            }
            // Порт переоткрыт, но мотора сразу после переподключения может
            // не быть слышно: повторяем попытку.
            continue;
        }
        // Мотор жив, но не ответил: первый запрос после переподключения
        // иногда пропадает, поэтому просто повторяем, не трогая порт.
    }
    return false;
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

SerialPort::SerialPort(const std::string& port)
    : port_(port), fd_(configure_serial_port(port.c_str())) {
	if (fd_ < 0) {
		// errno читается сразу и до построения строки: порядок вычисления
		// операндов operator+ не определён, а strerror отдаёт общий буфер.
		const int error = errno;
		throw ServoPortError("Failed to open serial port " + port + ": " +
		                     std::strerror(error));
	}
}

bool SerialPort::reopen() {
	if (fd_ >= 0) {
		close(fd_);
		fd_ = -1;
	}

	// После обрыва устройство переподключается само, и узел /dev/ttyACM*
	// появляется не мгновенно: сначала возвращается интерфейс, потом
	// до устройства доходит первый запрос. Поэтому ждём и пробуем снова.
	for (int attempt = 0; attempt < kReopenAttempts; ++attempt) {
		const int fd = configure_serial_port(port_.c_str());
		if (fd >= 0) {
			fd_ = fd;
			return true;
		}
		usleep(kReopenDelayMicroseconds);
	}
	return false;
}

SerialPort::~SerialPort() noexcept {
	if (fd_ >= 0) {
		close(fd_);
	}
}

SerialPort::SerialPort(SerialPort&& other) noexcept
    : port_(std::move(other.port_)), fd_(other.fd_) {
	// Путь переезжает вместе с дескриптором: перемещённый объект обязан
	// остаться пригодным для reopen(), иначе он попытался бы открыть пустую
	// строку.
	other.fd_ = -1;
}

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
	if (this != &other) {
		if (fd_ >= 0) {
			close(fd_);
		}
		port_ = std::move(other.port_);
		fd_ = other.fd_;
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

    cfsetospeed(&tty, kServoBusBaud);
    cfsetispeed(&tty, kServoBusBaud);
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
