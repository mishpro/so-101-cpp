#include "servo_bus.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace {
constexpr uint8_t INST_READ = 0x02;
constexpr uint8_t INST_WRITE = 0x03;
constexpr uint8_t REG_MIN_POSITION_LIMIT = 0x09;
constexpr uint8_t REG_MAX_POSITION_LIMIT = 0x0B;
constexpr uint8_t REG_GOAL_POSITION = 0x2A;
constexpr uint8_t REG_GOAL_VELOCITY = 0x2E;
constexpr uint8_t REG_PRESENT_POSITION = 0x38;
constexpr uint8_t REG_TORQUE_ENABLE = 0x28;
constexpr speed_t SERVO_BAUD = B1000000;

uint8_t checksum(const uint8_t* packet, int first, int last) {
    uint32_t sum = 0;
    for (int index = first; index <= last; ++index) {
        sum += packet[index];
    }
    return static_cast<uint8_t>(~sum);
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

bool read_register(int fd, uint8_t servo_id, uint8_t address,
                   uint16_t& value) {
    tcflush(fd, TCIOFLUSH);

    uint8_t request[8] = {
        0xFF, 0xFF, servo_id, 0x04, INST_READ, address, 0x02, 0x00
    };
    request[7] = checksum(request, 2, 6);

    if (!write_packet(fd, request, sizeof(request))) {
        return false;
    }

    uint8_t response[8] = {};
    int total_bytes_read = 0;
    int attempts = 0;
    while (total_bytes_read < 8 && attempts < 20) {
        int bytes_read = read(fd, response + total_bytes_read,
                              sizeof(response) - total_bytes_read);
        if (bytes_read > 0) {
            total_bytes_read += bytes_read;
        } else {
            usleep(1000);
            ++attempts;
        }
    }

    if (total_bytes_read < 8 || response[0] != 0xFF ||
        response[1] != 0xFF || response[2] != servo_id ||
        checksum(response, 2, 6) != response[7]) {
        return false;
    }
    if (response[4] != 0x00) {
        std::cerr << "Servo Joint " << static_cast<int>(servo_id)
                  << " reported error code: " << static_cast<int>(response[4])
                  << std::endl;
    }

    value = response[5] | (static_cast<uint16_t>(response[6]) << 8);
    return true;
}
}

SerialPort::SerialPort(const std::string& port)
    : fd_(configure_serial_port(port.c_str())) {
	if (fd_ < 0) {
		// errno читается сразу: configure_serial_port ничего не меняет после
		// неудачного open/tcgetattr, но полагаться на это не стоит.
		throw std::runtime_error("Failed to open serial port " + port + ": " +
		                         std::strerror(errno));
	}
}

SerialPort::~SerialPort() noexcept {
	if (fd_ >= 0) {
		close(fd_);
	}
}

SerialPort::SerialPort(SerialPort&& other) noexcept : fd_(other.fd_) {
	other.fd_ = -1;
}

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
	if (this != &other) {
		if (fd_ >= 0) {
			close(fd_);
		}
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

    cfsetospeed(&tty, SERVO_BAUD);
    cfsetispeed(&tty, SERVO_BAUD);
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

bool read_servo_limits(int fd, uint8_t servo_id, ServoLimits& limits) {
    if (!read_register(fd, servo_id, REG_MIN_POSITION_LIMIT,
                       limits.min_position)) {
        std::cerr << "Joint " << static_cast<int>(servo_id)
                  << ": failed to read the min position register (0x09)"
                  << std::endl;
        return false;
    }
    if (!read_register(fd, servo_id, REG_MAX_POSITION_LIMIT,
                       limits.max_position)) {
        std::cerr << "Joint " << static_cast<int>(servo_id)
                  << ": failed to read the max position register (0x0B)"
                  << std::endl;
        return false;
    }
    if (limits.max_position <= limits.min_position) {
        std::cerr << "Joint " << static_cast<int>(servo_id)
                  << ": register limits are empty (" << limits.min_position
                  << ".." << limits.max_position << ")" << std::endl;
        return false;
    }
    return true;
}

bool read_all_servo_limits(int fd, ServoLimits limits[6]) {
    for (uint8_t servo_id = 1; servo_id <= 6; ++servo_id) {
        if (!read_servo_limits(fd, servo_id, limits[servo_id - 1])) {
            std::cerr << "Failed to read register limits from Joint "
                      << static_cast<int>(servo_id) << std::endl;
            return false;
        }
        usleep(1000);
    }
    return true;
}

bool read_joint_angle(int fd, uint8_t servo_id, const ServoLimits& limits,
                      float& angle_degrees) {
    uint16_t present_position = 0;
    if (!read_register(fd, servo_id, REG_PRESENT_POSITION, present_position)) {
        return false;
    }
    if (present_position < limits.min_position ||
        present_position > limits.max_position) {
        std::cerr << "Joint " << static_cast<int>(servo_id)
                  << " position is outside servo register limits: "
                  << present_position << " (limits " << limits.min_position
                  << ".." << limits.max_position << ")" << std::endl;
        return false;
    }

    angle_degrees = limits.position_to_degrees(present_position);
    return true;
}

bool write_register(int fd, uint8_t servo_id, uint8_t address, uint16_t value) {
    uint8_t packet[9] = {
        0xFF, 0xFF, servo_id, 0x05, INST_WRITE, address,
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF), 0x00
    };
    packet[8] = checksum(packet, 2, 7);

    return write_packet(fd, packet, sizeof(packet));
}

bool write_byte_register(int fd, uint8_t servo_id, uint8_t address,
                         uint8_t value) {
    uint8_t packet[8] = {
        0xFF, 0xFF, servo_id, 0x04, INST_WRITE, address, value, 0x00
    };
    packet[7] = checksum(packet, 2, 6);
    return write_packet(fd, packet, sizeof(packet));
}

bool disable_servo_torque(int fd, uint8_t servo_id) {
    if (!write_byte_register(fd, servo_id, REG_TORQUE_ENABLE, 0)) {
        std::cerr << "Failed to disable torque on Joint "
                  << static_cast<int>(servo_id) << std::endl;
        return false;
    }

    std::cout << "Torque disabled on Joint "
              << static_cast<int>(servo_id) << std::endl;
    return true;
}

bool write_joint_angle(int fd, uint8_t servo_id, float angle_degrees,
                       uint16_t speed, const ServoLimits& limits) {
    if (!std::isfinite(angle_degrees) || angle_degrees < 0.0f) return false;

    const float max_angle = limits.span_degrees();
    if (angle_degrees > max_angle) {
        std::cerr << "Angle " << angle_degrees << " is outside Joint "
                  << static_cast<int>(servo_id) << " range 0.." << max_angle
                  << " degrees" << std::endl;
        return false;
    }

    const uint16_t steps = limits.degrees_to_position(angle_degrees);
    if (!write_register(fd, servo_id, REG_GOAL_VELOCITY, speed)) {
        std::cerr << "Failed to transmit speed to Joint "
                  << static_cast<int>(servo_id) << std::endl;
        return false;
    }
    usleep(3000);

    if (!write_register(fd, servo_id, REG_GOAL_POSITION, steps)) {
        std::cerr << "Failed to transmit packet to Joint "
                  << static_cast<int>(servo_id) << std::endl;
        return false;
    }

    std::cout << "Joint " << static_cast<int>(servo_id)
              << " commanded to " << angle_degrees << " degrees ("
              << steps << " steps, speed " << speed << ")." << std::endl;
    return true;
}
