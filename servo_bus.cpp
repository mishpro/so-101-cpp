#include "servo_bus.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <regex>
#include <sstream>
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

int configure_serial_port(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Failed to open serial port " << port << ": "
                  << std::strerror(errno) << std::endl;
        return -1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Failed to get terminal attributes: "
                  << std::strerror(errno) << std::endl;
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
        std::cerr << "Failed to set terminal attributes: "
                  << std::strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

bool read_servo_limits(int fd, uint8_t servo_id, ServoLimits& limits) {
    if (!read_register(fd, servo_id, REG_MIN_POSITION_LIMIT,
                       limits.min_position) ||
        !read_register(fd, servo_id, REG_MAX_POSITION_LIMIT,
                       limits.max_position) ||
        limits.max_position <= limits.min_position) {
        return false;
    }
    return true;
}

bool load_servo_limits(const std::string& path, ServoLimits limits[6]) {
    std::ifstream file(path);
    if (!file) return false;

    std::stringstream contents;
    contents << file.rdbuf();
    const std::string text = contents.str();
    const std::regex entry(
        R"("joint"\s*:\s*(\d+)\s*,\s*"min"\s*:\s*(\d+)\s*,\s*"max"\s*:\s*(\d+))");
    bool found[6] = {};
    for (std::sregex_iterator it(text.begin(), text.end(), entry), end;
         it != end; ++it) {
        const int joint = std::stoi((*it)[1].str());
        const unsigned long min_position = std::stoul((*it)[2].str());
        const unsigned long max_position = std::stoul((*it)[3].str());
        if (joint < 1 || joint > 6 || min_position > UINT16_MAX ||
            max_position > UINT16_MAX || max_position <= min_position ||
            found[joint - 1]) {
            return false;
        }
        limits[joint - 1] = {static_cast<uint16_t>(min_position),
                              static_cast<uint16_t>(max_position)};
        found[joint - 1] = true;
    }
    for (bool joint_found : found) {
        if (!joint_found) return false;
    }
    return true;
}

bool save_servo_limits(const std::string& path, const ServoLimits limits[6]) {
    std::ofstream file(path);
    if (!file) return false;
    file << "{\n  \"servos\": [\n";
    for (int index = 0; index < 6; ++index) {
        file << "    { \"joint\": " << index + 1
             << ", \"min\": " << limits[index].min_position
             << ", \"max\": " << limits[index].max_position << " }"
             << (index == 5 ? "\n" : ",\n");
    }
    file << "  ]\n}\n";
    return file.good();
}

bool load_or_read_servo_limits(int fd, const std::string& path,
                               ServoLimits limits[6]) {
    if (load_servo_limits(path, limits)) return true;

    std::ifstream existing_file(path);
    if (existing_file) {
        std::cerr << "Invalid servo limits file: " << path << std::endl;
        return false;
    }

    for (uint8_t servo_id = 1; servo_id <= 6; ++servo_id) {
        if (!read_servo_limits(fd, servo_id, limits[servo_id - 1])) {
            std::cerr << "Failed to read limits from Joint "
                      << static_cast<int>(servo_id) << std::endl;
            return false;
        }
    }
    if (!save_servo_limits(path, limits)) {
        std::cerr << "Failed to save servo limits to " << path << std::endl;
        return false;
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
                  << " position is outside saved limits: " << present_position
                  << " (limits " << limits.min_position << ".."
                  << limits.max_position << ")" << std::endl;
        return false;
    }

    const float position_from_min = static_cast<float>(present_position) -
                                    static_cast<float>(limits.min_position);
    angle_degrees = position_from_min * (360.0f / 4095.0f);
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

    const float max_angle =
        static_cast<float>(limits.max_position - limits.min_position) *
        (360.0f / 4095.0f);
    if (angle_degrees > max_angle) {
        std::cerr << "Angle " << angle_degrees << " is outside Joint "
                  << static_cast<int>(servo_id) << " range 0.." << max_angle
                  << " degrees" << std::endl;
        return false;
    }

    const uint16_t steps = static_cast<uint16_t>(
        static_cast<float>(limits.min_position) +
        angle_degrees * (4095.0f / 360.0f));
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
