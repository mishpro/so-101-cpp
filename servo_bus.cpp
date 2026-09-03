#include "servo_bus.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

namespace {
constexpr uint8_t INST_READ = 0x02;
constexpr uint8_t INST_WRITE = 0x03;
constexpr uint8_t REG_GOAL_POSITION = 0x2A;
constexpr uint8_t REG_GOAL_VELOCITY = 0x2E;
constexpr uint8_t REG_PRESENT_POSITION = 0x38;
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

float read_joint_angle(int fd, uint8_t servo_id) {
    tcflush(fd, TCIOFLUSH);

    uint8_t request[8] = {
        0xFF, 0xFF, servo_id, 0x04, INST_READ,
        REG_PRESENT_POSITION, 0x02, 0x00
    };
    request[7] = checksum(request, 2, 6);

    if (!write_packet(fd, request, sizeof(request))) {
        std::cerr << "Failed to send read command to Joint "
                  << static_cast<int>(servo_id) << std::endl;
        return -1.0f;
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

    if (total_bytes_read < 8) {
        std::cerr << "Timeout or incomplete packet from Joint "
                  << static_cast<int>(servo_id) << " (Got "
                  << total_bytes_read << " bytes)" << std::endl;
        return -1.0f;
    }
    if (response[0] != 0xFF || response[1] != 0xFF || response[2] != servo_id) {
        std::cerr << "Invalid response frame from Joint "
                  << static_cast<int>(servo_id) << std::endl;
        return -1.0f;
    }
    if (checksum(response, 2, 6) != response[7]) {
        std::cerr << "Checksum mismatch from Joint "
                  << static_cast<int>(servo_id) << std::endl;
        return -1.0f;
    }
    if (response[4] != 0x00) {
        std::cerr << "Servo Joint " << static_cast<int>(servo_id)
                  << " reported error code: " << static_cast<int>(response[4])
                  << std::endl;
    }

    uint16_t steps = response[5] | (static_cast<uint16_t>(response[6]) << 8);
    return static_cast<float>(steps) * (360.0f / 4095.0f);
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

bool write_joint_angle(int fd, uint8_t servo_id, float angle_degrees,
                       uint16_t speed) {
    if (angle_degrees < 0.0f) angle_degrees = 0.0f;
    if (angle_degrees > 360.0f) angle_degrees = 360.0f;

    uint16_t steps = static_cast<uint16_t>(angle_degrees * (4095.0f / 360.0f));
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
