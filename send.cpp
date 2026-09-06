#include "servo_bus.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>

int main(int argc, char** argv) {
    const char* port = "/dev/ttyACM0";
    uint16_t speed = 100;
    std::vector<std::pair<uint8_t, float>> commands;

    for (int argument_index = 1; argument_index < argc; ++argument_index) {
        std::string argument = argv[argument_index];

        if (argument == "-p") {
            if (argument_index + 1 >= argc) {
                std::cerr << "Missing port after -p" << std::endl;
                return 1;
            }
            port = argv[++argument_index];
            continue;
        }

        if (argument == "-s") {
            if (argument_index + 1 >= argc) {
                std::cerr << "Missing speed after -s" << std::endl;
                return 1;
            }
            char* speed_end = nullptr;
            long speed_value = std::strtol(argv[++argument_index], &speed_end, 10);
            if (*speed_end != '\0' || speed_value < 1 || speed_value > 1000) {
                std::cerr << "Invalid speed: expected a value from 1 to 1000"
                          << std::endl;
                return 1;
            }
            speed = static_cast<uint16_t>(speed_value);
            continue;
        }

        if (argument.size() < 3 || argument[0] != '-' || argument[1] != 'j') {
            std::cerr << "Usage: " << argv[0]
                      << " [-p PORT] [-s SPEED] [-j1..-j6 ANGLE]..."
                      << std::endl;
            return 1;
        }

        char* joint_end = nullptr;
        long joint_id = std::strtol(argument.c_str() + 2, &joint_end, 10);
        if (*joint_end != '\0' || joint_id < 1 || joint_id > 6 ||
            argument_index + 1 >= argc) {
            std::cerr << "Invalid joint option: " << argument << std::endl;
            return 1;
        }

        char* angle_end = nullptr;
        float angle = std::strtof(argv[++argument_index], &angle_end);
        if (*angle_end != '\0' || !std::isfinite(angle) || angle < 0.0f) {
            std::cerr << "Invalid angle for " << argument
                      << ": expected a finite value from 0 degrees" << std::endl;
            return 1;
        }

        commands.emplace_back(static_cast<uint8_t>(joint_id), angle);
    }

    if (commands.empty()) {
        std::cout << "No joints specified. Nothing was sent." << std::endl;
        std::cout << "Usage: " << argv[0]
                  << " [-p PORT] [-s SPEED] [-j1..-j6 ANGLE]..."
                  << std::endl;
        return 0;
    }

    int serial_fd = configure_serial_port(port);
    if (serial_fd < 0) return 1;

    ServoLimits limits[6]{};
    if (!load_or_read_servo_limits(serial_fd, "servo_limits.json", limits)) {
        close(serial_fd);
        return 1;
    }

    std::cout << "Successfully connected to SO-101 Bus!" << std::endl;

    for (const auto& command : commands) {
        write_joint_angle(serial_fd, command.first, command.second, speed,
                  limits[command.first - 1]);
        usleep(10000);
    }

    close(serial_fd);
    return 0;
}
