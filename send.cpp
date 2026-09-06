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
    bool set_middle = false;
    bool set_base = false;
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

        if (argument == "--set-middle") {
            if (set_middle || set_base || !commands.empty()) {
                std::cerr << "--set-middle cannot be combined with another preset"
                          << std::endl;
                return 1;
            }
            set_middle = true;
            continue;
        }

        if (argument == "--set-base") {
            if (set_middle || set_base || !commands.empty()) {
                std::cerr << "--set-base cannot be combined with another preset"
                          << std::endl;
                return 1;
            }
            set_base = true;
            continue;
        }

        if (set_middle || set_base || argument.size() < 3 || argument[0] != '-' ||
            argument[1] != 'j') {
            if (set_middle || set_base) {
                std::cerr << "Preset cannot be combined with joint commands"
                          << std::endl;
                return 1;
            }
            std::cerr << "Usage: " << argv[0]
                      << " [-p PORT] [-s SPEED] [--set-middle | --set-base | -j1..-j6 ANGLE]..."
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

    if (commands.empty() && !set_middle && !set_base) {
        std::cout << "No joints specified. Nothing was sent." << std::endl;
        std::cout << "Usage: " << argv[0]
                  << " [-p PORT] [-s SPEED] [--set-middle | --set-base | -j1..-j6 ANGLE]..."
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

    if (set_middle) {
        for (uint8_t joint_id = 1; joint_id <= 6; ++joint_id) {
            const ServoLimits& joint_limits = limits[joint_id - 1];
            const float middle_angle =
                static_cast<float>(joint_limits.max_position -
                                   joint_limits.min_position) *
                (180.0f / 4095.0f);
            commands.emplace_back(joint_id, middle_angle);
        }
    }

    if (set_base) {
        constexpr float base_positions[] = {
            0.5f, 0.0f, 1.0f, 0.75f, 0.5f, 0.0f
        };
        for (uint8_t joint_id = 1; joint_id <= 6; ++joint_id) {
            const ServoLimits& joint_limits = limits[joint_id - 1];
            const float range_degrees =
                static_cast<float>(joint_limits.max_position -
                                   joint_limits.min_position) *
                (360.0f / 4095.0f);
            commands.emplace_back(
                joint_id, range_degrees * base_positions[joint_id - 1]);
        }
    }

    bool all_commands_succeeded = true;
    for (const auto& command : commands) {
        if (!write_joint_angle(serial_fd, command.first, command.second, speed,
                               limits[command.first - 1])) {
            all_commands_succeeded = false;
        }
        usleep(10000);
    }

    close(serial_fd);
    return all_commands_succeeded ? 0 : 1;
}
