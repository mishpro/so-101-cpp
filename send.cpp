#include "servo_bus.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <exception>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>

namespace {
using json = nlohmann::json;

struct SendConfig {
    std::string port;
    uint16_t speed;
    float joint_speed_divisor;
};

bool load_send_config(const std::string& path, SendConfig& config) {
    std::ifstream file(path);
    if (!file) return false;

    try {
        const json data = json::parse(file);
        config.port = data.at("port").get<std::string>();

        const unsigned int speed = data.at("speed").get<unsigned int>();
        if (speed < 1 || speed > 1000) return false;
        config.speed = static_cast<uint16_t>(speed);

        config.joint_speed_divisor =
            data.at("joint_speed_divisor").get<float>();
        return !config.port.empty() &&
               std::isfinite(config.joint_speed_divisor) &&
               config.joint_speed_divisor > 0.0f;
    } catch (const std::exception&) {
        return false;
    }
}

// Эталонный снимок границ: из него генерируется URDF и им же проверяется
// согласованность офлайн. Само управление роботом от него не зависит.
constexpr const char* kServoLimitsPath = "servo_limits.json";

// Печатает границы из регистров рядом с эталонным файлом и завершает
// вердиктом о расхождении. Раньше --show-limits показывал только файл, чем
// скрывал возможный разрыв между настройками моторов и описанием робота.
void print_limits(const ServoLimits registers[6]) {
    ServoLimits reference[6]{};
    const bool has_reference = load_servo_limits(kServoLimitsPath, reference);

    std::cout << "Joint | Source     | min step | max step | max deg\n";
    std::cout << "------+------------+----------+----------+---------\n";
    std::cout << std::fixed << std::setprecision(2);

    int mismatches = 0;
    for (uint8_t joint_id = 1; joint_id <= 6; ++joint_id) {
        const ServoLimits& from_registers = registers[joint_id - 1];
        std::cout << std::setw(5) << static_cast<int>(joint_id) << " | "
                  << std::setw(10) << "registers" << " | "
                  << std::setw(8) << from_registers.min_position << " | "
                  << std::setw(8) << from_registers.max_position << " | "
                  << std::setw(7) << from_registers.span_degrees() << '\n';

        if (!has_reference) continue;

        const ServoLimits& from_file = reference[joint_id - 1];
        const bool differs =
            from_file.min_position != from_registers.min_position ||
            from_file.max_position != from_registers.max_position;
        std::cout << "       | " << std::setw(10) << "reference" << " | " << std::setw(8) << from_file.min_position << " | "
                  << std::setw(8) << from_file.max_position << " | "
                  << std::setw(7) << from_file.span_degrees()
                  << (differs ? "  <- differs" : "") << '\n';
        if (differs) ++mismatches;
    }

    if (!has_reference) {
        std::cout << "\nNo reference file " << kServoLimitsPath
                  << "; create it on the stand with ./build/send --save-limits"
                  << std::endl;
        return;
    }
    if (mismatches == 0) {
        std::cout << "\nOK: servo registers match " << kServoLimitsPath
                  << std::endl;
        return;
    }

    std::cout << "\nWarning: " << mismatches
              << " joint(s) differ between the servo registers and "
              << kServoLimitsPath << ":\n";
    for (uint8_t joint_id = 1; joint_id <= 6; ++joint_id) {
        const float delta = registers[joint_id - 1].span_degrees() -
                            reference[joint_id - 1].span_degrees();
        if (std::fabs(delta) < 0.005f) continue;
        std::cout << "  joint " << static_cast<int>(joint_id) << ": registers "
                  << registers[joint_id - 1].span_degrees()
                  << " deg, reference "
                  << reference[joint_id - 1].span_degrees() << " deg, delta "
                  << (delta > 0 ? "+" : "") << delta << "\n";
    }
    std::cout << "The robot is driven by the registers. If they were re-tuned, "
                 "run\n  ./build/send --save-limits"
                 "\n  python3 tools/sync_urdf_limits.py" << std::endl;
}
}

int main(int argc, char** argv) {
    SendConfig config{};
    if (!load_send_config("send_config.json", config)) {
        std::cerr << "Failed to load valid send_config.json" << std::endl;
        return 1;
    }

    std::string port = config.port;
    uint16_t speed = config.speed;
    bool set_middle = false;
    bool set_base = false;
    bool disable_torque = false;
    bool show_limits = false;
    bool save_limits = false;
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
            if (set_middle || set_base || show_limits || save_limits ||
                !commands.empty()) {
                std::cerr << "--set-middle cannot be combined with another preset"
                          << std::endl;
                return 1;
            }
            set_middle = true;
            continue;
        }

        if (argument == "--set-base") {
            if (set_middle || set_base || show_limits || save_limits ||
                !commands.empty()) {
                std::cerr << "--set-base cannot be combined with another preset"
                          << std::endl;
                return 1;
            }
            set_base = true;
            continue;
        }

        if (argument == "--disable-torque") {
            if (disable_torque || show_limits || set_middle || set_base ||
                save_limits || !commands.empty()) {
                std::cerr << "--disable-torque cannot be combined with another command"
                          << std::endl;
                return 1;
            }
            disable_torque = true;
            continue;
        }

        if (argument == "--show-limits") {
            if (show_limits || disable_torque || set_middle || set_base ||
                save_limits || !commands.empty()) {
                std::cerr << "--show-limits cannot be combined with another command"
                          << std::endl;
                return 1;
            }
            show_limits = true;
            continue;
        }

        if (argument == "--save-limits") {
            if (save_limits || show_limits || disable_torque || set_middle ||
                set_base || !commands.empty()) {
                std::cerr << "--save-limits cannot be combined with another command"
                          << std::endl;
                return 1;
            }
            save_limits = true;
            continue;
        }

        if (show_limits || disable_torque || set_middle || set_base ||
            save_limits || argument.size() < 3 ||
            argument[0] != '-' ||
            argument[1] != 'j') {
            if (show_limits || disable_torque || set_middle || set_base ||
                save_limits) {
                std::cerr << "Command cannot be combined with joint commands"
                          << std::endl;
                return 1;
            }
            std::cerr << "Usage: " << argv[0]
                      << " [-p PORT] [-s SPEED] [--set-middle | --set-base | --disable-torque | --show-limits | --save-limits | -j1..-j6 ANGLE]..."
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

    if (commands.empty() && !set_middle && !set_base && !disable_torque &&
        !show_limits && !save_limits) {
        std::cout << "No joints specified. Nothing was sent." << std::endl;
        std::cout << "Usage: " << argv[0]
                  << " [-p PORT] [-s SPEED] [--set-middle | --set-base | --disable-torque | --show-limits | --save-limits | -j1..-j6 ANGLE]..."
                  << std::endl;
        return 0;
    }

    try {
        SerialPort port_handle(port);

        // Границы протокола — из регистров моторов. send ничего не
        // откалибровывает, поэтому и файл эталонной калибровки тут не
        // источник истины: иначе один и тот же физический угол ушёл бы на
        // разные отсчёты в зависимости от того, какая программа пишет.
        ServoLimits limits[6]{};
        if (!read_all_servo_limits(port_handle.fd(), limits)) {
            return 1;
        }

        std::cout << "Successfully connected to SO-101 Bus!" << std::endl;

        if (save_limits) {
            if (!save_servo_limits(kServoLimitsPath, limits)) {
                std::cerr << "Failed to write " << kServoLimitsPath
                          << std::endl;
                return 1;
            }
            std::cout << "Servo registers saved to " << kServoLimitsPath
                      << std::endl;
            std::cout << "Regenerate the URDF to match them: "
                         "python3 tools/sync_urdf_limits.py"
                      << std::endl;
            return 0;
        }

        if (show_limits) {
            print_limits(limits);
            return 0;
        }

        if (disable_torque) {
            bool all_commands_succeeded = true;
            for (uint8_t joint_id = 1; joint_id <= 6; ++joint_id) {
                if (!disable_servo_torque(port_handle.fd(), joint_id)) {
                    all_commands_succeeded = false;
                }
                usleep(10000);
            }
            return all_commands_succeeded ? 0 : 1;
        }

        if (set_middle) {
            for (uint8_t joint_id = 1; joint_id <= 6; ++joint_id) {
                commands.emplace_back(
                    joint_id, limits[joint_id - 1].span_degrees() / 2.0f);
            }
        }

        if (set_base) {
            constexpr float base_positions[] = {
                0.5f, 0.0f, 1.0f, 0.75f, 0.5f, 0.0f
            };
            for (uint8_t joint_id = 1; joint_id <= 6; ++joint_id) {
                commands.emplace_back(
                    joint_id,
                    limits[joint_id - 1].span_degrees() *
                        base_positions[joint_id - 1]);
            }
        }

        bool all_commands_succeeded = true;
        for (const auto& command : commands) {
            const float speed_divisor =
                std::pow(config.joint_speed_divisor, command.first - 1);
            const uint16_t joint_speed = std::max<uint16_t>(
                1, static_cast<uint16_t>(speed / speed_divisor));
            if (!write_joint_angle(port_handle.fd(), command.first, command.second,
                                   joint_speed, limits[command.first - 1])) {
                all_commands_succeeded = false;
            }
            usleep(10000);
        }

        return all_commands_succeeded ? 0 : 1;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << std::endl;
        return 1;
    }

    return 0;
}
