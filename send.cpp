#include "arm_model.hpp"
#include "control_config.hpp"
#include "servo_bus.hpp"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <exception>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr const char* kConfigPath = "send_config.json";

// Печатает границы регистров рядом с границами URDF и завершается вердиктом.
// Сравнение идёт против URDF: он и есть источник истины для углов, а файл
// калибровки в проекте нет вовсе — границы лежат в самих моторах.
void print_register_rows(const ServoLimits registers[6]) {
    for (uint8_t servo_id = 1; servo_id <= 6; ++servo_id) {
        const ServoLimits& from_registers = registers[servo_id - 1];
        std::cout << std::setw(5) << static_cast<int>(servo_id) << " | "
                  << std::setw(10) << "registers" << " | "
                  << std::setw(8) << from_registers.min_position << " | "
                  << std::setw(8) << from_registers.max_position << " | "
                  << std::setw(8) << from_registers.span_degrees() << '\n';
    }
}

bool print_limits(const ServoLimits registers[6], const ControlConfig& config) {
    std::cout << "Joint | Source    | min step | max step | span deg\n";
    std::cout << "------+------------+----------+----------+---------\n";
    std::cout << std::fixed << std::setprecision(2);

    so101::ArmModel arm{};
    std::string error;
    if (!so101::load_arm_model(config.urdf_path, config.end_effector, arm, error)) {
        print_register_rows(registers);
        std::cout << '\n';
        std::cerr << "URDF comparison impossible: " << error << '\n';
        std::cerr << "Границы URDF берутся из send_config.json -> urdf_path"
                  << std::endl;
        return false;
    }

    // Из URDF нельзя выписать отсчёты напрямую — там радианы. Обратный
    // пересчёт даёт то количество шагов, которое описал бы мотор с таким
    // диапазоном, и делает строки таблицы сопоставимыми.
    for (uint8_t servo_id = 1; servo_id <= 6; ++servo_id) {
        const ServoLimits& from_registers = registers[servo_id - 1];
        std::cout << std::setw(5) << static_cast<int>(servo_id) << " | "
                  << std::setw(10) << "registers" << " | "
                  << std::setw(8) << from_registers.min_position << " | "
                  << std::setw(8) << from_registers.max_position << " | "
                  << std::setw(8) << from_registers.span_degrees() << '\n';

        const double span =
            arm.upper_position_limit(servo_id) -
            arm.lower_position_limit(servo_id);
        const ServoLimits implied{0,
                                   static_cast<uint16_t>(std::lround(
                                       span * kStepsPerRadian))};
        const bool differs =
            implied.min_position != from_registers.min_position ||
            implied.max_position != from_registers.max_position;
        std::cout << "       | " << std::setw(10) << "urdf" << " | "
                  << std::setw(8) << implied.min_position << " | "
                  << std::setw(8) << implied.max_position << " | "
                  << std::setw(8) << implied.span_degrees()
                  << (differs ? "  <- differs" : "") << '\n';
    }

    std::string mismatches;
    const bool agree = so101::check_joint_limits(arm, registers, 0.5 * kPi / 180.0,
                                          mismatches);
    if (agree) {
        std::cout << "\nOK: servo registers match " << config.urdf_path
                  << std::endl;
        return true;
    }

    std::cout << '\n' << mismatches << '\n';
    std::cout << "Углы берутся из URDF, поэтому робот работает, но описание "
                 "устарело.\nОбновить:\n"
                 "  python3 tools/sync_urdf_limits.py" << std::endl;
    return true;
}
}

int main(int argc, char** argv) {
    ControlConfig config{};
    if (!load_config(kConfigPath, config)) {
        std::cerr << "Failed to load valid " << kConfigPath << std::endl;
        return 1;
    }

    std::string port = config.port;
    uint16_t speed = config.speed;
    bool set_middle = false;
    bool set_base = false;
    bool disable_torque = false;
    bool show_limits = false;
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
            if (set_middle || set_base || show_limits || !commands.empty()) {
                std::cerr << "--set-middle cannot be combined with another preset"
                          << std::endl;
                return 1;
            }
            set_middle = true;
            continue;
        }

        if (argument == "--set-base") {
            if (set_middle || set_base || show_limits || !commands.empty()) {
                std::cerr << "--set-base cannot be combined with another preset"
                          << std::endl;
                return 1;
            }
            set_base = true;
            continue;
        }

        if (argument == "--disable-torque") {
            if (disable_torque || show_limits || set_middle || set_base ||
                !commands.empty()) {
                std::cerr << "--disable-torque cannot be combined with another command"
                          << std::endl;
                return 1;
            }
            disable_torque = true;
            continue;
        }

        if (argument == "--show-limits") {
            if (show_limits || disable_torque || set_middle || set_base ||
                !commands.empty()) {
                std::cerr << "--show-limits cannot be combined with another command"
                          << std::endl;
                return 1;
            }
            show_limits = true;
            continue;
        }

        if (show_limits || disable_torque || set_middle || set_base ||
            argument.size() < 3 ||
            argument[0] != '-' ||
            argument[1] != 'j') {
            if (show_limits || disable_torque || set_middle || set_base) {
                std::cerr << "Command cannot be combined with joint commands"
                          << std::endl;
                return 1;
            }
            std::cerr << "Usage: " << argv[0]
                      << " [-p PORT] [-s SPEED] [--set-middle | --set-base | --disable-torque | --show-limits | -j1..-j6 ANGLE]..."
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
        !show_limits) {
        std::cout << "No joints specified. Nothing was sent." << std::endl;
        std::cout << "Usage: " << argv[0]
                  << " [-p PORT] [-s SPEED] [--set-middle | --set-base | --disable-torque | --show-limits | -j1..-j6 ANGLE]..."
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

        if (show_limits) {
            return print_limits(limits, config) ? 0 : 1;
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
