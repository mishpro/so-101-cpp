#include "arm_model.hpp"
#include "control_config.hpp"
#include "servo_bus.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <exception>
#include <optional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <unistd.h>

namespace {
constexpr const char* kConfigPath = "send_config.json";

// Пауза между командами разным суставам: шина получает время на передачу.
constexpr int kInterCommandDelayMicroseconds = 10000;

// Разница меньше половины последнего разряда печати не отличается от нуля,
// но знак минус читается как расхождение. Такие дельты обнуляем.
constexpr double kDeltaDisplayEpsilon = 0.005;

// Печатает границы мотора рядом с тем, что описано в URDF, и завершается
// вердиктом. Сравнение идёт против URDF: он и есть источник истины для углов,
// а файла калибровки в проекте нет — границы лежат в самих моторах.
void print_register_rows(const std::array<ServoLimits, kServoCount>& registers) {
    for (std::size_t index = 0; index < kServoCount; ++index) {
        const int servo_id = static_cast<int>(index + 1);
        const ServoLimits& from_registers = registers[index];
        std::cout << std::setw(5) << servo_id << " | "
                  << std::setw(12)
                  << (std::to_string(from_registers.min_position) + ".." +
                      std::to_string(from_registers.max_position))
                  << " | " << std::setw(7) << from_registers.span_degrees()
                  << " |\n";
    }
}

bool print_limits(const std::array<ServoLimits, kServoCount>& registers,
                  const ControlConfig& config) {
    const double tolerance_deg =
        so101::radians_to_degrees(so101::kJointLimitToleranceRad);

    std::cout << "Joint |  Registers  |  span,° |   URDF,° | delta,°\n";
    std::cout << "------+--------------+---------+----------+---------\n";
    std::cout << std::fixed << std::setprecision(2);

    so101::ArmModel arm{};
    std::string error;
    if (!so101::load_arm_model(config.urdf_path, config.end_effector, arm,
                               error)) {
        print_register_rows(registers);
        std::cout << std::fixed << std::setprecision(3) << "  | "
                  << config.urdf_path << " | " << "  (нет URDF, сравнение невозможно)\n";
        std::cerr << "\nURDF comparison impossible: " << error << '\n';
        std::cerr << "Границы URDF берутся из send_config.json -> urdf_path"
                  << '\n';
        return false;
    }

    // Сравниваются только ДИАПАЗОНЫ. Абсолютный минимум мотора восстановить из
    // URDF нельзя: там хранятся градусы, а не отсчёты, и lower остаётся тем,
    // что было в исходной модели. Поэтому строка URDF показывает диапазон
    // в градусах, а пометка ставится по разнице диапазонов — ровно с тем же
    // допуском, что и в итоговом вердикте.
    for (std::size_t index = 0; index < kServoCount; ++index) {
        const int servo_id = static_cast<int>(index + 1);
        const ServoLimits& from_registers = registers[index];
        const double span_rad =
            arm.upper_position_limit(servo_id) -
            arm.lower_position_limit(servo_id);
        const double urdf_degrees = so101::radians_to_degrees(span_rad);
        double delta = from_registers.span_degrees() - urdf_degrees;
        // Разница в тысячные градуса печатается как «-0.00», что читается как
        // расхождение, хотя его нет.
        if (std::fabs(delta) < kDeltaDisplayEpsilon) delta = 0.0;
        const bool differs =
            std::fabs(delta) >
                so101::radians_to_degrees(so101::kJointLimitToleranceRad);

        std::cout << std::setw(5) << servo_id << " | "
                  << std::setw(12)
                  << (std::to_string(from_registers.min_position) + ".." +
                      std::to_string(from_registers.max_position))
                  << " | " << std::setw(7) << from_registers.span_degrees() << " | "
                  << std::setw(8) << urdf_degrees << " | "
                  << std::showpos << std::setw(7) << delta
                  << std::noshowpos
                  << (differs ? "  <- differs" : "") << '\n';
    }

    std::string mismatches;
    const bool agree =
        so101::check_joint_limits(arm, registers,
                                  so101::kJointLimitToleranceRad, mismatches);
    if (agree) {
        std::cout << "\nOK: servo registers match " << config.urdf_path
                  << " (допуск " << tolerance_deg << "°)" << '\n';
        return true;
    }

    std::cout << '\n' << mismatches
              << "\nДопуск " << tolerance_deg << "°. Углы берутся из URDF, "
                 "поэтому робот работает, но описание устарело.\nОбновить:\n"
                 "  python3 tools/sync_urdf_limits.py" << '\n';
    return true;
}
}

int main(int argc, char** argv) {
    ControlConfig config{};
    if (!load_config(kConfigPath, config)) {
        std::cerr << "Failed to load valid " << kConfigPath << '\n';
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
        const std::string argument = argv[argument_index];

        if (argument == "-p") {
            if (argument_index + 1 >= argc) {
                std::cerr << "Missing port after -p" << '\n';
                return 1;
            }
            port = argv[++argument_index];
            continue;
        }

        if (argument == "-s") {
            if (argument_index + 1 >= argc) {
                std::cerr << "Missing speed after -s" << '\n';
                return 1;
            }
            char* speed_end = nullptr;
            long speed_value = std::strtol(argv[++argument_index], &speed_end, 10);
            if (*speed_end != '\0' ||
                speed_value < static_cast<long>(kMinMotorSpeed) ||
                speed_value > static_cast<long>(kMaxMotorSpeed)) {
                std::cerr << "Invalid speed: expected a value from "
                          << kMinMotorSpeed << " to " << kMaxMotorSpeed
                          << '\n';
                return 1;
            }
            speed = static_cast<uint16_t>(speed_value);
            continue;
        }

        if (argument == "--set-middle") {
            if (set_middle || set_base || show_limits || !commands.empty()) {
                std::cerr << "--set-middle cannot be combined with another preset"
                          << '\n';
                return 1;
            }
            set_middle = true;
            continue;
        }

        if (argument == "--set-base") {
            if (set_middle || set_base || show_limits || !commands.empty()) {
                std::cerr << "--set-base cannot be combined with another preset"
                          << '\n';
                return 1;
            }
            set_base = true;
            continue;
        }

        if (argument == "--disable-torque") {
            if (disable_torque || show_limits || set_middle || set_base ||
                !commands.empty()) {
                std::cerr << "--disable-torque cannot be combined with another command"
                          << '\n';
                return 1;
            }
            disable_torque = true;
            continue;
        }

        if (argument == "--show-limits") {
            if (show_limits || disable_torque || set_middle || set_base ||
                !commands.empty()) {
                std::cerr << "--show-limits cannot be combined with another command"
                          << '\n';
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
                          << '\n';
                return 1;
            }
            std::cerr << "Usage: " << argv[0]
                      << " [-p PORT] [-s SPEED] [--set-middle | --set-base | --disable-torque | --show-limits | -j1..-j6 ANGLE]..."
                      << '\n';
            return 1;
        }

        char* joint_end = nullptr;
        long joint_id = std::strtol(argument.c_str() + 2, &joint_end, 10);
        if (*joint_end != '\0' || joint_id < 1 || joint_id > 6 ||
            argument_index + 1 >= argc) {
            std::cerr << "Invalid joint option: " << argument << '\n';
            return 1;
        }

        char* angle_end = nullptr;
        float angle = std::strtof(argv[++argument_index], &angle_end);
        if (*angle_end != '\0' || !std::isfinite(angle) || angle < 0.0f) {
            std::cerr << "Invalid angle for " << argument
                      << ": expected a finite value from 0 degrees" << '\n';
            return 1;
        }

        commands.emplace_back(static_cast<uint8_t>(joint_id), angle);
    }

    if (commands.empty() && !set_middle && !set_base && !disable_torque &&
        !show_limits) {
        std::cout << "No joints specified. Nothing was sent." << '\n';
        std::cout << "Usage: " << argv[0]
                  << " [-p PORT] [-s SPEED] [--set-middle | --set-base | --disable-torque | --show-limits | -j1..-j6 ANGLE]..."
                  << '\n';
        return 0;
    }

    try {
        SerialPort port_handle(port);

        // Границы протокола — из регистров моторов. send ничего не
        // откалибровывает, поэтому и файл эталонной калибровки тут не
        // источник истины: иначе один и тот же физический угол ушёл бы на
        // разные отсчёты в зависимости от того, какая программа пишет.
        const std::optional<std::array<ServoLimits, kServoCount>> limits =
            read_all_servo_limits(port_handle);
        if (!limits) {
            return 1;
        }

        std::cout << "Successfully connected to SO-101 Bus!" << '\n';

        if (show_limits) {
            return print_limits(*limits, config) ? 0 : 1;
        }

        if (disable_torque) {
            bool all_commands_succeeded = true;
            for (std::size_t index = 0; index < kServoCount; ++index) {
                if (!disable_servo_torque(port_handle,
                                         static_cast<uint8_t>(index + 1))) {
                    all_commands_succeeded = false;
                }
                usleep(kInterCommandDelayMicroseconds);
            }
            return all_commands_succeeded ? 0 : 1;
        }

        if (set_middle) {
            for (std::size_t index = 0; index < kServoCount; ++index) {
                commands.emplace_back(static_cast<uint8_t>(index + 1),
                                      (*limits)[index].span_degrees() / 2.0f);
            }
        }

        if (set_base) {
            constexpr std::array<float, kServoCount> base_positions{
                0.5f, 0.0f, 1.0f, 0.75f, 0.5f, 0.0f
            };
            for (std::size_t index = 0; index < kServoCount; ++index) {
                commands.emplace_back(
                    static_cast<uint8_t>(index + 1),
                    (*limits)[index].span_degrees() * base_positions[index]);
            }
        }

        bool all_commands_succeeded = true;
        for (const auto& command : commands) {
            const uint16_t motor_speed = joint_speed(
                speed, config.joint_speed_divisor,
                static_cast<int>(command.first) - 1);
            if (!write_joint_angle(port_handle, command.first, command.second,
                                   motor_speed, (*limits)[command.first - 1])) {
                all_commands_succeeded = false;
            }
            usleep(kInterCommandDelayMicroseconds);
        }

        return all_commands_succeeded ? 0 : 1;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }

    return 0;
}
