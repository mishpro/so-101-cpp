#include "servo_bus.hpp"
#include "cartesian_ik.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <termios.h>
#include <vector>
#include <unistd.h>

namespace {
using json = nlohmann::json;

struct SendConfig {
    std::string port;
    uint16_t speed;
    float joint_speed_divisor;
    CartesianIkConfig cartesian_ik;
    double cartesian_step;
};

std::string expand_home(const std::string& path) {
    if (path.rfind("~/", 0) != 0) return path;
    const char* home = std::getenv("HOME");
    return home == nullptr ? path : std::string(home) + path.substr(1);
}

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
        config.cartesian_ik.urdf_path =
            expand_home(data.at("urdf_path").get<std::string>());
        config.cartesian_ik.end_effector_frame =
            data.at("end_effector_frame").get<std::string>();
        config.cartesian_ik.max_iterations =
            data.at("ik_max_iterations").get<int>();
        config.cartesian_ik.damping = data.at("ik_damping").get<double>();
        config.cartesian_ik.position_tolerance =
            data.at("ik_position_tolerance").get<double>();
        config.cartesian_step = data.at("cartesian_step").get<double>();
        return !config.port.empty() &&
               std::isfinite(config.joint_speed_divisor) &&
               config.joint_speed_divisor > 0.0f &&
               !config.cartesian_ik.urdf_path.empty() &&
               !config.cartesian_ik.end_effector_frame.empty() &&
               config.cartesian_ik.max_iterations > 0 &&
               std::isfinite(config.cartesian_ik.damping) &&
               config.cartesian_ik.damping > 0.0 &&
               std::isfinite(config.cartesian_ik.position_tolerance) &&
               config.cartesian_ik.position_tolerance > 0.0 &&
               std::isfinite(config.cartesian_step) &&
               config.cartesian_step > 0.0;
    } catch (const std::exception&) {
        return false;
    }
}

void print_send_config(const SendConfig& config) {
    std::cout << "Configuration:" << std::endl
              << "  port: " << config.port << std::endl
              << "  speed: " << config.speed << std::endl
              << "  joint_speed_divisor: " << config.joint_speed_divisor
              << std::endl
              << "  urdf_path: " << config.cartesian_ik.urdf_path << std::endl
              << "  end_effector_frame: "
              << config.cartesian_ik.end_effector_frame << std::endl
              << "  ik_max_iterations: "
              << config.cartesian_ik.max_iterations << std::endl
              << "  ik_damping: " << config.cartesian_ik.damping << std::endl
              << "  ik_position_tolerance: "
              << config.cartesian_ik.position_tolerance << std::endl
              << "  cartesian_step: " << config.cartesian_step << std::endl;
}

class RawTerminal {
public:
    bool enable() {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0) {
            return false;
        }
        termios raw = saved_;
        raw.c_lflag &= ~(ICANON | ECHO | ISIG);
        raw.c_cc[VMIN] = 1;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return false;
        enabled_ = true;
        return true;
    }

    ~RawTerminal() {
        if (enabled_) tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    }

private:
    termios saved_{};
    bool enabled_ = false;
};

bool send_cartesian_commands(int fd, uint16_t speed, const SendConfig& config,
                             const ServoLimits limits[6],
                             const std::array<double, 6>& joint_angles) {
    constexpr double radians_to_degrees = 180.0 / 3.14159265358979323846;
    constexpr std::array<double, 6> urdf_lower_limits = {
        -1.91986, -1.74533, -1.69, -1.65806, -2.74385, -0.174533
    };

    bool succeeded = true;
    for (uint8_t joint_id = 1; joint_id <= 6; ++joint_id) {
        const float angle = static_cast<float>(
            (joint_angles[joint_id - 1] -
             urdf_lower_limits[joint_id - 1]) * radians_to_degrees);
        const float speed_divisor =
            std::pow(config.joint_speed_divisor, joint_id - 1);
        const uint16_t joint_speed = std::max<uint16_t>(
            1, static_cast<uint16_t>(speed / speed_divisor));
        if (!write_joint_angle(fd, joint_id, angle, joint_speed,
                               limits[joint_id - 1])) {
            succeeded = false;
        }
        usleep(10000);
    }
    return succeeded;
}

std::array<float, 6> cartesian_target_degrees(
    const std::array<double, 6>& joint_angles) {
    constexpr double radians_to_degrees = 180.0 / 3.14159265358979323846;
    constexpr std::array<double, 6> urdf_lower_limits = {
        -1.91986, -1.74533, -1.69, -1.65806, -2.74385, -0.174533
    };

    std::array<float, 6> result{};
    for (uint8_t joint_id = 1; joint_id <= 6; ++joint_id) {
        result[joint_id - 1] = static_cast<float>(
            (joint_angles[joint_id - 1] -
             urdf_lower_limits[joint_id - 1]) * radians_to_degrees);
    }
    return result;
}

void print_cartesian_joint_status(
    char key, const std::array<double, 3>& target_position,
    const std::array<double, 3>& current_position,
    const std::array<float, 6>& current_angles,
    const std::array<float, 6>* target_angles) {
    std::cout << "\nButton: " << key << "\nCurrent XYZ: ["
              << current_position[0] << ", " << current_position[1] << ", "
              << current_position[2] << "]\nCurrent angles: ";
    for (float angle : current_angles) std::cout << angle << " ";
    std::cout << "\nTarget XYZ: ["
              << target_position[0] << ", " << target_position[1] << ", "
              << target_position[2] << "]\nTarget Angles: ";
    if (target_angles == nullptr) {
        std::cout << "unavailable";
    } else {
        for (float angle : *target_angles) std::cout << angle << " ";
    }
    std::cout << std::endl;
}

int run_cartesian_control(int fd, uint16_t speed, const SendConfig& config,
                          const ServoLimits limits[6]) {
    constexpr double degrees_to_radians =
        3.14159265358979323846 / 180.0;
    constexpr std::array<double, 6> urdf_lower_limits = {
        -1.91986, -1.74533, -1.69, -1.65806, -2.74385, -0.174533
    };

    std::array<float, 6> initial_angles{};
    for (uint8_t joint_id = 1; joint_id <= 6; ++joint_id) {
        if (!read_joint_angle(fd, joint_id, limits[joint_id - 1],
                              initial_angles[joint_id - 1])) {
            std::cerr << "Failed to read current angle for joint "
                      << static_cast<int>(joint_id) << std::endl;
            return 1;
        }
    }

    std::array<double, 6> initial_joint_radians{};
    for (size_t index = 0; index < initial_joint_radians.size(); ++index) {
        initial_joint_radians[index] =
            urdf_lower_limits[index] +
            static_cast<double>(initial_angles[index]) * degrees_to_radians;
    }

    std::array<double, 3> target{};
    std::string ik_error;
    if (!get_cartesian_position(config.cartesian_ik, initial_joint_radians,
                                target, ik_error)) {
        std::cerr << ik_error << std::endl;
        return 1;
    }

    RawTerminal terminal;
    if (!terminal.enable()) {
        std::cerr << "Interactive Cartesian control requires a terminal"
                  << std::endl;
        return 1;
    }

    std::cout << "Cartesian control: T/G X, D/A Y, W/S Z, Q quit"
              << std::endl;
    std::cout << std::fixed << std::setprecision(3)
              << "Target: [" << target[0] << ", " << target[1] << ", "
              << target[2] << "]" << std::endl;

    while (true) {
        char key = '\0';
        const ssize_t bytes_read = read(STDIN_FILENO, &key, 1);
        if (bytes_read < 0 && errno == EINTR) continue;
        if (bytes_read != 1) {
            std::cerr << "Failed to read keyboard input" << std::endl;
            return 1;
        }
        if (key == 'q' || key == 'Q') break;

        std::array<double, 3> next_target = target;
        if (key == 't' || key == 'T') next_target[0] += config.cartesian_step;
        else if (key == 'g' || key == 'G') next_target[0] -= config.cartesian_step;
        else if (key == 'd' || key == 'D') next_target[1] += config.cartesian_step;
        else if (key == 'a' || key == 'A') next_target[1] -= config.cartesian_step;
        else if (key == 'w' || key == 'W') next_target[2] += config.cartesian_step;
        else if (key == 's' || key == 'S') next_target[2] -= config.cartesian_step;
        else continue;

        std::array<float, 6> current_angles{};
        for (uint8_t joint_id = 1; joint_id <= 6; ++joint_id) {
            if (!read_joint_angle(fd, joint_id, limits[joint_id - 1],
                                  current_angles[joint_id - 1])) {
                std::cerr << "\nFailed to read current angle for joint "
                          << static_cast<int>(joint_id) << std::endl;
                return 1;
            }
        }

        std::array<double, 6> current_joint_radians{};
        for (size_t index = 0; index < current_joint_radians.size(); ++index) {
            current_joint_radians[index] =
                urdf_lower_limits[index] +
                static_cast<double>(current_angles[index]) * degrees_to_radians;
        }

        std::array<double, 3> current_position{};
        if (!get_cartesian_position(config.cartesian_ik, current_joint_radians,
                                    current_position, ik_error)) {
            std::cerr << "\n" << ik_error << std::endl;
            return 1;
        }
        print_cartesian_joint_status(key, next_target, current_position,
                                     current_angles, nullptr);

        std::array<double, 6> joint_angles{};
        if (!solve_cartesian_position_from_angles(
                config.cartesian_ik, next_target, current_joint_radians,
                joint_angles, ik_error)) {
            print_cartesian_joint_status(key, next_target, current_position,
                                         current_angles, nullptr);
            std::cerr << "\n" << ik_error << std::endl;
            continue;
        }
        const std::array<float, 6> target_angles =
            cartesian_target_degrees(joint_angles);
        print_cartesian_joint_status(key, next_target, current_position,
                                     current_angles, &target_angles);
        if (!send_cartesian_commands(fd, speed, config, limits, joint_angles)) {
            return 1;
        }
        target = next_target;
        std::cout << "\rTarget: [" << target[0] << ", " << target[1] << ", "
                  << target[2] << "]   " << std::flush;
    }
    std::cout << std::endl;
    return 0;
}
}

int main(int argc, char** argv) {
    SendConfig config{};
    if (!load_send_config("send_config.json", config)) {
        std::cerr << "Failed to load valid send_config.json" << std::endl;
        return 1;
    }
    print_send_config(config);

    std::string port = config.port;
    uint16_t speed = config.speed;
    bool set_middle = false;
    bool set_base = false;
    bool disable_torque = false;
    bool show_limits = false;
    bool cartesian = false;
    bool cartesian_control = false;
    std::array<double, 3> cartesian_target{};
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

        if (argument == "--cartesian") {
            if (cartesian || cartesian_control || set_middle || set_base || disable_torque ||
                show_limits || !commands.empty() || argument_index + 3 >= argc) {
                std::cerr << "Invalid combination or arguments for --cartesian"
                          << std::endl;
                return 1;
            }
            for (size_t coordinate = 0; coordinate < cartesian_target.size();
                 ++coordinate) {
                char* coordinate_end = nullptr;
                cartesian_target[coordinate] =
                    std::strtod(argv[++argument_index], &coordinate_end);
                if (*coordinate_end != '\0' ||
                    !std::isfinite(cartesian_target[coordinate])) {
                    std::cerr << "Invalid Cartesian coordinate" << std::endl;
                    return 1;
                }
            }
            cartesian = true;
            continue;
        }

        if (argument == "--cartesian-control") {
            if (cartesian || cartesian_control || set_middle || set_base ||
                disable_torque || show_limits || !commands.empty()) {
                std::cerr << "--cartesian-control cannot be combined with another command"
                          << std::endl;
                return 1;
            }
            cartesian_control = true;
            continue;
        }

        if (argument == "--set-middle") {
            if (set_middle || set_base || cartesian || cartesian_control ||
                !commands.empty()) {
                std::cerr << "--set-middle cannot be combined with another preset"
                          << std::endl;
                return 1;
            }
            set_middle = true;
            continue;
        }

        if (argument == "--set-base") {
            if (set_middle || set_base || cartesian || cartesian_control ||
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
                cartesian || cartesian_control ||
                !commands.empty()) {
                std::cerr << "--disable-torque cannot be combined with another command"
                          << std::endl;
                return 1;
            }
            disable_torque = true;
            continue;
        }

        if (argument == "--show-limits") {
            if (show_limits || disable_torque || set_middle || set_base || cartesian ||
                cartesian_control ||
                !commands.empty()) {
                std::cerr << "--show-limits cannot be combined with another command"
                          << std::endl;
                return 1;
            }
            show_limits = true;
            continue;
        }

        if (show_limits || disable_torque || set_middle || set_base || cartesian ||
            argument.size() < 3 ||
            argument[0] != '-' ||
            argument[1] != 'j') {
            if (show_limits || disable_torque || set_middle || set_base) {
                std::cerr << "Command cannot be combined with joint commands"
                          << std::endl;
                return 1;
            }
            std::cerr << "Usage: " << argv[0]
                      << " [-p PORT] [-s SPEED] [--set-middle | --set-base | --disable-torque | --show-limits | --cartesian X Y Z | --cartesian-control | -j1..-j6 ANGLE]..."
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
        !show_limits && !cartesian && !cartesian_control) {
        std::cout << "No joints specified. Nothing was sent." << std::endl;
        std::cout << "Usage: " << argv[0]
                  << " [-p PORT] [-s SPEED] [--set-middle | --set-base | --disable-torque | --show-limits | --cartesian X Y Z | --cartesian-control | -j1..-j6 ANGLE]..."
                  << std::endl;
        return 0;
    }

    if (cartesian) {
        std::array<double, 6> joint_angles{};
        std::string ik_error;
        if (!solve_cartesian_position(config.cartesian_ik, cartesian_target,
                                      joint_angles, ik_error)) {
            std::cerr << ik_error << std::endl;
            return 1;
        }
        constexpr double radians_to_degrees = 180.0 / 3.14159265358979323846;
        constexpr std::array<double, 6> urdf_lower_limits = {
            -1.91986, -1.74533, -1.69, -1.65806, -2.74385, -0.174533
        };
        for (uint8_t joint_id = 1; joint_id <= 6; ++joint_id) {
            const float angle = static_cast<float>(
                (joint_angles[joint_id - 1] -
                 urdf_lower_limits[joint_id - 1]) * radians_to_degrees);
            commands.emplace_back(joint_id, angle);
        }
    }

    int serial_fd = configure_serial_port(port.c_str());
    if (serial_fd < 0) return 1;

    ServoLimits limits[6]{};
    if (!load_or_read_servo_limits(serial_fd, "servo_limits.json", limits)) {
        close(serial_fd);
        return 1;
    }

    std::cout << "Successfully connected to SO-101 Bus!" << std::endl;

    if (show_limits) {
        std::cout << "Joint | min degrees | max degrees | min step | max step"
                  << std::endl;
        std::cout << "------+-------------+-------------+----------+---------"
                  << std::endl;
        std::cout << std::fixed << std::setprecision(2);
        for (uint8_t joint_id = 1; joint_id <= 6; ++joint_id) {
            const ServoLimits& joint_limits = limits[joint_id - 1];
            const float max_angle =
                static_cast<float>(joint_limits.max_position -
                                   joint_limits.min_position) *
                (360.0f / 4095.0f);
            std::cout << std::setw(5) << static_cast<int>(joint_id) << " | "
                      << std::setw(11) << 0.0f << " | "
                      << std::setw(11) << max_angle << " | "
                      << std::setw(8) << joint_limits.min_position << " | "
                      << std::setw(8) << joint_limits.max_position << std::endl;
        }
        close(serial_fd);
        return 0;
    }

    if (disable_torque) {
        bool all_commands_succeeded = true;
        for (uint8_t joint_id = 1; joint_id <= 6; ++joint_id) {
            if (!disable_servo_torque(serial_fd, joint_id)) {
                all_commands_succeeded = false;
            }
            usleep(10000);
        }
        close(serial_fd);
        return all_commands_succeeded ? 0 : 1;
    }

    if (cartesian_control) {
        const int result = run_cartesian_control(serial_fd, speed, config, limits);
        close(serial_fd);
        return result;
    }

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
        const float speed_divisor =
            std::pow(config.joint_speed_divisor, command.first - 1);
        const uint16_t joint_speed = std::max<uint16_t>(
            1, static_cast<uint16_t>(speed / speed_divisor));
        if (!write_joint_angle(serial_fd, command.first, command.second,
                               joint_speed, limits[command.first - 1])) {
            all_commands_succeeded = false;
        }
        usleep(10000);
    }

    close(serial_fd);
    return all_commands_succeeded ? 0 : 1;
}
