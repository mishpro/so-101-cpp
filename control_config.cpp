#include "control_config.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

namespace {
using json = nlohmann::json;

// Верхняя граница числа итераций: опечатка в конфиге не должна превращать
// нажатие клавиши в многосекундное зависание.
constexpr int kMaxAllowedIterations = 10000;
}  // namespace

std::string expand_home(const std::string& path) {
    if (path.rfind("~/", 0) != 0) return path;
    const char* home = std::getenv("HOME");
    return home == nullptr ? path : std::string(home) + path.substr(1);
}

bool load_config(const std::string& path, ControlConfig& config) {
    std::ifstream file(path);
    if (!file) return false;

    try {
        const json data = json::parse(file);
        config.port = data.at("port").get<std::string>();
        config.urdf_path = expand_home(data.at("urdf_path").get<std::string>());
        config.end_effector = data.at("end_effector").get<std::string>();
        const unsigned int speed = data.at("speed").get<unsigned int>();
        config.speed = static_cast<uint16_t>(speed);
        config.joint_speed_divisor =
            data.at("joint_speed_divisor").get<float>();
        config.cartesian_step_m = data.at("cartesian_step_m").get<double>();
        config.ik_damping = data.at("ik_damping").get<double>();
        config.ik_max_iterations = data.at("ik_max_iterations").get<int>();
        config.ik_position_tolerance_m =
            data.at("ik_position_tolerance_m").get<double>();
        config.ik_max_joint_step_rad =
            data.at("ik_max_joint_step_rad").get<double>();
        return !config.port.empty() && !config.end_effector.empty() &&
               speed >= 1 && speed <= 1000 &&
               std::isfinite(config.joint_speed_divisor) &&
               config.joint_speed_divisor > 0.0f &&
               std::isfinite(config.cartesian_step_m) &&
               config.cartesian_step_m > 0.0 &&
               std::isfinite(config.ik_damping) && config.ik_damping > 0.0 &&
               config.ik_max_iterations >= 1 &&
               config.ik_max_iterations <= kMaxAllowedIterations &&
               std::isfinite(config.ik_position_tolerance_m) &&
               config.ik_position_tolerance_m > 0.0 &&
               std::isfinite(config.ik_max_joint_step_rad) &&
               config.ik_max_joint_step_rad > 0.0;
    } catch (const std::exception& error) {
        std::cerr << "Invalid " << path << ": " << error.what() << std::endl;
        return false;
    }
}