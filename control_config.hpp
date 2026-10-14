#pragma once

#include <cstdint>
#include <string>

struct ControlConfig {
    std::string port;
    std::string urdf_path;
    std::string end_effector;
    uint16_t speed;
    float joint_speed_divisor;
    double cartesian_step_m;
    double ik_damping;
    int ik_max_iterations;
    double ik_position_tolerance_m;
    double ik_max_joint_step_rad;
};

// Разворачивает ведущий ~/ в пути.
[[nodiscard]] std::string expand_home(const std::string& path);

// Читает конфигурацию и проверяет все поля. При ошибке печатает причину в
// stderr и возвращает false.
[[nodiscard]] bool load_config(const std::string& path, ControlConfig& config);