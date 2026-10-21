#pragma once

#include <cstdint>
#include <string>

// Допустимый диапазон скорости мотора (регистр GOAL_VELOCITY). Живёт здесь,
// потому что проверяется в двух местах: при чтении конфига и при разборе
// ключа -s у send. Две копии правила разъехались бы при первой же правке.
inline constexpr unsigned kMinMotorSpeed = 1;
inline constexpr unsigned kMaxMotorSpeed = 1000;

struct ControlConfig {
    std::string port;
    std::string urdf_path;
    std::string end_effector;
    uint16_t speed{0};
    // double, а не float: делитель в степени номера сустава даёт скорость,
    // и во float 1.2 превращается в 1.20000004768 — скорость первого сустава
    // на делителе 1.2 выходила бы на шаг меньше.
    double joint_speed_divisor{0.0};
    double cartesian_step_m{0.0};
    double ik_damping{0.0};
    int ik_max_iterations{0};
    double ik_position_tolerance_m{0.0};
    double ik_max_joint_step_rad{0.0};
};

// Разворачивает ведущий ~/ в пути.
[[nodiscard]] std::string expand_home(const std::string& path);

// Читает конфигурацию и проверяет все поля. При ошибке печатает причину в
// stderr и возвращает false.
[[nodiscard]] bool load_config(const std::string& path, ControlConfig& config);

// Скорость мотора для сустава с номером joint_index (нумерация с нуля).
// Суставы дальше первого идут медленнее: делитель в степени номера.
//
// Делитель берётся из конфига и может оказаться меньше единицы, тогда
// скорость для дальних суставов уходит за пределы регистра. Поэтому
// результат ограничивается диапазоном мотора: приведение double к uint16_t
// вне диапазона — неопределённое поведение.
[[nodiscard]] uint16_t joint_speed(uint16_t base_speed, double divisor,
                                   int joint_index);
