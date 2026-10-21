#pragma once

// Pinocchio 4.x: omnibus-заголовок. Точечные <pinocchio/multibody/data.hpp>
// и <pinocchio/multibody/model.hpp> больше не существуют как таковые —
// они переехали в deprecated/ и подключаются через шим, который сам
// pinocchio добавляет в include-путь.
#include <pinocchio/multibody.hpp>

#include <array>
#include <cstddef>
#include <string>

#include "servo_bus.hpp"

namespace so101 {

// Число сервоприводов определено в servo_bus.hpp: это свойство шины, и
// транспортному слою оно нужно не меньше, чем модели. Здесь имя доступно и
// в пространстве so101, чтобы код модели не зависел от того, где объявлено.
using ::kServoCount;

// Суставы, по которым решается IK. wrist_roll и gripper исключены намеренно:
//   - gripper — челюсть захвата, она находится ниже gripper_frame_link по
//     дереву кинематики и на положение целевого фрейма не влияет вообще
//     (её столбец в якобиане тождественно нулевой);
//   - wrist_roll сидит почти на оси TCP (плечо 7.9 мм/рад против 150-350 мм/рад
//     у остальных), поэтому в позиционной задаче он только шумит: минимально
//     нормированный DLS всё равно даёт ему ненулевой dq, и поворот накапливается
//     от нажатия к нажатию. Управляется отдельно, как -j5 в send.
inline constexpr std::size_t kIkJointCount = 4;

// Допуск расхождения диапазона мотора и диапазона URDF: 0.5°.
//
// Константа одна на весь проект намеренно. Пока порог жил отдельно в
// cartesian_control.cpp и дублировался в send.cpp, таблица --show-limits
// могла пометить расхождение там, где вердикт считал его допустимым: маркер
// сравнивал абсолютные отсчёты, вердикт — span. Теперь и подпись в таблице,
// и итоговая строка считают одно и то же.
inline constexpr double kJointLimitToleranceRad = 0.008726646259971648; // 0.5°

// M_PI не входит в стандарт C++17, поэтому константа своя.
inline constexpr double kPi = 3.14159265358979323846;

// Градусы в радианы и обратно. Один пересчёт на весь проект: иначе
// «180.0 / kPi» расползается по файлам и однажды разъедется с URDF.
[[nodiscard]] constexpr double degrees_to_radians(double degrees) noexcept {
	return degrees * kPi / 180.0;
}
[[nodiscard]] constexpr double radians_to_degrees(double radians) noexcept {
	return radians * 180.0 / kPi;
}

struct ArmModel {
	pinocchio::Model model;
	pinocchio::Data data;
	pinocchio::FrameIndex frame_id{};

	// Индекс = servo_id - 1. Полное соответствие всем шести моторам.
	std::array<pinocchio::JointIndex, kServoCount> servo_joints{};
	// Первые kIkJointCount элементов servo_joints — те, по которым решается IK.
	std::array<pinocchio::JointIndex, kIkJointCount> ik_joints{};

	// Границы сустава в радианах из URDF.
	[[nodiscard]] double lower_position_limit(int servo_id) const;
	[[nodiscard]] double upper_position_limit(int servo_id) const;

	// Перевод угла мотора (градусы от откалиброванного минимума) в q URDF и
	// обратно. Диапазон URDF обязан совпадать с диапазоном мотора — это
	// проверяет check_joint_limits.
	[[nodiscard]] double servo_degrees_to_q(int servo_id, double degrees) const;
	[[nodiscard]] double q_to_servo_degrees(int servo_id, double q) const;

	// Полный диапазон мотора в радианах: (max_position - min_position)·2π/4095.
	[[nodiscard]] double servo_span_rad(const ServoLimits& limits) const;
};

// Загружает URDF, находит целевой фрейм и строит соответствие servo <-> сустав.
[[nodiscard]] bool load_arm_model(const std::string& urdf_path,
                                  const std::string& frame_name, ArmModel& arm,
                                  std::string& error);

// Сверяет границы URDF с границами из регистров сервоприводов. URDF —
// источник истины для углов, поэтому расхождение здесь не повод отказываться
// работать, а повод предупредить. error заполняется всеми расхождениями
// сразу, а не только первым.
[[nodiscard]] bool check_joint_limits(
    const ArmModel& arm, const std::array<ServoLimits, kServoCount>& limits,
    double tolerance_rad, std::string& error);

struct IkParams {
	double damping{1e-4};
	int max_iterations{100};
	double position_tolerance_m{0.005};
	double max_joint_step_rad{0.1};
};

struct IkResult {
	Eigen::VectorXd q{Eigen::VectorXd::Zero(6)};
	double initial_error_m{0.0};
	double final_error_m{0.0};
	bool converged{false};
	int iterations{0};
};

// Решает позиционную задачу Damped Least Squares по суставам ik_joints
// (3x4), начиная с initial_q. Управление и состояние Pinocchio обновляются
// внутри: после вызова arm.data соответствует arm.q из результата.
[[nodiscard]] IkResult solve_ik(ArmModel& arm, const Eigen::Vector3d& target,
                                const Eigen::VectorXd& initial_q,
                                const IkParams& params);

}  // namespace so101