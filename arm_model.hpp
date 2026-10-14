#pragma once

// Pinocchio 4.x: omnibus-заголовок. Точечные <pinocchio/multibody/data.hpp>
// и <pinocchio/multibody/model.hpp> больше не существуют как таковые —
// они переехали в deprecated/ и подключаются через шим, который сам
// pinocchio добавляет в include-путь.
#include <pinocchio/multibody.hpp>

#include <array>
#include <string>

#include "servo_bus.hpp"

namespace so101 {

// SO-101: шесть сервоприводов с номерами 1..6.
inline constexpr std::size_t kServoCount = 6;

// Суставы, по которым решается IK. wrist_roll и gripper исключены намеренно:
//   - gripper — челюсть захвата, она находится ниже gripper_frame_link по
//     дереву кинематики и на положение целевого фрейма не влияет вообще
//     (её столбец в якобиане тождественно нулевой);
//   - wrist_roll сидит почти на оси TCP (плечо 7.9 мм/рад против 150-350 мм/рад
//     у остальных), поэтому в позиционной задаче он только шумит: минимально
//     нормированный DLS всё равно даёт ему ненулевой dq, и поворот накапливается
//     от нажатия к нажатию. Управляется отдельно, как -j5 в send.
inline constexpr std::size_t kIkJointCount = 4;

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

// Сверяет границы URDF с калибровкой моторов. Расхождение больше tolerance_rad
// означает, что URDF и servo_limits.json разошлись: такой файл надо пересоздать
// через tools/sync_urdf_limits.py.
[[nodiscard]] bool check_joint_limits(const ArmModel& arm,
                                      const ServoLimits limits[kServoCount],
                                      double tolerance_rad, std::string& error);

struct IkParams {
	double damping{1e-4};
	int max_iterations{100};
	double position_tolerance_m{0.005};
	double max_joint_step_rad{0.1};
};

struct IkResult {
	Eigen::VectorXd q;
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