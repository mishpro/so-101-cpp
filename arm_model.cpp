#include "arm_model.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace so101 {
namespace {

// Servo id -> имя сустава в URDF. Соответствие совпадает с блоками
// <transmission> в so101_irs_ksant.urdf (<actuator name="motorN">).
constexpr std::array<const char*, kServoCount> kServoJointNames{
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex",   "wrist_roll",    "gripper",
};

// Feetech STS3215: угол мотора считается как steps · 360 / 4095. Константа
// берётся из servo_bus.hpp — иначе URDF разойдётся с прошивкой.

int checked_index(int servo_id) {
	return (servo_id < 1 || servo_id > static_cast<int>(kServoCount))
	           ? -1
	           : servo_id - 1;
}

}  // namespace

double ArmModel::lower_position_limit(int servo_id) const {
	const int index = checked_index(servo_id);
	return index < 0 ? 0.0
	                 : model.lowerPositionLimit[
	                       model.joints[servo_joints[index]].idx_q()];
}

double ArmModel::upper_position_limit(int servo_id) const {
	const int index = checked_index(servo_id);
	return index < 0 ? 0.0
	                 : model.upperPositionLimit[
	                       model.joints[servo_joints[index]].idx_q()];
}

double ArmModel::servo_span_rad(const ServoLimits& limits) const {
	return degrees_to_radians(limits.span_degrees());
}

// С диапазонами URDF, совпадающими с диапазонами моторов (это гарантирует
// check_joint_limits), градус мотора и градус URDF совпадают: минимум мотора
// попадает в lower, максимум — в upper, и пересчёт — это только перевод
// градусов в радианы, без масштабирования.
double ArmModel::servo_degrees_to_q(int servo_id, double degrees) const {
	const int index = checked_index(servo_id);
	if (index < 0) return 0.0;

	const auto& joint = model.joints[servo_joints[index]];
	return model.lowerPositionLimit[joint.idx_q()] + degrees_to_radians(degrees);
}

double ArmModel::q_to_servo_degrees(int servo_id, double q) const {
	const int index = checked_index(servo_id);
	if (index < 0) return 0.0;

	const auto& joint = model.joints[servo_joints[index]];
	const double lower = model.lowerPositionLimit[joint.idx_q()];
	const double upper = model.upperPositionLimit[joint.idx_q()];
	if (upper <= lower) return 0.0;

	const double fraction = std::clamp((q - lower) / (upper - lower), 0.0, 1.0);
	return radians_to_degrees(fraction * (upper - lower));
}

bool load_arm_model(const std::string& urdf_path, const std::string& frame_name,
                    ArmModel& arm, std::string& error) {
	try {
		pinocchio::urdf::buildModel(urdf_path, arm.model);
	} catch (const std::exception& exception) {
		error = "Failed to build Pinocchio model from " + urdf_path + ": " +
		        exception.what();
		return false;
	}

	if (frame_name.empty()) {
		error = "End-effector frame name is empty";
		return false;
	}
	if (!arm.model.existFrame(frame_name)) {
		error = "URDF " + urdf_path + " does not contain frame " + frame_name;
		return false;
	}
	arm.frame_id = arm.model.getFrameId(frame_name);

	for (std::size_t servo_id = 0; servo_id < kServoCount; ++servo_id) {
		const char* name = kServoJointNames[servo_id];
		if (!arm.model.existJointName(name)) {
			error = "URDF " + urdf_path + " does not contain joint " + name;
			return false;
		}
		arm.servo_joints[servo_id] = arm.model.getJointId(name);
	}
	for (std::size_t index = 0; index < kIkJointCount; ++index) {
		arm.ik_joints[index] = arm.servo_joints[index];
	}

	arm.data = pinocchio::Data(arm.model);
	return true;
}

bool check_joint_limits(const ArmModel& arm,
                        const std::array<ServoLimits, kServoCount>& limits,
                        double tolerance_rad, std::string& error) {
	error.clear();
	for (std::size_t servo_id = 0; servo_id < kServoCount; ++servo_id) {
		if (limits[servo_id].max_position <= limits[servo_id].min_position) {
			error = "servo register limits of joint " +
			        std::to_string(servo_id + 1) + " are empty";
			return false;
		}

		const double urdf_span = arm.upper_position_limit(
		                             static_cast<int>(servo_id) + 1) -
		                         arm.lower_position_limit(
		                             static_cast<int>(servo_id) + 1);
		const double servo_span = arm.servo_span_rad(limits[servo_id]);
		if (std::abs(urdf_span - servo_span) <= tolerance_rad) continue;

		// Расхождения накапливаются: при предупреждении полезно увидеть все
		// суставы сразу, а не перезапускаться ради каждого по отдельности.
		std::ostringstream message;
		message << std::fixed << std::setprecision(2) << "joint "
		        << static_cast<int>(servo_id + 1) << " ("
		        << kServoJointNames[servo_id] << "): servo registers give "
		        << radians_to_degrees(servo_span) << " deg, URDF says "
		        << radians_to_degrees(urdf_span) << " deg";
		if (!error.empty()) error += "; ";
		error += message.str();
	}
	return error.empty();
}

IkResult solve_ik(ArmModel& arm, const Eigen::Vector3d& target,
                  const Eigen::VectorXd& initial_q, const IkParams& params) {
	IkResult result;
	result.q = initial_q;

	const auto current_error = [&arm, &target]() {
		return (target - arm.data.oMf[arm.frame_id].translation()).norm();
	};

	pinocchio::forwardKinematics(arm.model, arm.data, result.q);
	pinocchio::updateFramePlacements(arm.model, arm.data);
	result.initial_error_m = current_error();
	result.final_error_m = result.initial_error_m;
	result.converged = result.final_error_m <= params.position_tolerance_m;

	// Якобиан фрейма: шесть строк (xyz) и по одному столбцу на степень
	// свободы. Data::Matrix6x здесь не годится: у неё фиксированные шесть
	// столбцов, и при nv != 6 getFrameJacobian писал бы за её пределы —
	// в релизной сборке без проверок Eigen это тихая порча памяти.
	Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, arm.model.nv);

	// Несколько итераций Damped Least Squares позволяют приблизить конечную
	// точку к цели точнее, чем один шаг за вызов.
	while (result.iterations < params.max_iterations && !result.converged) {
		// Именно эта пара вызовов даёт правильный якобиан фрейма. Вариант
		// computeFrameJacobian(model, data, q, frame_id, rf, J) в pinocchio
		// 4.1.0 возвращает матрицу, не совпадающую с производной прямой
		// кинематики: нормы колонок расходились в разы, и решатель уходил
		// от цели. Совпадение проверяется тестом test_frame_jacobian_matches_finite_differences.
		pinocchio::computeJointJacobians(arm.model, arm.data, result.q);
		pinocchio::getFrameJacobian(arm.model, arm.data, arm.frame_id,
		                            pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
		const Eigen::Vector3d error =
		    target - arm.data.oMf[arm.frame_id].translation();

		// Берём только линейные строки якобиана и только суставы, входящие в
		// IK: wrist_roll и gripper исключены намеренно (см. kIkJointCount).
		Eigen::MatrixXd arm_jacobian(3, static_cast<Eigen::Index>(kIkJointCount));
		for (std::size_t index = 0; index < kIkJointCount; ++index) {
			arm_jacobian.col(index) = jacobian.topRows(3).col(
			    arm.model.joints[arm.ik_joints[index]].idx_v());
		}

		// Damped Least Squares устойчив к вырожденным положениям якобиана.
		Eigen::Matrix3d system = arm_jacobian * arm_jacobian.transpose();
		system += params.damping * params.damping * Eigen::Matrix3d::Identity();
		Eigen::VectorXd dq_arm =
		    arm_jacobian.transpose() * system.ldlt().solve(error);

		// Шаг ограничивается целиком, а не по одному суставу: покоординатное
		// ограничение меняет направление dq, и решатель перестаёт сходиться
		// (ошибка растёт от итерации к итерации). Однородное масштабирование
		// сохраняет направление решения.
		const double largest_step = dq_arm.cwiseAbs().maxCoeff();
		if (largest_step > params.max_joint_step_rad) {
			dq_arm *= params.max_joint_step_rad / largest_step;
		}

		Eigen::VectorXd dq = Eigen::VectorXd::Zero(arm.model.nv);
		for (std::size_t index = 0; index < kIkJointCount; ++index) {
			dq[arm.model.joints[arm.ik_joints[index]].idx_v()] = dq_arm[index];
		}

		// integrate обновляет конфигурацию Pinocchio. После каждой итерации
		// ограничиваем суставы диапазонами из URDF.
		result.q = pinocchio::integrate(arm.model, result.q, dq);
		for (std::size_t index = 0; index < kIkJointCount; ++index) {
			const auto joint = arm.model.joints[arm.ik_joints[index]];
			result.q[joint.idx_q()] =
			    std::clamp(result.q[joint.idx_q()],
			               arm.model.lowerPositionLimit[joint.idx_q()],
			               arm.model.upperPositionLimit[joint.idx_q()]);
		}
		++result.iterations;

		pinocchio::forwardKinematics(arm.model, arm.data, result.q);
		pinocchio::updateFramePlacements(arm.model, arm.data);
		result.final_error_m = current_error();
		result.converged = result.final_error_m <= params.position_tolerance_m;
	}

	return result;
}

}  // namespace so101
