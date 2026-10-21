#include "arm_model.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <optional>
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

// Матрица кососимметричного произведения: v × w = skew(v) · w.
Eigen::Matrix3d skew_matrix(const Eigen::Vector3d& v) {
	Eigen::Matrix3d m;
	m << 0.0, -v.z(), v.y(),
	     v.z(), 0.0, -v.x(),
	     -v.y(), v.x(), 0.0;
	return m;
}

// Решатель Damped Least Squares.
//
// Rows = 3 — только позиция точки; Rows = 6 — позиция и направление
// инструмента.
//
// Ветка для трёх строк повторяет прежний решатель как есть: та же
// арифметика, тот же порядок вычислений, тот же результат. Общий код с
// параметром шаблона даёт то же самое, но переписывать рабочий путь
// решателя ради «унификации» незачем: цена ошибки тут — неверная траектория
// руки, а не дублирование тридцати строк.
template <int Rows>
IkResult solve_ik_impl(ArmModel& arm, const Eigen::Vector3d& target,
                        const std::optional<Eigen::Vector3d>& tool_direction,
                        const Eigen::VectorXd& initial_q, const IkParams& params) {
	IkResult result;
	result.q = initial_q;

	const auto position_error = [&arm, &target]() {
		return target - arm.data.oMf[arm.frame_id].translation();
	};
	// Остаток по направлению: часть желаемого направления, перпендикулярная
	// текущей оси инструмента. Поворот вокруг самой оси направление не
	// меняет, поэтому такая составляющая смысла не имеет.
	//
	// Желаемое направление нормируется один раз и в обычный Vector3d: внутри
	// цикла оно не меняется, а ленивое выражение normalized() в этой задаче
	// лишнее.
	Eigen::Vector3d desired_direction = Eigen::Vector3d::UnitZ();
	if (tool_direction) {
		const Eigen::Vector3d raw = *tool_direction;
		const double length = raw.norm();
		if (length > 0.0) {
			desired_direction = raw / length;
		}
	}
	const auto direction_error = [&arm, &desired_direction]() {
		const Eigen::Vector3d axis = tool_axis(arm);
		// Результат материализуется в обычный вектор: возвращать выражение
		// Eigen нельзя — оно ссылается на локальную axis, которая умирает
		// вместе с лямбдой. Вызывающий получал мусор (в отладочной сборке
		// случайно правильный, в оптимизированной — нет).
		return Eigen::Vector3d(
		    desired_direction - axis.dot(desired_direction) * axis);
	};

	pinocchio::forwardKinematics(arm.model, arm.data, result.q);
	pinocchio::updateFramePlacements(arm.model, arm.data);
	result.initial_error_m = position_error().norm();
	result.final_error_m = result.initial_error_m;
	if constexpr (Rows == 6) {
		result.initial_orientation_error_rad = direction_error().norm();
		result.final_orientation_error_rad =
		    result.initial_orientation_error_rad;
	}
	result.converged =
	    result.final_error_m <= params.position_tolerance_m &&
	    result.final_orientation_error_rad <= params.orientation_tolerance_rad;

	// Якобиан фрейма: шесть строк (xyz) и по одному столбцу на степень
	// свободы. Data::Matrix6x здесь не годится: у неё фиксированные шесть
	// столбцов, и при nv != 6 getFrameJacobian писал бы за её пределы —
	// в релизной сборке без проверок Eigen это тихая порча памяти.
	Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, arm.model.nv);

	// Строки задачи сравниваются в своих допусках: остаток в единицу означает
	// «в допуске». Строка позиции остаётся единичной, строка направления
	// домножается на отношение допусков — так вклады сравнимы.
	double direction_weight = 1.0;
	if constexpr (Rows == 6) {
		direction_weight = params.position_tolerance_m /
		                   params.orientation_tolerance_rad;
	}

	// Несколько итераций Damped Least Squares позволяют приблизить конечную
	// точку к цели точнее, чем один шаг за вызов.
	while (result.iterations < params.max_iterations && !result.converged) {
		// Именно эта пара вызовов даёт правильный якобиан фрейма. Вариант
		// computeFrameJacobian(model, data, q, frame_id, rf, J) в pinocchio
		// 4.1.0 возвращает матрицу, не совпадающую с производной прямой
		// кинематики: нормы столбцов расходились в разы, и решатель уходил
		// от цели. Совпадение проверяется тестом test_frame_jacobian_matches_finite_differences.
		pinocchio::computeJointJacobians(arm.model, arm.data, result.q);
		pinocchio::getFrameJacobian(arm.model, arm.data, arm.frame_id,
		                            pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
		const Eigen::Vector3d error = position_error();

		// Берём только линейные строки якобиана и только суставы, входящие в
		// IK: wrist_roll и gripper исключены намеренно (см. kIkJointCount).
		//
		// Строки копируются в обычные векторы перед записью. Ленивое
		// выражение Eigen, присваиваемое в под-блок, — место, где значение
		// легко взять не оттуда, откуда ожидаешь; здесь это стоило долгого
		// поиска, и материализация стоит двух строк.
		Eigen::MatrixXd task_jacobian(Rows, static_cast<Eigen::Index>(kIkJointCount));
		for (std::size_t index = 0; index < kIkJointCount; ++index) {
			const auto joint = arm.model.joints[arm.ik_joints[index]];
			const Eigen::Vector3d linear = jacobian.topRows(3).col(joint.idx_v());
			task_jacobian.col(index).topRows(3) = linear;
		}
		Eigen::VectorXd task_error(Rows);
		task_error.head<3>() = error;
		if constexpr (Rows == 6) {
			// Ось инструмента меняется от угловой скорости: d_dot = ω × d =
			// -skew(d) · ω. Угловые строки якобиана LOCAL_WORLD_ALIGNED
			// выражены в мире — проверено тестом
			// test_frame_jacobian_angular_rows_match_finite_differences.
			const Eigen::Vector3d axis = tool_axis(arm);
			const Eigen::Matrix3d skew = skew_matrix(axis);
			for (std::size_t index = 0; index < kIkJointCount; ++index) {
				const auto joint = arm.model.joints[arm.ik_joints[index]];
				const Eigen::Vector3d angular =
				    -skew * jacobian.bottomRows(3).col(joint.idx_v());
				task_jacobian.col(index).bottomRows(3) = angular;
			}
			task_jacobian.bottomRows<3>() *= direction_weight;
			task_error.tail<3>() = direction_weight * direction_error();
		}

		// Damped Least Squares устойчив к вырожденным положениям якобиана.
		Eigen::Matrix<double, Rows, Rows> system =
		    task_jacobian * task_jacobian.transpose();
		system += params.damping * params.damping *
		          Eigen::Matrix<double, Rows, Rows>::Identity();
		Eigen::VectorXd dq_arm =
		    task_jacobian.transpose() * system.ldlt().solve(task_error);

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
		result.final_error_m = position_error().norm();
		if constexpr (Rows == 6) {
			result.final_orientation_error_rad = direction_error().norm();
		}
		result.converged =
		    result.final_error_m <= params.position_tolerance_m &&
		    result.final_orientation_error_rad <=
		        params.orientation_tolerance_rad;
	}

	return result;
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
	return solve_ik_impl<3>(arm, target, std::nullopt, initial_q, params);
}

IkResult solve_ik(ArmModel& arm, const Eigen::Vector3d& target,
                  const Eigen::Vector3d& tool_direction,
                  const Eigen::VectorXd& initial_q, const IkParams& params) {
	return solve_ik_impl<6>(arm, target, tool_direction, initial_q, params);
}

Eigen::Vector3d tool_axis(const ArmModel& arm) {
	const Eigen::Matrix3d frame_rotation = arm.data.oMf[arm.frame_id].rotation();
	// Третий столбец поворота — это ось фрейма в мире. Берём элементы по
	// одному, а не произведением матрицы на вектор: gcc 13 ругается на
	// ленивое вычисление произведения Eigen (ложное предупреждение
	// -Wmaybe-uninitialized), а сборка с -Werror такую руготу не прощает.
	return Eigen::Vector3d(frame_rotation(0, 2), frame_rotation(1, 2),
	                       frame_rotation(2, 2));
}

}  // namespace so101
