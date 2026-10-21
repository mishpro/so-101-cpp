#include "drawing_execution.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <unistd.h>

namespace so101 {
namespace {

constexpr double kPiExecution = 3.14159265358979323846;

// Угол между осью инструмента и вертикалью вниз, в градусах.
double tilt_degrees(const Eigen::Vector3d& axis) {
	const double cosine = std::clamp(-axis.z(), -1.0, 1.0);
	return std::acos(cosine) * 180.0 / kPiExecution;
}

// Переводит показания моторов в конфигурацию Pinocchio.
Eigen::VectorXd angles_to_q(const ArmModel& model, const JointDegrees& degrees) {
	Eigen::VectorXd q = pinocchio::neutral(model.model);
	for (std::size_t index = 0; index < kServoCount; ++index) {
		const int servo_id = static_cast<int>(index) + 1;
		const auto joint = model.model.joints[model.servo_joints[index]];
		q[joint.idx_q()] =
		    model.servo_degrees_to_q(servo_id, degrees[index]);
	}
	return q;
}

// Насколько поза отличается от требуемой: в градусах по худшему суставу.
double worst_joint_difference(const JointDegrees& actual,
                               const JointDegrees& wanted) {
	double worst = 0.0;
	for (std::size_t index = 0; index < kServoCount; ++index) {
		worst = std::max(worst, std::abs(actual[index] - wanted[index]));
	}
	return worst;
}

// Панель на экране: в терминале перерисовывается на месте, в потоке без
// терминала печатается одной строкой, чтобы не утонуть в логе.
class Panel {
public:
	Panel(std::ostream& stream, bool interactive)
	    : stream_(stream), interactive_(interactive) {}

	void update(const std::vector<std::string>& lines) {
		if (!interactive_) {
			// Без терминала панель не рисуем: она утонула бы в логе.
			return;
		}
		if (drawn_ > 0) {
			stream_ << "\033[" << drawn_ << "A";
		}
		for (const std::string& line : lines) {
			stream_ << "\033[K" << line << '\n';
		}
		stream_.flush();
		drawn_ = lines.size();
	}

	// Убирает панель с экрана: курсор остаётся там, где был текст.
	void clear() {
		if (!interactive_ || drawn_ == 0) return;
		stream_ << "\033[" << drawn_ << "A";
		for (std::size_t index = 0; index < drawn_; ++index) {
			stream_ << "\033[K\n";
		}
		stream_ << "\033[" << drawn_ << "A" << std::flush;
		drawn_ = 0;
	}

private:
	std::ostream& stream_;
	bool interactive_{false};
	std::size_t drawn_{0};
};

std::string format_degrees(double value) {
	std::ostringstream text;
	text << std::fixed << std::setprecision(2) << std::setw(8) << value << "°";
	return text.str();
}

}  // namespace

CaptureResult capture_plane(ArmFeedback& arm, ArmModel& model,
                            const IkParams& ik_params,
                            const CaptureOptions& options,
                            const KeyReader& read_key) {
	CaptureResult result;

	// Расслабляются только суставы, которые двигает человек: IK-суставы.
	// wrist_roll и gripper остаются под удержанием — они не провисают и не
	// влияют на направление инструмента, но удерживают вектор от TCP до
	// острия карандаша.
	JointMask relaxed{};
	for (std::size_t index = 0; index < model.ik_joints.size(); ++index) {
		relaxed[index] = true;
	}

	if (!arm.relax_motors(relaxed)) {
		result.error = "Failed to relax the motors";
		return result;
	}

	Panel panel(std::cout, isatty(STDOUT_FILENO) != 0);
	std::cout << "\nМоторы расслаблены — руку держите рукой. Поставьте остриё "
	             "карандаша на поверхность и выровняйте карандаш вертикально.\n"
	             "Enter — взять руку моторами, q — выход.\n"
	          << std::flush;

	while (true) {
		const std::optional<JointDegrees> angles = arm.read_joint_degrees();
		if (!angles) {
			result.error = "Failed to read the joint angles";
			break;
		}

		Eigen::VectorXd q = angles_to_q(model, *angles);
		pinocchio::forwardKinematics(model.model, model.data, q);
		pinocchio::updateFramePlacements(model.model, model.data);
		const Eigen::Vector3d axis = tool_axis(model);
		const double tilt = tilt_degrees(axis);
		const Eigen::Vector3d position = model.data.oMf[model.frame_id].translation();

		// Требуемые углы: то же положение точки TCP, но с вертикальным
		// инструментом. Разница между фактом и целью — что подкрутить.
		const IkResult wanted =
		    solve_ik(model, position, kToolDirectionDown, q, ik_params);

		const std::vector<std::string> lines = [&]() {
			std::vector<std::string> text;
			text.push_back(
			    "--- захват плоскости: моторы 1..4 расслаблены, держите руку ---");
			std::ostringstream header;
			header << std::left << std::setw(16) << "  факт" << std::right
			       << std::setw(10) << "цель" << std::setw(10) << "Δ";
			text.push_back(header.str());
			for (std::size_t index = 0; index < model.ik_joints.size(); ++index) {
				const int servo_id = static_cast<int>(index) + 1;
				const auto joint = model.model.joints[model.ik_joints[index]];
				const double actual = (*angles)[index];
				const double target =
				    model.q_to_servo_degrees(servo_id, wanted.q[joint.idx_q()]);
				std::ostringstream row;
				row << "  " << std::left << std::setw(16)
				    << (model.model.names[model.ik_joints[index]] + ": " +
				        format_degrees(actual))
				    << std::right << std::setw(10)
				    << format_degrees(target) << std::setw(10)
				    << format_degrees(target - actual);
				text.push_back(row.str());
			}
			std::ostringstream tcp;
			tcp << "  TCP: (" << std::fixed << std::setprecision(4)
			    << position.x() << ", " << position.y() << ", " << position.z()
			    << ") м";
			text.push_back(tcp.str());

			std::ostringstream tilt_line;
			tilt_line << "  наклон инструмента: " << std::fixed
			          << std::setprecision(2) << tilt << "° (допуск "
			          << options.tilt_tolerance_deg << "°)  ";
			if (tilt <= options.tilt_tolerance_deg) {
				tilt_line << "✓ ЗАМОК ОТКРЫТ — Enter, программа возьмёт руку";
			} else {
				tilt_line << "подровняйте кисть (остриё сдвинется на несколько "
			             "миллиметров)";
			}
			text.push_back(tilt_line.str());
			return text;
		}();
		panel.update(lines);

		const int key = read_key ? read_key() : 0;
		if (key < 0) {
			result.error = "No operator at the keyboard";
			break;
		}
		if (key == 'q' || key == 'Q') {
			result.error = "Capture cancelled by the operator";
			break;
		}
		if (key != '\n' && key != '\r') {
			std::this_thread::sleep_for(
			    std::chrono::milliseconds(options.refresh_ms));
			continue;
		}

		if (tilt > options.tilt_tolerance_deg) {
			std::cout << "\nНаклон " << std::fixed << std::setprecision(2) << tilt
			          << "° больше допуска " << options.tilt_tolerance_deg
			          << "°. Подровняйте карандаш и нажмите Enter ещё раз.\n"
			          << std::flush;
			continue;
		}

		// Захват состоялся. Цель записывается до включения удержания:
		// тогда моторы берут руку ровно там, где её оставил человек, без
		// скачка.
		if (!arm.write_joint_degrees(*angles)) {
			result.error = "Failed to hold the current pose";
			break;
		}
		if (!arm.hold_motors(relaxed)) {
			result.error = "Failed to restore the motor torque";
			break;
		}

		result.plane.tcp_height_m = position.z();
		result.plane.contact_x_m = position.x();
		result.plane.contact_y_m = position.y();
		result.plane.tilt_deg = tilt;
		result.joint_degrees = *angles;
		result.accepted = true;
		break;
	}

	panel.clear();
	if (!result.accepted) {
		// Рука осталась бы висеть на человеке. Возвращаем удержание в любом
		// случае: выход из захвата не должен оставлять моторы расслабленными.
		arm.hold_motors(relaxed);
	}
	return result;
}

TrajectoryCheck check_trajectory(ArmModel& model,
                                 const std::vector<Eigen::Vector3d>& targets,
                                 const IkParams& ik_params,
                                 const Eigen::VectorXd& initial_q) {
	TrajectoryCheck check;
	if (targets.empty()) return check;

	Eigen::VectorXd q = initial_q;
	for (const Eigen::Vector3d& target : targets) {
		const IkResult ik = solve_ik(model, target, kToolDirectionDown, q, ik_params);
		if (!ik.converged) return check;
		q = ik.q;
		check.max_position_error_m =
		    std::max(check.max_position_error_m, ik.final_error_m);
		check.max_tilt_deg = std::max(
		    check.max_tilt_deg, ik.final_orientation_error_rad * 180.0 / kPiExecution);
	}
	check.reachable = true;
	return check;
}

bool draw_trajectory(ArmFeedback& arm, ArmModel& model,
                     const std::vector<Eigen::Vector3d>& targets,
                     const IkParams& ik_params, const DrawOptions& options,
                     DrawReport& report, std::string& error) {
	const std::optional<JointDegrees> current = arm.read_joint_degrees();
	if (!current) {
		error = "Failed to read the current joint angles";
		return false;
	}

	Eigen::VectorXd q = angles_to_q(model, *current);
	JointDegrees degrees = *current;

	for (std::size_t index = 0; index < targets.size(); ++index) {
		const IkResult ik =
		    solve_ik(model, targets[index], kToolDirectionDown, q, ik_params);
		if (!ik.converged) {
			error = "Point " + std::to_string(index + 1) + " of " +
			        std::to_string(targets.size()) +
			        " is unreachable: position error " +
			        std::to_string(ik.final_error_m) + " m, tilt " +
			        std::to_string(ik.final_orientation_error_rad) + " rad";
			return false;
		}
		q = ik.q;

		// Команда формируется из решения IK по четырём суставам; остальные
		// два остаются там, где их оставил оператор.
		for (std::size_t joint_index = 0; joint_index < model.ik_joints.size();
		     ++joint_index) {
			const int servo_id = static_cast<int>(joint_index) + 1;
			const auto joint = model.model.joints[model.ik_joints[joint_index]];
			degrees[joint_index] = model.q_to_servo_degrees(
			    servo_id, q[joint.idx_q()]);
		}

		if (!arm.write_joint_degrees(degrees)) {
			error = "Failed to send the joint command for point " +
			        std::to_string(index + 1);
			return false;
		}

		// Ожидание достижения. Первая точка может быть далеко: руку только
		// что поставили рукой, и первый ход самый длинный.
		const int timeout =
		    index == 0 ? options.reach_timeout_ms * 3 : options.reach_timeout_ms;
		const auto deadline = std::chrono::steady_clock::now() +
		                      std::chrono::milliseconds(timeout);
		bool reached = false;
		std::optional<JointDegrees> actual;
		while (true) {
			actual = arm.read_joint_degrees();
			if (!actual) {
				error = "Lost the servo bus while moving to point " +
				        std::to_string(index + 1);
				return false;
			}
			if (worst_joint_difference(*actual, degrees) <=
			    options.reach_tolerance_deg) {
				reached = true;
				break;
			}
			if (std::chrono::steady_clock::now() > deadline) {
				++report.timeouts;
				error = "Point " + std::to_string(index + 1) + " of " +
				        std::to_string(targets.size()) +
				        " was not reached in " + std::to_string(timeout) +
				        " ms; the arm is held in place";
				return false;
			}
			std::this_thread::sleep_for(
			    std::chrono::milliseconds(options.poll_interval_ms));
		}

		// Фактическая точность: где оказался TCP и каков наклон.
		Eigen::VectorXd reached_q = angles_to_q(model, *actual);
		pinocchio::forwardKinematics(model.model, model.data, reached_q);
		pinocchio::updateFramePlacements(model.model, model.data);
		const Eigen::Vector3d position =
		    model.data.oMf[model.frame_id].translation();
		report.max_position_error_m = std::max(
		    report.max_position_error_m, (targets[index] - position).norm());
		report.max_tilt_deg = std::max(
		    report.max_tilt_deg, tilt_degrees(tool_axis(model)));
		++report.points_drawn;

		std::cout << "  точка " << std::setw(3) << (index + 1) << "/"
		          << targets.size() << "  ошибка "
		          << std::fixed << std::setprecision(3)
		          << (targets[index] - position).norm() * 1000.0 << " мм"
		          << "  наклон " << std::setprecision(2)
		          << tilt_degrees(tool_axis(model)) << "°" << std::flush;
		if (reached) {
			std::cout << "  достигнута" << std::endl;
		}
	}

	report.completed = true;
	return true;
}

}  // namespace so101
