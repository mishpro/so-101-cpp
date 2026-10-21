// Проверки рисования окружности без робота: нужны только URDF и псевдо-рука.
//
// Три слоя: геометрия (окружность и плоскость), исполнение (цикл по
// траектории на заглушке) и кинематика (IK по окружности на настоящей
// модели).

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "arm_feedback.hpp"
#include "arm_model.hpp"
#include "control_config.hpp"
#include "drawing.hpp"
#include "drawing_execution.hpp"
#include "test_support.hpp"

namespace {

using so101::JointDegrees;
using so101::JointMask;

std::string test_urdf_path() {
	ControlConfig config{};
	if (!load_config("send_config.json", config)) {
		std::cerr << "Не удалось прочитать send_config.json из рабочего каталога\n";
		std::exit(EXIT_FAILURE);
	}
	return config.urdf_path;
}

std::string test_end_effector() {
	ControlConfig config{};
	if (!load_config("send_config.json", config)) std::exit(EXIT_FAILURE);
	return config.end_effector;
}

// Рука без железа: помнит, что ей сказали, и отвечает тем же.
class StubArm final : public so101::ArmFeedback {
public:
	std::optional<JointDegrees> angles;
	std::vector<JointDegrees> written;
	JointMask relaxed{};
	JointMask held{};
	bool fail_reads{false};
	bool fail_writes{false};

	std::optional<JointDegrees> read_joint_degrees() override {
		if (fail_reads) return std::nullopt;
		return angles;
	}

	bool write_joint_degrees(const JointDegrees& degrees) override {
		if (fail_writes) return false;
		written.push_back(degrees);
		angles = degrees;
		return true;
	}

	bool relax_motors(const JointMask& joints) override {
		relaxed = joints;
		return true;
	}

	bool hold_motors(const JointMask& joints) override {
		held = joints;
		return true;
	}
};

// Углы моторов из конфигурации Pinocchio.
JointDegrees q_to_degrees(const so101::ArmModel& model,
                          const Eigen::VectorXd& q) {
	JointDegrees degrees{};
	for (std::size_t index = 0; index < so101::kServoCount; ++index) {
		const int servo_id = static_cast<int>(index) + 1;
		const auto joint = model.model.joints[model.servo_joints[index]];
		degrees[index] =
		    model.q_to_servo_degrees(servo_id, q[joint.idx_q()]);
	}
	return degrees;
}

// Окружность: точки лежат на окружности, на одной высоте, круг замкнут.
void test_circle_geometry() {
	so101::Plane plane;
	plane.tcp_height_m = 0.043;
	plane.contact_x_m = 0.18;
	plane.contact_y_m = 0.0;

	so101::CircleSpec circle;
	circle.center_x_m = 0.20;
	circle.center_y_m = 0.0;
	circle.radius_m = 0.05;
	circle.points = 36;

	const Eigen::Vector3d current{0.18, 0.0, 0.043};
	const std::vector<Eigen::Vector3d> targets =
	    so101::circle_targets(plane, circle, 0.0002, current);
	CHECK(targets.size() == circle.points);

	for (const Eigen::Vector3d& target : targets) {
		const double radius =
		    std::hypot(target.x() - circle.center_x_m, target.y() - circle.center_y_m);
		CHECK_NEAR(radius, circle.radius_m, 1e-9);
		// Высота: плоскость минус нажатие.
		CHECK_NEAR(target.z(), plane.tcp_height_m - 0.0002, 1e-12);
	}

	// Круг замкнут: последняя точка перед первой.
	const Eigen::Vector3d first = targets.front();
	const Eigen::Vector3d last = targets.back();
	const double chord =
	    std::hypot(last.x() - first.x(), last.y() - first.y());
	// Хорда между соседними точками: 2R sin(pi/N). Длина дуги была бы
	// 2*pi*R/N — это чуть больше хорды, и сравнивать надо с хордой.
	const double expected_chord =
	    2.0 * circle.radius_m *
	    std::sin(3.14159265358979323846 / static_cast<double>(circle.points));
	CHECK_NEAR(chord, expected_chord, 1e-9);

	// Начальная точка — ближайшая к текущему положению.
	double closest = 1e9;
	for (const Eigen::Vector3d& target : targets) {
		closest = std::min(closest, (target - current).norm());
	}
	CHECK_NEAR((targets.front() - current).norm(), closest, 1e-12);
}

// Плоскость пишется и читается без потерь; файл после записи цел.
void test_plane_file() {
	const std::string path = "test_draw_plane.json";
	so101::Plane plane;
	plane.tcp_height_m = 0.04321;
	plane.contact_x_m = 0.18125;
	plane.contact_y_m = -0.0043;
	plane.tilt_deg = 1.75;

	std::string error;
	CHECK(so101::save_plane(path, plane, error));
	CHECK(error.empty());

	so101::Plane loaded;
	CHECK(so101::load_plane(path, loaded, error));
	CHECK_NEAR(loaded.tcp_height_m, plane.tcp_height_m, 1e-12);
	CHECK_NEAR(loaded.contact_x_m, plane.contact_x_m, 1e-12);
	CHECK_NEAR(loaded.contact_y_m, plane.contact_y_m, 1e-12);
	CHECK_NEAR(loaded.tilt_deg, plane.tilt_deg, 1e-12);

	// Временный файл после переименования не остаётся.
	std::ifstream leftover(path + ".tmp");
	CHECK(!leftover.good());

	std::remove(path.c_str());

	// Отсутствующий файл — не ошибка молчания, а сообщение.
	so101::Plane missing;
	CHECK(!so101::load_plane("no_such_plane_file.json", missing, error));
	CHECK(!error.empty());
}

// Цикл по траектории шлёт цели и доводит каждую до конца.
void test_draw_trajectory_on_stub(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	so101::Plane plane;
	plane.tcp_height_m = 0.043;
	so101::CircleSpec circle;
	circle.center_x_m = 0.20;
	circle.center_y_m = 0.0;
	circle.radius_m = 0.05;
	circle.points = 12;

	// Стартовая поза: решение IK для вертикального инструмента.
	Eigen::VectorXd start = pinocchio::neutral(model.model);
	const so101::IkResult start_solution = so101::solve_ik(
	    model, Eigen::Vector3d{0.25, 0.0, 0.02}, so101::kToolDirectionDown,
	    start, params);
	CHECK(start_solution.converged);

	StubArm arm;
	arm.angles = q_to_degrees(model, start_solution.q);

	const std::vector<Eigen::Vector3d> targets = so101::circle_targets(
	    plane, circle, 0.0, Eigen::Vector3d{0.25, 0.0, 0.02});

	so101::DrawOptions options;
	so101::DrawReport report;
	std::string error;
	CHECK(so101::draw_trajectory(arm, model, targets, params, options, report,
	                              error));
	CHECK(report.completed);
	CHECK(report.points_drawn == targets.size());
	CHECK(arm.written.size() == targets.size());
	CHECK_NEAR(report.max_position_error_m, 0.0, 0.001);
	CHECK(report.max_tilt_deg < 0.5);
}

// Обрыв связи останавливает рисование, а не продолжает вслепую.
void test_draw_trajectory_stops_on_lost_bus(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	StubArm arm;
	arm.angles = q_to_degrees(model, pinocchio::neutral(model.model));
	arm.fail_reads = true;

	const std::vector<Eigen::Vector3d> targets{
	    Eigen::Vector3d{0.25, 0.0, 0.02}, Eigen::Vector3d{0.24, 0.0, 0.02}};

	so101::DrawOptions options;
	so101::DrawReport report;
	std::string error;
	CHECK(!so101::draw_trajectory(arm, model, targets, params, options, report,
	                               error));
	CHECK(!error.empty());
	CHECK(!report.completed);
}

// Захват плоскости: моторы расслабляются, человек «нажимает Enter», и
// программа возвращает моторы в работу, записав текущую позу целью.
void test_capture_plane_on_stub(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	Eigen::VectorXd start = pinocchio::neutral(model.model);
	const so101::IkResult solution = so101::solve_ik(
	    model, Eigen::Vector3d{0.25, 0.0, 0.02}, so101::kToolDirectionDown,
	    start, params);
	CHECK(solution.converged);

	StubArm arm;
	arm.angles = q_to_degrees(model, solution.q);

	so101::CaptureOptions options;
	options.tilt_tolerance_deg = 3.0;
	int polls = 0;
	const so101::CaptureResult result = so101::capture_plane(
	    arm, model, params, options,
	    [&polls]() { return polls++ < 2 ? 0 : '\n'; });

	CHECK(result.accepted);
	CHECK(result.error.empty());
	// Расслаблены только IK-суставы: первые четыре.
	CHECK(result.plane.tcp_height_m > 0.0);
	CHECK(result.plane.tilt_deg <= options.tilt_tolerance_deg);
	for (std::size_t index = 0; index < arm.relaxed.size(); ++index) {
		CHECK(arm.relaxed[index] == (index < 4));
		CHECK(arm.held[index] == (index < 4));
	}
	// Поза записана целью до возврата удержания: рука остаётся на месте.
	CHECK(arm.written.size() == 1);
}

// IK по окружности: точки достижимы, инструмент вертикален, круг замкнут.
void test_ik_along_the_circle(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.0001;
	params.orientation_tolerance_rad = 0.001;

	so101::Plane plane;
	plane.tcp_height_m = 0.043;
	so101::CircleSpec circle;
	circle.center_x_m = 0.20;
	circle.center_y_m = 0.0;
	circle.radius_m = 0.05;
	circle.points = 36;

	const std::vector<Eigen::Vector3d> targets = so101::circle_targets(
	    plane, circle, 0.0, Eigen::Vector3d{0.25, 0.0, 0.043});

	Eigen::VectorXd q = pinocchio::neutral(model.model);
	double worst_position = 0.0;
	double worst_tilt = 0.0;
	for (const Eigen::Vector3d& target : targets) {
		const so101::IkResult result = so101::solve_ik(
		    model, target, so101::kToolDirectionDown, q, params);
		CHECK(result.converged);
		q = result.q;
		worst_position = std::max(worst_position, result.final_error_m);
		worst_tilt = std::max(worst_tilt, result.final_orientation_error_rad);
	}
	CHECK(worst_position < 0.001);
	CHECK(worst_tilt < 0.01);

	// Суставы остались в пределах URDF.
	for (std::size_t index = 0; index < so101::kIkJointCount; ++index) {
		const auto joint = model.model.joints[model.ik_joints[index]];
		CHECK(q[joint.idx_q()] >= model.model.lowerPositionLimit[joint.idx_q()] - 1e-9);
		CHECK(q[joint.idx_q()] <= model.model.upperPositionLimit[joint.idx_q()] + 1e-9);
	}

	// Круг замкнут: решение для первой точки снова даёт первую точку.
	const so101::IkResult closing = so101::solve_ik(
	    model, targets.front(), so101::kToolDirectionDown, q, params);
	CHECK(closing.converged);
	for (std::size_t index = 0; index < so101::kIkJointCount; ++index) {
		const auto joint = model.model.joints[model.ik_joints[index]];
		CHECK_NEAR(std::abs(closing.q[joint.idx_q()] - q[joint.idx_q()]), 0.0,
		           0.35);
	}
}

}  // namespace

int main() {
	so101::ArmModel arm{};
	std::string error;
	if (!so101::load_arm_model(test_urdf_path(), test_end_effector(), arm,
	                           error)) {
		std::cerr << "Не удалось загрузить модель: " << error << '\n';
		return EXIT_FAILURE;
	}

	test_circle_geometry();
	test_plane_file();
	test_draw_trajectory_on_stub(arm);
	test_draw_trajectory_stops_on_lost_bus(arm);
	test_capture_plane_on_stub(arm);
	test_ik_along_the_circle(arm);

	return so101_test::finish("draw_circle");
}
