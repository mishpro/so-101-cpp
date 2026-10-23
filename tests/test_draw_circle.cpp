// Проверки рисования окружности без робота: нужны только URDF и псевдо-рука.
//
// Три слоя: геометрия (окружность и плоскость), исполнение (цикл по
// траектории на заглушке) и кинематика (IK по окружности на настоящей
// модели).
//
// Исполнение проверяется и на терпимость: нет сигнала нагрузки — работа
// идёт по высоте из плоскости, нажим не проверяется; точка вне
// досягаемости — дырка в круге, а не повод бросить круг.

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
#include "servo_bus.hpp"
#include "control_config.hpp"
#include "drawing.hpp"
#include "drawing_execution.hpp"
#include "surface_contact.hpp"
#include "test_support.hpp"

namespace {

using so101::JointDegrees;
using so101::JointMask;

// Насколько нагрузка на моторы растёт при нажатии на бумагу: единиц
// сигнала на миллиметр нажима. Глубина в метрах, поэтому множитель — на
// миллиметры.
constexpr double kReactionScale = 1000000.0;

// Насколько остриё проваливается в бумагу: толщина листа. Дальше нагрузка
// не растёт — бумага не резиновая.
constexpr double kPaperPenetrationM = 0.0005;

// Сколько моторов держат карандаш: первые четыре.
constexpr std::size_t kHoldingMotors = 4;

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

// Обратное преобразование: показания моторов в конфигурацию Pinocchio.
Eigen::VectorXd degrees_to_q(const so101::ArmModel& model,
                              const JointDegrees& degrees) {
	Eigen::VectorXd q = pinocchio::neutral(model.model);
	for (std::size_t index = 0; index < so101::kServoCount; ++index) {
		const int servo_id = static_cast<int>(index) + 1;
		const auto joint = model.model.joints[model.servo_joints[index]];
		q[joint.idx_q()] = model.servo_degrees_to_q(servo_id, degrees[index]);
	}
	return q;
}

// Наибольшее расхождение между двумя командами, градусы.
double largest_difference(const JointDegrees& left, const JointDegrees& right) {
	double largest = 0.0;
	for (std::size_t index = 0; index < left.size(); ++index) {
		largest = std::max(largest, std::abs(left[index] - right[index]));
	}
	return largest;
}

// Высота TCP в позе, заданной углами моторов.
double tcp_height(so101::ArmModel& model, const JointDegrees& degrees) {
	const Eigen::VectorXd q = degrees_to_q(model, degrees);
	pinocchio::forwardKinematics(model.model, model.data, q);
	pinocchio::updateFramePlacements(model.model, model.data);
	return model.data.oMf[model.frame_id].translation().z();
}

// Рука без железа.
//
// Моделируются две особенности настоящего STS3215, из-за которых рисование
// раньше ломалось:
//   - мёртвая зона: мотор до своей цели не доходит, а останавливается на
//     deadband_deg раньше;
//   - реакция бумаги: когда остриё уходит ниже бумаги, нагрузка на моторы
//     растёт.
class StubArm : public so101::ArmFeedback {
public:
	std::optional<JointDegrees> angles;
	std::vector<JointDegrees> written;
	JointMask relaxed{};
	JointMask held{};
	bool fail_reads{false};
	bool fail_writes{false};
	// Рука принимает команду, но не едет: как зажатая бумагой.
	bool ignore_writes{false};
	// Насколько большой ход рука ещё может выполнить, в градусах сустава.
	// Ноль — любой ход. Нужно, чтобы рука двигалась во время поиска
	// касания (шаги там мелкие) и не могла переехать к следующей точке:
	// тогда виден путь «точка не достигнута».
	double max_step_deg{0.0};
	// Мотор не доходит до своей цели: останавливается на мёртвой зоне
	// раньше. Это настоящее поведение сервопривода, и именно оно делало
	// рисование бесконечным: допуск в 0.5° был мельче зоны.
	double deadband_deg{0.0};
	// Шум сигнала нагрузки: столько единиц добавляется и вычитается
	// поочерёдно. Ноль — сигнал идеальный.
	std::int32_t noise_units{0};
	// Моторы не отдают нагрузку: настоящий STS3215 так умеет. Раньше на этом
	// рисование останавливалось, теперь обязано продолжаться.
	bool torque_readable{true};
	// Сколько раз читалась нагрузка: по нему видно, что программа её
	// действительно не читала.
	std::size_t torque_reads{0};
	// Высота бумаги в базовой системе. Ниже неё бумага давит в ответ.
	double paper_z{0.0};
	// Модель: нужна, чтобы знать, где оказался TCP.
	so101::ArmModel* model{nullptr};
	// Цель мотора: то, что он запомнил. При lose_commands мотор команду не
	// получает, и цель остаётся прежней.
	JointDegrees goal{};
	bool lose_commands{false};

	std::optional<JointDegrees> read_joint_degrees() override {
		if (fail_reads) return std::nullopt;
		if (!angles) return std::nullopt;
		// Регистр положения STS3215 имеет 4095 шагов на оборот: показание
		// квантуется, и на кончике карандаша это около 0.3 мм дрожания.
		// Без этого в заглушке не видно, что критерий «рука остановилась»
		// обязан терпеть этот шаг.
		//
		// Дрожание детерминированное: ±шаг, по очереди. Оно нужно именно
		// для проверки остановки — с постоянным показанием два соседних
		// опроса совпадают и критерий проверяется вхолостую.
		JointDegrees quantized = *angles;
		// Знак дрожания чередуется от опроса к опросу. Счётчик один на
		// опрос, а не на сустав: иначе шесть прибавлений за опрос делали бы
		// чётность постоянной, и соседние показания совпадали бы до
		// миллиметра.
		const double jitter = (++position_reads_ % 2 == 0) ? 1.0 : -1.0;
		for (double& value : quantized) {
			const double steps = std::round(value * kStepsPerDegree);
			value = (steps + jitter) / kStepsPerDegree;
		}
		return quantized;
	}

	bool write_joint_degrees(const JointDegrees& degrees) override {
		if (fail_writes) return false;
		written.push_back(degrees);
		if (ignore_writes) return true;
		if (max_step_deg > 0.0 && angles) {
			double largest = 0.0;
			for (std::size_t index = 0; index < degrees.size(); ++index) {
				largest = std::max(largest,
				                   std::abs(degrees[index] - (*angles)[index]));
			}
			// Ход неподъёмный: рука зажата и не может переехать.
			if (largest > max_step_deg) return true;
		}
		if (angles) {
			JointDegrees moved = *angles;
			for (std::size_t index = 0; index < moved.size(); ++index) {
				const double step = degrees[index] - (*angles)[index];
				// Мотор идёт к цели, но останавливается на мёртвой зоне
				// раньше: до цели он не доходит.
				if (std::abs(step) > deadband_deg) {
					moved[index] =
					    degrees[index] - std::copysign(deadband_deg, step);
				}
			}
			angles = moved;
		}
		if (!lose_commands) goal = degrees;
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

	std::optional<JointDegrees> read_goal_degrees() override {
		if (fail_reads) return std::nullopt;
		return goal;
	}

	std::optional<so101::TorqueSignal> read_torque_signals(
	    std::size_t motors) override {
		++torque_reads;
		if (fail_reads) return std::nullopt;
		if (!torque_readable) return std::nullopt;
		so101::TorqueSignal signal{};
		if (!model || !angles) return signal;
		const Eigen::VectorXd q = degrees_to_q(*model, *angles);
		pinocchio::forwardKinematics(model->model, model->data, q);
		pinocchio::updateFramePlacements(model->model, model->data);
		const double tcp_z = model->data.oMf[model->frame_id].translation().z();
		const double depth = paper_z - tcp_z;
		// Бумага не бесконечно жёсткая: остриё проваливается в неё не
		// больше, чем на толщину листа, и дальше нагрузка не растёт.
		const double penetration = std::min(depth, kPaperPenetrationM);
		const std::int32_t reaction =
		    penetration > 0.0
		        ? static_cast<std::int32_t>(penetration * kReactionScale)
		        : 0;
		// Шум сигнала: детерминированный, чтобы порог «по шуму» можно было
		// проверить. Знак меняется от замера к замеру, а не от мотора к
		// мотору: иначе сумма моторов шум гасит, и шум измеряется нулём.
		const std::int32_t jitter =
		    (++reads_ % 2 == 0) ? noise_units : -noise_units;
		for (std::size_t index = 0; index < motors && index < kHoldingMotors;
		     ++index) {
			signal[index] = reaction + jitter;
		}
		return signal;
	}

	// Счётчики опросов: по ним дрожание идёт по очереди, а не случайно.
	std::size_t reads_{0};
	std::size_t position_reads_{0};
};

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
	    so101::circle_targets(plane, circle, current);
	CHECK(targets.size() == circle.points);

	for (const Eigen::Vector3d& target : targets) {
		const double radius =
		    std::hypot(target.x() - circle.center_x_m, target.y() - circle.center_y_m);
		CHECK_NEAR(radius, circle.radius_m, 1e-9);
		// Высота цели — высота бумаги. Нажатие добавляет исполнение: его
		// величина не известна заранее, потому что она ищется касанием.
		CHECK_NEAR(target.z(), plane.tcp_height_m, 1e-12);
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

// Общие приготовления рисования: плоскость, окружность, поза.
struct DrawingSetup {
	so101::Plane plane;
	so101::CircleSpec circle;
	std::vector<Eigen::Vector3d> targets;
	Eigen::VectorXd start;
};

DrawingSetup drawing_setup(so101::ArmModel& model,
                           const so101::IkParams& params) {
	DrawingSetup setup;
	setup.plane.tcp_height_m = 0.043;
	setup.circle.center_x_m = 0.20;
	setup.circle.center_y_m = 0.0;
	setup.circle.radius_m = 0.05;
	setup.circle.points = 12;

	setup.start = pinocchio::neutral(model.model);
	const so101::IkResult start_solution = so101::solve_ik(
	    model, Eigen::Vector3d{0.25, 0.0, 0.043}, so101::kToolDirectionDown,
	    setup.start, params);
	CHECK(start_solution.converged);
	setup.start = start_solution.q;

	setup.targets = so101::circle_targets(
	    setup.plane, setup.circle, Eigen::Vector3d{0.25, 0.0, 0.043});
	return setup;
}

// Рисование доходит до конца, хотя мотор не доходит до своих целей.
//
// Это главная проверка: настоящий сервопривод останавливается на мёртвой
// зоне (0.35° на разгруженном суставе, 1.4° на локте под весом руки), а
// критерий достижения был 0.5° по углам суставов. Разница меньше зоны —
// и рисование останавливалось на первой точке с «остаток превышает допуск»,
// хотя рука давно доехала.
void test_draw_trajectory_survives_deadband(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	const DrawingSetup setup = drawing_setup(model, params);

	StubArm arm;
	arm.model = &model;
	arm.angles = q_to_degrees(model, setup.start);
	arm.paper_z = setup.plane.tcp_height_m;
	arm.deadband_deg = 1.4;

	so101::DrawOptions options;
	so101::DrawReport report;
	std::string error;
	CHECK_MSG(so101::draw_trajectory(arm, model, setup.targets, params, options,
	                                 report, error),
	          error);
	CHECK(error.empty());
	CHECK(report.completed);
	CHECK(report.points_drawn == setup.targets.size());
	// Ошибка после подстройки — миллиметры, а не половина окружности: без
	// подстройки остаток равен недобору мотора.
	CHECK(report.max_position_error_m <= 0.0025);
	// Подстройка действительно работала.
	CHECK(report.compensated > 0);
	// Карандаш прижимался к бумаге: нагрузка на точках не нулевая.
	CHECK(report.min_press_level > 0);
}

// Без подстройки окружность выходит меньше заданной: недобор сервопривода
// не компенсируется сам. Проверка нужна, чтобы подстройку нельзя было
// выкинуть: она единственное, что держит радиус.
void test_undershoot_without_compensation(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	const DrawingSetup setup = drawing_setup(model, params);

	StubArm arm;
	arm.model = &model;
	arm.angles = q_to_degrees(model, setup.start);
	arm.paper_z = setup.plane.tcp_height_m;
	arm.deadband_deg = 1.4;

	so101::DrawOptions options;
	options.compensate = false;
	so101::DrawReport report;
	std::string error;
	CHECK(so101::draw_trajectory(arm, model, setup.targets, params, options,
	                              report, error));
	CHECK(report.completed);
	// Без подстройки остаток заметный: это и есть недобор.
	CHECK(report.max_position_error_m > 0.002);
}

// Обрыв связи останавливает рисование, а не продолжает вслепую.
void test_draw_trajectory_stops_on_lost_bus(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	StubArm arm;
	arm.model = &model;
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

// Рука не доходит до цели: в сообщении должен быть остаток — по каким
// углам и на сколько миллиметров. Без него «не достигнуто» ничего не
// объясняет: и зажатая бумагой рука, и слишком строгий допуск выглядят
// одинаково.
void test_draw_trajectory_reports_residual(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	// Поза — как в остальных проверках: решение IK с вертикальным
	// инструментом. Из нейтральной позы решатель идёт в другую ветвь, и
	// проверка ловила бы не то, что задумано.
	const DrawingSetup setup = drawing_setup(model, params);

	// Рука двигается, пока идёт поиск касания, и замирает потом: как
	// зажатая бумагой рука.
	StubArm arm;
	arm.model = &model;
	arm.angles = q_to_degrees(model, setup.start);
	arm.paper_z = setup.plane.tcp_height_m;
	arm.max_step_deg = 3.0;

	const std::vector<Eigen::Vector3d> targets{
	    Eigen::Vector3d{0.25, 0.0, 0.043}, Eigen::Vector3d{0.20, 0.0, 0.043},
	    Eigen::Vector3d{0.15, 0.0, 0.043}};

	so101::DrawOptions options;
	options.reach_timeout_ms = 60;
	options.poll_interval_ms = 5;
	so101::DrawReport report;
	std::string error;
	CHECK(!so101::draw_trajectory(arm, model, targets, params, options, report,
	                               error));
	CHECK(!error.empty());
	// Это именно «точка не достигнута», а не сбой связи.
	CHECK(error.find("was not reached") != std::string::npos);
	// И разбор неудачи: остаток по суставам и расстояние до цели.
	CHECK(error.find("остаток по суставам") != std::string::npos);
	CHECK(error.find("горизонтальное расстояние от TCP до цели") !=
	      std::string::npos);
	// Цель мотора показана: по ней видно, что команда дошла.
	CHECK(error.find("цель мотора") != std::string::npos);
}

// Шум сигнала не должен выдаваться за касание: порог строится по шуму,
// измеренному на свободной высоте.
void test_contact_noise_is_not_a_contact(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	const DrawingSetup setup = drawing_setup(model, params);

	// Шум в 400 единиц на мотор: разброс базы — 3200, порог втрое выше —
	// 9600. Реакция бумаги такой порог не превышает, и касание не должно
	// быть найдено: иначе шум выдавал бы себя за касание.
	StubArm arm;
	arm.model = &model;
	arm.angles = q_to_degrees(model, setup.start);
	arm.paper_z = setup.plane.tcp_height_m;
	arm.noise_units = 400;

	so101::DrawOptions options;
	so101::DrawReport report;
	std::string error;
	// Шум не выдаётся за касание — но и не роняет рисование: высоты касания
	// нет, работа идёт по высоте плоскости минус глубина нажима, точки
	// считаются «без нажима».
	CHECK(so101::draw_trajectory(arm, model, setup.targets, params, options,
	                               report, error));
	CHECK(error.empty());
	// Касание не нашлось — и это теперь написано в отчёте, а не только в
	// сообщении на экране.
	CHECK(report.contact_failed);
	CHECK(report.points_drawn == setup.targets.size());
	CHECK(report.contact_drops == static_cast<int>(setup.targets.size()));

	// Без шума порог нулевой: касанием считается любое изменение.
	StubArm quiet;
	quiet.model = &model;
	quiet.angles = q_to_degrees(model, setup.start);
	quiet.paper_z = setup.plane.tcp_height_m;

	so101::DrawReport quiet_report;
	std::string quiet_error;
	CHECK(so101::draw_trajectory(quiet, model, setup.targets, params, options,
	                              quiet_report, quiet_error));
	CHECK(quiet_report.contact_threshold == 0);
	CHECK(quiet_report.min_press_level > 0);
}

// Нагрузка на моторы не читается — рисование всё равно идёт.
//
// Раньше программа на этом останавливалась: без нагрузки нельзя отличить
// «карандаш пишет» от «карандаш висит в воздухе». Теперь нагрузка
// необязательна: работа продолжается по высоте из плоскости, нажим не
// проверяется, а точки считаются «без нажима».
void test_draw_trajectory_survives_unreadable_torque(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	const DrawingSetup setup = drawing_setup(model, params);

	StubArm arm;
	arm.model = &model;
	arm.angles = q_to_degrees(model, setup.start);
	arm.paper_z = setup.plane.tcp_height_m;
	// Моторы отвечают на положение, но не на нагрузку.
	arm.torque_readable = false;

	so101::DrawOptions options;
	so101::DrawReport report;
	std::string error;
	CHECK_MSG(so101::draw_trajectory(arm, model, setup.targets, params, options,
	                                 report, error),
	          error);
	CHECK(error.empty());
	CHECK(report.completed);
	// Все точки пройдены: рисование дошло до конца.
	CHECK(report.points_drawn == setup.targets.size());
	// Нажим не зафиксирован: нагрузку прочитать не удалось.
	CHECK(report.min_press_level == 0);
	// Точки посчитаны «без нажима».
	CHECK(report.contact_drops == static_cast<int>(report.points_drawn));
}

// Флаг use_contact выключен: нагрузка не читается вовсе, нажим не
// проверяется, а рисование всё равно проходит.
//
// Отдельный флаг нужен для машин, у которых регистра нагрузки нет вовсе:
// там читать нечего, и программа не должна даже пытаться.
void test_draw_trajectory_without_contact_check(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	const DrawingSetup setup = drawing_setup(model, params);

	StubArm arm;
	arm.model = &model;
	arm.angles = q_to_degrees(model, setup.start);
	arm.paper_z = setup.plane.tcp_height_m;
	// Моторы не отдают нагрузку — и читать её не нужно.
	arm.torque_readable = false;

	so101::DrawOptions options;
	options.use_contact = false;
	so101::DrawReport report;
	std::string error;
	CHECK_MSG(so101::draw_trajectory(arm, model, setup.targets, params, options,
	                                 report, error),
	          error);
	CHECK(error.empty());
	CHECK(report.completed);
	CHECK(report.points_drawn == setup.targets.size());
	// Нагрузка не читалась ни разу: выключенный флаг — это не «прочитал и
	// не понял», а «не читал вовсе».
	CHECK(arm.torque_reads == 0);
	// Нажим не зафиксирован: проверять его было нечем.
	CHECK(report.min_press_level == 0);
}

// Точка, до которой IK не сходится, больше не останавливает программу.
//
// Программа предупреждает, командует лучшее приближение и идёт дальше по
// траектории: недостижимая точка — это дырка в окружности, а не причина
// бросить всё.
void test_draw_trajectory_continues_past_unreachable_point(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	const DrawingSetup setup = drawing_setup(model, params);

	StubArm arm;
	arm.model = &model;
	arm.angles = q_to_degrees(model, setup.start);
	arm.paper_z = setup.plane.tcp_height_m;

	// Средняя точка недостижима с вертикальным инструментом: там, куда
	// рука тянется вбок, вертикальный карандаш не получается. Лучшее
	// приближение рядом — до самой точки рука дотягивается, поэтому
	// проверка не зависит от того, ждёт ли программа прибытия в
	// недостижимую точку или сразу идёт дальше.
	const std::vector<Eigen::Vector3d> targets{
	    Eigen::Vector3d{0.25, 0.0, 0.043}, Eigen::Vector3d{0.24, 0.18, 0.043},
	    Eigen::Vector3d{0.23, 0.02, 0.043}};

	so101::DrawOptions options;
	so101::DrawReport report;
	std::string error;
	CHECK_MSG(so101::draw_trajectory(arm, model, targets, params, options, report,
	                                 error),
	          error);
	CHECK(error.empty());
	CHECK(report.completed);
	// Недостижимая точка посчитана.
	CHECK(report.unreachable_points > 0);
	// Рисование продолжилось: точки после недостижимой пройдены.
	CHECK(report.points_drawn >= 2);
}

// Недостижимая точка: моторам уходит лучшее приближение, а не поза
// предыдущей точки.
//
// Раньше команда строилась из seed'а, а seed на несошедшемся IK намеренно
// остаётся последним сошедшимся: моторам уходила ровно та же поза, что и на
// предыдущей точке. Рука не двигалась вовсе, а сообщение «моторам отправлено
// лучшее приближение» было неправдой.
void test_unreachable_point_gets_its_own_command(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	const DrawingSetup setup = drawing_setup(model, params);

	StubArm arm;
	arm.model = &model;
	arm.angles = q_to_degrees(model, setup.start);
	arm.paper_z = setup.plane.tcp_height_m;

	// Средняя точка недостижима с вертикальным инструментом.
	const std::vector<Eigen::Vector3d> targets{
	    Eigen::Vector3d{0.25, 0.0, 0.043}, Eigen::Vector3d{0.23, 0.18, 0.043},
	    Eigen::Vector3d{0.23, 0.02, 0.043}};

	so101::DrawOptions options;
	// Без поиска касания и без подстройки: на точку уходит ровно одна
	// команда, и она видна в списке записанных.
	options.use_contact = false;
	options.compensate = false;
	so101::DrawReport report;
	std::string error;
	CHECK_MSG(so101::draw_trajectory(arm, model, targets, params, options, report,
	                                 error),
	          error);
	CHECK(report.unreachable_points == 1);
	CHECK(arm.written.size() == targets.size());
	// Команда на недостижимой точке отличается от команды предыдущей точки:
	// моторам ушло лучшее приближение, а не копия позы, в которой рука уже
	// стояла.
	CHECK(largest_difference(arm.written[1], arm.written[0]) > 1.0);
}

// Seed после неудачного IK остаётся последним сошедшимся.
//
// Seed — поза, от которой решается следующая точка. На несошедшемся IK он
// намеренно не меняется: иначе следующая точка решалась бы из предела
// сустава, и неудача рассыпалась бы на весь круг.
//
// Проверяется там, где это видно: после недостижимой точки следующая точка
// всё равно решается, а разбор траектории даёт те же числа, что и рисование.
// С другим правилом seed'а (после неудачи seed равен пределу) последняя
// точка не решилась бы, и недостижимых получилось бы две.
void test_seed_stays_at_last_converged_solution(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	const DrawingSetup setup = drawing_setup(model, params);

	StubArm arm;
	arm.model = &model;
	arm.angles = q_to_degrees(model, setup.start);
	arm.paper_z = setup.plane.tcp_height_m;

	const std::vector<Eigen::Vector3d> targets{
	    Eigen::Vector3d{0.25, 0.0, 0.043}, Eigen::Vector3d{0.23, 0.18, 0.043},
	    Eigen::Vector3d{0.23, 0.02, 0.043}};

	so101::DrawOptions options;
	options.use_contact = false;
	options.compensate = false;

	// Разбор идёт по той же высоте, на какой рисует исполнитель, и от той
	// же позы: без поиска касания рука никуда не уходила.
	const double press_z = targets.front().z() - options.press_depth_m;
	std::vector<Eigen::Vector3d> pressed;
	for (const Eigen::Vector3d& target : targets) {
		Eigen::Vector3d pressed_target = target;
		pressed_target.z() = press_z;
		pressed.push_back(pressed_target);
	}
	const so101::TrajectoryPlan plan =
	    so101::plan_trajectory(model, pressed, params, setup.start);
	// Недостижима только средняя точка.
	CHECK(plan.unreachable_points == 1);

	// Рисование даёт те же числа: разбор и рисование считают
	// недостижимость по одному правилу.
	so101::DrawReport report;
	std::string error;
	CHECK_MSG(so101::draw_trajectory(arm, model, targets, params, options, report,
	                                 error),
	          error);
	CHECK(report.unreachable_points == plan.unreachable_points);
	// Точка после недостижимой пройдена: seed остался последним сошедшимся.
	CHECK(report.points_drawn == targets.size());
}

// Счётчики отчёта накапливаются между кругами: два круга — вдвое больше
// точек.
//
// Круги идут один за другим, и каждый начинается со своего поиска касания,
// но отчёт у них общий: оператор видит, сколько точек пройдено за все круги
// сразу.
void test_report_accumulates_between_laps(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	const DrawingSetup setup = drawing_setup(model, params);

	StubArm arm;
	arm.model = &model;
	arm.angles = q_to_degrees(model, setup.start);
	arm.paper_z = setup.plane.tcp_height_m;

	so101::DrawOptions options;
	so101::DrawReport report;
	std::string error;
	CHECK_MSG(so101::draw_trajectory(arm, model, setup.targets, params, options,
	                                 report, error),
	          error);
	CHECK(report.points_drawn == setup.targets.size());
	const std::size_t first_lap = report.points_drawn;

	// Второй круг: тот же отчёт, те же точки.
	CHECK_MSG(so101::draw_trajectory(arm, model, setup.targets, params, options,
	                                 report, error),
	          error);
	CHECK(report.points_drawn == 2 * first_lap);
	CHECK(report.points_drawn == 2 * setup.targets.size());
}

// Запасной путь: нагрузка недоступна — рабочая высота это плоскость минус
// глубина нажима.
//
// Раньше здесь ставилась ровно высота плоскости, а это высота TCP в тот
// момент, когда остриё стояло на бумаге: то есть кончик на бумаге, без
// нажима. С учётом мёртвой зоны мотора рука встала бы выше бумаги, карандаш
// не коснулся бы её, и круг не получился бы вовсе.
//
// Без сигнала нагрузки нажим задаётся глубиной: сигнал задаёт усилие, а
// глубина — глубину, и при отсутствии сигнала остаётся то, что есть.
void test_fallback_height_is_plane_minus_press_depth(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	const DrawingSetup setup = drawing_setup(model, params);

	StubArm arm;
	arm.model = &model;
	arm.angles = q_to_degrees(model, setup.start);
	arm.paper_z = setup.plane.tcp_height_m;
	// Моторы не отдают нагрузку: высоту касания уточнить нечем.
	arm.torque_readable = false;

	so101::DrawOptions options;
	options.press_depth_m = 0.005;
	options.compensate = false;
	so101::DrawReport report;
	std::string error;
	CHECK_MSG(so101::draw_trajectory(arm, model, setup.targets, params, options,
	                                 report, error),
	          error);
	CHECK(report.completed);
	CHECK(report.points_drawn == setup.targets.size());
	// Рука стоит ниже плоскости — на глубину нажима.
	CHECK_NEAR(tcp_height(model, *arm.angles),
	           setup.plane.tcp_height_m - options.press_depth_m, 0.002);
}

// Разбор траектории без движения: недостижимая точка попадает в список
// проблем, а весь разбор — в счётчик.
void test_plan_trajectory_reports_unreachable_point(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	const DrawingSetup setup = drawing_setup(model, params);

	const std::vector<Eigen::Vector3d> targets{
	    Eigen::Vector3d{0.25, 0.0, 0.043}, Eigen::Vector3d{0.24, 0.18, 0.043},
	    Eigen::Vector3d{0.23, 0.02, 0.043}};

	const so101::TrajectoryPlan plan =
	    so101::plan_trajectory(model, targets, params, setup.start);
	CHECK(!plan.reachable);
	CHECK(plan.unreachable_points == 1);
	CHECK(!plan.problems.empty());
}

// Разбор траектории, до которой рука дотягивается: проблем нет.
void test_plan_trajectory_accepts_reachable_circle(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	const DrawingSetup setup = drawing_setup(model, params);

	const so101::TrajectoryPlan plan =
	    so101::plan_trajectory(model, setup.targets, params, setup.start);
	CHECK(plan.reachable);
	CHECK(plan.unreachable_points == 0);
	CHECK(plan.problems.empty());
	// Ошибка по окружности — миллиметры: точки решаются вертикальным
	// инструментом.
	CHECK(plan.max_position_error_m < 0.002);
	CHECK(plan.max_tilt_deg < 1.0);
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
	    model, Eigen::Vector3d{0.25, 0.0, 0.043}, so101::kToolDirectionDown,
	    start, params);
	CHECK(solution.converged);

	StubArm arm;
	arm.model = &model;
	arm.angles = q_to_degrees(model, solution.q);
	arm.paper_z = 0.043;

	so101::CaptureOptions options;
	options.tilt_tolerance_deg = 3.0;
	// Высота бумаги касанием не ищется: проверяется сам захват.
	options.touch_calibrate = false;
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

// Высота бумаги, найденная касанием, отличается от оценки человека: после
// подъёма и спуска рука оказывается там, где бумага на самом деле.
void test_capture_plane_finds_paper_by_touch(const so101::ArmModel& model_template) {
	so101::ArmModel model = model_template;
	so101::IkParams params;
	params.max_iterations = 300;
	params.position_tolerance_m = 0.001;
	params.orientation_tolerance_rad = 0.01;

	Eigen::VectorXd start = pinocchio::neutral(model.model);
	const so101::IkResult solution = so101::solve_ik(
	    model, Eigen::Vector3d{0.25, 0.0, 0.048}, so101::kToolDirectionDown,
	    start, params);
	CHECK(solution.converged);

	StubArm arm;
	arm.model = &model;
	arm.angles = q_to_degrees(model, solution.q);
	// Бумага ниже, чем её оценил человек: на пять миллиметров.
	arm.paper_z = 0.043;

	so101::CaptureOptions options;
	options.tilt_tolerance_deg = 3.0;
	options.touch_calibrate = true;
	int polls = 0;
	const so101::CaptureResult result = so101::capture_plane(
	    arm, model, params, options,
	    [&polls]() { return polls++ < 2 ? 0 : '\n'; });

	CHECK(result.accepted);
	CHECK(result.touch_calibrated);
	// Найденная высота близка к настоящей высоте бумаги.
	CHECK_NEAR(result.plane.tcp_height_m, 0.043, 0.002);
}

// Критерий касания: реакция бумаги отличается от её отсутствия, и слишком
// сильный нажим опознаётся отдельно.
void test_contact_criteria() {
	so101::TorqueSignal baseline{};
	so101::TorqueSignal touching{};
	for (std::size_t index = 0; index < kHoldingMotors; ++index) {
		touching[index] = 100;
	}

	so101::ContactOptions options;
	options.press_min = 10;
	options.press_max = 1000;

	// Реакции нет: карандаш в воздухе.
	so101::ContactReading free_reading =
	    so101::evaluate_contact(baseline, baseline, options);
	CHECK(free_reading.state == so101::ContactState::Free);
	CHECK(free_reading.delta == 0);

	// Реакция есть: карандаш на бумаге.
	so101::ContactReading touching_reading =
	    so101::evaluate_contact(touching, baseline, options);
	CHECK(touching_reading.state == so101::ContactState::Touching);
	CHECK(touching_reading.delta == 400);
	CHECK(touching_reading.level == 400);

	// Реакция огромная: моторы перегружены.
	so101::TorqueSignal heavy{};
	for (std::size_t index = 0; index < kHoldingMotors; ++index) {
		heavy[index] = 400;
	}
	so101::ContactReading heavy_reading =
	    so101::evaluate_contact(heavy, baseline, options);
	CHECK(heavy_reading.state == so101::ContactState::TooHard);

	// Шум не должен выдаваться за касание: порог считается по шуму.
	so101::TorqueSignal noisy{};
	noisy[0] = 5;
	so101::ContactReading noisy_reading =
	    so101::evaluate_contact(noisy, baseline, options);
	CHECK(noisy_reading.state == so101::ContactState::Free);
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
	    plane, circle, Eigen::Vector3d{0.25, 0.0, 0.043});

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
	test_contact_criteria();
	test_draw_trajectory_survives_deadband(arm);
	test_undershoot_without_compensation(arm);
	test_draw_trajectory_stops_on_lost_bus(arm);
	test_draw_trajectory_reports_residual(arm);
	test_contact_noise_is_not_a_contact(arm);
	test_draw_trajectory_survives_unreadable_torque(arm);
	test_draw_trajectory_without_contact_check(arm);
	test_draw_trajectory_continues_past_unreachable_point(arm);
	test_unreachable_point_gets_its_own_command(arm);
	test_seed_stays_at_last_converged_solution(arm);
	test_report_accumulates_between_laps(arm);
	test_fallback_height_is_plane_minus_press_depth(arm);
	test_plan_trajectory_reports_unreachable_point(arm);
	test_plan_trajectory_accepts_reachable_circle(arm);
	test_capture_plane_on_stub(arm);
	test_capture_plane_finds_paper_by_touch(arm);
	test_ik_along_the_circle(arm);

	return so101_test::finish("draw_circle");
}
