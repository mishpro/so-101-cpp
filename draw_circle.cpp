// Рисование окружности карандашом в горизонтальной плоскости базы.
//
// Режимы:
//   --set-plane   измерить плоскость рисования: оператор ставит остриё
//                 карандаша на поверхность, программа запоминает высоту;
//   --probe       показать регистры моторов вживую: по ним видно, что и
//                 как измеряется, и подбираются пороги нажима;
//   --dry-run     разбор без движения: траектория решается от текущей позы,
//                 моторы не включаются, печатается, какие точки достижимы;
//   --no-contact  рисование без сигнала нагрузки: высота берётся из
//                 плоскости, нажим не проверяется; при захвате плоскости
//                 высота касанием тоже не уточняется;
//   (по умолчанию) нарисовать окружность по измеренной плоскости.
//
// Программа — корень сборки: здесь разбираются аргументы, читается
// конфиг, поднимается шина и связываются части. Вся логика живёт в
// библиотеках: arm_model (кинематика и IK), drawing (что и где рисуем),
// drawing_execution (как рисуем), surface_contact (касание и нажим).

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "arm_feedback.hpp"
#include "arm_model.hpp"
#include "control_config.hpp"
#include "drawing.hpp"
#include "drawing_execution.hpp"
#include "serial_arm.hpp"
#include "servo_bus.hpp"

namespace {

using so101::kServoCount;

// Скорость мотора по умолчанию ограничена: в конфиге стоит 600, а это больше
// половины диапазона регистра (0..1000). Для рисования быстро не нужно, а
// на большой скорости рука в линейке проскакивает мимо точки.
constexpr uint16_t kDefaultMaxSpeed = 500;
constexpr double kDefaultPressMeters = 0.001;
constexpr std::size_t kDefaultPoints = 36;

// Сколько моторов держат карандаш: первые четыре. По ним же считается
// критерий касания.
constexpr std::size_t kHoldingMotors = 4;

// Предел единиц сигнала нагрузки: регистр 16-битный, и знаковый разбор
// даёт диапазон от −32768 до 32767. Модуль — до 32767.
constexpr long kMaxTorqueUnits = 32767;

// Клавиатура без эха и без ожидания: символ читается, только если он уже
// нажат.
class Keyboard {
public:
	Keyboard() {
		// Если stdin не терминал, настройки не трогаем и не восстанавливаем:
		// иначе в деструктор уйдёт нулевой termios.
		enabled_ = tcgetattr(STDIN_FILENO, &old_termios_) == 0;
		if (!enabled_) return;
		new_termios_ = old_termios_;
		new_termios_.c_lflag &= ~(ICANON | ECHO);
		new_termios_.c_cc[VMIN] = 0;
		new_termios_.c_cc[VTIME] = 0;
		tcsetattr(STDIN_FILENO, TCSANOW, &new_termios_);
	}

	~Keyboard() {
		if (enabled_) tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
	}

	Keyboard(const Keyboard&) = delete;
	Keyboard& operator=(const Keyboard&) = delete;
	Keyboard(Keyboard&&) = delete;
	Keyboard& operator=(Keyboard&&) = delete;

	// Символ, если он готов; 0 — нажатия ещё нет; -1 — ввод закрыт
	// (оператора за клавиатурой нет).
	//
	// Готовность спрашивается poll(2), а не чтением: при неблокирующем
	// read() и «нет данных», и «конец ввода» выглядят одинаково — нулём, —
	// а различать их надо: конец ввода означает, что ждать некого.
	int poll() const {
		pollfd descriptor{};
		descriptor.fd = STDIN_FILENO;
		descriptor.events = POLLIN;
		const int ready = ::poll(&descriptor, 1, 0);
		if (ready == 0) return 0;
		if (ready < 0) return -1;
		if (descriptor.revents & POLLIN) {
			char key = 0;
			const ssize_t read_bytes = read(STDIN_FILENO, &key, 1);
			return read_bytes > 0 ? static_cast<unsigned char>(key) : -1;
		}
		return -1;
	}

private:
	termios old_termios_{};
	termios new_termios_{};
	bool enabled_{false};
};

void print_usage(const char* program) {
	std::cout
	    << "Usage: " << program << " --set-plane [--plane FILE]\n"
	    << "       " << program
	    << " --center X Y --radius R [--points N] [--press M] [--speed S]\n"
	    << "                 [--laps N] [--plane FILE] [--port P] [--no-confirm]\n"
	    << "                 [--dry-run] [--no-contact]\n"
	    << "       " << program << " --probe [--port P]\n"
	    << "\n"
	    << "  --set-plane     измерить плоскость: оператор ставит остриё\n"
	    << "                  карандаша на поверхность, программа запоминает\n"
	    << "                  высоту и сохраняет её в файл плоскости\n"
	    << "  --center X Y    центр окружности в базовой системе, метры\n"
	    << "  --radius R      радиус окружности, метры\n"
	    << "  --points N      число точек окружности (по умолчанию "
	    << kDefaultPoints << ")\n"
	    << "  --press M       глубина нажима ниже найденного касания, метры\n"
	    << "                  (по умолчанию " << kDefaultPressMeters << ")\n"
	    << "  --speed S       скорость мотора, 1..1000 (по умолчанию из "
	       "конфига, не выше "
	    << kDefaultMaxSpeed << ")\n"
	    << "  --laps N        число кругов (по умолчанию 1)\n"
	    << "  --tilt-tolerance D  допустимый наклон при захвате, градусы\n"
	    << "                  (по умолчанию 3; погрешность высоты ~L(1-cos D))\n"
	    << "  --plane FILE    файл плоскости (по умолчанию draw_plane.json)\n"
	    << "  --port P        порт шины (по умолчанию из конфига)\n"
	    << "  --no-confirm    не спрашивать подтверждения перед движением\n"
	    << "  --dry-run       разбор без движения: траектория решается от\n"
	    << "                  текущей позы, моторы не включаются, рука не\n"
	    << "                  двигается; печатается, какие точки достижимы,\n"
	    << "                  а какие нет, с остатками по каждой\n"
	    << "  --no-contact    рисование без сигнала нагрузки: высота берётся\n"
	    << "                  из плоскости, нажим не проверяется, точки\n"
	    << "                  считаются без нажима; при захвате плоскости\n"
	    << "                  высота касанием тоже не уточняется\n"
	    << "\n"
	    << "Критерии достижения (подбираются замером --probe):\n"
	    << "  --reach-tolerance M  допуск по горизонтали, метры (0.010)\n"
	    << "  --reach-tilt D       допуск наклона инструмента, градусы (5)\n"
	    << "  --press-min N        порог касания по нагрузке (0 — по шуму)\n"
	    << "  --press-load N       желаемая реакция бумаги (0 — только касание)\n"
	    << "  --press-drop F       доля падения уровня нажима, при которой карандаш\n"
	    << "                       считается оторвавшимся от бумаги (0.5)\n"
	    << "  --press-max N        предел нажима по нагрузке (0 — без предела)\n"
	    << "  --press-noise-factor F  порог касания во столько раз выше шума (3)\n"
	    << "  --search-step M      шаг поиска касания, метры (0.001)\n"
	    << "  --search-max-depth M глубина поиска касания, метры (0.008)\n"
	    << "  --search-clearance M свободная высота над плоскостью, метры (0.01)\n"
	    << "  --no-compensate      не подстраивать цель по недобору\n"
	    << "  --compensate-tolerance M  допуск подстройки, метры (0.0015)\n"
	    << "  --compensate-iterations N подстроек на точку (2)\n"
	    << "  --no-touch-calibrate не искать высоту бумаги касанием\n"
	    << "  --load-source A      регистр сигнала нагрузки: 0x3c или 0x3e\n";
}

bool parse_double(const char* text, double& value) {
	char* end = nullptr;
	const double parsed = std::strtod(text, &end);
	if (end == text || *end != '\0') return false;
	// nan и inf — не числа: через них в допуски попадает мусор, и ни одна
	// точка потом не признаётся достигнутой.
	if (!std::isfinite(parsed)) return false;
	value = parsed;
	return true;
}

bool parse_long(const char* text, long& value) {
	char* end = nullptr;
	errno = 0;
	const long parsed = std::strtol(text, &end, 10);
	// Переполнение strtol возвращает LONG_MAX и ставит errno: без проверки
	// такой молча становится огромным числом, а потом — приведением к
	// int32_t без всякого смысла.
	if (end == text || *end != '\0' || errno == ERANGE) return false;
	value = parsed;
	return true;
}

// Допуски IK для рисования: на порядок tighter, чем у ручного шага в
// сантиметр. Позиция — сотые доли миллиметра, наклон — сотые доли градуса.
so101::IkParams drawing_ik_params(const ControlConfig& config) {
	so101::IkParams params;
	params.damping = config.ik_damping;
	params.max_iterations = config.ik_max_iterations;
	params.position_tolerance_m = 0.0001;
	params.orientation_tolerance_rad = 0.001;
	params.max_joint_step_rad = config.ik_max_joint_step_rad;
	return params;
}

// Текущие углы моторов в конфигурацию Pinocchio.
//
// Один и тот же путь и для рисования, и для сухого прогона: разбор должен
// идти от той же позы, от которой пойдёт рука, иначе «достижимо» означало
// бы «достижимо откуда-то ещё».
Eigen::VectorXd angles_to_q(const so101::ArmModel& model,
                            const so101::JointDegrees& degrees) {
	Eigen::VectorXd q = pinocchio::neutral(model.model);
	for (std::size_t index = 0; index < kServoCount; ++index) {
		const auto joint = model.model.joints[model.servo_joints[index]];
		q[joint.idx_q()] = model.servo_degrees_to_q(
		    static_cast<int>(index) + 1, degrees[index]);
	}
	return q;
}

// Текущее положение TCP: от него выбирается ближайшая точка окружности,
// чтобы первый ход был коротким.
Eigen::Vector3d tcp_position(so101::ArmModel& model,
                              const Eigen::VectorXd& q) {
	pinocchio::forwardKinematics(model.model, model.data, q);
	pinocchio::updateFramePlacements(model.model, model.data);
	return model.data.oMf[model.frame_id].translation();
}

// Программа закончилась — рука больше не нужна, и моторы расслабляются:
// иначе её нельзя поправить руками, а сервоприводы держат позицию с
// моментом около 11 кг·см. Рука при этом провисает под тяжестью — её надо
// держать.
void release_arm(so101::SerialArmFeedback& arm) {
	so101::JointMask all{};
	all.fill(true);
	if (arm.relax_motors(all)) {
		std::cout << "\nМоторы расслаблены: руку можно двигать рукой. Держите "
		             "её — она провиснет.\n"
		          << std::flush;
	}
}

// --- Режим разведки: регистры моторов вживую ---------------------------

// Читает один регистр мотора. Отсутствие ответа — не повод молчать: в
// разведке важно видеть и то, что не прочиталось.
std::optional<std::int32_t> probe_register(SerialPort& port, uint8_t servo_id,
                                            Register address) {
	const std::optional<std::uint16_t> raw =
	    read_raw_register(port, servo_id, address);
	if (!raw) return std::nullopt;
	return static_cast<std::int32_t>(*raw);
}

// Разведка: что показывают регистры моторов.
//
// Смысл режима в том, что единицы нагрузки неизвестны заранее: их и
// устанавливает этот режим — в трёх состояниях (рука в воздухе, остриё на
// бумаге, нажатие сильнее). По разнице между ними выбираются пороги
// касания и нажима.
int probe_mode(ControlConfig& config, Register torque_register) {
	SerialPort port(config.port);
	const std::optional<std::array<ServoLimits, kServoCount>> limits =
	    read_all_servo_limits(port);
	if (!limits) {
		std::cerr << "Failed to read joint limits from the servos\n";
		return 1;
	}

	std::cout << "Разведка регистров: " << port.path()
	          << "  (q — выход)\n"
	          << "Положите остриё на бумагу, потом надавите сильнее: по\n"
	          << "разнице нагрузки выбираются пороги касания и нажима.\n\n";

	Keyboard keyboard;
	// Управляющие последовательности — только в терминал: в файле или в
	// потоке они превращаются в мусор.
	const bool interactive = isatty(STDOUT_FILENO) != 0;
	const std::string clear = interactive ? "\033[K" : "";
	std::size_t drawn = 0;
	while (true) {
		if (drawn > 0 && interactive) std::cout << "\033[" << drawn << "A";

		std::cout << clear << "--- " << std::fixed << std::setprecision(2)
		          << "мотор   позиция      цель    нагрузка      ток    скорость"
		          << " ---\n";
		++drawn;
		std::int32_t load_sum = 0;
		std::int32_t current_sum = 0;
		for (std::size_t index = 0; index < kServoCount; ++index) {
			const auto servo_id = static_cast<std::uint8_t>(index + 1);
			const std::optional<std::int32_t> position =
			    probe_register(port, servo_id, Register::present_position);
			const std::optional<std::int32_t> goal =
			    probe_register(port, servo_id, Register::goal_position);
			const std::optional<std::int32_t> load =
			    read_servo_torque_signal(port, servo_id, torque_register);
			// Знак учитывается: ток упирающегося мотора отрицателен, и
			// беззнаковое прочтение дало бы 65500 вместо минуса.
			const std::optional<std::int32_t> current =
			    read_servo_torque_signal(port, servo_id,
			                             Register::present_torque_3c);
			const std::optional<std::int32_t> speed =
			    probe_register(port, servo_id, Register::present_speed);

			std::cout << clear << "  " << std::setw(2) << (index + 1) << "   ";
			if (position) {
				std::cout << std::setw(9) << std::fixed << std::setprecision(2)
				          << limits->at(index).position_to_degrees(
				                 static_cast<std::uint16_t>(*position))
				          << "°";
			} else {
				std::cout << std::setw(10) << "---";
			}
			std::cout << std::setw(10);
			if (goal) {
				std::cout << std::fixed << std::setprecision(2)
				          << limits->at(index).position_to_degrees(
				                 static_cast<std::uint16_t>(*goal))
				          << "°";
			} else {
				std::cout << "---";
			}
			std::cout << std::setw(11);
			if (load) {
				std::cout << *load;
				// Суммируются первые четыре мотора: карандаш держат
				// именно они, и по ним же считается критерий касания.
				if (index < kHoldingMotors) load_sum += *load;
			} else {
				std::cout << "---";
			}
			std::cout << std::setw(10);
			if (current) {
				std::cout << *current;
				if (index < kHoldingMotors) current_sum += *current;
			} else {
				std::cout << "---";
			}
			std::cout << std::setw(11);
			if (speed) {
				std::cout << *speed;
			} else {
				std::cout << "---";
			}
			std::cout << '\n';
			++drawn;
		}
		std::cout << clear << "  сумма по моторам 1..4: нагрузка " << load_sum
		          << ", ток " << current_sum << '\n';
		++drawn;
		std::cout << std::flush;

		const int key = keyboard.poll();
		if (key == 'q' || key == 'Q' || key < 0) break;
		usleep(150000);
	}
	return 0;
}

int capture_mode(ControlConfig& config, so101::ArmModel& model,
                 const std::string& plane_path, double tilt_tolerance_deg,
                 bool touch_calibrate, bool use_contact,
                 const so101::ContactSearchOptions& search,
                 Register torque_register) {
	SerialPort port(config.port);
	const std::optional<std::array<ServoLimits, kServoCount>> limits =
	    read_all_servo_limits(port);
	if (!limits) {
		std::cerr << "Failed to read joint limits from the servos\n";
		return 1;
	}

	so101::SerialArmFeedback arm(port, *limits, config.speed,
	                             config.joint_speed_divisor, torque_register);
	Keyboard keyboard;

	so101::CaptureOptions options;
	options.tilt_tolerance_deg = tilt_tolerance_deg;
	// Без сигнала нагрузки высоту касанием уточнить нечем: она остаётся
	// той, что задал оператор рукой. Флаг --no-contact действует и на
	// захват, а не только на рисование: иначе «--set-plane --no-contact»
	// всё равно читал бы нагрузку при уточнении высоты.
	options.touch_calibrate = touch_calibrate && use_contact;
	options.search = search;
	const so101::CaptureResult result = so101::capture_plane(
	    arm, model, drawing_ik_params(config), options,
	    [&keyboard]() { return keyboard.poll(); });

	if (!result.accepted) {
		std::cerr << "Plane was not captured: " << result.error << '\n';
		// Захват не состоялся, но программа закончилась: руку надо оставить
		// в руках человека.
		release_arm(arm);
		return 1;
	}

	std::string error;
	if (!so101::save_plane(plane_path, result.plane, error)) {
		std::cerr << "Failed to save the plane: " << error << '\n';
		// Программа закончилась — руку надо оставить в руках человека.
		release_arm(arm);
		return 1;
	}

	std::cout << std::fixed << std::setprecision(4)
	          << "Plane captured:\n"
	          << "  TCP height      " << result.plane.tcp_height_m << " m\n"
	          << "  contact point   (" << result.plane.contact_x_m << ", "
	          << result.plane.contact_y_m << ") m\n"
	          << "  tilt at contact " << std::setprecision(2)
	          << result.plane.tilt_deg << " deg\n"
	          << "  by touch        " << (result.touch_calibrated ? "yes" : "no")
	          << '\n'
	          << "Saved to " << plane_path << '\n';

	// Захват закончен: рука больше не нужна, моторы расслабляются.
	release_arm(arm);
	return 0;
}

// --- Режим сухого прогона: разбор без движения ----------------------------

// Разбор достижимости.
//
// Числа печатаются по-русски и в тех же единицах, что и весь остальной
// вывод: миллиметры и градусы. По каждой недостижимой точке печатается
// остаток: без него «недостижима» ничем не лучше молчания — не видно, чего
// именно не хватило.
void print_plan(const so101::TrajectoryPlan& plan, std::size_t total_points) {
	const int total = static_cast<int>(total_points);
	const int unreachable = plan.unreachable_points;
	std::cout << std::fixed << std::setprecision(4)
	          << "Точек всего: " << total << '\n'
	          << "Достижимы: " << (total - unreachable) << '\n'
	          << "Недостижимы: " << unreachable << '\n'
	          << "Максимальная ошибка позиции: " << std::setprecision(3)
	          << plan.max_position_error_m * 1000.0 << " мм\n"
	          << "Максимальный наклон: " << std::setprecision(3)
	          << plan.max_tilt_deg << " °\n";

	if (!plan.problems.empty()) {
		std::cout << "Проблемы по точкам:\n";
		for (const std::string& problem : plan.problems) {
			std::cout << "  " << problem << '\n';
		}
	}

	if (plan.reachable) {
		std::cout << "Итог: вся траектория достижима.\n";
	} else {
		std::cout << "Итог: часть точек недостижима — рисование пойдёт с "
		             "пропусками.\n";
	}
}

// Сухой прогон: траектория решается, рука не двигается.
//
// Моторы при этом только читаются: ни одна команда на них не уходит, и
// состояние руки остаётся ровно тем, каким было. Расслаблять моторы здесь
// нельзя — это уже изменение состояния, а сухой прогон не должен ничего
// менять.
//
// Траектория решается от текущей позы: от той же, от которой пошла бы рука
// при рисовании. Иначе «достижимо» означало бы «достижимо откуда-то ещё».
int dry_run_mode(ControlConfig& config, so101::ArmModel& model,
                 const std::string& plane_path, const so101::CircleSpec& circle,
                 bool use_contact, Register torque_register) {
	so101::Plane plane;
	std::string error;
	if (!so101::load_plane(plane_path, plane, error)) {
		std::cerr << "Failed to read the drawing plane: " << error
		          << "\nRun " << "./build/draw_circle --set-plane first\n";
		return 1;
	}

	SerialPort port(config.port);
	const std::optional<std::array<ServoLimits, kServoCount>> limits =
	    read_all_servo_limits(port);
	if (!limits) {
		std::cerr << "Failed to read joint limits from the servos\n";
		return 1;
	}

	so101::SerialArmFeedback arm(port, *limits, config.speed,
	                             config.joint_speed_divisor, torque_register);

	const std::optional<so101::JointDegrees> current = arm.read_joint_degrees();
	if (!current) {
		std::cerr << "Failed to read the current joint angles\n";
		return 1;
	}

	const Eigen::VectorXd q = angles_to_q(model, *current);
	const Eigen::Vector3d position = tcp_position(model, q);

	const std::vector<Eigen::Vector3d> targets =
	    so101::circle_targets(plane, circle, position);
	if (targets.empty()) {
		std::cerr << "The circle has no points\n";
		return 1;
	}

	std::cout << "Разбор траектории: моторы не включаются, рука не двигается.\n";
	if (!use_contact) {
		std::cout << "Нажим не учитывается: сигнал нагрузки не читается, "
		             "точки считаются без нажима.\n";
	}
	std::cout << std::fixed << std::setprecision(4)
	          << "Plane from " << plane_path << ": TCP height "
	          << plane.tcp_height_m << " m, tilt at capture "
	          << std::setprecision(2) << plane.tilt_deg << " deg\n"
	          << "Circle: centre (" << circle.center_x_m << ", "
	          << circle.center_y_m << ") m, radius " << circle.radius_m
	          << " m, " << targets.size() << " points\n";

	const so101::IkParams ik_params = drawing_ik_params(config);
	const so101::TrajectoryPlan plan =
	    so101::plan_trajectory(model, targets, ik_params, q);
	print_plan(plan, targets.size());
	return 0;
}

int draw_mode(ControlConfig& config, so101::ArmModel& model,
              const std::string& plane_path, const so101::CircleSpec& circle,
              uint16_t speed, std::size_t laps, bool confirm,
              const so101::DrawOptions& options, Register torque_register) {
	so101::Plane plane;
	std::string error;
	if (!so101::load_plane(plane_path, plane, error)) {
		std::cerr << "Failed to read the drawing plane: " << error
		          << "\nRun " << "./build/draw_circle --set-plane first\n";
		return 1;
	}

	SerialPort port(config.port);
	const std::optional<std::array<ServoLimits, kServoCount>> limits =
	    read_all_servo_limits(port);
	if (!limits) {
		std::cerr << "Failed to read joint limits from the servos\n";
		return 1;
	}

	so101::SerialArmFeedback arm(port, *limits, speed, config.joint_speed_divisor,
	                             torque_register);

	// С этого момента программа обязана оставить руку в руках человека, как
	// бы ни кончилась: иначе её нельзя поправить руками. Аварийный выход
	// (исключение, сигнал) моторы оставит включёнными — рука удержит
	// позицию и не провиснет.
	const std::optional<so101::JointDegrees> current = arm.read_joint_degrees();
	if (!current) {
		std::cerr << "Failed to read the current joint angles\n";
		release_arm(arm);
		return 1;
	}

	// Текущее положение TCP — от него считается ближайшая точка окружности,
	// чтобы первый ход был коротким.
	const Eigen::VectorXd q = angles_to_q(model, *current);
	const Eigen::Vector3d position = tcp_position(model, q);

	const std::vector<Eigen::Vector3d> targets =
	    so101::circle_targets(plane, circle, position);
	if (targets.empty()) {
		std::cerr << "The circle has no points\n";
		release_arm(arm);
		return 1;
	}

	// Разбор достижимости печатает исполнитель: он же решает, что делать с
	// недостижимыми точками. Здесь отказа быть не должно: недостижимая
	// точка — не повод отказаться рисовать, а повод нарисовать остальное и
	// сказать, чего не вышло.
	const so101::IkParams ik_params = drawing_ik_params(config);

	std::cout << std::fixed << std::setprecision(4)
	          << "Plane from " << plane_path << ": TCP height "
	          << plane.tcp_height_m << " m, tilt at capture "
	          << std::setprecision(2) << plane.tilt_deg << " deg\n"
	          << "Circle: centre (" << circle.center_x_m << ", "
	          << circle.center_y_m << ") m, radius " << circle.radius_m
	          << " m, " << targets.size() << " points\n"
	          << "Criteria: horizontal tolerance "
	          << std::setprecision(1) << options.reach_position_tolerance_m * 1000.0
	          << " mm, tilt " << options.reach_tilt_tolerance_deg << " deg, press "
	          << std::setprecision(2) << options.press_depth_m * 1000.0 << " mm\n";

	if (!options.use_contact) {
		std::cout << "Рисование идёт без проверки нажима: высота берётся из "
		             "плоскости, сигнал нагрузки не читается.\n";
	}

	if (confirm) {
		std::cout << "Start drawing? Press Enter, or Ctrl-C to abort.\n"
		          << std::flush;
		Keyboard keyboard;
		char key = 0;
		while (key != '\n' && key != '\r') {
			key = keyboard.poll();
			if (key == 0) usleep(20000);
		}
	}

	so101::DrawReport report;
	for (std::size_t lap = 0; lap < laps; ++lap) {
		std::cout << "Lap " << (lap + 1) << " of " << laps << '\n';
		if (!so101::draw_trajectory(arm, model, targets, ik_params, options,
		                            report, error)) {
			std::cerr << "Drawing stopped: " << error << '\n';
			// Рисование закончено, чем бы оно ни кончилось: рука больше не
			// нужна, и её надо оставить в руках человека.
			release_arm(arm);
			return 1;
		}
	}

	std::cout << "Done: " << report.points_drawn << " points, max horizontal error "
	          << std::fixed << std::setprecision(3)
	          << report.max_position_error_m * 1000.0 << " mm, max tilt "
	          << std::setprecision(3) << report.max_tilt_deg << " deg, compensated "
	          << report.compensated << ", points without press "
	          << report.contact_drops << ", unreachable points "
	          << report.unreachable_points << '\n';

	// Программа закончилась — моторы расслабляются, руку можно править
	// руками.
	release_arm(arm);
	return 0;
}

}  // namespace

int main(int argc, char** argv) {
	bool set_plane = false;
	bool probe = false;
	bool dry_run = false;
	bool confirm = true;
	bool touch_calibrate = true;
	std::string plane_path = "draw_plane.json";
	so101::CircleSpec circle;
	bool have_center = false;
	double press_m = kDefaultPressMeters;
	std::size_t laps = 1;
	long speed_override = 0;
	std::string port_override;
	double tilt_tolerance_deg = 3.0;

	so101::DrawOptions draw_options;
	so101::ContactSearchOptions search_options;
	Register torque_register = kTorqueSignalRegister;

	for (int index = 1; index < argc; ++index) {
		const std::string argument = argv[index];
		if (argument == "--set-plane") {
			set_plane = true;
		} else if (argument == "--probe") {
			probe = true;
		} else if (argument == "--dry-run") {
			dry_run = true;
		} else if (argument == "--no-contact") {
			// Нажим не проверяется: высоту задаёт плоскость, а сигнал
			// нагрузки не читается вовсе.
			draw_options.use_contact = false;
		} else if (argument == "--no-confirm") {
			confirm = false;
		} else if (argument == "--no-touch-calibrate") {
			touch_calibrate = false;
		} else if (argument == "--help" || argument == "-h") {
			print_usage(argv[0]);
			return 0;
		} else if (argument == "--center" && index + 2 < argc) {
			if (!parse_double(argv[index + 1], circle.center_x_m) ||
			    !parse_double(argv[index + 2], circle.center_y_m)) {
				std::cerr << "--center expects two numbers\n";
				return 1;
			}
			have_center = true;
			index += 2;
		} else if (argument == "--radius" && index + 1 < argc) {
			if (!parse_double(argv[index + 1], circle.radius_m)) {
				std::cerr << "--radius expects a number\n";
				return 1;
			}
			index += 1;
		} else if (argument == "--points" && index + 1 < argc) {
			long points = 0;
			if (!parse_long(argv[index + 1], points) || points <= 0 ||
			    points > 10000) {
				std::cerr << "--points expects a number in 1..10000\n";
				return 1;
			}
			circle.points = static_cast<std::size_t>(points);
			index += 1;
		} else if (argument == "--press" && index + 1 < argc) {
			if (!parse_double(argv[index + 1], press_m) || press_m < 0.0 ||
			    press_m > 0.05) {
				std::cerr << "--press expects a number in 0..0.05\n";
				return 1;
			}
			index += 1;
		} else if (argument == "--speed" && index + 1 < argc) {
			if (!parse_long(argv[index + 1], speed_override) ||
			    speed_override < 1 || speed_override > 1000) {
				std::cerr << "--speed expects a number in 1..1000\n";
				return 1;
			}
			index += 1;
		} else if (argument == "--tilt-tolerance" && index + 1 < argc) {
			if (!parse_double(argv[index + 1], tilt_tolerance_deg) ||
			    tilt_tolerance_deg < 0.0 || tilt_tolerance_deg > 90.0) {
				std::cerr << "--tilt-tolerance expects a number in 0..90\n";
				return 1;
			}
			index += 1;
		} else if (argument == "--laps" && index + 1 < argc) {
			long value = 0;
			if (!parse_long(argv[index + 1], value) || value <= 0) {
				std::cerr << "--laps expects a positive number\n";
				return 1;
			}
			laps = static_cast<std::size_t>(value);
			index += 1;
		} else if (argument == "--plane" && index + 1 < argc) {
			plane_path = argv[index + 1];
			index += 1;
		} else if (argument == "--port" && index + 1 < argc) {
			// Порт из аргумента перекрывает конфиг: он применяется позже, к
			// уже загруженной конфигурации.
			port_override = argv[index + 1];
			index += 1;
		} else if (argument == "--reach-tolerance" && index + 1 < argc) {
			if (!parse_double(argv[index + 1],
			                  draw_options.reach_position_tolerance_m) ||
			    draw_options.reach_position_tolerance_m <= 0.0) {
				std::cerr << "--reach-tolerance expects a positive number\n";
				return 1;
			}
			index += 1;
		} else if (argument == "--reach-tilt" && index + 1 < argc) {
			if (!parse_double(argv[index + 1],
			                  draw_options.reach_tilt_tolerance_deg) ||
			    draw_options.reach_tilt_tolerance_deg <= 0.0 ||
			    draw_options.reach_tilt_tolerance_deg > 90.0) {
				std::cerr << "--reach-tilt expects a number in 0..90\n";
				return 1;
			}
			index += 1;
		} else if (argument == "--press-min" && index + 1 < argc) {
			long value = 0;
			if (!parse_long(argv[index + 1], value) || value < 0 ||
			    value > kMaxTorqueUnits) {
				std::cerr << "--press-min expects a number in 0.."
				          << kMaxTorqueUnits << "\n";
				return 1;
			}
			search_options.contact.press_min = static_cast<std::int32_t>(value);
			index += 1;
		} else if (argument == "--press-max" && index + 1 < argc) {
			long value = 0;
			if (!parse_long(argv[index + 1], value) || value < 0 ||
			    value > kMaxTorqueUnits) {
				std::cerr << "--press-max expects a number in 0.."
				          << kMaxTorqueUnits << "\n";
				return 1;
			}
			search_options.contact.press_max = static_cast<std::int32_t>(value);
			index += 1;
		} else if (argument == "--press-load" && index + 1 < argc) {
			long value = 0;
			if (!parse_long(argv[index + 1], value) || value < 0 ||
			    value > kMaxTorqueUnits) {
				std::cerr << "--press-load expects a number in 0.."
				          << kMaxTorqueUnits << "\n";
				return 1;
			}
			search_options.contact.press_target = static_cast<std::int32_t>(value);
			index += 1;
		} else if (argument == "--press-drop" && index + 1 < argc) {
			if (!parse_double(argv[index + 1], draw_options.press_drop_fraction) ||
			    draw_options.press_drop_fraction < 0.0 ||
			    draw_options.press_drop_fraction > 1.0) {
				std::cerr << "--press-drop expects a number in 0..1\n";
				return 1;
			}
			index += 1;
		} else if (argument == "--press-noise-factor" && index + 1 < argc) {
			if (!parse_double(argv[index + 1],
			                  search_options.press_noise_factor) ||
			    search_options.press_noise_factor < 0.0 ||
			    search_options.press_noise_factor > 1000.0) {
				std::cerr << "--press-noise-factor expects a number in 0..1000\n";
				return 1;
			}
			index += 1;
		} else if (argument == "--search-step" && index + 1 < argc) {
			if (!parse_double(argv[index + 1], search_options.search_step_m) ||
			    search_options.search_step_m <= 0.0) {
				std::cerr << "--search-step expects a positive number\n";
				return 1;
			}
			index += 1;
		} else if (argument == "--search-max-depth" && index + 1 < argc) {
			if (!parse_double(argv[index + 1],
			                  search_options.search_max_depth_m) ||
			    search_options.search_max_depth_m <= 0.0) {
				std::cerr << "--search-max-depth expects a positive number\n";
				return 1;
			}
			index += 1;
		} else if (argument == "--search-clearance" && index + 1 < argc) {
			if (!parse_double(argv[index + 1],
			                  search_options.search_clearance_m) ||
			    search_options.search_clearance_m <= 0.0) {
				std::cerr << "--search-clearance expects a positive number\n";
				return 1;
			}
			index += 1;
		} else if (argument == "--no-compensate") {
			draw_options.compensate = false;
		} else if (argument == "--compensate-tolerance" && index + 1 < argc) {
			if (!parse_double(argv[index + 1],
			                  draw_options.compensate_tolerance_m) ||
			    draw_options.compensate_tolerance_m <= 0.0) {
				std::cerr << "--compensate-tolerance expects a positive number\n";
				return 1;
			}
			index += 1;
		} else if (argument == "--compensate-iterations" && index + 1 < argc) {
			long value = 0;
			if (!parse_long(argv[index + 1], value) || value < 0 ||
			    value > 10) {
				std::cerr << "--compensate-iterations expects a number in 0..10\n";
				return 1;
			}
			draw_options.compensate_iterations = static_cast<int>(value);
			index += 1;
		} else if (argument == "--load-source" && index + 1 < argc) {
			const std::string source = argv[index + 1];
			if (source == "0x3c" || source == "3c") {
				torque_register = Register::present_torque_3c;
			} else if (source == "0x3e" || source == "3e") {
				torque_register = Register::present_torque_3e;
			} else {
				std::cerr << "--load-source expects 0x3c or 0x3e\n";
				return 1;
			}
			index += 1;
		} else {
			std::cerr << "Unknown argument: " << argument << '\n';
			print_usage(argv[0]);
			return 1;
		}
	}

	try {
		ControlConfig config{};
		if (!load_config("send_config.json", config)) {
			std::cerr << "Failed to load valid send_config.json\n";
			return 1;
		}
		if (!port_override.empty()) config.port = port_override;

		if (probe) {
			return probe_mode(config, torque_register);
		}

		so101::ArmModel model{};
		std::string error;
		if (!so101::load_arm_model(config.urdf_path, config.end_effector, model,
		                           error)) {
			std::cerr << error << '\n';
			return 1;
		}

		if (set_plane) {
			return capture_mode(config, model, plane_path, tilt_tolerance_deg,
			                    touch_calibrate, draw_options.use_contact,
			                    search_options, torque_register);
		}

		if (!have_center) {
			std::cerr << "--center X Y is required to draw or to analyse a circle\n";
			print_usage(argv[0]);
			return 1;
		}
		if (circle.radius_m <= 0.0) {
			std::cerr << "--radius must be positive\n";
			return 1;
		}

		if (dry_run) {
			// Сухой прогон ничего не спрашивает и ничего не двигает: он
			// нужен, чтобы увидеть разбор, и выходить сразу.
			return dry_run_mode(config, model, plane_path, circle,
			                    draw_options.use_contact, torque_register);
		}

		// Скорость по умолчанию ограничена, явно заданная — как есть.
		const uint16_t speed = speed_override > 0
		                           ? static_cast<uint16_t>(speed_override)
		                           : static_cast<uint16_t>(
		                                 std::min<int>(config.speed,
		                                                kDefaultMaxSpeed));
		draw_options.press_depth_m = press_m;
		draw_options.search = search_options;
		return draw_mode(config, model, plane_path, circle, speed, laps, confirm,
		                 draw_options, torque_register);
	} catch (const std::exception& exception) {
		std::cerr << exception.what() << '\n';
		return 1;
	}
}
