// Рисование окружности карандашом в горизонтальной плоскости базы.
//
// Два режима:
//   --set-plane   измерить плоскость рисования: оператор ставит остриё
//                 карандаша на поверхность, программа запоминает высоту;
//   (по умолчанию) нарисовать окружность по измеренной плоскости.
//
// Программа — корень сборки: здесь разбираются аргументы, читается
// конфиг, поднимается шина и связываются части. Вся логика живёт в
// библиотеках: arm_model (кинематика и IK), drawing (что и где рисуем),
// drawing_execution (как рисуем).

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstdlib>
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
constexpr double kDefaultPressMeters = 0.0002;
constexpr std::size_t kDefaultPoints = 36;

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
	    << "\n"
	    << "  --set-plane     измерить плоскость: оператор ставит остриё\n"
	    << "                  карандаша на поверхность, программа запоминает\n"
	    << "                  высоту и сохраняет её в файл плоскости\n"
	    << "  --center X Y    центр окружности в базовой системе, метры\n"
	    << "  --radius R      радиус окружности, метры\n"
	    << "  --points N      число точек окружности (по умолчанию "
	    << kDefaultPoints << ")\n"
	    << "  --press M       нажатие карандаша на бумагу, метры (по умолчанию "
	    << kDefaultPressMeters << ")\n"
	    << "  --speed S       скорость мотора, 1..1000 (по умолчанию из "
	       "конфига, не выше "
	    << kDefaultMaxSpeed << ")\n"
	    << "  --laps N        число кругов (по умолчанию 1)\n"
	    << "  --tilt-tolerance D  допустимый наклон при захвате, градусы\n"
	    << "                  (по умолчанию 3; погрешность высоты ~L(1-cos D))\n"
	    << "  --plane FILE    файл плоскости (по умолчанию draw_plane.json)\n"
	    << "  --port P        порт шины (по умолчанию из конфига)\n"
	    << "  --no-confirm    не спрашивать подтверждения перед движением\n";
}

bool parse_double(const char* text, double& value) {
	char* end = nullptr;
	const double parsed = std::strtod(text, &end);
	if (end == text || *end != '\0') return false;
	value = parsed;
	return true;
}

bool parse_long(const char* text, long& value) {
	char* end = nullptr;
	const long parsed = std::strtol(text, &end, 10);
	if (end == text || *end != '\0') return false;
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

int capture_mode(ControlConfig& config, so101::ArmModel& model,
                 const std::string& plane_path, double tilt_tolerance_deg) {
	SerialPort port(config.port);
	const std::optional<std::array<ServoLimits, kServoCount>> limits =
	    read_all_servo_limits(port);
	if (!limits) {
		std::cerr << "Failed to read joint limits from the servos\n";
		return 1;
	}

	so101::SerialArmFeedback arm(port, *limits, config.speed,
	                             config.joint_speed_divisor);
	Keyboard keyboard;

	so101::CaptureOptions options;
	options.tilt_tolerance_deg = tilt_tolerance_deg;
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
	          << "Saved to " << plane_path << '\n';

	// Захват закончен: рука больше не нужна, моторы расслабляются.
	release_arm(arm);
	return 0;
}

int draw_mode(ControlConfig& config, so101::ArmModel& model,
              const std::string& plane_path, const so101::CircleSpec& circle,
              double press_m, uint16_t speed, std::size_t laps, bool confirm) {
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

	so101::SerialArmFeedback arm(port, *limits, speed, config.joint_speed_divisor);

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
	Eigen::VectorXd q = pinocchio::neutral(model.model);
	for (std::size_t index = 0; index < kServoCount; ++index) {
		const auto joint = model.model.joints[model.servo_joints[index]];
		q[joint.idx_q()] = model.servo_degrees_to_q(
		    static_cast<int>(index) + 1, (*current)[index]);
	}
	pinocchio::forwardKinematics(model.model, model.data, q);
	pinocchio::updateFramePlacements(model.model, model.data);
	const Eigen::Vector3d position = model.data.oMf[model.frame_id].translation();

	const std::vector<Eigen::Vector3d> targets =
	    so101::circle_targets(plane, circle, press_m, position);
	if (targets.empty()) {
		std::cerr << "The circle has no points\n";
		release_arm(arm);
		return 1;
	}

	// Проверка всей траектории до первого движения: если хоть одна точка
	// недостижима, руку двигать нельзя.
	const so101::IkParams ik_params = drawing_ik_params(config);
	const so101::TrajectoryCheck check =
	    so101::check_trajectory(model, targets, ik_params, q);
	if (!check.reachable) {
		std::cerr << "The circle is not reachable from the current pose: "
		          << "some point cannot be solved with a vertical tool.\n";
		release_arm(arm);
		return 1;
	}

	std::cout << std::fixed << std::setprecision(4)
	          << "Plane from " << plane_path << ": TCP height "
	          << plane.tcp_height_m << " m, tilt at capture "
	          << std::setprecision(2) << plane.tilt_deg << " deg\n"
	          << "Circle: centre (" << circle.center_x_m << ", "
	          << circle.center_y_m << ") m, radius " << circle.radius_m
	          << " m, " << targets.size() << " points, press "
	          << press_m * 1000.0 << " mm\n"
	          << "Trajectory checked: max position error "
	          << std::setprecision(4) << check.max_position_error_m * 1000.0
	          << " mm, max tilt " << std::setprecision(3)
	          << check.max_tilt_deg << " deg\n";

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

	const so101::DrawOptions options;
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

	std::cout << "Done: " << report.points_drawn << " points, max position error "
	          << std::fixed << std::setprecision(3)
	          << report.max_position_error_m * 1000.0 << " mm, max tilt "
	          << std::setprecision(3) << report.max_tilt_deg << " deg\n";

	// Программа закончилась — моторы расслабляются, руку можно править
	// руками.
	release_arm(arm);
	return 0;
}

}  // namespace

int main(int argc, char** argv) {
	bool set_plane = false;
	bool confirm = true;
	std::string plane_path = "draw_plane.json";
	so101::CircleSpec circle;
	bool have_center = false;
	double press_m = kDefaultPressMeters;
	std::size_t laps = 1;
	long speed_override = 0;
	std::string port_override;
	double tilt_tolerance_deg = 3.0;

	for (int index = 1; index < argc; ++index) {
		const std::string argument = argv[index];
		if (argument == "--set-plane") {
			set_plane = true;
		} else if (argument == "--no-confirm") {
			confirm = false;
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
			if (!parse_long(argv[index + 1], points) || points <= 0) {
				std::cerr << "--points expects a positive number\n";
				return 1;
			}
			circle.points = static_cast<std::size_t>(points);
			index += 1;
		} else if (argument == "--press" && index + 1 < argc) {
			if (!parse_double(argv[index + 1], press_m)) {
				std::cerr << "--press expects a number\n";
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

		so101::ArmModel model{};
		std::string error;
		if (!so101::load_arm_model(config.urdf_path, config.end_effector, model,
		                           error)) {
			std::cerr << error << '\n';
			return 1;
		}

		if (set_plane) {
			return capture_mode(config, model, plane_path, tilt_tolerance_deg);
		}

		if (!have_center) {
			std::cerr << "--center X Y is required to draw a circle\n";
			print_usage(argv[0]);
			return 1;
		}
		if (circle.radius_m <= 0.0) {
			std::cerr << "--radius must be positive\n";
			return 1;
		}

		// Скорость по умолчанию ограничена, явно заданная — как есть.
		const uint16_t speed = speed_override > 0
		                           ? static_cast<uint16_t>(speed_override)
		                           : static_cast<uint16_t>(
		                                 std::min<int>(config.speed,
		                                                kDefaultMaxSpeed));
		return draw_mode(config, model, plane_path, circle, press_m, speed, laps,
		                 confirm);
	} catch (const std::exception& exception) {
		std::cerr << exception.what() << '\n';
		return 1;
	}
}
