#include "drawing_execution.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>
#include <unistd.h>

namespace so101 {
namespace {

constexpr double kPiExecution = 3.14159265358979323846;

// Сколько ждать одной ступени поиска касания: шаг маленький, рука едет
// быстро, но наклон инструмента успевает устояться.
constexpr int kSearchStepTimeoutMs = 2000;

// Сколько замеров нагрузки снимается на свободной высоте, чтобы узнать шум
// сигнала, и с какой паузой между ними.
constexpr int kNoiseSamples = 5;
constexpr int kNoiseSampleDelayMs = 50;

// Насколько расстояние и наклон должны перестать меняться, чтобы считать, что
// рука остановилась.
//
// Допуск заведомо больше шага квантования: регистр положения STS3215 имеет
// 4095 шагов на оборот, то есть 0.088° на шаг, и на кончике карандаша это
// около 0.3 мм. Меньший допуск не сработает никогда: показание всё равно
// дрожит на шаг квантования, и рука никогда не будет признана остановившейся.
//
// Допуск вдвое больше шага: соседние показания отличаются на шаг туда и
// шаг обратно, то есть на два шага.
//
// Проверка односторонняя: рука считается остановившейся, если расстояние
// не уменьшилось заметно. Пока рука едет к цели, расстояние падает на всю
// величину шага квантования и больше, и такая рука ещё не считается
// остановившейся. Двустороннее сравнение не годится: квантование дрожит в
// обе стороны, и расстояние «скачет» туда-обратно.
constexpr double kSettleEpsilonM = 0.002;
constexpr double kSettleEpsilonDeg = 0.6;

// Меньший шаг поиска бессмысленен: цель почти не сдвигается, а поиск
// становится долгим. Больше — спуск мимо касания.
constexpr double kMinimumSearchStepM = 0.0001;

// Сколько шагов поиска не больше: страховка от шага, заданного слишком
// мелким.
constexpr int kMaxSearchSteps = 2000;

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

// Где оказалась рука и как наклонён инструмент.
struct ArmState {
	Eigen::Vector3d position{Eigen::Vector3d::Zero()};
	double tilt_deg{0.0};
};

// Состояние руки по измеренным углам.
ArmState arm_state(ArmModel& model, const JointDegrees& angles) {
	ArmState state;
	const Eigen::VectorXd q = angles_to_q(model, angles);
	pinocchio::forwardKinematics(model.model, model.data, q);
	pinocchio::updateFramePlacements(model.model, model.data);
	state.position = model.data.oMf[model.frame_id].translation();
	state.tilt_deg = tilt_degrees(tool_axis(model));
	return state;
}

// Горизонтальная ошибка: насколько TCP ушёл от цели по плоскости.
//
// Вертикаль сюда не входит намеренно: высоту TCP задаёт нажим, и до своей
// цели по высоте сервопривод всё равно не доходит. Горизонталь — это то, что
// определяет форму окружности.
Eigen::Vector2d horizontal_error(const Eigen::Vector3d& target,
                                  const Eigen::Vector3d& position) {
	return Eigen::Vector2d(target.x() - position.x(), target.y() - position.y());
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

// Остаток с разбором: команда, цель мотора, факт.
//
// Три числа рядом объясняют то, чего не объясняют одни углы: команда — то,
// что посчитал решатель; цель мотора — то, что мотору велели и что он
// запомнил; факт — то, где мотор стоит. Расхождение цели мотора и команды
// означает, что команда не дошла; расхождение факта и цели — что мотор не
// доехал.
std::string describe_motors(const ArmModel& model,
                            const JointDegrees& actual,
                            const JointDegrees& commanded,
                            const std::optional<JointDegrees>& goal,
                            double horizontal_mm, double tilt_deg) {
	std::ostringstream text;
	text << "\n  остаток по суставам (факт / команда / Δ):";
	for (std::size_t index = 0; index < model.ik_joints.size(); ++index) {
		text << "\n    " << model.model.names[model.ik_joints[index]] << ": "
		     << std::fixed << std::setprecision(2) << actual[index] << " / "
		     << commanded[index] << " / "
		     << (actual[index] - commanded[index]) << "°";
	}
	if (goal) {
		text << "\n  цель мотора (регистр 0x2A) / команда / Δ:";
		for (std::size_t index = 0; index < model.ik_joints.size(); ++index) {
			text << "\n    " << model.model.names[model.ik_joints[index]] << ": "
			     << std::fixed << std::setprecision(2) << (*goal)[index] << " / "
			     << commanded[index] << " / "
			     << ((*goal)[index] - commanded[index]) << "°";
		}
	}
	text << "\n  горизонтальное расстояние от TCP до цели: " << std::fixed
	     << std::setprecision(1) << horizontal_mm << " мм,  наклон: "
	     << std::setprecision(2) << tilt_deg << "°";
	return text.str();
}

// Состояние сессии рисования: конфигурация, команда, база нагрузки.
//
// Собрано в одну структуру, потому что иначе одни и те же параметры ходят
// из функции в функцию, и в какой-то момент какой-нибудь забывают
// обновить.
struct Session {
	Session(ArmFeedback& arm_in, ArmModel& model_in, const IkParams& ik_params_in,
	        const ContactSearchOptions& search_in)
	    : arm(arm_in),
	      model(model_in),
	      ik_params(ik_params_in),
	      search_options(search_in),
	      q(pinocchio::neutral(model_in.model)) {}

	ArmFeedback& arm;
	ArmModel& model;
	const IkParams& ik_params;
	const ContactSearchOptions& search_options;

	// Текущая конфигурация: ею засевается решатель, и она же ведёт цепочку
	// точек от одной к другой.
	Eigen::VectorXd q;
	// Команда моторам. wrist_roll и gripper в ней не меняются: ими не
	// двигают, иначе кисть уводит точку TCP на миллиметры.
	JointDegrees degrees{};
	// База нагрузки: то, что показывают моторы, когда карандаш в воздухе.
	TorqueSignal baseline{};
	// Пороги касания, действующие сейчас: они считаются по шуму сигнала и
	// потому отличаются от заданных снаружи.
	ContactOptions contact{};

	// Критерий прибытия. Допуск заведомо больше мёртвой зоны сервопривода:
	// до своей цели он не доходит, и меньший допуск рука не проходит
	// никогда.
	double position_tolerance_m{0.010};
	double tilt_tolerance_deg{5.0};
	int poll_interval_ms{20};
	// Сколько ждать прибытия: первый ход самый длинный, руку только что
	// поставили рукой.
	int reach_timeout_ms{5000};
};

// Что получилось на точке: команда ушла моторам, а IK сошёлся или нет.
struct CommandOutcome {
	// Команда отправлена моторам.
	bool sent{false};
	// IK не сошёлся: моторам отправлено лучшее приближение, точка
	// недостижима.
	bool unreachable{false};
	// Остаток по позиции, м, и по наклону инструмента, °.
	double residual_m{0.0};
	double residual_tilt_deg{0.0};
};

// Решение IK и команда моторам.
//
// Несошедшийся IK — не поломка и не повод останавливаться: моторам уходит
// лучшее приближение, о точке печатается предупреждение, рисование
// продолжается. Так поступает и сам решатель: приближение всё равно ближе
// к цели, чем ничего, а отказ останавливает круг целиком из-за одного места.
//
// Поле sent говорит, ушла ли команда: false означает, что отправить её не
// удалось, и это единственная причина остановить рисование. Номер точки
// (point_number) — для предупреждения; ноль означает точку вне траектории,
// у которой номера нет.
CommandOutcome solve_and_command(Session& session, const Eigen::Vector3d& target,
                                 std::string& error, int point_number = 0) {
	CommandOutcome outcome;
	const IkResult ik = solve_ik(session.model, target, kToolDirectionDown,
	                             session.q, session.ik_params);
	outcome.unreachable = !ik.converged;
	outcome.residual_m = ik.final_error_m;
	outcome.residual_tilt_deg =
	    ik.final_orientation_error_rad * 180.0 / kPiExecution;
	if (outcome.unreachable) {
		std::ostringstream warning;
		warning << "  ⚠ ";
		if (point_number > 0) warning << "точка " << point_number << ": ";
		warning << "IK не сошёлся, моторам отправлено лучшее приближение "
		           "(остаток "
		        << std::fixed << std::setprecision(1)
		        << outcome.residual_m * 1000.0 << " мм, наклон "
		        << std::setprecision(2) << outcome.residual_tilt_deg << "°)\n";
		std::cout << warning.str() << std::flush;
	}
	// Конфигурация для следующей точки — та, на которой решатель сошёлся.
	// Если решатель не сошёлся, его результат (зажатый в пределы сустава)
	// за seed не берём: следующая точка тогда решалась бы из предела, и
	// неудача рассыпалась бы на весь круг.
	if (ik.converged) {
		session.q = ik.q;
	}

	// Команда моторам строится из самого решения, а не из seed'а.
	//
	// Seed и команда — разные вещи, и путать их нельзя. Seed на
	// несошедшемся IK остаётся последним сошедшимся (об этом выше), и
	// команда, построенная из него, была бы копией предыдущей позы:
	// моторам уходило бы ровно то же, что и на предыдущей точке, рука не
	// двигалась бы вовсе, а сообщение о лучшем приближении было бы
	// неправдой. Лучшее приближение — это как раз решение решателя, и
	// оно уже зажато в пределы суставов.
	for (std::size_t joint_index = 0;
	     joint_index < session.model.ik_joints.size(); ++joint_index) {
		const int servo_id = static_cast<int>(joint_index) + 1;
		const auto joint = session.model.model.joints
		                        [session.model.ik_joints[joint_index]];
		session.degrees[joint_index] = session.model.q_to_servo_degrees(
		    servo_id, ik.q[joint.idx_q()]);
	}

	if (!session.arm.write_joint_degrees(session.degrees)) {
		error = "failed to send the joint command";
		return outcome;
	}
	outcome.sent = true;
	return outcome;
}

// Ожидание прибытия в точку.
struct Arrival {
	bool reached{false};
	// Шина пропала: ждать больше нечего.
	bool bus_lost{false};
	Eigen::Vector2d horizontal{Eigen::Vector2d::Zero()};
	double tilt_deg{0.0};
	Eigen::Vector3d position{Eigen::Vector3d::Zero()};
	JointDegrees angles{};
};

// Ждать, пока рука окажется в точке: горизонтальное расстояние в допуске и
// наклон инструмента в допуске.
//
// «Близко» недостаточно: сервопривод подходит к цели плавно, и одно только
// расстояние срабатывает, пока рука ещё в пути. Поэтому прибытие — это ещё
// и остановка: расстояние перестало меняться.
//
// Состояние возвращается в любом случае: по нему видно, где рука
// остановилась, даже если до цели не дошла.
Arrival wait_for_arrival(Session& session, const Eigen::Vector3d& target,
                          double position_tolerance_m, int timeout_ms) {
	Arrival arrival;
	bool have_previous = false;
	double previous_error = 0.0;
	double previous_tilt = 0.0;
	const auto deadline =
	    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
	while (true) {
		const std::optional<JointDegrees> actual =
		    session.arm.read_joint_degrees();
		if (!actual) {
			arrival.bus_lost = true;
			return arrival;
		}
		const ArmState state = arm_state(session.model, *actual);
		arrival.angles = *actual;
		arrival.position = state.position;
		arrival.horizontal = horizontal_error(target, state.position);
		arrival.tilt_deg = state.tilt_deg;

		const double error = arrival.horizontal.norm();
		const bool settled =
		    have_previous &&
		    error >= previous_error - kSettleEpsilonM &&
		    arrival.tilt_deg >= previous_tilt - kSettleEpsilonDeg;
		previous_error = error;
		previous_tilt = arrival.tilt_deg;
		have_previous = true;

		if (error <= position_tolerance_m &&
		    arrival.tilt_deg <= session.tilt_tolerance_deg && settled) {
			arrival.reached = true;
			return arrival;
		}
		if (std::chrono::steady_clock::now() > deadline) {
			return arrival;
		}
		std::this_thread::sleep_for(
		    std::chrono::milliseconds(session.poll_interval_ms));
	}
}

// База нагрузки в том виде, в каком её видит человек.
std::int32_t contact_baseline(const Session& session) {
	return sum_signal(session.baseline, session.search_options.contact.motors);
}

// Поиск касания: спуск до тех пор, пока бумага не начнёт давить в ответ.
struct ContactSearch {
	bool found{false};
	ContactState state{ContactState::Free};
	// Нагрузка моторов прочитать не удалось: высоту касания уточнить нечем.
	// Это не поломка программы, а отсутствие сигнала: работа продолжается
	// по высоте плоскости, а нажим не проверяется.
	bool load_unavailable{false};
	// Высота цели, на которой поиск остановился.
	double target_z{0.0};
	// Высота цели, на которой касание началось.
	double contact_z{0.0};
	// Измеренная высота TCP в этот момент: это и есть высота бумаги.
	double measured_z{0.0};
	std::int32_t level{0};
	std::int32_t delta{0};
	// Уровень нагрузки на свободной высоте: если он не похож на свободный,
	// то база снята с бумагой, и касание не найдётся никогда.
	std::int32_t level_at_free_height{0};
	int steps{0};
};

// Поиск касания с поверхностью.
//
// Рука поднимается на свободную высоту: там касания заведомо нет, и по
// нагрузке висящей руки снимается база. Дальше цель опускается шаг за
// шагом, и на каждом шаге читается нагрузка: как только бумага начинает
// давить в ответ — касание найдено.
//
// Спуск ограничен глубиной поиска и нагрузкой: выше предела моторы
// перегружены, и поиск прекращается.
//
// Нагрузка нужна только для поиска: если её не прочитать, поиск
// заканчивается как есть (load_unavailable), и высота остаётся той, что
// задал оператор. Ошибкой это не считается.
ContactSearch search_contact(Session& session, const Eigen::Vector3d& target_xy,
                             double start_z, double end_z, double press_depth_m,
                             std::string& error) {
	ContactSearch search;

	// Свободная высота: база нагрузки снимается здесь.
	Eigen::Vector3d free_target = target_xy;
	free_target.z() = start_z;
	if (!solve_and_command(session, free_target, error).sent) return search;
	const Arrival free_arrival =
	    wait_for_arrival(session, free_target, session.position_tolerance_m,
	                      session.reach_timeout_ms);
	if (!free_arrival.reached) {
		error = "the arm did not reach the free height" +
		        describe_motors(session.model, free_arrival.angles, session.degrees,
		                        std::nullopt,
		                        free_arrival.horizontal.norm() * 1000.0,
		                        free_arrival.tilt_deg);
		return search;
	}
	const std::size_t motors = session.search_options.contact.motors;
	const std::optional<TorqueSignal> base =
	    session.arm.read_torque_signals(motors);
	if (!base) {
		// Нагрузка недоступна: высоту касания уточнить нечем, но и повод
		// останавливаться нет — высота плоскости уже задана оператором.
		search.load_unavailable = true;
		return search;
	}
	session.baseline = *base;
	// Уровень на свободной высоте запоминается сразу: по нему видно, что
	// «свободная» высота свободна не была (бумага выше неё), и в этом
	// случае касание не найдётся никогда. Показывать его надо и там, где
	// поиск оборвался на середине, а не только в конце спуска.
	search.level_at_free_height = sum_signal(*base, motors);

	// Шум сигнала: с висящей рукой он всё равно немного дрожит. Касанием это
	// дрожание считаться не должно, поэтому порог поднимается над шумом.
	//
	// Шум меряется здесь же: рука в воздухе — то самое состояние, в котором
	// касания нет.
	TorqueSignal lowest = session.baseline;
	TorqueSignal highest = session.baseline;
	for (int sample = 1; sample < kNoiseSamples; ++sample) {
		std::this_thread::sleep_for(std::chrono::milliseconds(kNoiseSampleDelayMs));
		const std::optional<TorqueSignal> reading =
		    session.arm.read_torque_signals(motors);
		if (!reading) {
			// Нагрузка стала недоступна посреди поиска: как и в начале,
			// высоту уточнить нечем, и это не ошибка.
			search.load_unavailable = true;
			return search;
		}
		for (std::size_t index = 0; index < kServoCount; ++index) {
			lowest[index] = std::min(lowest[index], (*reading)[index]);
			highest[index] = std::max(highest[index], (*reading)[index]);
		}
	}
	std::int32_t noise = 0;
	for (std::size_t index = 0; index < motors && index < kServoCount; ++index) {
		noise += highest[index] - lowest[index];
	}
	session.contact = session.search_options.contact;
	const double threshold =
	    std::ceil(static_cast<double>(noise) *
	              session.search_options.press_noise_factor);
	session.contact.press_min = std::max(
	    session.contact.press_min,
	    static_cast<std::int32_t>(std::min(
	        threshold, static_cast<double>(std::numeric_limits<std::int32_t>::max()))));
	if (session.contact.press_max > 0 &&
	    session.contact.press_max <= session.contact.press_min) {
		// Предел нажима ниже порога касания — порог опускается под него, а
		// не наоборот: предохранитель от перегрузки должен остаться
		// строгим, иначе шум сигнала тихо ослабил бы его.
		session.contact.press_min = session.contact.press_max - 1;
		std::cout << "  порог касания выше предела нажима: опущен до "
		          << session.contact.press_min << "\n"
		          << std::flush;
	}

	// Спуск: цель опускается шаг за шагом, и на каждом шаге читается
	// нагрузка. Касание нашлось — когда бумага начала давить в ответ.
	//
	// Дальше спуск продолжается ещё на press_depth_m ниже касания: мотор
	// не доходит до своей цели, и без этого сдвига карандаш просто лежит
	// на бумаге, не оставляя следа. Глубина спуска, в свою очередь, велика
	// лишь потому, что мёртвая зона мотора измеряется в углах суставов: по
	// карандашу это миллиметры.
	//
	// Шагов ровно столько, сколько помещается в диапазон: шаг считается
	// целым числом шагов, а не вычитанием из числа с плавающей точкой —
	// иначе шаг меньше точности double не сдвинул бы цель вовсе и цикл
	// стал бы бесконечным.
	const double step = std::max(session.search_options.search_step_m,
	                             kMinimumSearchStepM);
	const int steps = static_cast<int>(std::min(
	    std::ceil((start_z - end_z) / step), static_cast<double>(kMaxSearchSteps)));
	for (int index = 0; index < steps; ++index) {
		const double z = start_z - step * static_cast<double>(index + 1);
		Eigen::Vector3d target = target_xy;
		target.z() = z;
		if (!solve_and_command(session, target, error).sent) return search;
		const Arrival arrival = wait_for_arrival(
		    session, target, session.position_tolerance_m, kSearchStepTimeoutMs);
		if (!arrival.reached) {
			error = "the arm did not reach the search height" +
			        describe_motors(session.model, arrival.angles, session.degrees,
			                        std::nullopt,
			                        arrival.horizontal.norm() * 1000.0,
			                        arrival.tilt_deg);
			return search;
		}
		const std::optional<TorqueSignal> signal =
		    session.arm.read_torque_signals(motors);
		if (!signal) {
			// Нагрузка пропала в ходе спуска: дальше идти незачем — касание
			// всё равно не отличить от шума. Высота остаётся той, что задал
			// оператор.
			search.load_unavailable = true;
			return search;
		}
		const ContactReading reading =
		    evaluate_contact(*signal, session.baseline, session.contact);
		++search.steps;
		search.level = reading.level;
		search.delta = reading.delta;
		search.target_z = z;
		search.measured_z = arrival.position.z();

		if (reading.state == ContactState::TooHard) {
			search.state = ContactState::TooHard;
			error = "the press is too strong: the motors are overloaded (load " +
			        std::to_string(reading.level) + ", change from the base " +
			        std::to_string(reading.delta) + ")";
			return search;
		}
		if (reading.state != ContactState::Touching) continue;

		// Касание нашлось. Запоминаем, на какой высоте цели.
		if (!search.found) {
			search.found = true;
			search.state = ContactState::Touching;
			search.contact_z = z;
		}
		// Дальше спускаемся, пока нажим не дойдёт до заданного уровня. Если
		// уровень не задан, спуск ограничен глубиной нажима: без неё мотор
		// не сдвинется (его мёртвая зона), и карандаш просто лежит на
		// бумаге.
		const bool press_reached =
		    session.search_options.contact.press_target > 0 &&
		    signal_magnitude(reading.delta) >=
		        session.search_options.contact.press_target;
		const bool depth_reached =
		    session.search_options.contact.press_target <= 0 &&
		    search.contact_z - z >= press_depth_m;
		if (press_reached || depth_reached) {
			search.target_z = z;
			search.measured_z = arrival.position.z();
			return search;
		}
	}

	search.state = ContactState::Free;
	search.target_z = end_z;
	return search;
}

// Подстройка цели по недобору сервопривода: что получилось.
//
// Шаги — это попытки, а не результат: если подстроечный IK не сошёлся,
// моторам уходит прежняя поза, рука не двигается, и такую точку нельзя
// считать подстроенной. Результат — reached: недобор убран, рука встала в
// точку.
struct Compensation {
	// Сколько шагов подстройки сделано.
	int steps{0};
	// Недобор убран: рука встала в точку.
	bool reached{false};
	// Сбой связи: рисование надо остановить.
	bool failed{false};
};

// Подстройка цели по недобору сервопривода.
//
// Сервопривод всегда останавливается чуть не до цели, и недобор у него
// постоянный. Значит, если сдвинуть цель на измеренный недобор, мотор
// встанет туда, куда надо: второй раз он остановится на то же расстояние до
// цели, но уже с другой стороны.
//
// Ошибка всегда отмеряется от точки окружности, а не от сдвинутой цели: цель
// после подстройки уехала, а карандаш должен встать именно в точку.
//
// Номер точки передаётся в предупреждение о несходящемся IK: подстройка —
// это продолжение той же точки, и её номер должен быть виден.
Compensation compensate_undershoot(Session& session, const Eigen::Vector3d& point,
                                    Eigen::Vector3d& goal,
                                    const DrawOptions& options,
                                    Arrival& arrival, std::string& error,
                                    int point_number = 0) {
	Compensation compensation;
	while (compensation.steps < options.compensate_iterations) {
		if (arrival.horizontal.norm() <= options.compensate_tolerance_m) break;

		// Цель уезжает на измеренный недобор — в ту же сторону, куда рука
		// не доехала.
		goal.x() += arrival.horizontal.x();
		goal.y() += arrival.horizontal.y();
		if (!solve_and_command(session, goal, error, point_number).sent) {
			compensation.failed = true;
			return compensation;
		}

		const Arrival after =
		    wait_for_arrival(session, point, options.compensate_tolerance_m,
		                     options.compensate_timeout_ms);
		if (after.bus_lost) {
			error = "the servo bus was lost while compensating the undershoot";
			compensation.failed = true;
			return compensation;
		}
		arrival = after;
		++compensation.steps;
		// Подстройка удалась, только если рука после неё встала в точку.
		compensation.reached = after.reached;
		if (!after.reached) {
			// Рука не дошла до точки за отведённое время: дальше подстраивать
			// бессмысленно, позиция принимается как есть.
			break;
		}
	}
	return compensation;
}

// Причина, по которой касание не нашлось.
//
// Пустая ошибка означает, что спуск просто не увидел реакции бумаги: шаги
// кончились, а бумага так и не ответила. Молчать об этом нельзя —
// «касание не найдено ()» ничего не объясняет.
std::string contact_search_reason(const ContactSearch& search,
                                  const std::string& error) {
	if (!error.empty()) return error;
	return "бумага не дала реакции за " + std::to_string(search.steps) +
	       " шагов спуска";
}

// Разбор неудачи: что именно не сработало.
std::string diagnose_failure(const Arrival& arrival, double horizontal_mm,
                             int timeout_ms, double position_tolerance_m,
                             double tilt_tolerance_deg) {
	std::ostringstream text;
	if (arrival.bus_lost) {
		text << "\n  the servo bus was lost";
		return text.str();
	}
	if (horizontal_mm > position_tolerance_m) {
		text << "\n  the arm is far from the target: it is stuck or clamped "
		        "(by the paper, the table) — or the motors are relaxed";
		return text.str();
	}
	if (arrival.tilt_deg > tilt_tolerance_deg) {
		text << "\n  the arm is in place, but the tool is tilted by "
		     << std::fixed << std::setprecision(2) << arrival.tilt_deg
		     << " deg: a vertical tool did not come out";
		return text.str();
	}
	text << "\n  the arm is in place, and the target was reached within "
	     << std::fixed << std::setprecision(2) << horizontal_mm
	     << " mm, but the point was not confirmed within " << timeout_ms
	     << " ms";
	return text.str();
}

// Разбор одной точки траектории: сошёлся ли IK и с каким остатком.
struct PointSolution {
	bool solved{false};
	// Остаток по позиции, м, и по наклону инструмента, °.
	double position_error_m{0.0};
	double tilt_deg{0.0};
};

// Разбор траектории по цепочке: каждая точка решается от предыдущей, так
// же, как при рисовании.
//
// Правило обновления seed'а — то же, что и в исполнителе: seed меняется
// только на сошедшемся IK, а на несошедшемся остаётся последним сошедшимся.
// Иначе разбор и рисование считали бы недостижимость по разным правилам:
// --dry-run показывал бы не то, что будет при рисовании, а оператор
// принимал бы решение по неверным числам.
std::vector<PointSolution> solve_chain(ArmModel& model,
                                       const std::vector<Eigen::Vector3d>& targets,
                                       const IkParams& ik_params,
                                       const Eigen::VectorXd& initial_q) {
	std::vector<PointSolution> solutions;
	solutions.reserve(targets.size());
	Eigen::VectorXd q = initial_q;
	for (const Eigen::Vector3d& target : targets) {
		const IkResult ik =
		    solve_ik(model, target, kToolDirectionDown, q, ik_params);
		PointSolution solution;
		solution.solved = ik.converged;
		solution.position_error_m = ik.final_error_m;
		solution.tilt_deg =
		    ik.final_orientation_error_rad * 180.0 / kPiExecution;
		// Seed'ом следующей точки становится только сошедшееся решение:
		// несошедшееся зажато в пределы сустава, и от него следующая точка
		// решалась бы из заведомо неверной позы.
		if (ik.converged) {
			q = ik.q;
		}
		solutions.push_back(solution);
	}
	return solutions;
}

// Описание проблемной точки: номер, остаток по позиции и по наклону.
std::string describe_point_problem(std::size_t index,
                                   const PointSolution& solution) {
	std::ostringstream text;
	text << "точка " << (index + 1) << ": остаток " << std::fixed
	     << std::setprecision(1) << solution.position_error_m * 1000.0
	     << " мм, наклон " << std::setprecision(2) << solution.tilt_deg << "°";
	return text.str();
}

// Разбор траектории целиком: что достижимо, что нет и какие проблемы.
struct TrajectoryAnalysis {
	bool reachable{true};
	int unreachable_points{0};
	double max_position_error_m{0.0};
	double max_tilt_deg{0.0};
	std::vector<std::string> problems;
};

TrajectoryAnalysis analyse_trajectory(ArmModel& model,
                                       const std::vector<Eigen::Vector3d>& targets,
                                       const IkParams& ik_params,
                                       const Eigen::VectorXd& initial_q) {
	TrajectoryAnalysis analysis;
	const std::vector<PointSolution> solutions =
	    solve_chain(model, targets, ik_params, initial_q);
	for (std::size_t index = 0; index < solutions.size(); ++index) {
		const PointSolution& solution = solutions[index];
		analysis.max_position_error_m =
		    std::max(analysis.max_position_error_m, solution.position_error_m);
		analysis.max_tilt_deg =
		    std::max(analysis.max_tilt_deg, solution.tilt_deg);
		if (solution.solved) continue;
		++analysis.unreachable_points;
		analysis.reachable = false;
		analysis.problems.push_back(describe_point_problem(index, solution));
	}
	return analysis;
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
				tilt_line << "✓ принят и будет зафиксирован";
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
		return result;
	}

	// Уточнение высоты бумаги касанием.
	//
	// Человек ставит остриё примерно, и его оценка высоты бумаги — это оценка.
	// Программа может найти поверхность сама: поднять руку, снять базу
	// нагрузки и опустить до тех пор, пока бумага не начнёт давить в ответ.
	if (options.touch_calibrate) {
		// Поиск при захвате ищет поверхность, а не нажим: заданный уровень
		// нажима к нему отношения не имеет, и с ним захват нажимал бы на
		// бумагу сильнее, чем нужно.
		ContactSearchOptions capture_search = options.search;
		capture_search.contact.press_target = 0;
		Session session(arm, model, ik_params, capture_search);
		session.q = angles_to_q(model, result.joint_degrees);
		session.degrees = result.joint_degrees;
		session.tilt_tolerance_deg = options.tilt_tolerance_deg;

		Eigen::Vector3d contact_xy(result.plane.contact_x_m,
		                           result.plane.contact_y_m, 0.0);
		const double start_z = result.plane.tcp_height_m +
		                       options.search.search_clearance_m;
		const double end_z = result.plane.tcp_height_m -
		                     options.search.search_max_depth_m;

		std::string error;
		const ContactSearch search = search_contact(
		    session, contact_xy, start_z, end_z, 0.0, error);
		// Поиск при захвате не давит: он ищет поверхность, а не нажим.
		if (search.load_unavailable) {
			// Нагрузку прочитать не удалось, и уточнить высоту нечем. Это не
			// поломка: высоту задал оператор, и за её правильность отвечает
			// он же — сообщение говорит, что работа пойдёт по этой высоте.
			std::cout << "Высота бумаги касанием не уточнена: сигнал нагрузки "
			             "моторов не читается.\n  работаем по высоте, заданной "
			             "оператором: "
			          << std::fixed << std::setprecision(4)
			          << result.plane.tcp_height_m << " м\n"
			          << std::flush;
		} else if (search.found) {
			result.plane.tcp_height_m = search.measured_z;
			result.touch_calibrated = true;
			std::cout << "Высота бумаги уточнена касанием: "
			          << std::fixed << std::setprecision(4)
			          << result.plane.tcp_height_m << " м (касание на "
			          << std::setprecision(2) << search.delta
			          << " единицы нагрузки)\n"
			          << std::flush;
		} else {
			// Плоскость остаётся той, что записал человек: лучше грубая
			// оценка человека, чем пустое место.
			//
			// Уровень нагрузки на свободной высоте печатается рядом с
			// причиной: если он велик, то «свободная» высота свободна не
			// была (бумага выше неё), и дело не в порогах, а в плоскости.
			std::cout << "Высота бумаги касанием не уточнена: "
			          << contact_search_reason(search, error)
			          << "\n  the load at the free height was "
			          << search.level_at_free_height
			          << "\n  остаётся высота, измеренная по месту касания "
			          << std::fixed << std::setprecision(4)
			          << result.plane.tcp_height_m << " м\n"
			          << std::flush;
		}
	}

	return result;
}

TrajectoryCheck check_trajectory(ArmModel& model,
                                 const std::vector<Eigen::Vector3d>& targets,
                                 const IkParams& ik_params,
                                 const Eigen::VectorXd& initial_q) {
	const TrajectoryAnalysis analysis =
	    analyse_trajectory(model, targets, ik_params, initial_q);
	TrajectoryCheck check;
	check.max_position_error_m = analysis.max_position_error_m;
	check.max_tilt_deg = analysis.max_tilt_deg;
	check.reachable = analysis.reachable;
	check.unreachable_points = analysis.unreachable_points;
	check.problems = analysis.problems;
	return check;
}

TrajectoryPlan plan_trajectory(ArmModel& model,
                               const std::vector<Eigen::Vector3d>& targets,
                               const IkParams& ik_params,
                               const Eigen::VectorXd& initial_q) {
	const TrajectoryAnalysis analysis =
	    analyse_trajectory(model, targets, ik_params, initial_q);
	TrajectoryPlan plan;
	plan.reachable = analysis.reachable;
	plan.unreachable_points = analysis.unreachable_points;
	plan.max_position_error_m = analysis.max_position_error_m;
	plan.max_tilt_deg = analysis.max_tilt_deg;
	plan.problems = analysis.problems;
	return plan;
}

bool draw_trajectory(ArmFeedback& arm, ArmModel& model,
                     const std::vector<Eigen::Vector3d>& targets,
                     const IkParams& ik_params, const DrawOptions& options,
                     DrawReport& report, std::string& error) {
	if (targets.empty()) {
		error = "There are no points to draw";
		return false;
	}

	const std::optional<JointDegrees> current = arm.read_joint_degrees();
	if (!current) {
		error = "Failed to read the current joint angles";
		return false;
	}

	Session session(arm, model, ik_params, options.search);
	session.q = angles_to_q(model, *current);
	session.degrees = *current;
	const Eigen::VectorXd initial_q = session.q;
	session.position_tolerance_m = options.reach_position_tolerance_m;
	session.tilt_tolerance_deg = options.reach_tilt_tolerance_deg;
	session.poll_interval_ms = options.poll_interval_ms;
	session.reach_timeout_ms = options.reach_timeout_ms;

	// Высота плоскости: все точки лежат на ней.
	const double plane_z = targets.front().z();

	// Поиск касания: он выполняется на каждый круг.
	//
	// Бумага плоская, и высота касания от круга к кругу почти не меняется, но
	// поиск всё равно повторяется на каждом круге: рука за круг может уйти
	// (нажим, продавленная бумага), а высота, на которой карандаш касается
	// бумаги, должна быть измерена заново, а не взята из прошлого круга.
	//
	// Рука поднимается на свободную высоту, там снимается база нагрузки, и
	// дальше цель опускается, пока бумага не начнёт давить в ответ. Высота,
	// на которой касание нашлось, уже включает недобор сервопривода: до
	// своей цели он не доходит, поэтому «цель ниже бумаги» означает цель
	// ещё ниже, чем кажется.
	//
	// Нагрузка нужна только для этого и для проверки нажима. Если её нет или
	// касание не нашлось, работа идёт по высоте плоскости (её задал оператор
	// при захвате) минус глубина нажима, а нажим по нагрузке не проверяется:
	// за положение инструмента и за нажим отвечает тот, кто его ставил.
	//
	// Рабочая высота в запасном пути — плоскость минус глубина нажима.
	//
	// Раньше здесь ставилась ровно высота плоскости, а это высота TCP в тот
	// момент, когда остриё стояло на бумаге: то есть кончик на бумаге, без
	// нажима. С учётом мёртвой зоны мотора рука встала бы выше бумаги,
	// карандаш не коснулся бы её, и круг не получился бы вовсе. Без сигнала
	// нагрузки нажим задаётся глубиной: сигнал нагрузки задаёт усилие, а
	// глубина — глубину, и при отсутствии сигнала остаётся то, что есть.
	const double fallback_press_z = plane_z - options.press_depth_m;
	double press_z = fallback_press_z;
	// Читается ли нагрузка вообще. На false переходим, когда сигнала нет:
	// дальше читать его бессмысленно, а нажим всё равно не проверить.
	bool load_readable = options.use_contact;
	// Проверяется ли нажим по нагрузке. Для этого нужно ещё и найденное
	// касание: без него уровень нагрузки не с чем сравнивать, и нажим по
	// нагрузке не проверяется.
	bool load_available = options.use_contact;
	if (options.use_contact) {
		Eigen::Vector3d first = targets.front();
		first.z() = plane_z + options.search.search_clearance_m;
		const double end_z = plane_z - options.search.search_max_depth_m;
		std::string search_error;
		const ContactSearch search = search_contact(
		    session, first, first.z(), end_z, options.press_depth_m, search_error);
		if (search.load_unavailable) {
			// Высоту касания уточнить не удалось: работаем по высоте
			// плоскости минус глубина нажима. Это не ошибка — плоскость уже
			// задана, а нажим задаётся глубиной.
			load_readable = false;
			load_available = false;
			press_z = fallback_press_z;
		} else if (!search.found) {
			// Касание не нашлось: бумага не дала реакции. Это не повод бросать
			// рисование — точки недостижимые просто будут пропущены, а
			// остальные пройдут. Работаем по высоте плоскости минус глубина
			// нажима.
			//
			// Нагрузка при этом читается, но нажим по ней не проверяется:
			// касания нет, а значит нет и уровня, с которым можно сравнивать
			// уровень на точке.
			std::cout << "  ⚠ касание не найдено ("
			          << contact_search_reason(search, search_error)
			          << "): работаем по высоте плоскости минус глубина "
			             "нажима, нажим по нагрузке не проверяется\n"
			          << "    the load at the free height was "
			          << search.level_at_free_height << "\n"
			          << std::flush;
			load_available = false;
			press_z = fallback_press_z;
			report.contact_failed = true;
		} else {
			// Высота цели, на которой карандаш пишет.
			press_z = search.target_z;
			report.contact_threshold = session.contact.press_min;
			std::cout << "  касание найдено за " << search.steps << " шагов: высота "
			          << "цели " << std::fixed << std::setprecision(4)
			          << search.contact_z << " м (TCP " << std::setprecision(4)
			          << search.measured_z << " м), цель для нажима "
			          << std::setprecision(4) << press_z << " м, нагрузка "
			          << search.level << " (база " << contact_baseline(session)
			          << ", Δ " << search.delta << "), порог касания "
			          << session.contact.press_min << "\n"
			          << std::flush;
		}
	}

	// Опорный уровень нажима: по нему оценивается, не оторвался ли карандаш
	// от бумаги на других точках.
	//
	// Берётся максимум из ненулевых измерений, а не первый замер: первый
	// замер мог оказаться нулём (карандаш в этой точке ещё не прижался), и
	// тогда опорного уровня не было бы вовсе, а отрыв от бумаги не
	// определялся бы никогда. При всех нулевых замерах опорного уровня нет,
	// и отрыва нет: нажима не было ни разу, отрываться не от чего.
	std::int32_t reference_level = 0;

	// Измерялся ли нажим хоть раз: без этого флага минимум нажима нельзя
	// отличить от «нажима не было ни разу».
	bool have_press_measurement = report.min_press_level > 0;

	// Траектория проверяется ещё раз — на высоте нажима. Проверка на высоте
	// плоскости не покрывает высоты, на которых рука реально работает.
	{
		std::vector<Eigen::Vector3d> pressed;
		pressed.reserve(targets.size());
		for (const Eigen::Vector3d& target : targets) {
			Eigen::Vector3d pressed_target = target;
			pressed_target.z() = press_z;
			pressed.push_back(pressed_target);
		}
		const TrajectoryCheck press_check =
		    check_trajectory(model, pressed, ik_params, initial_q);
		if (!press_check.reachable) {
			// Недостижимость — не повод отказываться: на таких точках
			// моторам уйдёт лучшее приближение, а в отчёте они посчитаются
			// отдельно. Останавливать круг из-за одного места нельзя.
			std::cout << "  ⚠ на высоте нажима (" << std::fixed
			          << std::setprecision(4) << press_z << " м) недостижимы "
			          << press_check.unreachable_points << " из "
			          << pressed.size() << " точек:\n";
			for (const std::string& problem : press_check.problems) {
				std::cout << "    " << problem << "\n";
			}
			std::cout << std::flush;
		}
	}

	for (std::size_t index = 0; index < targets.size(); ++index) {
		Eigen::Vector3d target = targets[index];
		target.z() = press_z;

		// Несошедшийся IK не останавливает круг: моторам уходит лучшее
		// приближение, а точка попадает в счётчик недостижимых.
		const CommandOutcome outcome = solve_and_command(
		    session, target, error, static_cast<int>(index + 1));
		if (!outcome.sent) return false;
		if (outcome.unreachable) ++report.unreachable_points;

		// Ожидание прибытия. Первая точка может быть далеко: руку только
		// что поставили рукой, и первый ход самый длинный.
		const int timeout =
		    index == 0 ? options.reach_timeout_ms * 3 : options.reach_timeout_ms;
		Arrival arrival =
		    wait_for_arrival(session, target, options.reach_position_tolerance_m,
		                      timeout);
		if (!arrival.reached) {
			if (outcome.unreachable && !arrival.bus_lost) {
				// Точка заведомо недостижима, и рука не дошла даже до
				// приближения. Это дырка в окружности, а не повод бросить
				// круг: точка пропускается, идём к следующей.
				std::cout << "  ⚠ точка " << (index + 1)
				          << ": рука не дошла даже до приближения, точка "
				             "пропущена\n"
				          << std::flush;
				continue;
			}
			const std::optional<JointDegrees> goal = arm.read_goal_degrees();
			const double horizontal_mm = arrival.horizontal.norm() * 1000.0;
			error = "Point " + std::to_string(index + 1) + " of " +
			        std::to_string(targets.size()) + " was not reached in " +
			        std::to_string(timeout) + " ms; the arm is held in place" +
			        describe_motors(model, arrival.angles, session.degrees, goal,
			                        horizontal_mm, arrival.tilt_deg) +
			        diagnose_failure(arrival, horizontal_mm, timeout,
			                         options.reach_position_tolerance_m,
			                         options.reach_tilt_tolerance_deg);
			return false;
		}

		// Касание: бумага должна давить в ответ.
		//
		// Если не давит — значит, остриё не дошло до бумаги: сервопривод
		// остановился выше, чем нужно.
		//
		// Нагрузки может не быть вовсе: она нужна только для проверки нажима,
		// и её отсутствие — не повод встать. Точка считается взятой без
		// нажима, а рисование продолжается.
		ContactReading contact;
		bool have_contact = false;
		if (load_available) {
			const std::optional<TorqueSignal> signal = arm.read_torque_signals(
			    options.search.contact.motors);
			if (signal) {
				contact =
				    evaluate_contact(*signal, session.baseline, session.contact);
				have_contact = true;
			} else {
				// Нагрузка пропала: нажим больше не проверяется. Дальше
				// сигнал читать не будем — каждая неудачная попытка стоит
				// времени, а проверить нажим без него всё равно нечем.
				load_readable = false;
				load_available = false;
			}
		}

		// Касание на других точках оценивается по уровню, а не по изменению
		// относительно базы: база снята в позе первой точки, а вес руки по
		// окружности меняется, и это изменение не отличить от реакции
		// бумаги. Уровень же — прямо то, что показывает нажим.
		const bool have_reference = reference_level > 0;
		const std::int32_t floor_level =
		    have_reference
		        ? static_cast<std::int32_t>(reference_level *
		                                   (1.0 - options.press_drop_fraction))
		        : 0;
		const bool dropped = have_contact && have_reference &&
		                     options.press_drop_fraction > 0.0 &&
		                     contact.level < floor_level;
		if (!have_contact) {
			// Точка без нажима: нажим не проверялся, и проверить его было
			// нечем. В отчёте она считается вместе с отрывами от бумаги.
			++report.contact_drops;
		} else {
			if (dropped) {
				// Уровень упал ниже опорного: карандаш оторвался от бумаги
				// (бумага неровная) или изменился вес руки по окружности.
				//
				// Точка помечается, но рисование не рвётся и нажим не
				// углубляется: и то и другое сделало бы хуже. Поиск касания
				// заново здесь не нужен — он поднял бы руку над бумагой, и в
				// круге появился бы разрыв.
				++report.contact_drops;
			}
			// Опорный уровень — максимум из ненулевых замеров. Обновляется
			// после проверки на отрыв: иначе текущая точка сама была бы
			// опорой для себя, и отрыв не определялся бы никогда.
			if (contact.level > 0) {
				reference_level = std::max(reference_level, contact.level);
			}
			if (contact.state == ContactState::TooHard) {
				error = "The press is too strong at point " +
				        std::to_string(index + 1) +
				        ": the motors are overloaded (load " +
				        std::to_string(contact.level) + ", change from the base " +
				        std::to_string(contact.delta) + ")";
				return false;
			}
		}

		// Подстройка цели по недобору сервопривода: без неё окружность
		// выходит меньше заданной на ширину недобора.
		if (options.compensate) {
			const Compensation compensation = compensate_undershoot(
			    session, targets[index], target, options, arrival, error,
			    static_cast<int>(index + 1));
			if (compensation.failed) return false;
			if (compensation.steps > 0) {
				if (compensation.reached) {
					// Подстройка удалась: рука встала в точку. Такие точки и
					// считаются подстроенными.
					++report.compensated;
					std::cout << "  точка " << std::setw(3) << (index + 1) << "/"
					          << targets.size() << "  недобор подстроен, шагов "
					          << compensation.steps << ", остаток "
					          << std::fixed << std::setprecision(2)
					          << arrival.horizontal.norm() * 1000.0 << " мм\n"
					          << std::flush;
				} else {
					// Подстройка не удалась: рука в точку не встала, и
					// недобор остался. Попытка — не результат, и в счётчик
					// такая точка не идёт.
					std::cout << "  точка " << std::setw(3) << (index + 1) << "/"
					          << targets.size() << "  недобор не убран, шагов "
					          << compensation.steps << ", остаток "
					          << std::fixed << std::setprecision(2)
					          << arrival.horizontal.norm() * 1000.0 << " мм\n"
					          << std::flush;
				}
			}
		}

		// Фактическая точность: где оказался TCP и каков наклон.
		//
		// Ошибка отмеряется от точки окружности, а не от цели: цель после
		// подстройки уехала, а карандаш должен встать в точку.
		const ArmState state = arm_state(model, arrival.angles);
		const double horizontal_mm =
		    horizontal_error(targets[index], state.position).norm() * 1000.0;
		report.max_position_error_m =
		    std::max(report.max_position_error_m, horizontal_mm / 1000.0);
		report.max_tilt_deg = std::max(report.max_tilt_deg, state.tilt_deg);
		// Наименьший уровень нажима: настоящий минимум по точкам, где нажим
		// измерен. Нулевые замеры в минимум не идут: один замер без нажима
		// обнулял бы весь отчёт. Ноль в отчёте означает, что нажима не
		// было ни разу.
		if (have_contact && contact.level > 0) {
			if (!have_press_measurement ||
			    contact.level < report.min_press_level) {
				report.min_press_level = contact.level;
			}
			have_press_measurement = true;
		}
		++report.points_drawn;

		std::cout << "  точка " << std::setw(3) << (index + 1) << "/"
		          << targets.size() << "  ошибка " << std::fixed
		          << std::setprecision(3) << horizontal_mm << " мм"
		          << "  наклон " << std::setprecision(2) << state.tilt_deg << "°";
		if (have_contact) {
			std::cout << "  нагрузка " << contact.level << " (база "
			          << contact.baseline << ", Δ " << contact.delta << ")";
		} else if (!load_readable) {
			// Сигнала нагрузки нет: проверять нажим нечем.
			std::cout << "  нагрузка недоступна, нажим не проверяется";
		} else {
			// Сигнал читается, но касание не найдено: сравнивать уровень
			// не с чем, и нажим по нагрузке не проверяется. Это не то же
			// самое, что «сигнала нет»: сигнал есть, и об этом честнее
			// сказать отдельно.
			std::cout << "  нагрузка доступна, нажим по ней не проверяется: "
			             "касание не найдено";
		}
		std::cout << std::flush;
		const bool touching =
		    have_reference ? contact.level >= floor_level && !dropped
		                   : contact.state == ContactState::Touching;
		// «Пишет» и «не касается» говорить можно только там, где нажим
		// проверялся. Иначе про нажим уже сказано в строке выше, и вторая
		// половина фразы была бы повтором того же самого.
		if (!have_contact) {
			std::cout << std::endl;
		} else if (touching) {
			std::cout << "  пишет" << std::endl;
		} else {
			std::cout << "  НЕ КАСАЕТСЯ БУМАГИ" << std::endl;
		}
	}

	report.completed = true;
	return true;
}

}  // namespace so101
