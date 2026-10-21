#pragma once

// Как рисуем: захват плоскости и обход траектории.
//
// Модуль работает через контракт ArmFeedback и не знает, что за ним шина.
// Всё, что касается железа, приходит снаружи: углы читаются, цели
// отправляются, клавиатура опрашивается через колбэк.

#include <Eigen/Dense>

#include <functional>
#include <string>
#include <vector>

#include "arm_feedback.hpp"
#include "arm_model.hpp"
#include "drawing.hpp"

namespace so101 {

// Направление, в котором «смотрит» карандаш: ось TCP в мире смотрит вниз.
// Z-ось TCP направлена от основания кисти к острию, поэтому вертикальный
// карандаш — это именно (0, 0, -1).
inline const Eigen::Vector3d kToolDirectionDown{0.0, 0.0, -1.0};

// Возвращает символ, если клавиатура готова его отдать (0 — нажатия ещё
// нет), либо -1, если ввод закрыт: оператора за клавиатурой нет и ждать
// бессмысленно.
using KeyReader = std::function<int()>;

struct CaptureOptions {
	// Допустимый наклон инструмента при касании. Погрешность высоты от
	// наклона пропорциональна L(1 - cos θ): при 3° это около 0.07 мм.
	double tilt_tolerance_deg{3.0};
	// Пауза между кадрами живого экрана.
	int refresh_ms{80};
};

struct CaptureResult {
	Plane plane;
	JointDegrees joint_degrees{};
	// Захват состоялся: наклон в допуске и рука взята моторами.
	bool accepted{false};
	// Причина отказа, если захват не состоялся.
	std::string error;
};

// Живой захват плоскости.
//
// Программа показывает текущие углы, требуемые углы (решение IK с
// вертикальным инструментом) и наклон оси. Человек рукой ставит остриё на
// поверхность и подкручивает кисть, пока наклон не станет допустимым; после
// Enter программа записывает текущую позу целью и возвращает моторам
// удержание — рука остаётся ровно там, где её оставил человек.
//
// Моторы на время захвата расслабляются: с удержанием их не сдвинуть
// (мотор держит позицию с моментом около 11 кг·см против 2 кг·см веса руки).
[[nodiscard]] CaptureResult capture_plane(ArmFeedback& arm, ArmModel& model,
                                           const IkParams& ik_params,
                                           const CaptureOptions& options,
                                           const KeyReader& read_key);

struct DrawOptions {
	// Допуск достижения точки по углам суставов, в градусах.
	double reach_tolerance_deg{0.5};
	// Сколько ждать достижения точки.
	int reach_timeout_ms{5000};
	// Пауза между опросами положения.
	int poll_interval_ms{20};
};

struct DrawReport {
	std::size_t points_drawn{0};
	double max_position_error_m{0.0};
	double max_tilt_deg{0.0};
	int timeouts{0};
	bool completed{false};
};

// Проверка траектории без движения: решается IK по всем целям, как при
// рисовании, но ничего не отправляется. Нужна до первого движения: если
// какая-то точка недостижима, руку двигать нельзя.
//
// Проверка идёт от начальной конфигурации — той же, от которой пойдёт
// рисование: нельзя объявить траекторию недостижимой, если она не
// достижима из позы, в которой рука стоит сейчас.
struct TrajectoryCheck {
	double max_position_error_m{0.0};
	double max_tilt_deg{0.0};
	bool reachable{false};
};

[[nodiscard]] TrajectoryCheck check_trajectory(
    ArmModel& model, const std::vector<Eigen::Vector3d>& targets,
    const IkParams& ik_params, const Eigen::VectorXd& initial_q);

// Обход траектории: на каждую цель решается IK с вертикальным
// инструментом, цели отправляются, и программа ждёт фактического
// достижения. Возврат false означает, что рисование остановлено.
[[nodiscard]] bool draw_trajectory(ArmFeedback& arm, ArmModel& model,
                                   const std::vector<Eigen::Vector3d>& targets,
                                   const IkParams& ik_params,
                                   const DrawOptions& options,
                                   DrawReport& report, std::string& error);

}  // namespace so101
