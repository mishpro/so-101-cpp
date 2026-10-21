#pragma once

// Контракт руки для логики рисования.
//
// Логика рисования не знает, что за ней шина: она читает положения
// суставов и отправляет цели. Конкретная реализация поверх SerialPort
// лежит в serial_arm.hpp, заглушка для тестов — в самом тесте.
//
// Зависимость направлена внутрь: исполнитель (рисующий цикл) зависит от
// этого интерфейса, а не от транспорта. Благодаря этому цикл проверяется
// без железа, а симулятор руки подставляется без правок исполнителя.

#include <array>
#include <cstddef>
#include <optional>

#include "servo_bus.hpp"

namespace so101 {

// Углы в градусах от калиброванного нуля мотора — в тех же единицах, что
// и write_joint_angle.
using JointDegrees = std::array<double, kServoCount>;

// Какие моторы расслабить или зафиксировать. Индекс 0 — мотор 1.
using JointMask = std::array<bool, kServoCount>;

class ArmFeedback {
public:
	virtual ~ArmFeedback() = default;

	// Текущие положения всех моторов. Пустой optional — мотор не ответил.
	[[nodiscard]] virtual std::optional<JointDegrees> read_joint_degrees() = 0;

	// Абсолютные цели положений. Команда идемпотентна: повтор той же цели
	// не двигает руку дальше, поэтому её безопасно повторять при сбое связи.
	virtual bool write_joint_degrees(const JointDegrees& degrees) = 0;

	// Расслабить моторы: удержание момента выключается, суставы двигаются
	// рукой. Нужно для ручной установки карандаша.
	virtual bool relax_motors(const JointMask& joints) = 0;

	// Вернуть удержание момента. После ручной установки это обязательно:
	// оставленная расслабленной рука провисает под тяжестью.
	virtual bool hold_motors(const JointMask& joints) = 0;
};

}  // namespace so101
