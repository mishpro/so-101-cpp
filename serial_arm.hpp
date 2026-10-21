#pragma once

// Реализация контракта ArmFeedback поверх шины сервоприводов.
//
// Единственное место в проекте, где «логика рисования» встречается с
// SerialPort. Всё остальное работает через контракт, поэтому цикл
// рисования проверяется без железа.

#include <array>
#include <cstdint>

#include "arm_feedback.hpp"
#include "servo_bus.hpp"

namespace so101 {

class SerialArmFeedback final : public ArmFeedback {
public:
	// base_speed и joint_speed_divisor задают скорость мотора: скорость
	// сустава N равна base_speed / divisor^(N-1), то есть дальние суставы
	// движутся медленнее.
	SerialArmFeedback(SerialPort& port,
	                  const std::array<ServoLimits, kServoCount>& limits,
	                  uint16_t base_speed, double joint_speed_divisor)
	    : port_(port),
	      limits_(limits),
	      base_speed_(base_speed),
	      joint_speed_divisor_(joint_speed_divisor) {}

	[[nodiscard]] std::optional<JointDegrees> read_joint_degrees() override;
	bool write_joint_degrees(const JointDegrees& degrees) override;
	bool relax_motors(const JointMask& joints) override;
	bool hold_motors(const JointMask& joints) override;

private:
	SerialPort& port_;
	std::array<ServoLimits, kServoCount> limits_;
	uint16_t base_speed_;
	double joint_speed_divisor_;
};

}  // namespace so101
