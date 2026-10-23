#include "serial_arm.hpp"

#include <unistd.h>

#include <algorithm>

#include "control_config.hpp"

namespace so101 {

std::optional<JointDegrees> SerialArmFeedback::read_joint_degrees() {
	JointDegrees degrees{};
	for (std::size_t index = 0; index < kServoCount; ++index) {
		const auto servo_id = static_cast<uint8_t>(index + 1);
		const std::optional<float> angle =
		    read_joint_angle(port_, servo_id, limits_[index]);
		if (!angle) return std::nullopt;
		degrees[index] = static_cast<double>(*angle);
	}
	return degrees;
}

bool SerialArmFeedback::write_joint_degrees(const JointDegrees& degrees) {
	bool succeeded = true;
	for (std::size_t index = 0; index < kServoCount; ++index) {
		const auto servo_id = static_cast<uint8_t>(index + 1);
		const uint16_t speed = joint_speed(
		    base_speed_, joint_speed_divisor_, static_cast<int>(index));
		succeeded = write_joint_angle(port_, servo_id,
		                              static_cast<float>(degrees[index]), speed,
		                              limits_[index], false) &&
		            succeeded;
		// Моторы не успевают переваривать кадры подряд: без паузы часть
		// команд теряется, и рука едет мимо цели.
		usleep(kMotorCommandDelayMicroseconds);
	}
	return succeeded;
}

bool SerialArmFeedback::relax_motors(const JointMask& joints) {
	bool succeeded = true;
	for (std::size_t index = 0; index < kServoCount; ++index) {
		if (!joints[index]) continue;
		const auto servo_id = static_cast<uint8_t>(index + 1);
		succeeded = disable_servo_torque(port_, servo_id) && succeeded;
	}
	return succeeded;
}

bool SerialArmFeedback::hold_motors(const JointMask& joints) {
	bool succeeded = true;
	for (std::size_t index = 0; index < kServoCount; ++index) {
		if (!joints[index]) continue;
		const auto servo_id = static_cast<uint8_t>(index + 1);
		succeeded = enable_servo_torque(port_, servo_id) && succeeded;
	}
	return succeeded;
}

std::optional<JointDegrees> SerialArmFeedback::read_goal_degrees() {
	JointDegrees degrees{};
	for (std::size_t index = 0; index < kServoCount; ++index) {
		const auto servo_id = static_cast<uint8_t>(index + 1);
		const std::optional<float> angle =
		    read_servo_goal_angle(port_, servo_id, limits_[index]);
		if (!angle) return std::nullopt;
		degrees[index] = static_cast<double>(*angle);
	}
	return degrees;
}

std::optional<TorqueSignal> SerialArmFeedback::read_torque_signals(
    std::size_t motors) {
	TorqueSignal signal{};
	const std::size_t count = std::min(motors, kServoCount);
	for (std::size_t index = 0; index < count; ++index) {
		const auto servo_id = static_cast<uint8_t>(index + 1);
		const std::optional<int32_t> value =
		    read_servo_torque_signal(port_, servo_id, torque_register_);
		if (!value) return std::nullopt;
		signal[index] = *value;
	}
	return signal;
}

}  // namespace so101
