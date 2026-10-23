#include "surface_contact.hpp"

#include <algorithm>
#include <limits>

namespace so101 {

std::int32_t signal_magnitude(std::int32_t value) {
	return value < 0 ? (value == std::numeric_limits<std::int32_t>::min()
	                       ? std::numeric_limits<std::int32_t>::max()
	                       : -value)
	                 : value;
}

std::int32_t sum_signal(const TorqueSignal& signal, std::size_t motors) {
	const std::size_t count = std::min(motors, signal.size());
	std::int32_t sum = 0;
	for (std::size_t index = 0; index < count; ++index) {
		// Модули, а не сами значения: при нажатии одни суставы упираются в
		// одну сторону, другие — в другую, и сумма знаковых токов взаимно
		// уничтожается. Нагрузка без знака от этого не меняется.
		sum += signal_magnitude(signal[index]);
	}
	return sum;
}

ContactReading evaluate_contact(const TorqueSignal& signal,
                                 const TorqueSignal& baseline,
                                 const ContactOptions& options) {
	ContactReading reading;
	reading.level = sum_signal(signal, options.motors);
	reading.baseline = sum_signal(baseline, options.motors);
	reading.delta = reading.level - reading.baseline;

	// Знак реакции бумаги зависит от того, в какую сторону упирается мотор,
	// поэтому касание ищется по модулю изменения. Нагрузка без знака (0x3E)
	// по модулю не изменилась бы — она и не меняет знак.
	const std::int32_t change = signal_magnitude(reading.delta);

	// Предел нагрузки — предохранитель: при слишком сильном нажиме моторы
	// перегружены, и это надо остановить, а не рисовать. Предел не задан
	// (ноль) — предохранителя нет.
	if (options.press_max > 0 && change >= options.press_max) {
		reading.state = ContactState::TooHard;
		return reading;
	}
	// Строгое сравнение: нулевое изменение — это не касание, а шум, иначе
	// порог, равный нулю, выдавал бы за касание саму базу.
	if (change > options.press_min) {
		reading.state = ContactState::Touching;
		return reading;
	}
	reading.state = ContactState::Free;
	return reading;
}

}  // namespace so101
