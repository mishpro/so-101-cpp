#pragma once

#include <cstdint>
#include <string>

// Feetech STS3215 считает угол как steps * 360 / 4095 — именно так, а не через
// 4096. Константа одна на весь проект, иначе URDF разойдётся с прошивкой.
// Именно double: во float мантисса обрезается и сверка границ с URDF набирает
// погрешность порядка 1e-5 градуса.
inline constexpr double kDegreesPerStep = 360.0 / 4095.0;
inline constexpr double kStepsPerDegree = 4095.0 / 360.0;
// Отсчёты на радиан: нужно там, где диапазон приходит из URDF в радианах и
// его нужно пересчитать в то количество шагов, которое описал бы такой мотор.
inline constexpr double kStepsPerRadian = 4095.0 / (2.0 * 3.14159265358979323846);

// Бодовая скорость шины сервоприводов: 1 Мбит/с, фиксирована в SerialPort.
//
// Не путать с полем speed в send_config.json и с ключом -s у send: это скорость
// МОТОРА (регистр GOAL_VELOCITY), а не скорость передачи. Любой внешний инструмент,
// читающий регистры напрямую — например tools/sync_urdf_limits.py, — обязан
// открывать порт на этой скорости. На 600 бод, взятых из конфига по ошибке,
// сервоприводы не отвечают вовсе.
inline constexpr unsigned kServoBusBaud = 1000000;

// Границы протокола сервопривода: абсолютные отсчёты регистров 0x09 и 0x0B,
// при которых мотор считает себя нулём градусов и концом хода.
struct ServoLimits {
	uint16_t min_position;
	uint16_t max_position;

	// Полный ход мотора в градусах. Считается в double: эта величина
	// сравнивается с диапазонами URDF, где нужна точность выше, чем даёт float.
	[[nodiscard]] constexpr double span_degrees() const {
		return static_cast<double>(max_position - min_position) *
		       static_cast<double>(kDegreesPerStep);
	}

	// Градусы мотора для абсолютного отсчёта. Отсчёты вне границ дают
	// отрицательные или избыточные градусы — это ловит вызывающий код.
	[[nodiscard]] constexpr double position_to_degrees(uint16_t position) const {
		return static_cast<double>(position - min_position) *
		       static_cast<double>(kDegreesPerStep);
	}

	// Абсолютный отсчёт для угла мотора.
	[[nodiscard]] constexpr uint16_t degrees_to_position(double degrees) const {
		return static_cast<uint16_t>(
		    static_cast<double>(min_position) +
		    degrees * static_cast<double>(kStepsPerDegree));
	}
};

// Владеет файловым дескриптором tty-порта и закрывает его в деструкторе.
// Конструктор бросает std::runtime_error, если порт не открылся.
class SerialPort {
public:
	explicit SerialPort(const std::string& port);
	~SerialPort() noexcept;

	SerialPort(const SerialPort&) = delete;
	SerialPort& operator=(const SerialPort&) = delete;
	SerialPort(SerialPort&& other) noexcept;
	SerialPort& operator=(SerialPort&& other) noexcept;

	int fd() const noexcept { return fd_; }

private:
	int fd_;
};

int configure_serial_port(const char* port);

// Границы протокола сервопривода: абсолютные отсчёты регистров 0x09 и 0x0B,
// при которых мотор считает себя нулём градусов и концом хода.

// Читает границы одного мотора прямо из его регистров.
bool read_servo_limits(int fd, uint8_t servo_id, ServoLimits& limits);

// Читает границы всех шести моторов прямо из регистров. Это единственный
// источник границ протокола: в URDF те же значения переносятся генератором
// tools/sync_urdf_limits.py, который читает регистры напрямую.
bool read_all_servo_limits(int fd, ServoLimits limits[6]);

bool read_joint_angle(int fd, uint8_t servo_id, const ServoLimits& limits,
					  float& angle_degrees);
bool disable_servo_torque(int fd, uint8_t servo_id);
bool write_joint_angle(int fd, uint8_t servo_id, float angle_degrees,
					   uint16_t speed, const ServoLimits& limits);
