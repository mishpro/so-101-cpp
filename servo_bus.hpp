#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
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

// Окно ожидания возврата устройства после обрыва USB.
//
// Адаптер переподключается сам, и узел /dev/ttyACM* появляется не мгновенно.
// Окно в 60 секунд выбрано по измеренным разрывам: наблюдались окна в 54 и
// 13 секунд, поэтому и пять, и тридцать секунд проваливались бы на первом
// же из них. Правится одним числом.
inline constexpr int kDeviceWaitSeconds = 60;

// Пауза между попытками переоткрыть порт.
inline constexpr int kReopenDelayMicroseconds = 200000;

// Сколько раз повторить запрос молчащего сервопривода, прежде чем признать
// его не отвечающим. Порт при этом не трогаем: шина жива, молчит мотор.
inline constexpr int kSilentReadAttempts = 2;

// SO-101: шесть сервоприводов с номерами 1..6. Живёт здесь, а не в arm_model:
// это свойство шины, и транспортному слою оно нужно не меньше, чем модели.
inline constexpr std::size_t kServoCount = 6;

// Адреса регистров STS3215.
//
// enum class, а не набор констант: адрес не должен молча сложиться с целым
// числом, а в протокол попадает как есть, поэтому единственное место, где тип
// превращается обратно в байт, — граница обмена с железом.
enum class Register : uint8_t {
	min_position_limit = 0x09,
	max_position_limit = 0x0B,
	torque_enable = 0x28,
	goal_position = 0x2A,
	goal_velocity = 0x2E,
	present_position = 0x38,
};

// Границы протокола сервопривода: абсолютные отсчёты регистров 0x09 и 0x0B,
// при которых мотор считает себя нулём градусов и концом хода.
struct ServoLimits {
	uint16_t min_position{0};
	uint16_t max_position{0};

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

// Сбой связи с шиной сервоприводов. Отдельный тип, а не std::runtime_error:
// вызывающий код отличает «порт не открылся» от прочих ошибок и может
// показать пользователю осмысленное сообщение.
class ServoPortError : public std::runtime_error {
public:
	// Уналедованный конструктор std::runtime_error не был бы explicit, и
	// ошибка молча превратилась бы в любой строковый литерал.
	explicit ServoPortError(const std::string& what)
	    : std::runtime_error(what) {}
};

// Владеет файловым дескриптором tty-порта и закрывает его в деструкторе.
// Конструктор бросает ServoPortError, если порт не открылся.
//
// Объект умеет переоткрывать порт: адаптер иногда теряет связь с хабом
// («disabled by hub (EMI?)» в dmesg), после чего устройство переподключается
// само — старый дескриптор при этом умирает, а путь на короткое время
// исчезает. Без reopen() программа падала бы на первом же чтении после
// такого обрыва, и сообщение об ошибке не отличалось бы от «мотор не ответил».
class SerialPort {
public:
	// device_wait_seconds — сколько ждать возврата адаптера после обрыва.
	// По умолчанию kDeviceWaitSeconds; тестам задают меньше, чтобы не ждать
	// минуту там, где устройство заведомо не вернётся.
	explicit SerialPort(const std::string& port,
	                    int device_wait_seconds = kDeviceWaitSeconds);
	~SerialPort() noexcept;

	SerialPort(const SerialPort&) = delete;
	SerialPort& operator=(const SerialPort&) = delete;
	SerialPort(SerialPort&& other) noexcept;
	SerialPort& operator=(SerialPort&& other) noexcept;

	[[nodiscard]] int fd() const noexcept { return fd_; }
	[[nodiscard]] const std::string& path() const noexcept { return port_; }
	[[nodiscard]] int device_wait_seconds() const noexcept {
		return device_wait_seconds_;
	}

	// Переоткрывает порт после обрыва USB: закрывает текущий дескриптор и
	// открывает путь заново. Одной попыткой, без ожидания: сколько ждать
	// возврата устройства, решает вызывающий код.
	bool reopen();

private:
	std::string port_;
	int fd_{-1};
	int device_wait_seconds_{kDeviceWaitSeconds};
};

int configure_serial_port(const char* port);

// Жив ли порт: на месте ли устройство за дескриптором.
//
// Когда адаптер исчезает, дескриптор оказывается повешенным, и ioctl на нём
// возвращает EIO; на живом порту отвечает. Нужно, чтобы отличить «устройство
// пропало» от «сервопривод молчит»: в первом случае программа должна дождаться
// возврата адаптера, во втором — ждать бесполезно.
[[nodiscard]] bool is_port_alive(int fd);

// Читает границы одного мотора прямо из его регистров. Пустой optional —
// мотор не ответил.
[[nodiscard]] std::optional<ServoLimits> read_servo_limits(SerialPort& port,
                                                           uint8_t servo_id);

// Читает границы всех шести моторов прямо из регистров. Это единственный
// источник границ протокола: в URDF те же значения переносятся генератором
// tools/sync_urdf_limits.py, который читает регистры напрямую.
[[nodiscard]] std::optional<std::array<ServoLimits, kServoCount>>
read_all_servo_limits(SerialPort& port);

// Текущий угол мотора в градусах от откалиброванного нуля. Пустой optional —
// мотор не ответил либо стоит вне своих границ.
[[nodiscard]] std::optional<float> read_joint_angle(SerialPort& port,
                                                     uint8_t servo_id,
                                                     const ServoLimits& limits);

bool disable_servo_torque(SerialPort& port, uint8_t servo_id);
bool write_joint_angle(SerialPort& port, uint8_t servo_id, float angle_degrees,
					   uint16_t speed, const ServoLimits& limits);
