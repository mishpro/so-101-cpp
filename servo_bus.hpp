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
//
// Это числовое значение, для программы. Термиос хочет закодированную
// константу B1000000, и перевод один на другой — не дело этого заголовка.
inline constexpr unsigned kServoBusBaud = 1000000;

// Окно ожидания появления устройства, если узла порта нет.
//
// Адаптер переподключается сам, и узел /dev/ttyACM* появляется не мгновенно:
// после подключения — через ~300 мс, после обрыва — по измеренным разрывам
// самый длинный наблюдавшийся составил 13 секунд. Десяти секунд хватает на
// оба случая. Правится одним числом.
inline constexpr int kDeviceWaitSeconds = 10;

// Пауза между попытками переоткрыть порт.
inline constexpr int kReopenDelayMicroseconds = 200000;

// SO-101: шесть сервоприводов с номерами 1..6. Живёт здесь, а не в arm_model:
// это свойство шины, и транспортному слою оно нужно не меньше, чем модели.
inline constexpr std::size_t kServoCount = 6;

// Сигнал нагрузки по каждому мотору, в единицах того регистра, из которого
// он прочитан (ток или нагрузка на валу). Живёт рядом с числом моторов по
// той же причине: это свойство шины, а не модели.
//
// Знак у тока значащий: он говорит, в какую сторону мотор упирается.
using TorqueSignal = std::array<std::int32_t, kServoCount>;

// Пауза между командами разным моторам.
//
// STS3215 теряет кадры, если следующий приходит, пока предыдущий не
// переварен: запись в порт при этом удаётся, а мотор команду не выполняет.
// Поэтому команды моторам разводятся паузой.
inline constexpr int kMotorCommandDelayMicroseconds = 2000;

// Сколько раз повторяется команда удержания, если мотор не подтвердил
// результат чтением своего регистра.
inline constexpr int kTorqueCommandAttempts = 5;

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
	present_speed = 0x3A,
	// Два регистра с сигналом нагрузки: 0x3C и 0x3E. В документации STS3215
	// один назван «ток», другой — «нагрузка на валу», и в разных редакциях
	// они названы по-разному. Поэтому оба читаются одинаково: как знаковое
	// 16-битное число, откуда берётся модуль. Так результат верен при любом
	// раскладе, а какой регистр что означает — выясняется замером (--probe).
	present_torque_3c = 0x3C,
	present_torque_3e = 0x3E,
};

// Регистр, из которого по умолчанию берётся сигнал нагрузки на мотор.
//
// По умолчанию — 0x3E. Если замер покажет, что нужный сигнал лежит в 0x3C,
// регистр меняется ключом --load-source.
inline constexpr Register kTorqueSignalRegister = Register::present_torque_3e;

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
// Объект умеет переоткрывать порт. Это нужно не только при обрыве USB:
// первый open сразу после подачи питания роботу не получает ответа от
// сервоприводов, а следующий open — получает. Повтор запроса на том же
// дескрипторе не помогает, помогает только свежий open. Поэтому любая
// неудача чтения лечится переоткрытием.
class SerialPort {
public:
	// device_wait_seconds — сколько ждать связи с шиной. Окно используется в
	// двух местах: появление устройства, если узла порта нет, и восстановление
	// связи, когда молчит вся шина. По умолчанию kDeviceWaitSeconds; тестам
	// задают меньше, чтобы не ждать там, где устройство заведомо не появится.
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

	// Переоткрывает порт: закрывает текущий дескриптор и открывает путь
	// заново. Если узла нет (адаптер отвалился), ждёт его появления до
	// device_wait_seconds. false — устройство не появилось либо открыть не
	// удалось; причина уже выведена.
	bool reopen();

private:
	// Итог открытия порта.
	enum class OpenResult {
		Opened,        // порт открыт
		DeviceMissing, // узла нет: адаптер не подключён
		Failed,        // прочая ошибка: права, занятость порта
	};

	// Открывает порт, а если узла нет — ждёт появления устройства.
	OpenResult open_waiting(bool reconnecting);

	std::string port_;
	int fd_{-1};
	int device_wait_seconds_{kDeviceWaitSeconds};
	int last_error_{0};
};

int configure_serial_port(const char* port);

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

// Цель мотора: положение, в которое его **отправили**, в градусах от
// калиброванного нуля. Пустой optional — мотор не ответил.
//
// Это не то же самое, что текущий угол: разница между ними показывает, доехал
// мотор до цели или нет, и потому отличает «мотор получил команду и стоит на
// ней» от «мотор команду не получил». По этой же разнице видно, что мотор
// упирается: он не может закрыть разрыв, потому что упирается в бумагу.
[[nodiscard]] std::optional<float> read_servo_goal_angle(SerialPort& port,
                                                          uint8_t servo_id,
                                                          const ServoLimits& limits);

// Сырое значение любого регистра. Для разведки регистров на живой руке:
// пока неизвестно, что именно лежит в 0x3C и в 0x3E, это выясняется
// чтением, а не догадкой. Пустой optional — мотор не ответил.
[[nodiscard]] std::optional<uint16_t> read_raw_register(SerialPort& port,
                                                         uint8_t servo_id,
                                                         Register address);

// Сигнал нагрузки на мотор: значение регистра address в его собственном
// масштабе. Пустой optional — мотор не ответил.
//
// Значение всегда знаковое 16-битное: ток упирающегося мотора отрицателен,
// и беззнаковое прочтение превратило бы его в 65500. Нагрузка без знака от
// этого не меняется: её значения невелики. Единицы измерения определяются
// замером на живой руке, поэтому наружу выдаётся сырое значение, а не
// «ньютоны».
[[nodiscard]] std::optional<int32_t> read_servo_torque_signal(SerialPort& port,
                                                               uint8_t servo_id,
                                                               Register address);

// Расслабляет мотор: удержание момента выключается, сустав двигается рукой.
//
// Нужно для ручной установки карандаша: с включённым удержанием мотор
// держит позицию с моментом около 11 кг·см, а вес руки даёт на плече
// примерно 2 кг·см — сдвинуть руку руками нельзя, сервоприводы победят.
// Обратная операция — enable_servo_torque.
//
// Команда проверяется чтением регистра: шина теряет кадры, и об этом не
// узнать никак, кроме как спросить у мотора. Мотор, который не отпустил
// удержание, пробуют снова.
bool disable_servo_torque(SerialPort& port, uint8_t servo_id);

// Включает удержание момента на моторе. После ручной установки карандаша
// программа обязана вернуть моторы в работу: оставленная расслабленной рука
// провисает под тяжестью. Как и выключение, проверяется чтением регистра.
bool enable_servo_torque(SerialPort& port, uint8_t servo_id);

// Состояние удержания мотора: true — держит позицию, false — свободен.
// Пустой optional — мотор не ответил.
[[nodiscard]] std::optional<bool> read_servo_torque_enabled(SerialPort& port,
                                                            uint8_t servo_id);

// Отправляет абсолютное положение мотора. announce печатает команду в
// stdout: при плотном цикле (рисование круга) это лишний шум, и вызывающий
// печатает своё.
bool write_joint_angle(SerialPort& port, uint8_t servo_id, float angle_degrees,
					   uint16_t speed, const ServoLimits& limits,
					   bool announce = true);
