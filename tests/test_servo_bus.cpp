#include "servo_bus.hpp"
#include "test_support.hpp"

#include <pty.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <array>
#include <chrono>
#include <cstdio>
#include <optional>
#include <cstdlib>
#include <poll.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint8_t REG_MIN_POSITION_LIMIT = 0x09;
constexpr uint8_t REG_MAX_POSITION_LIMIT = 0x0B;
constexpr uint8_t REG_PRESENT_POSITION = 0x38;

// Длина кадра в байтах — как в настоящем протоколе STS3215.
constexpr std::size_t kFrameSize = 8;

uint8_t checksum(const uint8_t* packet, int first, int last) {
	uint32_t sum = 0;
	for (int index = first; index <= last; ++index) {
		sum += packet[index];
	}
	return static_cast<uint8_t>(~sum);
}

// Эмулятор шести моторов на втором конце псевдо-порта: отвечает на запросы
// чтения регистров ровно так, как это делает настоящий сервопривод.
class ServoEmulator {
public:
	ServoEmulator(int master, const std::vector<uint16_t>& minimums,
	              const std::vector<uint16_t>& maximums)
	    : master_(master), minimums_(minimums), maximums_(maximums),
	      present_(minimums.size(), 0) {}

	~ServoEmulator() { stop(); }

	ServoEmulator(const ServoEmulator&) = delete;
	ServoEmulator& operator=(const ServoEmulator&) = delete;
	ServoEmulator(ServoEmulator&&) = delete;
	ServoEmulator& operator=(ServoEmulator&&) = delete;

	void start() {
		running_ = true;
		thread_ = std::thread([this] { serve(); });
	}

	void stop() {
		running_ = false;
		if (thread_.joinable()) thread_.join();
	}

	// Код ошибки в ответе: настоящий мотор ставит его, когда запрос
	// не может быть выполнен, и параметры тогда бессмысленны.
	void set_error_code(uint8_t code) { error_code_ = code; }

	[[nodiscard]] int served() const { return served_.load(); }

	// Мотор перестаёт отвечать, но шина остаётся живой.
	void mute(uint8_t servo_id) { muted_ = servo_id; }

	// Обрывает связь, как это делает USB-хаб: master исчезает, и slave-конец
	// начинает отдавать вместо данных пустоту.
	//
	// Сначала поток: пока он жив, он висит в read() на master, и закрытие
	// дескриптора из чужого потока — гонка. После join() pending read не
	// остаётся, и slave получает честный EOF, а не EIO.
	void disconnect() {
		stop();
		if (master_ >= 0) {
			close(master_);
			master_ = -1;
		}
	}

private:
	void serve() {
		std::array<uint8_t, kFrameSize> request{};
		while (running_) {
			// poll, а не блокирующий read: поток обязан просыпаться и
			// завершаться по флагу running_, а не висеть на чтении.
			struct pollfd descriptor{master_, POLLIN, 0};
			const int ready = poll(&descriptor, 1, 50);
			if (ready <= 0) continue;

			const ssize_t count = read(master_, request.data(), request.size());
			if (count <= 0) {
				// master закрыт: связь оборвана.
				break;
			}
			if (count != static_cast<ssize_t>(request.size())) {
				continue;
			}
			++served_;
			const uint8_t servo_id = request[2];
			const uint8_t address = request[5];
			if (servo_id == muted_) {
				continue;  // молчим: на запрос не отвечаем вовсе
			}
			uint16_t value = 0;
			if (servo_id >= 1 && servo_id <= 6) {
				const std::size_t index = servo_id - 1;
				if (address == REG_MIN_POSITION_LIMIT) {
					value = minimums_[index];
				} else if (address == REG_MAX_POSITION_LIMIT) {
					value = maximums_[index];
				} else if (address == REG_PRESENT_POSITION) {
					value = present_[index];
				}
			}
			std::array<uint8_t, kFrameSize> reply{
				0xFF, 0xFF, servo_id, 0x00, error_code_,
				static_cast<uint8_t>(value & 0xFF),
				static_cast<uint8_t>(value >> 8), 0x00};
			reply[kFrameSize - 1] = checksum(reply.data(), 2, 6);
			(void)!write(master_, reply.data(), reply.size());
		}
	}

	int master_;
	std::vector<uint16_t> minimums_;
	std::vector<uint16_t> maximums_;
	std::vector<uint16_t> present_;
	uint8_t error_code_{0};
	uint8_t muted_{0};
	std::atomic<int> served_{0};
	std::atomic<bool> running_{false};
	std::thread thread_;
};

// Пара портов и путь к slave-устройству. Дескрипторы принадлежат объекту:
// при исключении из теста закрывать их вручную не придётся.
struct PtyPair {
	explicit PtyPair(int master_fd, int slave_fd, std::string slave_path)
	    : master(master_fd), slave(slave_fd), path(std::move(slave_path)) {}

	~PtyPair() {
		if (master >= 0) close(master);
		if (slave >= 0) close(slave);
	}

	PtyPair(const PtyPair&) = delete;
	PtyPair& operator=(const PtyPair&) = delete;
	PtyPair(PtyPair&&) = delete;
	PtyPair& operator=(PtyPair&&) = delete;

	int master = -1;
	int slave = -1;
	std::string path;
};

// Открывает пару портов. Пустой путь означает, что openpty не удался, и
// вызывающий обязан это учесть: конструировать SerialPort с пустым путём
// бессмысленно.
PtyPair open_pair() {
	int master = -1;
	int slave = -1;
	std::array<char, 256> name{};
	if (openpty(&master, &slave, name.data(), nullptr, nullptr) != 0) {
		return PtyPair(-1, -1, "");
	}
	return PtyPair(master, slave, name.data());
}

// Перехватывает stderr на время вызова, чтобы прочитать сообщения
// программы. Восстанавливает поток и возвращает собранный текст.
class CapturedStderr {
public:
	CapturedStderr() {
		saved_ = dup(STDERR_FILENO);
		capture_ = std::tmpfile();
		if (capture_ != nullptr) {
			fflush(stderr);
			dup2(fileno(capture_), STDERR_FILENO);
		}
	}

	~CapturedStderr() { restore(); }

	std::string restore() {
		if (capture_ == nullptr) return {};
		fflush(stderr);
		dup2(saved_, STDERR_FILENO);
		close(saved_);
		std::rewind(capture_);
		std::array<char, 4096> buffer{};
		const size_t read_bytes = std::fread(buffer.data(), 1, buffer.size(), capture_);
		std::fclose(capture_);
		capture_ = nullptr;
		return std::string(buffer.data(), read_bytes);
	}

private:
	int saved_{-1};
	std::FILE* capture_{nullptr};
};

// Чтение границ через эмулятор: обычный рабочий сценарий.
void test_read_limits_over_pty() {
	PtyPair pair = open_pair();
	if (pair.master < 0) {
		CHECK_MSG(false, "openpty failed: no pseudo-terminal available");
		return;
	}

	const std::vector<uint16_t> minimums = {767, 812, 886, 900, 260, 1420};
	const std::vector<uint16_t> maximums = {3328, 3197, 3107, 3166, 4050, 2620};
	ServoEmulator emulator(pair.master, minimums, maximums);
	emulator.start();

	{
		SerialPort port(pair.path);
		const std::optional<std::array<ServoLimits, kServoCount>> limits =
		    read_all_servo_limits(port);
		CHECK(limits.has_value());
		for (std::size_t index = 0; index < kServoCount; ++index) {
			CHECK((*limits)[index].min_position == minimums[index]);
			CHECK((*limits)[index].max_position == maximums[index]);
		}
	}

	emulator.stop();
}

// Главное: обрыв USB должен распознаваться как обрыв, а не как «мотор не
// ответил». После разрыва pty чтение slave-конца возвращает EIO.
void test_disconnect_is_detected() {
	PtyPair pair = open_pair();
	if (pair.master < 0) {
		CHECK_MSG(false, "openpty failed: no pseudo-terminal available");
		return;
	}

	const std::vector<uint16_t> minimums = {767, 812, 886, 900, 260, 1420};
	const std::vector<uint16_t> maximums = {3328, 3197, 3107, 3166, 4050, 2620};
	ServoEmulator emulator(pair.master, minimums, maximums);
	emulator.start();

	SerialPort port(pair.path, 2);
	const std::optional<std::array<ServoLimits, kServoCount>> limits =
	    read_all_servo_limits(port);
	CHECK(limits.has_value());

	// Обрыв: master исчезает, и slave-конец начинает отдавать ошибку.
	emulator.disconnect();
	emulator.stop();

	CHECK(!read_all_servo_limits(port).has_value());
}

// После обрыва устройство переподключается само, и порт надо открыть заново.
// reopen() обязан вернуть рабочий дескриптор.
void test_reopen_restores_port() {
	PtyPair pair = open_pair();
	if (pair.master < 0) {
		CHECK_MSG(false, "openpty failed: no pseudo-terminal available");
		return;
	}

	const std::vector<uint16_t> minimums = {767, 812, 886, 900, 260, 1420};
	const std::vector<uint16_t> maximums = {3328, 3197, 3107, 3166, 4050, 2620};
	ServoEmulator emulator(pair.master, minimums, maximums);
	emulator.start();

	SerialPort port(pair.path);
	const std::optional<std::array<ServoLimits, kServoCount>> limits =
	    read_all_servo_limits(port);
	CHECK(limits.has_value());

	const int previous_fd = port.fd();
	CHECK(port.reopen());
	CHECK(port.fd() >= 0);
	// Номер дескриптора после close+open вовсе не обязан измениться — ядро
	// часто отдаёт тот же свободный номер. Проверяем не это, а работоспособность
	// порта: дальше идёт настоящий обмен с эмулятором.
	(void)previous_fd;

	// После переоткрытия порт снова работает.
	const std::optional<std::array<ServoLimits, kServoCount>> reopened =
	    read_all_servo_limits(port);
	CHECK(reopened.has_value());
	for (std::size_t index = 0; index < kServoCount; ++index) {
		CHECK((*reopened)[index].min_position == minimums[index]);
		CHECK((*reopened)[index].max_position == maximums[index]);
	}

	emulator.stop();
}

// Мотор сообщил об ошибке: параметры бессмысленны, и принять их за границы
// нельзя. Раньше код ошибки печатался, но значение всё равно принималось —
// и мусор уходил в min_position и max_position.
void test_servo_error_is_not_accepted() {
	PtyPair pair = open_pair();
	if (pair.master < 0) {
		CHECK_MSG(false, "openpty failed: no pseudo-terminal available");
		return;
	}

	const std::vector<uint16_t> minimums = {767, 812, 886, 900, 260, 1420};
	const std::vector<uint16_t> maximums = {3328, 3197, 3107, 3166, 4050, 2620};
	ServoEmulator emulator(pair.master, minimums, maximums);
	emulator.set_error_code(0x01);
	emulator.start();

	SerialPort port(pair.path);
	CHECK(!read_all_servo_limits(port).has_value());

	emulator.stop();
}


// Различение «устройство пропало» и «мотор молчит» держится на том, что
// повешенный порт не отвечает на ioctl. Это и есть признак, по которому
// программа решает: ждать возврата адаптера или сдаваться.
void test_port_alive_distinguishes_disconnect() {
	PtyPair pair = open_pair();
	if (pair.master < 0) {
		CHECK_MSG(false, "openpty failed: no pseudo-terminal available");
		return;
	}

	SerialPort port(pair.path);
	CHECK(is_port_alive(port.fd()));

	// Master исчезает — дескриптор slave оказывается повешенным.
	close(pair.master);
	pair.master = -1;
	CHECK(!is_port_alive(port.fd()));
}

// Молчащий сервопривод при живой шине.
//
// Это не обрыв: шина работает, молчит один мотор. Порт в этом случае
// переоткрывать нельзя — иначе нормальная работа с остальными моторами
// сбрасывалась бы из-за одного неисправного. И ждать тут нечего: мотор
// не вернётся сам, сколько бы мы ни ждали адаптер.
void test_silent_motor_does_not_reopen_port() {
	PtyPair pair = open_pair();
	if (pair.master < 0) {
		CHECK_MSG(false, "openpty failed: no pseudo-terminal available");
		return;
	}

	const std::vector<uint16_t> minimums = {767, 812, 886, 900, 260, 1420};
	const std::vector<uint16_t> maximums = {3328, 3197, 3107, 3166, 4050, 2620};
	ServoEmulator emulator(pair.master, minimums, maximums);
	emulator.mute(1);
	emulator.start();

	SerialPort port(pair.path);
	CapturedStderr captured;
	const auto started = std::chrono::steady_clock::now();
	const auto limits = read_all_servo_limits(port);
	const double elapsed_ms = std::chrono::duration<double, std::milli>(
	                              std::chrono::steady_clock::now() - started)
	                              .count();
	const std::string output = captured.restore();

	CHECK(!limits.has_value());
	CHECK(output.find("did not answer") != std::string::npos);
	CHECK(output.find("disconnected") == std::string::npos);
	// Ожидание адаптера длиной в минуту здесь означало бы, что молчание
	// мотора принято за обрыв.
	CHECK(elapsed_ms < 5000.0);

	emulator.stop();
}

// Адаптер отваливается и возвращается.
//
// Настоящий переподключённый адаптер получает тот же путь /dev/ttyACM*, но
// другое устройство за ним. Это и моделируется: символическая ссылка на
// псевдо-порт переводится на новую пару, пока старая мертва.
void test_recovers_after_adapter_returns() {
	PtyPair first = open_pair();
	if (first.master < 0) {
		CHECK_MSG(false, "openpty failed: no pseudo-terminal available");
		return;
	}

	const std::vector<uint16_t> minimums = {767, 812, 886, 900, 260, 1420};
	const std::vector<uint16_t> maximums = {3328, 3197, 3107, 3166, 4050, 2620};

	// Путь, который видит программа, — символическая ссылка.
	const std::string link_path = "/tmp/so101_test_bus_link";
	std::remove(link_path.c_str());
	CHECK(symlink(first.path.c_str(), link_path.c_str()) == 0);

	ServoEmulator first_emulator(first.master, minimums, maximums);
	first_emulator.start();

	SerialPort port(link_path);
	std::optional<std::array<ServoLimits, kServoCount>> limits;

	// Обрыв: master исчезает, за узлом ничего не остаётся.
	first_emulator.disconnect();
	first_emulator.stop();

	// Возврат адаптера: другая пара портов и перевод ссылки на неё.
	PtyPair second = open_pair();
	if (second.master < 0) {
		CHECK_MSG(false, "openpty failed: no pseudo-terminal available");
		std::remove(link_path.c_str());
		return;
	}
	ServoEmulator second_emulator(second.master, minimums, maximums);
	second_emulator.start();
	std::remove(link_path.c_str());
	CHECK(symlink(second.path.c_str(), link_path.c_str()) == 0);

	// Программа должна дождаться адаптера и прочитать границы.
	limits = read_all_servo_limits(port);
	CHECK_MSG(limits.has_value(),
	          "the program must recover after the adapter returns");
	if (limits) {
		for (std::size_t index = 0; index < kServoCount; ++index) {
			CHECK((*limits)[index].min_position == minimums[index]);
			CHECK((*limits)[index].max_position == maximums[index]);
		}
	}

	std::remove(link_path.c_str());
	second_emulator.stop();
}

}  // namespace

int main() {
	test_read_limits_over_pty();
	test_servo_error_is_not_accepted();
	test_port_alive_distinguishes_disconnect();
	test_silent_motor_does_not_reopen_port();
	test_recovers_after_adapter_returns();
	test_disconnect_is_detected();
	test_reopen_restores_port();
	return so101_test::finish("servo_bus");
}
