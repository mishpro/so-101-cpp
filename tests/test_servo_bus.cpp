#include "servo_bus.hpp"
#include "test_support.hpp"

#include <pty.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <poll.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint8_t REG_MIN_POSITION_LIMIT = 0x09;
constexpr uint8_t REG_MAX_POSITION_LIMIT = 0x0B;
constexpr uint8_t REG_PRESENT_POSITION = 0x38;

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

	void start() {
		running_ = true;
		thread_ = std::thread([this] { serve(); });
	}

	void stop() {
		running_ = false;
		if (thread_.joinable()) thread_.join();
	}

	// Обрывает связь, как это делает USB-хаб: master исчезает, и slave-конец
	// начинает возвращать ошибку вместо данных.
	void disconnect() {
		if (master_ >= 0) {
			close(master_);
			master_ = -1;
		}
	}

private:
	void serve() {
		uint8_t request[8];
		while (running_) {
			// poll, а не блокирующий read: поток обязан просыпаться и
			// завершаться по флагу running_, а не висеть на чтении.
			struct pollfd descriptor{master_, POLLIN, 0};
			const int ready = poll(&descriptor, 1, 50);
			if (ready <= 0) continue;

			const ssize_t count = read(master_, request, sizeof(request));
			if (count <= 0) {
				// master закрыт: связь оборвана.
				break;
			}
			if (count != static_cast<ssize_t>(sizeof(request))) {
				continue;
			}
			const uint8_t servo_id = request[2];
			const uint8_t address = request[5];
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
			uint8_t reply[8] = {0xFF, 0xFF, servo_id, 0x00, 0x00,
			                    static_cast<uint8_t>(value & 0xFF),
			                    static_cast<uint8_t>(value >> 8), 0x00};
			reply[7] = checksum(reply, 2, 6);
			(void)!write(master_, reply, sizeof(reply));
		}
	}

	int master_;
	std::vector<uint16_t> minimums_;
	std::vector<uint16_t> maximums_;
	std::vector<uint16_t> present_;
	std::atomic<bool> running_{false};
	std::thread thread_;
};

// Пара портов и путь к slave-устройству.
struct PtyPair {
	int master = -1;
	int slave = -1;
	std::string path;
};

PtyPair open_pair() {
	PtyPair pair;
	char name[256];
	if (openpty(&pair.master, &pair.slave, name, nullptr, nullptr) != 0) {
		return pair;
	}
	pair.path = name;
	return pair;
}

// Чтение границ через эмулятор: обычный рабочий сценарий.
void test_read_limits_over_pty() {
	PtyPair pair = open_pair();
	CHECK(pair.master >= 0);

	const std::vector<uint16_t> minimums = {767, 812, 886, 900, 260, 1420};
	const std::vector<uint16_t> maximums = {3328, 3197, 3107, 3166, 4050, 2620};
	ServoEmulator emulator(pair.master, minimums, maximums);
	emulator.start();

	{
		SerialPort port(pair.path);
		ServoLimits limits[6]{};
		CHECK(read_all_servo_limits(port, limits));
		for (std::size_t index = 0; index < 6; ++index) {
			CHECK(limits[index].min_position == minimums[index]);
			CHECK(limits[index].max_position == maximums[index]);
		}
	}

	emulator.stop();
	close(pair.slave);
	close(pair.master);
}

// Главное: обрыв USB должен распознаваться как обрыв, а не как «мотор не
// ответил». После разрыва pty чтение slave-конца возвращает EIO.
void test_disconnect_is_detected() {
	PtyPair pair = open_pair();
	CHECK(pair.master >= 0);

	const std::vector<uint16_t> minimums = {767, 812, 886, 900, 260, 1420};
	const std::vector<uint16_t> maximums = {3328, 3197, 3107, 3166, 4050, 2620};
	ServoEmulator emulator(pair.master, minimums, maximums);
	emulator.start();

	SerialPort port(pair.path);
	ServoLimits limits[6]{};
	CHECK(read_all_servo_limits(port, limits));

	// Обрыв: master исчезает, и slave-конец начинает отдавать ошибку.
	emulator.disconnect();
	emulator.stop();

	CHECK(!read_all_servo_limits(port, limits));
}

// После обрыва устройство переподключается само, и порт надо открыть заново.
// reopen() обязан вернуть рабочий дескриптор.
void test_reopen_restores_port() {
	PtyPair pair = open_pair();
	CHECK(pair.master >= 0);

	const std::vector<uint16_t> minimums = {767, 812, 886, 900, 260, 1420};
	const std::vector<uint16_t> maximums = {3328, 3197, 3107, 3166, 4050, 2620};
	ServoEmulator emulator(pair.master, minimums, maximums);
	emulator.start();

	SerialPort port(pair.path);
	ServoLimits limits[6]{};
	CHECK(read_all_servo_limits(port, limits));

	const int previous_fd = port.fd();
	CHECK(port.reopen());
	CHECK(port.fd() >= 0);
	// Номер дескриптора после close+open вовсе не обязан измениться — ядро
	// часто отдаёт тот же свободный номер. Проверяем не это, а работоспособность
	// порта: дальше идёт настоящий обмен с эмулятором.
	(void)previous_fd;

	// После переоткрытия порт снова работает.
	ServoLimits reopened[6]{};
	CHECK(read_all_servo_limits(port, reopened));
	for (std::size_t index = 0; index < 6; ++index) {
		CHECK(reopened[index].min_position == minimums[index]);
		CHECK(reopened[index].max_position == maximums[index]);
	}

	emulator.stop();
	close(pair.slave);
	close(pair.master);
}

}  // namespace

int main() {
	test_read_limits_over_pty();
	test_disconnect_is_detected();
	test_reopen_restores_port();
	return so101_test::finish("servo_bus");
}
