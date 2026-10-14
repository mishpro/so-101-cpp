#pragma once

#include <cstdint>
#include <string>

struct ServoLimits {
	uint16_t min_position;
	uint16_t max_position;
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
bool read_servo_limits(int fd, uint8_t servo_id, ServoLimits& limits);
bool load_servo_limits(const std::string& path, ServoLimits limits[6]);
bool save_servo_limits(const std::string& path, const ServoLimits limits[6]);
bool load_or_read_servo_limits(int fd, const std::string& path,
							   ServoLimits limits[6]);
bool read_joint_angle(int fd, uint8_t servo_id, const ServoLimits& limits,
					  float& angle_degrees);
bool disable_servo_torque(int fd, uint8_t servo_id);
bool write_joint_angle(int fd, uint8_t servo_id, float angle_degrees,
					   uint16_t speed, const ServoLimits& limits);
