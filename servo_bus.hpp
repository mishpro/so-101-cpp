#pragma once

#include <cstdint>

int configure_serial_port(const char* port);
float read_joint_angle(int fd, uint8_t servo_id);
bool write_joint_angle(int fd, uint8_t servo_id, float angle_degrees,
					   uint16_t speed);
