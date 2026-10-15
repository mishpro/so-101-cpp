#include "servo_bus.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <unistd.h>

int main(int argc, char** argv) {
    const std::string port = argc > 1 ? argv[1] : "/dev/ttyACM0";

    try {
        SerialPort serial_port(port);

        // Границы берём из регистров моторов, а не из файла: base_angle
        // ничего не откалибровывает, он просто читает текущие положения.
        ServoLimits limits[6]{};
        if (!read_all_servo_limits(serial_port.fd(), limits)) {
            return 1;
        }

        std::cout << "Querying SO-101 Joint Angles..." << std::endl;

        // SO-101 has six daisy-chained Feetech servos with IDs 1 through 6.
        for (uint8_t servo_id = 1; servo_id <= 6; ++servo_id) {
            float angle = 0.0f;
            if (read_joint_angle(serial_port.fd(), servo_id,
                                 limits[servo_id - 1], angle)) {
                std::cout << "Joint " << static_cast<int>(servo_id)
                          << " angle: " << angle << " degrees" << std::endl;
            } else {
                std::cerr << "Joint " << static_cast<int>(servo_id)
                          << " angle read failed" << std::endl;
            }
        }
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << std::endl;
        return 1;
    }

    return 0;
}

