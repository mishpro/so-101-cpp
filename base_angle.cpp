#include "servo_bus.hpp"

#include <iostream>
#include <string>
#include <unistd.h>

int main(int argc, char** argv) {
    const char* port = argc > 1 ? argv[1] : "/dev/ttyACM0";
    int serial_fd = configure_serial_port(port);
    if (serial_fd < 0) return 1;

    ServoLimits limits[6]{};
    if (!load_or_read_servo_limits(serial_fd, "servo_limits.json", limits)) {
        close(serial_fd);
        return 1;
    }

    std::cout << "Querying SO-101 Joint Angles..." << std::endl;

    // SO-101 has six daisy-chained Feetech servos with IDs 1 through 6.
    for (uint8_t servo_id = 1; servo_id <= 6; ++servo_id) {
        float angle = 0.0f;
        if (read_joint_angle(serial_fd, servo_id, limits[servo_id - 1], angle)) {
            std::cout << "Joint " << static_cast<int>(servo_id)
                      << " angle: " << angle << " degrees" << std::endl;
        } else {
            std::cerr << "Joint " << static_cast<int>(servo_id)
                      << " angle read failed" << std::endl;
        }
    }

    close(serial_fd);
    return 0;
}

