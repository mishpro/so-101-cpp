#include "servo_bus.hpp"

#include <iostream>
#include <unistd.h>

int main(int argc, char** argv) {
    const char* port = argc > 1 ? argv[1] : "/dev/ttyACM0";
    int serial_fd = configure_serial_port(port);
    if (serial_fd < 0) return 1;

    std::cout << "Querying SO-101 Joint Angles..." << std::endl;

    // SO-101 has six daisy-chained Feetech servos with IDs 1 through 6.
    for (uint8_t servo_id = 1; servo_id <= 6; ++servo_id) {
        float angle = read_joint_angle(serial_fd, servo_id);
        if (angle >= 0.0f) {
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

