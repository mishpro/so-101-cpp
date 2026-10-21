#include "control_config.hpp"
#include "servo_bus.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
constexpr const char* kConfigPath = "send_config.json";
}

int main(int argc, char** argv) {
    // Порт берётся из send_config.json — как у всех остальных программ. Раньше
    // здесь был жёстко вписанный /dev/ttyACM0, из-за чего base_angle молча
    // обращался к настоящей руке там, где вызывающий ожидал эмулятор: путь
    // к устройству не подставлялся ни из конфига, ни из аргументов.
    std::string port;
    try {
        ControlConfig config{};
        if (!load_config(kConfigPath, config)) {
            std::cerr << "Failed to load valid " << kConfigPath << '\n';
            return 1;
        }
        port = config.port;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }

    if (argc > 1) {
        port = argv[1];
    }

    try {
        SerialPort serial_port(port);

        // Границы берём из регистров моторов, а не из файла: base_angle
        // ничего не откалибровывает, он просто читает текущие положения.
        const std::optional<std::array<ServoLimits, kServoCount>> limits =
            read_all_servo_limits(serial_port);
        if (!limits) {
            return 1;
        }

        std::cout << "Querying SO-101 Joint Angles...\n";

        // SO-101 has six daisy-chained Feetech servos with IDs 1 through 6.
        for (std::size_t index = 0; index < kServoCount; ++index) {
            const uint8_t servo_id = static_cast<uint8_t>(index + 1);
            const std::optional<float> angle =
                read_joint_angle(serial_port, servo_id, (*limits)[index]);
            if (angle) {
                std::cout << "Joint " << static_cast<int>(servo_id)
                          << " angle: " << *angle << " degrees\n";
            } else {
                std::cerr << "Joint " << static_cast<int>(servo_id)
                          << " angle read failed\n";
            }
        }
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }

    return 0;
}
