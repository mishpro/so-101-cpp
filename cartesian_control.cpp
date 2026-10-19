#include "arm_model.hpp"
#include "control_config.hpp"
#include "servo_bus.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace {
using so101::kIkJointCount;
using so101::kServoCount;

// Допуск расхождения границ URDF и калибровки моторов: 0.5°.
constexpr double kJointLimitToleranceRad = 0.008726646259971648;

class TerminalInput {
public:
    TerminalInput() {
        // Если stdin не терминал, настройки не трогаем и не восстанавливаем:
        // иначе в деструктор уйдёт нулевой termios.
        enabled_ = tcgetattr(STDIN_FILENO, &old_termios_) == 0;
        if (!enabled_) return;
        new_termios_ = old_termios_;
        new_termios_.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios_);
    }

    ~TerminalInput() {
        if (enabled_) tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
    }

    TerminalInput(const TerminalInput&) = delete;
    TerminalInput& operator=(const TerminalInput&) = delete;

    // Возвращает 0 при EOF или ошибке чтения — оба случая означают, что
    // управлять дальше нечем.
    char getch() {
        char key = 0;
        return read(STDIN_FILENO, &key, 1) > 0 ? key : 0;
    }

private:
    termios old_termios_{};
    termios new_termios_{};
    bool enabled_{false};
};

// Читает все шесть моторов и переводит их в конфигурацию Pinocchio.
bool read_servo_configuration(const SerialPort& port,
                              const std::array<ServoLimits, kServoCount>& limits,
                              const so101::ArmModel& arm, Eigen::VectorXd& q,
                              std::array<float, kServoCount>& degrees) {
    for (std::size_t index = 0; index < kServoCount; ++index) {
        const int servo_id = static_cast<int>(index) + 1;
        float angle_degrees = 0.0f;
        if (!read_joint_angle(port.fd(), static_cast<uint8_t>(servo_id),
                              limits[index], angle_degrees)) {
            return false;
        }
        degrees[index] = angle_degrees;

        const auto joint = arm.model.joints[arm.servo_joints[index]];
        q[joint.idx_q()] =
            arm.servo_degrees_to_q(servo_id, static_cast<double>(angle_degrees));
    }
    return true;
}

// Углы для отправки: первые четыре сустава берутся из решения IK,
// wrist_roll и gripper остаются там, где их оставил оператор.
std::array<float, kServoCount> configuration_to_servo_angles(
    const so101::ArmModel& arm, const Eigen::VectorXd& q,
    const std::array<float, kServoCount>& degrees) {
    std::array<float, kServoCount> angles = degrees;
    for (std::size_t index = 0; index < kIkJointCount; ++index) {
        const int servo_id = static_cast<int>(index) + 1;
        const auto joint = arm.model.joints[arm.servo_joints[index]];
        angles[index] =
            static_cast<float>(arm.q_to_servo_degrees(servo_id, q[joint.idx_q()]));
    }
    return angles;
}

bool send_configuration(const SerialPort& port,
                        const std::array<ServoLimits, kServoCount>& limits,
                        const ControlConfig& config,
                        const std::array<float, kServoCount>& angles) {
    bool succeeded = true;
    for (std::size_t index = 0; index < angles.size(); ++index) {
        const uint16_t joint_speed = std::max<uint16_t>(
            1, static_cast<uint16_t>(config.speed /
                                     std::pow(config.joint_speed_divisor,
                                              static_cast<int>(index))));
        // write_joint_angle самостоятельно переводит градусы в отсчёты,
        // добавляет минимальное положение и отправляет пакет через ttyACM.
        succeeded = write_joint_angle(port.fd(), static_cast<uint8_t>(index + 1),
                                      angles[index], joint_speed,
                                      limits[index]) &&
                    succeeded;
    }
    return succeeded;
}

void print_angles(const std::string& label,
                  const std::array<float, kServoCount>& angles) {
    std::cout << label << " [deg]: [";
    for (std::size_t index = 0; index < angles.size(); ++index) {
        if (index > 0) std::cout << ", ";
        std::cout << angles[index];
    }
    std::cout << "]\n";
}

so101::IkParams make_ik_params(const ControlConfig& config) {
    so101::IkParams params;
    params.damping = config.ik_damping;
    params.max_iterations = config.ik_max_iterations;
    params.position_tolerance_m = config.ik_position_tolerance_m;
    params.max_joint_step_rad = config.ik_max_joint_step_rad;
    return params;
}

void control_cartesian(const SerialPort& port,
                       const std::array<ServoLimits, kServoCount>& limits,
                       const ControlConfig& config, so101::ArmModel& arm) {
    const so101::IkParams ik_params = make_ik_params(config);

    Eigen::VectorXd q = pinocchio::neutral(arm.model);
    std::array<float, kServoCount> degrees{};
    if (!read_servo_configuration(port, limits, arm, q, degrees)) {
        throw std::runtime_error("Failed to read the current robot position");
    }

    // Получаем фактическое текущее положение крайней точки после загрузки
    // q из сервоприводов. Именно оно становится начальной целью, поэтому
    // запуск программы не вызывает скачка конечной точки.
    pinocchio::forwardKinematics(arm.model, arm.data, q);
    pinocchio::updateFramePlacements(arm.model, arm.data);
    Eigen::Vector3d target = arm.data.oMf[arm.frame_id].translation();

    std::cout << "\nCartesian SO-101 control at " << config.end_effector
              << "\nIK joints: ";
    for (std::size_t index = 0; index < kIkJointCount; ++index) {
        if (index > 0) std::cout << ", ";
        std::cout << arm.model.names[arm.ik_joints[index]];
    }
    std::cout << "\n(wrist_roll и gripper вне IK, управляются через send -j5/-j6)"
              << "\n[T/G] X forward/back, [A/D] Y left/right, [W/S] Z up/down\n"
              << "[Q] quit, step: " << config.cartesian_step_m << " m\n"
              << "Target: " << target.transpose() << std::endl;

    TerminalInput terminal;
    while (true) {
        const char key = terminal.getch();
        if (key == 0 || key == 'q' || key == 'Q') break;

        if (key == 't') target.x() += config.cartesian_step_m;
        else if (key == 'g') target.x() -= config.cartesian_step_m;
        else if (key == 'd') target.y() += config.cartesian_step_m;
        else if (key == 'a') target.y() -= config.cartesian_step_m;
        else if (key == 'w') target.z() += config.cartesian_step_m;
        else if (key == 's') target.z() -= config.cartesian_step_m;
        else continue;

        // Перед каждым IK заново читаем фактические углы сервоприводов.
        // Это учитывает запаздывание или неполное достижение предыдущей команды.
        if (!read_servo_configuration(port, limits, arm, q, degrees)) {
            throw std::runtime_error("Failed to read current servo angles");
        }

        pinocchio::forwardKinematics(arm.model, arm.data, q);
        pinocchio::updateFramePlacements(arm.model, arm.data);
        const Eigen::Vector3d current_xyz =
            arm.data.oMf[arm.frame_id].translation();

        const so101::IkResult ik = so101::solve_ik(arm, target, q, ik_params);
        q = ik.q;

        if (!ik.converged) {
            std::cerr << "\nIK did not reach the target in "
                      << ik.iterations << " iterations, residual "
                      << ik.final_error_m
                      << " m; the arm is most likely at a joint limit."
                      << std::endl;
        }

        const auto target_angles =
            configuration_to_servo_angles(arm, q, degrees);
        pinocchio::forwardKinematics(arm.model, arm.data, q);
        pinocchio::updateFramePlacements(arm.model, arm.data);
        const Eigen::Vector3d ik_error =
            target - arm.data.oMf[arm.frame_id].translation();

        if (!send_configuration(port, limits, config, target_angles)) {
            throw std::runtime_error("Failed to send a joint command");
        }

        std::cout << "\n" << std::fixed << std::setprecision(4);
        print_angles("Current angles", degrees);
        std::cout << "Current XYZ [m]: [" << current_xyz.transpose() << "]\n";
        print_angles("Target angles", target_angles);
        std::cout << "Target XYZ [m]: [" << target.transpose() << "]\n"
                  << "IK error before [m]: " << ik.initial_error_m << "\n"
                  << "IK error after  [m]: " << ik_error.norm() << std::endl;
    }
}
}  // namespace

int main() {
    try {
        ControlConfig config{};
        if (!load_config("send_config.json", config)) {
            std::cerr << "Failed to load valid send_config.json" << std::endl;
            return 1;
        }

        so101::ArmModel arm{};
        std::string error;
        if (!so101::load_arm_model(config.urdf_path, config.end_effector, arm,
                                   error)) {
            std::cerr << error << std::endl;
            return 1;
        }

        SerialPort port(config.port);

        // Границы протокола берём из регистров самих сервоприводов: файл
        // калибровки в проекте нет, а диапазон мотора хранится в нём самом.
        // В URDF те же значения переносит tools/sync_urdf_limits.py.
        std::array<ServoLimits, kServoCount> limits{};
        if (!read_all_servo_limits(port.fd(), limits.data())) {
            std::cerr << "Failed to read joint limits from the servos"
                      << std::endl;
            return 1;
        }

        std::ostringstream registers;
        registers << std::fixed << std::setprecision(2)
                  << "servo registers [deg]: ";
        for (std::size_t index = 0; index < kServoCount; ++index) {
            registers << limits[index].span_degrees() << (index + 1 < kServoCount ? " / " : "");
        }
        std::cout << registers.str() << std::endl;

        // URDF — источник истины для углов. Расхождение с регистрами означает,
        // что моторы перенастроены относительно модели: работаем дальше, но
        // говорим об этом.
        if (!so101::check_joint_limits(arm, limits.data(),
                                       kJointLimitToleranceRad, error)) {
            std::cerr << "\nWarning: servo registers disagree with the URDF: "
                      << error
                      << "\n         angles are taken from the URDF; if the arm "
                         "behaves unexpectedly, re-run "
                         "tools/sync_urdf_limits.py"
                      << std::endl;
        }

        try {
            control_cartesian(port, limits, config, arm);
        } catch (const std::exception& exception) {
            std::cerr << "\nCartesian control stopped: " << exception.what()
                      << std::endl;
            return 1;
        }
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << std::endl;
        return 1;
    }
    return 0;
}