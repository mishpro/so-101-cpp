#include "servo_bus.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace {
using json = nlohmann::json;

constexpr int kServoCount = 6;
constexpr int kArmJointCount = 5;
constexpr char kArmJointNames[kArmJointCount][14] = {
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
    "wrist_roll"};

struct ControlConfig {
    std::string port;
    std::string urdf_path;
    std::string end_effector;
    uint16_t speed;
    float joint_speed_divisor;
    double cartesian_step_m;
    double ik_damping;
    int ik_max_iterations;
    double ik_position_tolerance_m;
    double ik_max_joint_step_rad;
};

std::string expand_home(const std::string& path) {
    if (path.rfind("~/", 0) != 0) return path;
    const char* home = std::getenv("HOME");
    return home == nullptr ? path : std::string(home) + path.substr(1);
}

bool load_config(const std::string& path, ControlConfig& config) {
    std::ifstream file(path);
    if (!file) return false;

    try {
        const json data = json::parse(file);
        config.port = data.at("port").get<std::string>();
        config.urdf_path = expand_home(data.at("urdf_path").get<std::string>());
        config.end_effector =
            data.at("end_effector").get<std::string>();
        const unsigned int speed = data.at("speed").get<unsigned int>();
        config.speed = static_cast<uint16_t>(speed);
        config.joint_speed_divisor =
            data.at("joint_speed_divisor").get<float>();
        config.cartesian_step_m = data.at("cartesian_step_m").get<double>();
        config.ik_damping = data.at("ik_damping").get<double>();
        config.ik_max_iterations = data.at("ik_max_iterations").get<int>();
        config.ik_position_tolerance_m =
            data.at("ik_position_tolerance_m").get<double>();
        config.ik_max_joint_step_rad =
            data.at("ik_max_joint_step_rad").get<double>();
        return !config.port.empty() && !config.end_effector.empty() &&
               speed >= 1 && speed <= 1000 &&
               std::isfinite(config.joint_speed_divisor) &&
               config.joint_speed_divisor > 0.0f &&
               std::isfinite(config.cartesian_step_m) &&
               config.cartesian_step_m > 0.0 &&
               std::isfinite(config.ik_damping) && config.ik_damping > 0.0 &&
               config.ik_max_iterations >= 1 &&
               std::isfinite(config.ik_position_tolerance_m) &&
               config.ik_position_tolerance_m > 0.0 &&
               std::isfinite(config.ik_max_joint_step_rad) &&
               config.ik_max_joint_step_rad > 0.0;
    } catch (const std::exception&) {
        return false;
    }
}

class TerminalInput {
public:
    TerminalInput() {
        tcgetattr(STDIN_FILENO, &old_termios_);
        new_termios_ = old_termios_;
        new_termios_.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios_);
    }

    ~TerminalInput() { tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_); }

    char getch() {
        char key = 0;
        return read(STDIN_FILENO, &key, 1) > 0 ? key : 0;
    }

private:
    termios old_termios_{};
    termios new_termios_{};
};

bool find_joints(const pinocchio::Model& model,
                 std::vector<pinocchio::JointIndex>& joints) {
    for (const auto& name : kArmJointNames) {
        if (!model.existJointName(name)) {
            std::cerr << "Missing joint in URDF: " << name << std::endl;
            return false;
        }
        joints.push_back(model.getJointId(name));
    }
    return true;
}

bool load_current_configuration(int serial_fd,
                                const ServoLimits limits[kServoCount],
                                const pinocchio::Model& model,
                                const std::vector<pinocchio::JointIndex>& joints,
                                Eigen::VectorXd& q,
                                float& gripper_angle,
                                std::vector<float>& current_angles) {
    // Сервоприводы возвращают положение в градусах относительно сохранённого
    // минимума. Pinocchio, напротив, использует радианы и ограничения URDF.
    // Поэтому сначала нормализуем положение каждого мотора в [0, 1], а затем
    // переносим его в диапазон соответствующего сустава из URDF.
    current_angles.clear();
    for (int servo_id = 1; servo_id <= kServoCount; ++servo_id) {
        float angle_degrees = 0.0f;
        if (!read_joint_angle(serial_fd, servo_id, limits[servo_id - 1],
                              angle_degrees)) {
            return false;
        }
        current_angles.push_back(angle_degrees);

        if (servo_id <= kArmJointCount) {
            const auto joint_id = joints[servo_id - 1];
            // read_joint_angle уже преобразовал отсчёты мотора в градусы.
            // Разность max_position - min_position задаёт полный диапазон
            // сервопривода, а 360 / 4095 переводит отсчёты в градусы.
            const double fraction = angle_degrees /
                                    (static_cast<double>(limits[servo_id - 1].max_position -
                                                         limits[servo_id - 1].min_position) *
                                     (360.0 / 4095.0));
            const int q_index = model.joints[joint_id].idx_q();
            // q хранится в радианах: здесь градусы мотора не переводятся
            // напрямую, а масштабируются в диапазон lower..upper из URDF.
            q[q_index] = model.lowerPositionLimit[q_index] +
                         fraction * (model.upperPositionLimit[q_index] -
                                     model.lowerPositionLimit[q_index]);
        } else {
            // Для gripper IK не выполняется. Его текущее положение сохраняем
            // и отправляем обратно без изменения.
            gripper_angle = angle_degrees;
        }
    }
    return true;
}

std::vector<float> configuration_to_servo_angles(
    const ServoLimits limits[kServoCount], const pinocchio::Model& model,
    const Eigen::VectorXd& q, float gripper_angle,
    const std::vector<pinocchio::JointIndex>& joints) {
    std::vector<float> angles;
    angles.reserve(kServoCount);
    for (size_t index = 0; index < joints.size(); ++index) {
        const int q_index = model.joints[joints[index]].idx_q();
        const double lower = model.lowerPositionLimit[q_index];
        const double upper = model.upperPositionLimit[q_index];
        // После IK q задан в радианах и может выйти за пределы URDF. Сначала
        // переводим его в долю диапазона сустава, затем в градусы сервопривода.
        const double fraction = (q[q_index] - lower) / (upper - lower);
        const float angle_degrees = static_cast<float>(
            std::clamp(fraction, 0.0, 1.0) *
            static_cast<double>(limits[index].max_position -
                                limits[index].min_position) *
            (360.0 / 4095.0));
        angles.push_back(angle_degrees);
    }
    angles.push_back(gripper_angle);
    return angles;
}

bool send_configuration(int serial_fd, const ServoLimits limits[kServoCount],
                        const ControlConfig& config,
                        const std::vector<float>& angles) {
    bool succeeded = true;
    for (size_t index = 0; index < angles.size(); ++index) {
        const uint16_t joint_speed = std::max<uint16_t>(
            1, static_cast<uint16_t>(config.speed /
                                     std::pow(config.joint_speed_divisor,
                                              static_cast<int>(index))));
                    // write_joint_angle самостоятельно переводит градусы в отсчёты,
                    // добавляет минимальное положение и отправляет пакет через ttyACM.
                    succeeded = write_joint_angle(serial_fd, static_cast<uint8_t>(index + 1),
                                      angles[index], joint_speed,
                                      limits[index]) &&
                            succeeded;
    }
    return succeeded;
}

void print_angles(const std::string& label, const std::vector<float>& angles) {
    std::cout << label << " [deg]: [";
    for (size_t index = 0; index < angles.size(); ++index) {
        if (index > 0) std::cout << ", ";
        std::cout << angles[index];
    }
    std::cout << "]\n";
}

void control_cartesian(int serial_fd, const ServoLimits limits[kServoCount],
                       const ControlConfig& config, pinocchio::Model& model,
                       pinocchio::Data& data,
                       const std::vector<pinocchio::JointIndex>& joints) {
    const pinocchio::FrameIndex frame_id =
        model.getFrameId(config.end_effector);
    Eigen::VectorXd q = pinocchio::neutral(model);
    float gripper_angle = 0.0f;
    std::vector<float> current_angles;
    if (!load_current_configuration(serial_fd, limits, model, joints, q,
                                    gripper_angle, current_angles)) {
        throw std::runtime_error("Failed to read the current robot position");
    }

    // Получаем фактическое текущее положение крайней точки после загрузки
    // q из сервоприводов. Именно оно становится начальной целью, поэтому
    // запуск программы не вызывает скачка конечной точки.
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    Eigen::Vector3d target = data.oMf[frame_id].translation();
    pinocchio::Data::Matrix6x jacobian(6, model.nv);

    std::cout << "\nCartesian SO-101 control at "
              << config.end_effector << "\n"
              << "[T/G] X forward/back, [A/D] Y left/right, [W/S] Z up/down\n"
              << "[Q] quit, step: " << config.cartesian_step_m << " m\n"
              << "Target: " << target.transpose() << std::endl;

    TerminalInput terminal;
    while (true) {
        const char key = terminal.getch();
        if (key == 'q' || key == 'Q') break;

        if (key == 't') target.x() += config.cartesian_step_m;
        else if (key == 'g') target.x() -= config.cartesian_step_m;
        else if (key == 'd') target.y() += config.cartesian_step_m;
        else if (key == 'a') target.y() -= config.cartesian_step_m;
        else if (key == 'w') target.z() += config.cartesian_step_m;
        else if (key == 's') target.z() -= config.cartesian_step_m;
        else continue;

        // Перед каждым IK заново читаем фактические углы сервоприводов.
        // Это учитывает запаздывание или неполное достижение предыдущей команды.
        if (!load_current_configuration(serial_fd, limits, model, joints, q,
                                        gripper_angle, current_angles)) {
            throw std::runtime_error("Failed to read current servo angles");
        }

        // target задаётся в мировой декартовой системе координат. По текущему
        // q вычисляем положение gripper и его якобиан, связывающий скорости
        // суставов со скоростью крайней точки.
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        const Eigen::Vector3d current_xyz = data.oMf[frame_id].translation();
        const double initial_error = (target - current_xyz).norm();
        double final_error = initial_error;

        // Несколько итераций Damped Least Squares позволяют приблизить
        // конечную точку к цели точнее, чем один шаг на нажатие клавиши.
        for (int iteration = 0;
             iteration < config.ik_max_iterations &&
             final_error > config.ik_position_tolerance_m;
             ++iteration) {
            pinocchio::computeFrameJacobian(model, data, q, frame_id,
                                            pinocchio::ReferenceFrame::WORLD,
                                            jacobian);
            const Eigen::Vector3d error =
                target - data.oMf[frame_id].translation();
            Eigen::MatrixXd arm_jacobian(3, joints.size());
            for (size_t index = 0; index < joints.size(); ++index) {
                // Берём только линейные строки якобиана и только пять суставов
                // руки; вращение и gripper в расчёте положения не используются.
                arm_jacobian.col(index) = jacobian.topRows(3).col(
                    model.joints[joints[index]].idx_v());
            }

            // Damped Least Squares устойчив к вырожденным положениям
            // якобиана. Величина damping приходит из конфигурации.
            Eigen::Matrix3d system = arm_jacobian * arm_jacobian.transpose();
            system += config.ik_damping * config.ik_damping *
                      Eigen::Matrix3d::Identity();
            const Eigen::VectorXd dq_arm = arm_jacobian.transpose() *
                                           system.ldlt().solve(error);
            Eigen::VectorXd dq = Eigen::VectorXd::Zero(model.nv);
            for (size_t index = 0; index < joints.size(); ++index) {
                const double limited_step = std::clamp(
                    dq_arm[index], -config.ik_max_joint_step_rad,
                    config.ik_max_joint_step_rad);
                dq[model.joints[joints[index]].idx_v()] = limited_step;
            }

            // integrate обновляет конфигурацию Pinocchio. После каждой
            // итерации ограничиваем суставы диапазонами из URDF.
            q = pinocchio::integrate(model, q, dq);
            for (const auto joint_id : joints) {
                const int q_index = model.joints[joint_id].idx_q();
                q[q_index] = std::clamp(
                    q[q_index], model.lowerPositionLimit[q_index],
                    model.upperPositionLimit[q_index]);
            }

            pinocchio::forwardKinematics(model, data, q);
            pinocchio::updateFramePlacements(model, data);
            final_error =
                (target - data.oMf[frame_id].translation()).norm();
        }

        const std::vector<float> target_angles = configuration_to_servo_angles(
            limits, model, q, gripper_angle, joints);
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        const Eigen::Vector3d predicted_xyz =
            data.oMf[frame_id].translation();
        const Eigen::Vector3d ik_error = target - predicted_xyz;

        if (!send_configuration(serial_fd, limits, config, target_angles)) {
            throw std::runtime_error("Failed to send a joint command");
        }

        std::cout << "\n" << std::fixed << std::setprecision(4);
        print_angles("Current angles", current_angles);
        std::cout << "Current XYZ [m]: [" << current_xyz.transpose() << "]\n";
        print_angles("Target angles", target_angles);
        std::cout << "Target XYZ [m]: [" << target.transpose() << "]\n"
                  << "IK error before [m]: " << initial_error << "\n"
                  << "IK error after  [m]: " << ik_error.norm() << std::endl;
    }
}
}

int main() {
    ControlConfig config{};
    if (!load_config("send_config.json", config)) {
        std::cerr << "Failed to load valid send_config.json" << std::endl;
        return 1;
    }

    pinocchio::Model model;
    try {
        // buildModel строит кинематическую модель SO-101 из URDF, включая
        // ограничения суставов и настроенный end-effector frame.
        pinocchio::urdf::buildModel(config.urdf_path, model);
    } catch (const std::exception& error) {
        std::cerr << "Failed to build Pinocchio model: " << error.what()
                  << std::endl;
        return 1;
    }
    if (!model.existFrame(config.end_effector)) {
        std::cerr << "URDF does not contain configured end-effector frame: "
                  << config.end_effector << std::endl;
        return 1;
    }

    std::vector<pinocchio::JointIndex> joints;
    if (!find_joints(model, joints)) return 1;

    const int serial_fd = configure_serial_port(config.port.c_str());
    if (serial_fd < 0) return 1;
    ServoLimits limits[kServoCount]{};
    if (!load_or_read_servo_limits(serial_fd, "servo_limits.json", limits)) {
        close(serial_fd);
        return 1;
    }

    int result = 0;
    try {
        pinocchio::Data data(model);
        control_cartesian(serial_fd, limits, config, model, data, joints);
    } catch (const std::exception& error) {
        std::cerr << "\nCartesian control stopped: " << error.what() << std::endl;
        result = 1;
    }
    close(serial_fd);
    return result;
}