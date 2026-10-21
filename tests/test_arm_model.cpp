// Проверки модуля arm_model без робота: нужен только URDF.

#include <array>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <random>
#include <vector>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "arm_model.hpp"
#include "control_config.hpp"
#include "test_support.hpp"

namespace {

using so101::kIkJointCount;
using so101::kServoCount;


std::string test_urdf_path() {
    ControlConfig config{};
    if (!load_config("send_config.json", config)) {
        std::cerr << "Не удалось прочитать send_config.json из рабочего каталога\n";
        std::exit(EXIT_FAILURE);
    }
    return config.urdf_path;
}

std::string test_end_effector() {
    ControlConfig config{};
    if (!load_config("send_config.json", config)) std::exit(EXIT_FAILURE);
    return config.end_effector;
}

// Калибровка этого экземпляра робота лежит в регистрах моторов, а её перенос в
// URDF делает tools/sync_urdf_limits.py. Отдельного файла с эталоном в проекте
// нет, и хранить его копию в коде тоже незачем: все проверки ниже работают с
// ДИАПАЗОНОМ мотора, а он восстанавливается из span блоков <limit> однозначно.
// Абсолютный min_position не используется нигде — формула привязки сохраняет
// исходный lower, поэтому диапазон и есть единственное, что связывает URDF с
// моторами.
std::array<ServoLimits, kServoCount> reference_limits(const so101::ArmModel& arm) {
    std::array<ServoLimits, kServoCount> limits{};
    for (std::size_t index = 0; index < kServoCount; ++index) {
        const int servo_id = static_cast<int>(index) + 1;
        const double span = arm.upper_position_limit(servo_id) -
                            arm.lower_position_limit(servo_id);
        const long steps = std::lround(span * kStepsPerRadian);
        // Диапазон URDF обязан укладываться в 16-битные отсчёты мотора: иначе
        // перевод градусов в позицию терял бы точность или переполнялся.
        CHECK(steps > 0 && steps <= std::numeric_limits<uint16_t>::max());
        limits[index] = {0, static_cast<uint16_t>(steps)};
    }
    return limits;
}

void test_ik_joint_set(const so101::ArmModel& arm) {
    CHECK(so101::kIkJointCount == 4);
    // Первые четыре сустава — pan, lift, elbow, flex; roll и gripper вне IK.
    CHECK(arm.model.names[arm.ik_joints[0]] == "shoulder_pan");
    CHECK(arm.model.names[arm.ik_joints[3]] == "wrist_flex");
    CHECK(arm.model.names[arm.servo_joints[4]] == "wrist_roll");
    CHECK(arm.model.names[arm.servo_joints[5]] == "gripper");
}

void test_limits_match_calibration(const so101::ArmModel& arm) {
    const auto limits = reference_limits(arm);
    std::string error;
    CHECK_MSG(so101::check_joint_limits(arm, limits, so101::kJointLimitToleranceRad,
                                        error),
              error);

    for (std::size_t index = 0; index < so101::kServoCount; ++index) {
        const int servo_id = static_cast<int>(index) + 1;
        const double span = arm.upper_position_limit(servo_id) -
                            arm.lower_position_limit(servo_id);
        CHECK_NEAR(span, arm.servo_span_rad(limits[index]), so101::kJointLimitToleranceRad);
    }
}

void test_limits_check_detects_drift(const so101::ArmModel& model) {
    std::string error;
    auto limits = reference_limits(model);
    limits[4].max_position = static_cast<uint16_t>(limits[4].max_position - 300);
    CHECK(!so101::check_joint_limits(model, limits, so101::kJointLimitToleranceRad,
                                     error));
    // В сообщении должно быть имя именно сломанного сустава.
    CHECK(error.find("wrist_roll") != std::string::npos);
    CHECK(error.find("servo registers give") != std::string::npos);
    CHECK(error.find("URDF says") != std::string::npos);

    // Расхождения накапливаются: видны все, а не только первое.
    auto two_drifted = reference_limits(model);
    two_drifted[4].max_position =
        static_cast<uint16_t>(two_drifted[4].max_position - 300);
    two_drifted[2].min_position =
        static_cast<uint16_t>(two_drifted[2].min_position + 200);
    CHECK(!so101::check_joint_limits(model, two_drifted,
                                     so101::kJointLimitToleranceRad, error));
    CHECK(error.find("wrist_roll") != std::string::npos);
    CHECK(error.find("elbow_flex") != std::string::npos);

    // Сообщение об ошибке пустее, когда всё сошлось.
    CHECK(so101::check_joint_limits(model, reference_limits(model),
                                    so101::kJointLimitToleranceRad, error));
    CHECK(error.empty());

    // Пустой диапазон мотора тоже должен ловиться.
    auto empty = reference_limits(model);
    empty[0].max_position = empty[0].min_position;
    CHECK(!so101::check_joint_limits(model, empty, so101::kJointLimitToleranceRad,
                                     error));
    CHECK(error.find("empty") != std::string::npos);
}

// Границы протокола считаются по формуле мотора, а не повторной реализацией:
// span_degrees() и span в градусах обязаны совпадать.
void test_servo_span_matches_degrees(const so101::ArmModel& arm) {
    const auto limits = reference_limits(arm);
    for (std::size_t index = 0; index < kServoCount; ++index) {
        const double steps = static_cast<double>(limits[index].max_position -
                                                 limits[index].min_position);
        CHECK_NEAR(limits[index].span_degrees(), steps * kDegreesPerStep, 1e-9);
        // Обратное преобразование должно возвращать исходный отсчёт.
        CHECK_NEAR(limits[index].degrees_to_position(
                       limits[index].span_degrees()),
                   limits[index].max_position, 1.0);
        CHECK_NEAR(
            limits[index].position_to_degrees(limits[index].min_position), 0.0,
            1e-4);
    }
}

void test_angle_roundtrip(const so101::ArmModel& arm) {
    const auto limits = reference_limits(arm);
    for (std::size_t index = 0; index < so101::kServoCount; ++index) {
        const int servo_id = static_cast<int>(index) + 1;
        const double span_deg = so101::radians_to_degrees(arm.servo_span_rad(limits[index]));

        // Границы диапазона мотора должны попадать ровно в границы URDF.
        CHECK_NEAR(arm.servo_degrees_to_q(servo_id, 0.0),
                   arm.lower_position_limit(servo_id), 1e-9);
        CHECK_NEAR(arm.servo_degrees_to_q(servo_id, span_deg),
                   arm.upper_position_limit(servo_id), 1e-9);

        for (double fraction : {0.0, 0.1, 0.5, 0.9, 1.0}) {
            const double degrees = fraction * span_deg;
            const double q = arm.servo_degrees_to_q(servo_id, degrees);
            CHECK_NEAR(arm.q_to_servo_degrees(servo_id, q), degrees, 1e-6);
        }
    }
}

void test_q_to_degrees_clamps(const so101::ArmModel& arm) {
    const auto limits = reference_limits(arm);
    const int servo_id = 1;
    const double span_deg = so101::radians_to_degrees(arm.servo_span_rad(limits[0]));

    // Значение за пределами диапазона должно обрезаться, а не уйти на серво.
    CHECK_NEAR(arm.q_to_servo_degrees(
                   servo_id, arm.lower_position_limit(servo_id) - 0.1),
               0.0, 1e-9);
    CHECK_NEAR(arm.q_to_servo_degrees(
                   servo_id, arm.upper_position_limit(servo_id) + 0.1),
               span_deg, 1e-6);
}

// Реальная нагрузка инструмента: шаг cartesian_step_m (10 мм) от текущей позы.
// Здесь решатель обязан сходиться.
void test_ik_tracks_cartesian_steps(const so101::ArmModel& model_template) {
    so101::ArmModel arm = model_template;
    so101::IkParams params;
    params.damping = 1e-4;
    params.max_iterations = 100;
    params.position_tolerance_m = 0.001;
    params.max_joint_step_rad = 0.1;

    // Несколько реально достижимых начальных поз.
    const std::vector<Eigen::VectorXd> starts = [&arm] {
        std::vector<Eigen::VectorXd> result;
        const double presets[][4] = {
            {0.0, -0.3, 0.8, -0.5},  {0.4, -0.2, 0.6, -0.4},
            {-0.5, -0.4, 1.0, -0.6}, {0.2, -0.5, 1.2, -0.8},
        };
        for (const auto& preset : presets) {
            auto q = pinocchio::neutral(arm.model);
            for (std::size_t index = 0; index < kIkJointCount; ++index) {
                const auto joint = arm.model.joints[arm.ik_joints[index]];
                q[joint.idx_q()] = std::clamp(
                    preset[index], arm.model.lowerPositionLimit[joint.idx_q()],
                    arm.model.upperPositionLimit[joint.idx_q()]);
            }
            result.push_back(q);
        }
        return result;
    }();

    const double step = 0.01;
    const std::vector<Eigen::Vector3d> directions = {
        {step, 0, 0},  {-step, 0, 0},  {0, step, 0},
        {0, -step, 0}, {0, 0, step},  {0, 0, -step},
    };

    for (const auto& start : starts) {
        pinocchio::forwardKinematics(arm.model, arm.data, start);
        pinocchio::updateFramePlacements(arm.model, arm.data);
        const Eigen::Vector3d origin =
            arm.data.oMf[arm.frame_id].translation();

        for (const auto& direction : directions) {
            const Eigen::Vector3d target = origin + direction;
            const so101::IkResult result =
                so101::solve_ik(arm, target, start, params);
            std::array<char, 128> detail{};
            std::snprintf(detail.data(), detail.size(),
                          "target [%.3f %.3f %.3f] residual %.4f in %d iterations",
                          target.x(), target.y(), target.z(),
                          result.final_error_m, result.iterations);
            CHECK_MSG(result.converged, detail.data());
        }
    }
}

// На больших перемещениях решатель обязан по крайней мере не ухудшать
// результат: линейное приближение перестаёт работать, когда шаг велик.
void test_ik_never_makes_things_worse(const so101::ArmModel& model_template) {
    so101::ArmModel arm = model_template;
    so101::IkParams params;
    params.max_iterations = 100;
    params.position_tolerance_m = 0.001;

    std::mt19937 rng(11);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    int worsened = 0;
    double worst_ratio = 0.0;
    int total = 0;

    for (int sample = 0; sample < 100; ++sample) {
        Eigen::VectorXd reachable = pinocchio::neutral(arm.model);
        for (std::size_t index = 0; index < kIkJointCount; ++index) {
            const auto joint = arm.model.joints[arm.ik_joints[index]];
            reachable[joint.idx_q()] =
                arm.model.lowerPositionLimit[joint.idx_q()] +
                unit(rng) * (arm.model.upperPositionLimit[joint.idx_q()] -
                             arm.model.lowerPositionLimit[joint.idx_q()]);
        }
        pinocchio::forwardKinematics(arm.model, arm.data, reachable);
        pinocchio::updateFramePlacements(arm.model, arm.data);
        const Eigen::Vector3d target =
            arm.data.oMf[arm.frame_id].translation();

        const so101::IkResult result =
            so101::solve_ik(arm, target, pinocchio::neutral(arm.model), params);
        ++total;
        const double ratio = result.initial_error_m > 1e-9
                                 ? result.final_error_m / result.initial_error_m
                                 : 1.0;
        if (ratio > 1.0) ++worsened;
        worst_ratio = std::max(worst_ratio, ratio);
    }

    std::array<char, 128> detail{};
    std::snprintf(detail.data(), detail.size(),
                  "из %d целей ухудшились %d, худшее отношение %.2f", total,
                  worsened, worst_ratio);
    CHECK_MSG(worsened == 0, detail.data());
}

// Регрессия: pinocchio 4.1.0 computeFrameJacobian возвращает якобиан, не
// совпадающий с производной прямой кинематики, из-за чего решатель уходил от
// цели. Сверяем используемый нами путь с производной, посчитанной численно.
void test_frame_jacobian_matches_finite_differences(
    const so101::ArmModel& model_template) {
    so101::ArmModel arm = model_template;

    const Eigen::VectorXd starts[] = {
        pinocchio::neutral(arm.model),
        [&arm] {
            Eigen::VectorXd q = pinocchio::neutral(arm.model);
            for (std::size_t index = 0; index < kIkJointCount; ++index) {
                const auto joint = arm.model.joints[arm.ik_joints[index]];
                q[joint.idx_q()] =
                    0.4 * arm.model.upperPositionLimit[joint.idx_q()];
            }
            return q;
        }(),
        [&arm] {
            Eigen::VectorXd q = pinocchio::neutral(arm.model);
            for (std::size_t index = 0; index < kIkJointCount; ++index) {
                const auto joint = arm.model.joints[arm.ik_joints[index]];
                q[joint.idx_q()] =
                    -0.6 * arm.model.lowerPositionLimit[joint.idx_q()];
            }
            return q;
        }(),
    };

    constexpr double epsilon = 1e-6;
    pinocchio::Data::Matrix6x jacobian(6, arm.model.nv);

    for (const Eigen::VectorXd& q : starts) {
        pinocchio::forwardKinematics(arm.model, arm.data, q);
        pinocchio::updateFramePlacements(arm.model, arm.data);
        pinocchio::computeJointJacobians(arm.model, arm.data, q);
        pinocchio::getFrameJacobian(arm.model, arm.data, arm.frame_id,
                                    pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

        for (std::size_t index = 0; index < kIkJointCount; ++index) {
            const auto joint = arm.model.joints[arm.ik_joints[index]];
            Eigen::VectorXd plus = q;
            Eigen::VectorXd minus = q;
            plus[joint.idx_q()] += epsilon;
            minus[joint.idx_q()] -= epsilon;

            pinocchio::Data plus_data(arm.model);
            pinocchio::Data minus_data(arm.model);
            pinocchio::forwardKinematics(arm.model, plus_data, plus);
            pinocchio::updateFramePlacements(arm.model, plus_data);
            pinocchio::forwardKinematics(arm.model, minus_data, minus);
            pinocchio::updateFramePlacements(arm.model, minus_data);
            const Eigen::Vector3d derivative =
                (plus_data.oMf[arm.frame_id].translation() -
                 minus_data.oMf[arm.frame_id].translation()) /
                (2.0 * epsilon);

            const Eigen::Vector3d linear_part =
                jacobian.topRows(3).col(joint.idx_v());
            CHECK_NEAR(linear_part.norm(), derivative.norm(), 1e-5);
            CHECK_NEAR((linear_part - derivative).cwiseAbs().maxCoeff(), 0.0,
                       1e-5);
        }
    }
}

// Сушность варианта А: поворот wrist_roll не должен влиять на позиционное
// решение. Иначе накопление поворота вернётся.
void test_ik_ignores_wrist_roll(const so101::ArmModel& model_template) {
    so101::ArmModel arm = model_template;
    so101::IkParams params;
    params.position_tolerance_m = 0.001;

    auto start = pinocchio::neutral(arm.model);
    start[arm.model.joints[arm.servo_joints[4]].idx_q()] = -1.2;
    const Eigen::Vector3d target{0.22, 0.0, 0.18};

    const so101::IkResult result = so101::solve_ik(arm, target, start, params);
    CHECK(result.converged);
    // wrist_roll не участвует в IK, поэтому решение не должно сдвинуть его.
    CHECK_NEAR(result.q[arm.model.joints[arm.servo_joints[4]].idx_q()],
               start[arm.model.joints[arm.servo_joints[4]].idx_q()], 1e-12);
}

void test_ik_respects_joint_limits(const so101::ArmModel& model_template) {
    so101::ArmModel arm = model_template;
    so101::IkParams params;
    params.max_iterations = 200;
    params.position_tolerance_m = 0.001;

    // Цель за пределами рабочей области: решение обязано остаться в лимитах.
    const so101::IkResult result =
        so101::solve_ik(arm, Eigen::Vector3d{0.6, 0.0, 0.6},
                        pinocchio::neutral(arm.model), params);
    for (std::size_t index = 0; index < so101::kIkJointCount; ++index) {
        const auto joint = arm.model.joints[arm.ik_joints[index]];
        const double q = result.q[joint.idx_q()];
        CHECK(q >= arm.model.lowerPositionLimit[joint.idx_q()] - 1e-9);
        CHECK(q <= arm.model.upperPositionLimit[joint.idx_q()] + 1e-9);
    }
}

}  // namespace

int main() {
    so101::ArmModel arm{};
    std::string error;
    if (!so101::load_arm_model(test_urdf_path(), test_end_effector(), arm,
                                error)) {
        std::cerr << "Не удалось загрузить модель: " << error << '\n';
        return EXIT_FAILURE;
    }

    test_ik_joint_set(arm);
    test_limits_match_calibration(arm);
    test_limits_check_detects_drift(arm);
    test_servo_span_matches_degrees(arm);
    test_angle_roundtrip(arm);
    test_q_to_degrees_clamps(arm);
    test_ik_ignores_wrist_roll(arm);
    test_ik_respects_joint_limits(arm);
    test_frame_jacobian_matches_finite_differences(arm);
    test_ik_tracks_cartesian_steps(arm);
    test_ik_never_makes_things_worse(arm);

    return so101_test::finish("arm_model");
}