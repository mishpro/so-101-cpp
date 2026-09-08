#include "cartesian_ik.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <array>
#include <cmath>
#include <sstream>

namespace {
constexpr std::array<const char*, 6> kJointNames = {
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper"
};

bool load_model(const CartesianIkConfig& config, pinocchio::Model& model,
                std::string& error) {
    try {
        pinocchio::urdf::buildModel(config.urdf_path, model);
    } catch (const std::exception& exception) {
        error = "Failed to load URDF: ";
        error += exception.what();
        return false;
    }
    return true;
}
}

bool solve_cartesian_position_impl(
    const CartesianIkConfig& config, const std::array<double, 3>& target_position,
    const Eigen::VectorXd& initial_q,
    std::array<double, 6>& joint_angles_radians, std::string& error) {
    if (config.urdf_path.empty() || config.end_effector_frame.empty() ||
        config.max_iterations <= 0 || !std::isfinite(config.damping) ||
        config.damping <= 0.0 || !std::isfinite(config.position_tolerance) ||
        config.position_tolerance <= 0.0) {
        error = "Invalid Cartesian IK configuration";
        return false;
    }

    for (double coordinate : target_position) {
        if (!std::isfinite(coordinate)) {
            error = "Cartesian target must contain finite coordinates";
            return false;
        }
    }

    pinocchio::Model model;
    if (!load_model(config, model, error)) return false;

    const pinocchio::FrameIndex frame_id =
        model.getFrameId(config.end_effector_frame);
    if (frame_id >= model.nframes ||
        model.frames[frame_id].name != config.end_effector_frame) {
        error = "End-effector frame not found in URDF: " +
                config.end_effector_frame;
        return false;
    }

    std::array<pinocchio::JointIndex, 6> joint_ids{};
    for (size_t index = 0; index < kJointNames.size(); ++index) {
        joint_ids[index] = model.getJointId(kJointNames[index]);
        if (joint_ids[index] == 0) {
            error = "Joint not found in URDF: ";
            error += kJointNames[index];
            return false;
        }
    }

    if (model.nq < 6 || model.nv < 6) {
        error = "URDF model does not contain six actuated joints";
        return false;
    }

    Eigen::VectorXd q = initial_q;
    const Eigen::Vector3d target(target_position.data());
    pinocchio::Data data(model);
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

    for (int iteration = 0; iteration < config.max_iterations; ++iteration) {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacement(model, data, frame_id);

        const Eigen::Vector3d error_vector =
            target - data.oMf[frame_id].translation();
        if (!error_vector.allFinite()) {
            error = "Cartesian IK produced a non-finite position";
            return false;
        }
        if (error_vector.norm() <= config.position_tolerance) {
            for (size_t index = 0; index < joint_ids.size(); ++index) {
                joint_angles_radians[index] = q[model.idx_qs[joint_ids[index]]];
            }
            return true;
        }

        Eigen::MatrixXd frame_jacobian(6, model.nv);
        pinocchio::computeFrameJacobian(
            model, data, q, frame_id, pinocchio::WORLD, frame_jacobian);
        const Eigen::MatrixXd position_jacobian =
            frame_jacobian.block(0, 0, 3, model.nv);
        const Eigen::Matrix3d system =
            position_jacobian * position_jacobian.transpose() +
            config.damping * config.damping * identity;
        Eigen::LDLT<Eigen::Matrix3d> decomposition(system);
        if (decomposition.info() != Eigen::Success) {
            error = "Cartesian IK failed to factorize the Jacobian system";
            return false;
        }
        Eigen::VectorXd delta =
            position_jacobian.transpose() * decomposition.solve(error_vector);
        if (!delta.allFinite()) {
            error = "Cartesian IK produced a non-finite joint update";
            return false;
        }

        constexpr double max_joint_step = 0.1;
        const double step_norm = delta.norm();
        if (step_norm > max_joint_step) {
            delta *= max_joint_step / step_norm;
        }

        q = pinocchio::integrate(model, q, delta);
        q = q.cwiseMax(model.lowerPositionLimit)
             .cwiseMin(model.upperPositionLimit);
        if (!q.allFinite()) {
            error = "Cartesian IK produced an invalid joint configuration";
            return false;
        }
    }

    std::ostringstream message;
    message << "Cartesian IK did not converge within "
            << config.max_iterations << " iterations";
    error = message.str();
    return false;
}

bool solve_cartesian_position(const CartesianIkConfig& config,
                              const std::array<double, 3>& target_position,
                              std::array<double, 6>& joint_angles_radians,
                              std::string& error) {
    pinocchio::Model model;
    if (!load_model(config, model, error)) return false;
    return solve_cartesian_position_impl(
        config, target_position, pinocchio::neutral(model),
        joint_angles_radians, error);
}

bool solve_cartesian_position_from_angles(
    const CartesianIkConfig& config,
    const std::array<double, 3>& target_position,
    const std::array<double, 6>& initial_joint_angles_radians,
    std::array<double, 6>& joint_angles_radians,
    std::string& error) {
    pinocchio::Model model;
    if (!load_model(config, model, error)) return false;
    Eigen::VectorXd initial_q = pinocchio::neutral(model);
    for (size_t index = 0; index < initial_joint_angles_radians.size(); ++index) {
        const pinocchio::JointIndex joint_id =
            model.getJointId(kJointNames[index]);
        if (joint_id == 0) {
            error = "Joint not found in URDF: ";
            error += kJointNames[index];
            return false;
        }
        initial_q[model.idx_qs[joint_id]] =
            initial_joint_angles_radians[index];
    }
    return solve_cartesian_position_impl(
        config, target_position, initial_q, joint_angles_radians, error);
}

bool get_neutral_cartesian_position(const CartesianIkConfig& config,
                                    std::array<double, 3>& position,
                                    std::string& error) {
    if (config.urdf_path.empty() || config.end_effector_frame.empty()) {
        error = "Invalid Cartesian IK configuration";
        return false;
    }

    pinocchio::Model model;
    if (!load_model(config, model, error)) return false;

    const pinocchio::FrameIndex frame_id =
        model.getFrameId(config.end_effector_frame);
    if (frame_id >= model.nframes ||
        model.frames[frame_id].name != config.end_effector_frame) {
        error = "End-effector frame not found in URDF: " +
                config.end_effector_frame;
        return false;
    }

    pinocchio::Data data(model);
    const Eigen::VectorXd q = pinocchio::neutral(model);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacement(model, data, frame_id);
    const Eigen::Vector3d neutral_position = data.oMf[frame_id].translation();
    for (size_t index = 0; index < position.size(); ++index) {
        position[index] = neutral_position[static_cast<Eigen::Index>(index)];
    }
    return true;
}

bool get_cartesian_position(const CartesianIkConfig& config,
                            const std::array<double, 6>& joint_angles_radians,
                            std::array<double, 3>& position,
                            std::string& error) {
    if (config.urdf_path.empty() || config.end_effector_frame.empty()) {
        error = "Invalid Cartesian IK configuration";
        return false;
    }

    pinocchio::Model model;
    if (!load_model(config, model, error)) return false;

    const pinocchio::FrameIndex frame_id =
        model.getFrameId(config.end_effector_frame);
    if (frame_id >= model.nframes ||
        model.frames[frame_id].name != config.end_effector_frame) {
        error = "End-effector frame not found in URDF: " +
                config.end_effector_frame;
        return false;
    }

    Eigen::VectorXd q = pinocchio::neutral(model);
    for (size_t index = 0; index < kJointNames.size(); ++index) {
        const pinocchio::JointIndex joint_id =
            model.getJointId(kJointNames[index]);
        if (joint_id == 0) {
            error = "Joint not found in URDF: ";
            error += kJointNames[index];
            return false;
        }
        q[model.idx_qs[joint_id]] = joint_angles_radians[index];
    }

    pinocchio::Data data(model);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacement(model, data, frame_id);
    const Eigen::Vector3d frame_position = data.oMf[frame_id].translation();
    if (!frame_position.allFinite()) {
        error = "Forward kinematics produced a non-finite position";
        return false;
    }
    for (size_t index = 0; index < position.size(); ++index) {
        position[index] = frame_position[static_cast<Eigen::Index>(index)];
    }
    return true;
}
