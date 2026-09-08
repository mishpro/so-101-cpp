#pragma once

#include <array>
#include <string>

struct CartesianIkConfig {
    std::string urdf_path;
    std::string end_effector_frame;
    int max_iterations;
    double damping;
    double position_tolerance;
};

bool solve_cartesian_position(const CartesianIkConfig& config,
                              const std::array<double, 3>& target_position,
                              std::array<double, 6>& joint_angles_radians,
                              std::string& error);

bool solve_cartesian_position_from_angles(
    const CartesianIkConfig& config,
    const std::array<double, 3>& target_position,
    const std::array<double, 6>& initial_joint_angles_radians,
    std::array<double, 6>& joint_angles_radians,
    std::string& error);

bool get_neutral_cartesian_position(const CartesianIkConfig& config,
                                    std::array<double, 3>& position,
                                    std::string& error);

bool get_cartesian_position(const CartesianIkConfig& config,
                            const std::array<double, 6>& joint_angles_radians,
                            std::array<double, 3>& position,
                            std::string& error);
