// Analytical Franka inverse kinematics using q7 as redundant parameter
// - Yanhao He, February 2020

#pragma once

#ifndef FRANKA_IK_HE_HPP
#define FRANKA_IK_HE_HPP

#include "Eigen/Dense"
#include <array>

/**
 * @brief Inverse kinematics w.r.t. End Effector Frame (using Franka Hand data)
 * 
 * @param O_T_EE_array Transform from base to end effector
 * @param q7 Redundant parameter (7th joint angle)
 * @param q_actual_array Current joint configuration
 * @return Array of 4 possible joint configurations, each with 7 DOF
 */
std::array<std::array<double, 7>, 4> franka_IK_EE(Eigen::Affine3d O_T_EE_array,
                                                  double q7,
                                                  std::array<double, 7> q_actual_array);

/**
 * @brief "Case-Consistent" inverse kinematics w.r.t. End Effector Frame (using Franka Hand data)
 * 
 * This function selects a single solution that is consistent with the current joint configuration.
 * 
 * @param O_T_EE Transform from base to end effector
 * @param q7 Redundant parameter (7th joint angle)
 * @param q_actual_array Current joint configuration
 * @return Single joint configuration with 7 DOF, or NaN array if no solution exists
 */
std::array<double, 7> franka_IK_EE_CC(Eigen::Matrix<double, 4, 4> O_T_EE,
                                      double q7,
                                      std::array<double, 7> q_actual_array);

#endif // FRANKA_IK_HE_HPP
