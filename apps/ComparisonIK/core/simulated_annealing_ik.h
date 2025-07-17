#pragma once

#include <functional>
#include <random>
#include <limits>
#include <cmath>
#include <optional>
#include <array>
#include <Eigen/Dense>
#include "ik_cost_functions.h"
#include "TrajectoryLib/Robot/RobotArm.h"
#include "TrajectoryLib/Robot/franka_ik_He.h"

/**
 * @brief Parameters for simulated annealing IK optimization
 */
struct SimulatedAnnealingParams {
    double T_max = 0.5;                     ///< Initial temperature - reduced
    double T_min = 0.001;                   ///< Minimum temperature - smaller range
    double alpha = 0.9;                     ///< Cooling rate - much faster cooling
    int max_iterations = 1000;              ///< Maximum number of iterations - much fewer
    int max_no_improvement = 100;           ///< Early termination - very aggressive
    double search_min = -2.8973;           ///< Minimum search bound (q7 joint limit)
    double search_max = 2.8973;            ///< Maximum search bound (q7 joint limit)
    
    // Pose accuracy thresholds
    double max_position_error = 0.0001;    ///< Maximum position error (0.1mm)
    double max_orientation_error = 0.001;  ///< Maximum orientation error (~0.057 degrees)
};

/**
 * @brief Result of simulated annealing IK optimization
 */
struct SimulatedAnnealingResult {
    Eigen::Matrix<double, 7, 1> best_solution;  ///< Best joint configuration found
    double best_cost = std::numeric_limits<double>::infinity(); ///< Best cost achieved
    bool success = false;                   ///< Whether optimization succeeded
    int iterations_used = 0;                ///< Number of iterations used
    double final_q7 = 0.0;                 ///< Final q7 value that produced best solution
};

/**
 * @brief Performs simulated annealing optimization for IK solving
 * 
 * This function optimizes the q7 joint parameter using simulated annealing
 * to find the best inverse kinematics solution for a given target pose.
 * Uses the shared cost function for consistency with other IK methods.
 * 
 * @param target_pose Target end-effector pose
 * @param start_arm Initial robot arm configuration
 * @param path_planner Optional path planner for collision checking
 * @param params Optimization parameters
 * @param initial_q7 Optional initial q7 value (random if not provided)
 * @return SimulatedAnnealingResult containing the optimization result
 */
SimulatedAnnealingResult solveIKSimulatedAnnealing(
    const Eigen::Affine3d& target_pose,
    const RobotArm& start_arm,
    PathPlanner* path_planner = nullptr,
    const SimulatedAnnealingParams& params = SimulatedAnnealingParams(),
    const std::optional<double>& initial_q7 = std::nullopt
);
