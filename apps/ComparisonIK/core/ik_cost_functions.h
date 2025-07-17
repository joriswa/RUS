#pragma once

#include <Eigen/Dense>
#include "TrajectoryLib/Robot/RobotArm.h"
#include "TrajectoryLib/Planning/PathPlanner.h"

/**
 * @brief Shared cost function utilities for IK method comparison
 * 
 * This module provides reusable cost functions that can be used across
 * different IK solvers to ensure fair comparison. The cost functions
 * are based on the PathPlanner SA approach for consistency.
 */
class IKCostFunctions {
public:
    /**
     * @brief Compute comprehensive cost function for IK solutions
     * @param arm Robot arm in current configuration
     * @param target_pose Desired end-effector pose
     * @param path_planner Optional path planner for collision checking (can be nullptr)
     * @return Combined cost value (lower is better)
     */
    static double computeComprehensiveCost(const RobotArm& arm, 
                                         const Eigen::Affine3d& target_pose,
                                         PathPlanner* path_planner = nullptr);
    
    /**
     * @brief Compute pose error magnitude between current and target pose
     * @param arm Robot arm in current configuration
     * @param target_pose Desired end-effector pose
     * @return Pose error magnitude
     */
    static double computePoseErrorMagnitude(const RobotArm& arm, 
                                          const Eigen::Affine3d& target_pose);
    
    /**
     * @brief Compute clearance-based penalty for obstacle avoidance
     * @param arm Robot arm in current configuration
     * @param path_planner Path planner for obstacle checking
     * @return Clearance penalty (0 if no obstacles)
     */
    static double computeClearancePenalty(const RobotArm& arm, 
                                        PathPlanner* path_planner);
    
    /**
     * @brief Compute manipulability measure for the current configuration
     * @param arm Robot arm in current configuration
     * @return Manipulability value (higher is better, normalized 0-1)
     */
    static double computeManipulabilityMeasure(const RobotArm& arm);
    
    /**
     * @brief Joint limit distance measures
     * @param joint_angles Current joint configuration
     * @return Pair of (min_distance, avg_distance) to joint limits
     */
    static std::pair<double, double> computeJointLimitDistances(
        const Eigen::Matrix<double, 7, 1>& joint_angles);
    
    /**
     * @brief Compute gradient-friendly collision penalty for Newton's method
     * @param arm Robot arm in current configuration
     * @param path_planner Path planner for obstacle checking
     * @return Smooth collision penalty with good gradients
     */
    static double computeGradientFriendlyCollisionPenalty(const RobotArm& arm, 
                                                         PathPlanner* path_planner);

    /**
     * @brief Compute comprehensive cost with gradient-friendly collision penalty
     * @param arm Robot arm in current configuration
     * @param target_pose Desired end-effector pose
     * @param path_planner Optional path planner for collision checking (can be nullptr)
     * @return Combined cost value optimized for gradient-based methods
     */
    static double computeGradientFriendlyComprehensiveCost(const RobotArm& arm, 
                                                          const Eigen::Affine3d& target_pose,
                                                          PathPlanner* path_planner = nullptr);
};
