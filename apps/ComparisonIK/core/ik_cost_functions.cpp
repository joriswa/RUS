#include "ik_cost_functions.h"
#include <cmath>
#include <algorithm>
#include <limits>

double IKCostFunctions::computeComprehensiveCost(const RobotArm& arm, 
                                               const Eigen::Affine3d& target_pose,
                                               PathPlanner* path_planner) {
    double cost = 0.0;
    
    // 1. POSE ERROR (HIGHEST PRIORITY)
    double pose_error = computePoseErrorMagnitude(arm, target_pose);
    cost += 100.0 * pose_error; // Heavy weight to prioritize pose accuracy
    
    // 2. Clearance-based cost (if path planner is available)
    if (path_planner) {
        double clearance_penalty = computeClearancePenalty(arm, path_planner);
        cost += clearance_penalty;
    }
    
    // 3. Add manipulability (lowest priority)
    double manipulability = computeManipulabilityMeasure(arm);
    cost += 0.25 * (1.0 - manipulability);
    
    return cost;
}

double IKCostFunctions::computePoseErrorMagnitude(const RobotArm& arm, 
                                                const Eigen::Affine3d& target_pose) {
    Eigen::Affine3d current_pose = arm.getEndeffectorPose();
    
    // Position error
    Eigen::Vector3d position_error = target_pose.translation() - current_pose.translation();
    double pos_error = position_error.norm();
    
    // Orientation error (using angle-axis representation)
    Eigen::Matrix3d rotation_error = target_pose.linear() * current_pose.linear().transpose();
    Eigen::AngleAxisd angle_axis(rotation_error);
    double orientation_error = std::abs(angle_axis.angle());
    
    // Combined error with weighting
    return pos_error + 0.1 * orientation_error; // Weight orientation less than position
}

double IKCostFunctions::computeClearancePenalty(const RobotArm& arm, 
                                              PathPlanner* path_planner) {
    if (!path_planner) return 0.0;
    
    const double threshold = 0.3;
    double penalty = 0.0;
    
    // Need to cast away const for computeArmClearance API
    RobotArm& non_const_arm = const_cast<RobotArm&>(arm);
    
    // Get clearance metrics (excluding first two base links and end effector)
    auto clearance_metrics = path_planner->computeArmClearance(non_const_arm);
    
    // Penalty based on minimum clearance
    if (clearance_metrics.min_clearance < threshold) {
        double dist = clearance_metrics.min_clearance;
        penalty = 1.0 / (1.0 + std::exp(100 * (dist - threshold)));
    }
    
    return penalty;
}

double IKCostFunctions::computeManipulabilityMeasure(const RobotArm& arm) {
    // Use the robot's built-in manipulability computation
    return arm.computeManipulability();
}

std::pair<double, double> IKCostFunctions::computeJointLimitDistances(
    const Eigen::Matrix<double, 7, 1>& joint_angles) {
    
    // Official Franka Panda joint limits
    const double q_min[7] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    const double q_max[7] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
    
    double min_distance = std::numeric_limits<double>::max();
    double total_distance = 0.0;
    
    for (int i = 0; i < 7; i++) {
        // Distance to lower and upper limits
        double dist_to_lower = joint_angles(i) - q_min[i];
        double dist_to_upper = q_max[i] - joint_angles(i);
        
        // Minimum distance to either limit for this joint
        double joint_min_distance = std::min(dist_to_lower, dist_to_upper);
        
        // Update overall minimum
        min_distance = std::min(min_distance, joint_min_distance);
        total_distance += joint_min_distance;
    }
    
    double avg_distance = total_distance / 7.0;
    return std::make_pair(min_distance, avg_distance);
}

double IKCostFunctions::computeGradientFriendlyCollisionPenalty(const RobotArm& arm, 
                                                              PathPlanner* path_planner) {
    if (!path_planner) return 0.0;
    
    const double safe_distance = 0.3;  // Safe clearance distance
    const double danger_distance = 0.05; // Critical collision distance
    
    // Need to cast away const for computeArmClearance API
    RobotArm& non_const_arm = const_cast<RobotArm&>(arm);
    
    // Get clearance metrics
    auto clearance_metrics = path_planner->computeArmClearance(non_const_arm);
    double dist = clearance_metrics.min_clearance;
    
    // Gradient-friendly penalty function with reduced magnitudes
    if (dist >= safe_distance) {
        return 0.0; // No penalty in safe zone
    } else if (dist <= danger_distance) {
        // Reduced steep penalty in danger zone  
        double normalized_dist = (dist - danger_distance) / (safe_distance - danger_distance);
        return 5.0 * std::pow(1.0 - normalized_dist, 3); // Reduced from 50.0 to 5.0
    } else {
        // Reduced smooth transition zone
        double normalized_dist = (dist - danger_distance) / (safe_distance - danger_distance);
        return 1.0 * std::pow(1.0 - normalized_dist, 2); // Reduced from 10.0 to 1.0
    }
}

double IKCostFunctions::computeGradientFriendlyComprehensiveCost(const RobotArm& arm, 
                                                               const Eigen::Affine3d& target_pose,
                                                               PathPlanner* path_planner) {
    double cost = 0.0;
    
    // 1. POSE ERROR (HIGHEST PRIORITY)
    double pose_error = computePoseErrorMagnitude(arm, target_pose);
    cost += 100.0 * pose_error; // Heavy weight to prioritize pose accuracy
    
    // 2. Gradient-friendly clearance-based cost (if path planner is available)
    if (path_planner) {
        double clearance_penalty = computeGradientFriendlyCollisionPenalty(arm, path_planner);
        cost += clearance_penalty;
    }
    
    // 3. Add manipulability (lowest priority)
    double manipulability = computeManipulabilityMeasure(arm);
    cost += 0.25 * (1.0 - manipulability);
    
    return cost;
}
