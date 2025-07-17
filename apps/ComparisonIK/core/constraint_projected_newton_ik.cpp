#include "constraint_projected_newton_ik.h"
#include "ik_cost_functions.h"
#include <chrono>
#include <iostream>
#include <cmath>
#include <algorithm>

ConstraintProjectedNewtonIKResult ConstraintProjectedNewtonIK::solve(
    const Eigen::Affine3d& target_pose, 
    const Eigen::Matrix<double, 7, 1>& initial_guess) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    ConstraintProjectedNewtonIKResult result;
    result.success = false;
    result.iterations = 0;
    result.projection_iterations = 0;
    result.joint_angles = initial_guess;
    result.final_cost = std::numeric_limits<double>::infinity();
    
    // Ensure initial guess satisfies constraints
    result.joint_angles = projectOntoConstraints(result.joint_angles);
    
    double previous_cost = std::numeric_limits<double>::infinity();
    int stagnation_count = 0;
    const int max_stagnation = 5;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        result.iterations = iter + 1;
        
        // Set current joint angles and compute current pose
        robot.setJointAngles(result.joint_angles);
        Eigen::Affine3d current_pose = robot.getEndeffectorPose();
        
        // Compute pose error
        Eigen::Matrix<double, 6, 1> pose_error = computePoseError(current_pose, target_pose);
        double error_norm = pose_error.norm();
        
        // Compute total cost (pose error + constraint penalties)
        double total_cost = computeTotalCost(target_pose, result.joint_angles);
        
        // Store metrics for analysis
        result.error_history.push_back(error_norm);
        result.cost_history.push_back(total_cost);
        result.constraint_violations.push_back(countConstraintViolations(result.joint_angles));
        
        // Update error metrics
        result.position_error = pose_error.head<3>().norm();
        result.orientation_error = pose_error.tail<3>().norm();
        result.final_error = error_norm;
        result.final_cost = total_cost;
        
        // Check convergence
        if (result.position_error < position_tolerance && 
            result.orientation_error < orientation_tolerance) {
            result.success = true;
            break;
        }
        
        // Check for stagnation
        if (std::abs(total_cost - previous_cost) < 1e-12) {
            stagnation_count++;
            if (stagnation_count >= max_stagnation) {
                break; // Converged or stalled
            }
        } else {
            stagnation_count = 0;
        }
        
        // Compute Newton step with adaptive damping
        Eigen::Matrix<double, 7, 1> newton_direction;
        static double adaptive_damping = initial_damping; // Persist across iterations
        
        if (use_adaptive_damping) {
            // Use adaptive damping (Levenberg-Marquardt style)
            newton_direction = computeAdaptiveDampedStep(target_pose, result.joint_angles, adaptive_damping);
        } else {
            // Use traditional methods
            if (kinematic_only_mode) {
                // Pure pose error minimization
                newton_direction = computeNewtonStep(target_pose, result.joint_angles);
            } else if (collision_avoidance_only_mode) {
                // Total cost minimization (collision-focused)
                newton_direction = computeTotalCostNewtonStep(target_pose, result.joint_angles);
            } else {
                // Hybrid approach: pose accuracy primary + collision correction
                newton_direction = computeHybridNewtonStep(target_pose, result.joint_angles);
            }
        }
        
        // Perform line search to find optimal step size
        double step_size = 1.0;
        if (use_line_search) {
            step_size = performLineSearch(target_pose, result.joint_angles, newton_direction, total_cost);
        }
        
        result.step_sizes.push_back(step_size);
        
        // Add diagnostics
        double step_norm = (step_size * newton_direction).norm();
        result.step_norms.push_back(step_norm);
        
        if (!kinematic_only_mode) {
            // Track gradient norm for total cost function
            Eigen::Matrix<double, 7, 1> gradient = computeTotalCostGradient(target_pose, result.joint_angles);
            result.gradient_norms.push_back(gradient.norm());
        }
        
        // Check for minimum step size
        if (step_size < min_step_size) {
            break; // Step too small, likely converged
        }
        
        // Take step
        Eigen::Matrix<double, 7, 1> candidate_q = result.joint_angles + step_size * newton_direction;
        
        // Project onto constraints
        result.joint_angles = projectOntoConstraints(candidate_q);
        
        // Update adaptive damping if using it
        if (use_adaptive_damping) {
            double cost_improvement = previous_cost - total_cost;
            updateDamping(adaptive_damping, cost_improvement);
        }
        
        previous_cost = total_cost;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.solve_time = duration.count() / 1000.0; // Convert to milliseconds
    
    return result;
}

std::pair<RobotArm, bool> ConstraintProjectedNewtonIK::solveIK(const Eigen::Affine3d& target_pose) {
    // Use current robot configuration as initial guess
    Eigen::Matrix<double, 7, 1> initial_guess = robot.getJointAngles();
    
    // Solve IK
    ConstraintProjectedNewtonIKResult result = solve(target_pose, initial_guess);
    
    // Create result robot arm
    RobotArm result_robot = robot;
    if (result.success) {
        result_robot.setJointAngles(result.joint_angles);
    }
    
    return std::make_pair(result_robot, result.success);
}

Eigen::Matrix<double, 7, 1> ConstraintProjectedNewtonIK::computeNewtonStep(
    const Eigen::Affine3d& target_pose,
    const Eigen::Matrix<double, 7, 1>& current_q) {
    
    // Set robot configuration
    robot.setJointAngles(current_q);
    
    // Compute pose error
    Eigen::Affine3d current_pose = robot.getEndeffectorPose();
    Eigen::Matrix<double, 6, 1> pose_error = computePoseError(current_pose, target_pose);
    
    // Compute Jacobian
    Eigen::MatrixXd J;
    robot.computeJacobian(J);
    
    // Compute Newton step using damped pseudo-inverse
    // (J^T J + λ²I)^(-1) J^T * error
    Eigen::MatrixXd JTJ = J.transpose() * J;
    Eigen::MatrixXd damped_matrix = JTJ + damping_factor * damping_factor * Eigen::MatrixXd::Identity(7, 7);
    
    return damped_matrix.inverse() * J.transpose() * pose_error;
}

Eigen::Matrix<double, 7, 1> ConstraintProjectedNewtonIK::computeTotalCostGradient(
    const Eigen::Affine3d& target_pose,
    const Eigen::Matrix<double, 7, 1>& current_q) {
    
    const double eps = 1e-6; // Finite difference step size
    Eigen::Matrix<double, 7, 1> gradient;
    
    // Current cost
    double current_cost = computeTotalCost(target_pose, current_q);
    
    // Compute gradient using forward differences
    for (int i = 0; i < 7; i++) {
        Eigen::Matrix<double, 7, 1> q_plus = current_q;
        q_plus(i) += eps;
        
        double cost_plus = computeTotalCost(target_pose, q_plus);
        gradient(i) = (cost_plus - current_cost) / eps;
    }
    
    return gradient;
}

Eigen::Matrix<double, 7, 1> ConstraintProjectedNewtonIK::computeTotalCostNewtonStep(
    const Eigen::Affine3d& target_pose,
    const Eigen::Matrix<double, 7, 1>& current_q) {
    
    // Compute gradient of total cost function
    Eigen::Matrix<double, 7, 1> gradient = computeTotalCostGradient(target_pose, current_q);
    
    // For Newton's method, we need the Hessian, but that's expensive to compute
    // Use gradient descent with identity matrix as approximation to Hessian
    // Or use the pose error Jacobian as a reasonable approximation
    
    // Set robot configuration for Jacobian computation
    robot.setJointAngles(current_q);
    Eigen::MatrixXd J;
    robot.computeJacobian(J);
    
    // Use J^T J as approximation to Hessian of total cost
    Eigen::MatrixXd JTJ = J.transpose() * J;
    Eigen::MatrixXd damped_matrix = JTJ + damping_factor * damping_factor * Eigen::MatrixXd::Identity(7, 7);
    
    // Newton step: -H^(-1) * gradient (negative because we want to minimize)
    return -damped_matrix.inverse() * gradient;
}

Eigen::Matrix<double, 7, 1> ConstraintProjectedNewtonIK::computeHybridNewtonStep(
    const Eigen::Affine3d& target_pose,
    const Eigen::Matrix<double, 7, 1>& current_q) {
    
    // Primary step: Newton step for pose error
    Eigen::Matrix<double, 7, 1> pose_step = computeNewtonStep(target_pose, current_q);
    
    // Collision correction: only add if in collision or near collision
    if (path_planner) {
        robot.setJointAngles(current_q);
        auto clearance_metrics = path_planner->computeArmClearance(robot);
        
        if (clearance_metrics.min_clearance < 0.1) { // Only correct if close to collision
            // Compute collision penalty gradient
            const double eps = 1e-6;
            double current_collision_cost = IKCostFunctions::computeGradientFriendlyCollisionPenalty(robot, path_planner);
            
            Eigen::Matrix<double, 7, 1> collision_gradient;
            for (int i = 0; i < 7; i++) {
                Eigen::Matrix<double, 7, 1> q_plus = current_q;
                q_plus(i) += eps;
                robot.setJointAngles(q_plus);
                
                double cost_plus = IKCostFunctions::computeGradientFriendlyCollisionPenalty(robot, path_planner);
                collision_gradient(i) = (cost_plus - current_collision_cost) / eps;
            }
            
            // Add collision correction with small weight
            double collision_weight = 0.1; // Small weight to not overwhelm pose accuracy
            pose_step -= collision_weight * collision_gradient;
        }
    }
    
    return pose_step;
}

Eigen::Matrix<double, 7, 1> ConstraintProjectedNewtonIK::computeAdaptiveDampedStep(
    const Eigen::Affine3d& target_pose,
    const Eigen::Matrix<double, 7, 1>& current_q,
    double& current_damping) {
    
    // Set robot configuration
    robot.setJointAngles(current_q);
    
    if (kinematic_only_mode) {
        // For kinematic-only mode, use pose error with adaptive damping
        Eigen::Affine3d current_pose = robot.getEndeffectorPose();
        Eigen::Matrix<double, 6, 1> pose_error = computePoseError(current_pose, target_pose);
        
        // Compute Jacobian
        Eigen::MatrixXd J;
        robot.computeJacobian(J);
        
        // Adaptive damped least squares: (J^T J + λI)^(-1) J^T * error
        Eigen::MatrixXd JTJ = J.transpose() * J;
        Eigen::MatrixXd damped_matrix = JTJ + current_damping * Eigen::MatrixXd::Identity(7, 7);
        
        return damped_matrix.inverse() * J.transpose() * pose_error;
    } else {
        // For collision avoidance modes, use gradient-based approach with adaptive damping
        Eigen::Matrix<double, 7, 1> gradient = computeTotalCostGradient(target_pose, current_q);
        
        // Compute approximate Hessian using Jacobian
        Eigen::MatrixXd J;
        robot.computeJacobian(J);
        Eigen::MatrixXd JTJ = J.transpose() * J;
        
        // Adaptive damped step: -(J^T J + λI)^(-1) * gradient
        Eigen::MatrixXd damped_matrix = JTJ + current_damping * Eigen::MatrixXd::Identity(7, 7);
        
        return -damped_matrix.inverse() * gradient;
    }
}

void ConstraintProjectedNewtonIK::updateDamping(double& current_damping, double cost_improvement) {
    if (cost_improvement > 0) {
        // Cost decreased - reduce damping for faster convergence (more Newton-like)
        current_damping *= damping_decrease_factor;
        current_damping = std::max(current_damping, min_damping);
    } else {
        // Cost increased or no improvement - increase damping for stability (more gradient-like)
        current_damping *= damping_increase_factor;
        current_damping = std::min(current_damping, max_damping);
    }
}

Eigen::Matrix<double, 7, 1> ConstraintProjectedNewtonIK::projectOntoConstraints(
    const Eigen::Matrix<double, 7, 1>& q) {
    
    Eigen::Matrix<double, 7, 1> projected_q = q;
    
    // Simple projection: enforce joint limits (box constraints)
    projected_q = enforceJointLimits(projected_q);
    
    // For collision constraints, we use penalty methods rather than hard projection
    // This avoids the complexity of projecting onto the collision-free manifold
    
    // Iterative projection for multiple constraints could be added here
    // For now, joint limits are the primary hard constraints
    
    return projected_q;
}

double ConstraintProjectedNewtonIK::performLineSearch(
    const Eigen::Affine3d& target_pose,
    const Eigen::Matrix<double, 7, 1>& current_q,
    const Eigen::Matrix<double, 7, 1>& direction,
    double initial_cost) {
    
    const double alpha = 0.1; // Armijo parameter
    const double beta = 0.5;  // Backtracking parameter
    const int max_backtracks = 10;
    
    double step_size = std::min(max_step_size, 1.0);
    
    for (int i = 0; i < max_backtracks; ++i) {
        // Candidate point
        Eigen::Matrix<double, 7, 1> candidate_q = current_q + step_size * direction;
        candidate_q = projectOntoConstraints(candidate_q); // Ensure constraints are satisfied
        
        // Compute cost at candidate point
        double candidate_cost = computeTotalCost(target_pose, candidate_q);
        
        // Armijo condition for sufficient decrease
        double expected_decrease = alpha * step_size * (-initial_cost); // Negative gradient assumption
        
        if (candidate_cost <= initial_cost + expected_decrease) {
            return step_size; // Sufficient decrease achieved
        }
        
        // Reduce step size
        step_size *= beta;
        
        if (step_size < min_step_size) {
            break;
        }
    }
    
    return step_size;
}

double ConstraintProjectedNewtonIK::computeTotalCost(const Eigen::Affine3d& target_pose,
                                                    const Eigen::Matrix<double, 7, 1>& q) {
    // Set robot configuration
    robot.setJointAngles(q);
    
    if (kinematic_only_mode) {
        // Pure kinematic cost - only pose error
        Eigen::Affine3d current_pose = robot.getEndeffectorPose();
        Eigen::Matrix<double, 6, 1> pose_error = computePoseError(current_pose, target_pose);
        
        // Weight position error more heavily than orientation error
        double position_cost = pose_error.head<3>().squaredNorm();
        double orientation_cost = 0.1 * pose_error.tail<3>().squaredNorm(); // Reduced weight
        
        return position_cost + orientation_cost;
    } else if (collision_avoidance_only_mode) {
        // Collision-focused but with minimal pose accuracy to prevent drift
        if (path_planner) {
            double collision_cost = IKCostFunctions::computeGradientFriendlyCollisionPenalty(robot, path_planner);
            
            // Add minimal pose error to prevent drifting away from target
            Eigen::Affine3d current_pose = robot.getEndeffectorPose();
            Eigen::Matrix<double, 6, 1> pose_error = computePoseError(current_pose, target_pose);
            double minimal_pose_cost = 0.1 * pose_error.squaredNorm(); // Very low weight
            
            return collision_cost + minimal_pose_cost;
        } else {
            return 0.0; // No collision detection available
        }
    } else {
        // Use gradient-friendly comprehensive cost function for better Newton's method performance
        return IKCostFunctions::computeGradientFriendlyComprehensiveCost(robot, target_pose, path_planner);
    }
}

Eigen::Vector3d ConstraintProjectedNewtonIK::rotationMatrixToAxisAngle(const Eigen::Matrix3d& R) {
    // Convert rotation matrix to axis-angle representation
    Eigen::AngleAxisd axis_angle(R);
    return axis_angle.angle() * axis_angle.axis();
}

Eigen::Matrix<double, 6, 1> ConstraintProjectedNewtonIK::computePoseError(
    const Eigen::Affine3d& current_pose, 
    const Eigen::Affine3d& target_pose) {
    
    Eigen::Matrix<double, 6, 1> error;
    
    // Position error
    error.head<3>() = target_pose.translation() - current_pose.translation();
    
    // Orientation error using axis-angle representation
    Eigen::Matrix3d rotation_error = target_pose.rotation() * current_pose.rotation().transpose();
    error.tail<3>() = rotationMatrixToAxisAngle(rotation_error);
    
    return error;
}

Eigen::Matrix<double, 7, 1> ConstraintProjectedNewtonIK::enforceJointLimits(
    const Eigen::Matrix<double, 7, 1>& joint_angles) {
    
    auto joint_limits = robot.jointLimits();
    Eigen::Matrix<double, 7, 1> clamped_angles = joint_angles;
    
    for (int i = 0; i < 7; ++i) {
        double lower_limit = joint_limits[i].first + joint_limit_margin;
        double upper_limit = joint_limits[i].second - joint_limit_margin;
        clamped_angles(i) = std::clamp(clamped_angles(i), lower_limit, upper_limit);
    }
    
    return clamped_angles;
}

double ConstraintProjectedNewtonIK::computeJointLimitDistance(const Eigen::Matrix<double, 7, 1>& q) {
    auto joint_limits = robot.jointLimits();
    double min_distance = std::numeric_limits<double>::infinity();
    
    for (int i = 0; i < 7; ++i) {
        double lower_dist = q(i) - joint_limits[i].first;
        double upper_dist = joint_limits[i].second - q(i);
        double joint_min_dist = std::min(lower_dist, upper_dist);
        min_distance = std::min(min_distance, joint_min_dist);
    }
    
    return min_distance;
}

int ConstraintProjectedNewtonIK::countConstraintViolations(const Eigen::Matrix<double, 7, 1>& q) {
    int violations = 0;
    
    // Check joint limits
    auto joint_limits = robot.jointLimits();
    for (int i = 0; i < 7; ++i) {
        if (q(i) <= joint_limits[i].first + joint_limit_margin ||
            q(i) >= joint_limits[i].second - joint_limit_margin) {
            violations++;
        }
    }
    
    // Collision violations are now handled through the shared cost function
    // so we don't count them separately here
    
    return violations;
}
