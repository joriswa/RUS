#include "damped_least_squares.h"
#include <chrono>
#include <iostream>
#include <cmath>

DampedLeastSquaresIKResult DampedLeastSquaresIK::solve(const Eigen::Affine3d& target_pose, 
                                                      const Eigen::Matrix<double, 7, 1>& initial_guess) {
    DampedLeastSquaresIKResult result;
    result.success = false;
    result.iterations = 0;
    result.joint_angles = initial_guess;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        result.iterations = iter + 1;
        
        // Set current joint angles
        robot.setJointAngles(result.joint_angles);
        Eigen::Affine3d current_pose = robot.getEndeffectorPose();
        
        // Compute pose error
        Eigen::Matrix<double, 6, 1> pose_error = computePoseError(current_pose, target_pose);
        double error_norm = pose_error.norm();
        
        // Store error for analysis
        result.error_history.push_back(error_norm);
        
        // Check convergence
        result.position_error = pose_error.head<3>().norm();
        result.orientation_error = pose_error.tail<3>().norm();
        
        if (result.position_error < position_tolerance && 
            result.orientation_error < orientation_tolerance) {
            result.success = true;
            result.final_error = error_norm;
            break;
        }
        
        // Compute Jacobian
        Eigen::MatrixXd J;
        robot.computeJacobian(J);
        
        // Compute adaptive damping
        double current_damping = adaptive_damping ? 
            computeAdaptiveDamping(J, damping_factor) : damping_factor;
        result.damping_history.push_back(current_damping);
        
        // Correct Damped Least Squares formula for redundant systems: J_λ⁺ = (J^T J + λ²I)^(-1) J^T
        // This is more stable for overdetermined systems (7 joints, 6 DOF task space)
        Eigen::MatrixXd JTJ = J.transpose() * J;
        Eigen::MatrixXd damped_matrix = JTJ + current_damping * current_damping * Eigen::MatrixXd::Identity(7, 7);
        Eigen::Matrix<double, 7, 1> delta_q = damped_matrix.inverse() * J.transpose() * pose_error;
        
        // Limit step size
        double step_norm = delta_q.norm();
        if (step_norm > max_step_size) {
            delta_q *= (max_step_size / step_norm);
        }
        
        // Update joint angles
        result.joint_angles += delta_q;
        
        // Enforce joint limits
        if (enforce_joint_limits) {
            result.joint_angles = enforceJointLimits(result.joint_angles);
        }
        
        // Check for very small updates (convergence)
        if (delta_q.norm() < 1e-6) {
            break;
        }
    }
    
    result.final_error = result.error_history.empty() ? 
        std::numeric_limits<double>::infinity() : result.error_history.back();
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.solve_time = duration.count() / 1000.0; // Convert to milliseconds
    
    return result;
}

std::pair<RobotArm, bool> DampedLeastSquaresIK::solveIK(const Eigen::Affine3d& target_pose) {
    // Use current robot configuration as initial guess
    Eigen::Matrix<double, 7, 1> initial_guess = robot.getJointAngles();
    
    // Solve IK
    DampedLeastSquaresIKResult result = solve(target_pose, initial_guess);
    
    // Create result robot arm
    RobotArm result_robot = robot;
    if (result.success) {
        result_robot.setJointAngles(result.joint_angles);
    }
    
    return std::make_pair(result_robot, result.success);
}

Eigen::Matrix<double, 7, 1> DampedLeastSquaresIK::solve(const Eigen::MatrixXd& J, 
                                                       const Eigen::Matrix<double, 6, 1>& pose_error) {
    // Compute adaptive damping
    double current_damping = adaptive_damping ? 
        computeAdaptiveDamping(J, damping_factor) : damping_factor;
    
    // Correct Damped Least Squares formula for redundant systems: J_λ⁺ = (J^T J + λ²I)^(-1) J^T
    // This is more stable for overdetermined systems (7 joints, 6 DOF task space)
    Eigen::MatrixXd JTJ = J.transpose() * J;
    Eigen::MatrixXd damped_matrix = JTJ + current_damping * current_damping * Eigen::MatrixXd::Identity(7, 7);
    Eigen::Matrix<double, 7, 1> delta_q = damped_matrix.inverse() * J.transpose() * pose_error;
    
    return delta_q;
}

Eigen::Vector3d DampedLeastSquaresIK::rotationMatrixToAxisAngle(const Eigen::Matrix3d& R) {
    // Convert rotation matrix to axis-angle representation
    Eigen::AngleAxisd axis_angle(R);
    return axis_angle.angle() * axis_angle.axis();
}

Eigen::Matrix<double, 6, 1> DampedLeastSquaresIK::computePoseError(const Eigen::Affine3d& current_pose, 
                                                                   const Eigen::Affine3d& target_pose) {
    Eigen::Matrix<double, 6, 1> error;
    
    // Position error
    error.head<3>() = target_pose.translation() - current_pose.translation();
    
    // Orientation error using axis-angle representation
    Eigen::Matrix3d rotation_error = target_pose.rotation() * current_pose.rotation().transpose();
    error.tail<3>() = rotationMatrixToAxisAngle(rotation_error);
    
    return error;
}

Eigen::Matrix<double, 7, 1> DampedLeastSquaresIK::enforceJointLimits(const Eigen::Matrix<double, 7, 1>& joint_angles) {
    auto joint_limits = robot.jointLimits();
    Eigen::Matrix<double, 7, 1> clamped_angles = joint_angles;
    
    for (int i = 0; i < 7; ++i) {
        double lower_limit = joint_limits[i].first + joint_limit_margin;
        double upper_limit = joint_limits[i].second - joint_limit_margin;
        clamped_angles(i) = std::clamp(clamped_angles(i), lower_limit, upper_limit);
    }
    
    return clamped_angles;
}

double DampedLeastSquaresIK::computeAdaptiveDamping(const Eigen::MatrixXd& J, double base_damping) {
    // Compute condition number of Jacobian
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto singular_values = svd.singularValues();
    
    if (singular_values.size() > 0) {
        double max_sv = singular_values(0);
        double min_sv = singular_values(singular_values.size() - 1);
        
        if (min_sv > 1e-6) {
            double condition_number = max_sv / min_sv;
            
            // Increase damping for ill-conditioned Jacobians
            if (condition_number > 100.0) {
                return base_damping * std::sqrt(condition_number / 100.0);
            }
        } else {
            // Near-singular Jacobian, use high damping
            return base_damping * 10.0;
        }
    }
    
    return base_damping;
}
