#include "newton_raphson_ik.h"
#include <chrono>
#include <iostream>
#include <cmath>
#include <random>

NewtonRaphsonIKResult NewtonRaphsonIK::solve(const Eigen::Affine3d& target_pose, 
                                              const Eigen::Matrix<double, 7, 1>& initial_guess) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Try multiple initial guesses for better convergence
    std::vector<Eigen::Matrix<double, 7, 1>> initial_guesses;
    
    // Strategy 1: Use provided initial guess
    initial_guesses.push_back(initial_guess);
    
    // Strategy 2: Zero configuration
    initial_guesses.push_back(Eigen::Matrix<double, 7, 1>::Zero());
    
    // Strategy 3: Random configurations within joint limits
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-2.0, 2.0);
    
    for (int i = 0; i < 3; ++i) {
        Eigen::Matrix<double, 7, 1> random_guess;
        for (int j = 0; j < 7; ++j) {
            random_guess(j) = dis(gen);
        }
        initial_guesses.push_back(enforceJointLimits(random_guess));
    }
    
    NewtonRaphsonIKResult best_result;
    best_result.success = false;
    best_result.final_error = std::numeric_limits<double>::infinity();
    
    // Try each initial guess
    for (const auto& guess : initial_guesses) {
        NewtonRaphsonIKResult current_result = solveSingle(target_pose, guess);
        
        // Keep the best result (successful with lowest error, or best failure)
        if ((current_result.success && !best_result.success) ||
            (current_result.success == best_result.success && 
             current_result.final_error < best_result.final_error)) {
            best_result = current_result;
        }
        
        // Early exit if we found a good solution
        if (current_result.success && 
            current_result.position_error < position_tolerance/10 &&
            current_result.orientation_error < orientation_tolerance/10) {
            best_result = current_result;
            break;
        }
    }
    
    return best_result;
}

NewtonRaphsonIKResult NewtonRaphsonIK::solveSingle(const Eigen::Affine3d& target_pose, 
                                                   const Eigen::Matrix<double, 7, 1>& initial_guess) {
    NewtonRaphsonIKResult result;
    result.success = false;
    result.iterations = 0;
    result.joint_angles = initial_guess;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    double step_size = 1.0;
    const double min_step_size = 1e-6;
    const double step_reduction_factor = 0.5;
    double previous_error_norm = std::numeric_limits<double>::infinity();
    double stagnation_count = 0;
    const int max_stagnation = 3;
    
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
        
        // Choose between Newton-Raphson and Damped Least Squares methods
        Eigen::Matrix<double, 7, 1> delta_q;
        
        if (use_damped_least_squares) {
            // Damped Least Squares: (J^T J + λ²I)^(-1) J^T
            // This is the left pseudoinverse formulation, better for redundant systems
            Eigen::MatrixXd JTJ = J.transpose() * J;
            double effective_damping = adaptive_damping ? 
                computeAdaptiveDamping(J, damping_factor) : damping_factor;
            Eigen::MatrixXd damped_matrix = JTJ + effective_damping * effective_damping * Eigen::MatrixXd::Identity(7, 7);
            delta_q = damped_matrix.inverse() * J.transpose() * pose_error;
        } else {
            // Newton-Raphson: J^T(JJ^T + λI)^(-1) 
            // This is the right pseudoinverse formulation
            if (adaptive_damping) {
                // Check if we're near a singularity (small singular values)
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
                double min_sv = svd.singularValues().minCoeff();
                double effective_damping = (min_sv < 1e-3) ? damping_factor : 0.001;
                
                // Use damped pseudo-inverse only if near singularity
                Eigen::MatrixXd JTJ = J.transpose() * J;
                Eigen::MatrixXd damped_inverse = (JTJ + effective_damping * effective_damping * Eigen::MatrixXd::Identity(7, 7)).inverse();
                delta_q = damped_inverse * J.transpose() * pose_error;
            } else {
                // Pure Newton-Raphson: use Moore-Penrose pseudo-inverse
                // J^+ = J^T(JJ^T)^(-1) for over-determined system (6 equations, 7 unknowns)
                Eigen::MatrixXd JJT = J * J.transpose();
                
                // Add minimal damping for numerical stability only
                if (damping_factor > 0) {
                    JJT += damping_factor * damping_factor * Eigen::MatrixXd::Identity(6, 6);
                }
                
                delta_q = J.transpose() * JJT.inverse() * pose_error;
            }
        }
        
        // Adaptive step size
        double adaptive_step_size = computeAdaptiveStepSize(pose_error, delta_q, error_norm, previous_error_norm);
        result.step_sizes.push_back(adaptive_step_size);
        
        // Update joint angles
        result.joint_angles += adaptive_step_size * delta_q;
        
        // Enforce joint limits
        if (enforce_joint_limits) {
            result.joint_angles = enforceJointLimits(result.joint_angles);
        }
        
        // Check for convergence stalling
        if (adaptive_step_size < min_step_size) {
            break; // Converged or stalled
        }
        
        previous_error_norm = error_norm;
    }
    
    result.final_error = result.error_history.empty() ? 
        std::numeric_limits<double>::infinity() : result.error_history.back();
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.solve_time = duration.count() / 1000.0; // Convert to milliseconds
    
    return result;
}

std::pair<RobotArm, bool> NewtonRaphsonIK::solveIK(const Eigen::Affine3d& target_pose) {
    // Use current robot configuration as initial guess
    Eigen::Matrix<double, 7, 1> initial_guess = robot.getJointAngles();
    
    // Solve IK
    NewtonRaphsonIKResult result = solve(target_pose, initial_guess);
    
    // Create result robot arm
    RobotArm result_robot = robot;
    if (result.success) {
        result_robot.setJointAngles(result.joint_angles);
    }
    
    return std::make_pair(result_robot, result.success);
}

Eigen::Vector3d NewtonRaphsonIK::rotationMatrixToAxisAngle(const Eigen::Matrix3d& R) {
    // Convert rotation matrix to axis-angle representation
    Eigen::AngleAxisd axis_angle(R);
    return axis_angle.angle() * axis_angle.axis();
}

Eigen::Matrix<double, 6, 1> NewtonRaphsonIK::computePoseError(const Eigen::Affine3d& current_pose, 
                                                               const Eigen::Affine3d& target_pose) {
    Eigen::Matrix<double, 6, 1> error;
    
    // Position error
    error.head<3>() = target_pose.translation() - current_pose.translation();
    
    // Orientation error using axis-angle representation
    Eigen::Matrix3d rotation_error = target_pose.rotation() * current_pose.rotation().transpose();
    error.tail<3>() = rotationMatrixToAxisAngle(rotation_error);
    
    return error;
}

Eigen::Matrix<double, 7, 1> NewtonRaphsonIK::enforceJointLimits(const Eigen::Matrix<double, 7, 1>& joint_angles) {
    auto joint_limits = robot.jointLimits();
    Eigen::Matrix<double, 7, 1> clamped_angles = joint_angles;
    
    for (int i = 0; i < 7; ++i) {
        double lower_limit = joint_limits[i].first + joint_limit_margin;
        double upper_limit = joint_limits[i].second - joint_limit_margin;
        clamped_angles(i) = std::clamp(clamped_angles(i), lower_limit, upper_limit);
    }
    
    return clamped_angles;
}

double NewtonRaphsonIK::computeAdaptiveStepSize(const Eigen::Matrix<double, 6, 1>& error, 
                                                 const Eigen::Matrix<double, 7, 1>& delta_q,
                                                 double current_error_norm,
                                                 double previous_error_norm) {
    // Start with base step size
    double step_size = 1.0;
    
    // Reduce step size based on joint angle magnitude
    double max_joint_change = delta_q.cwiseAbs().maxCoeff();
    if (max_joint_change > max_step_size) {
        step_size = std::min(step_size, max_step_size / max_joint_change);
    }
    
    // Adaptive step size based on error improvement
    if (previous_error_norm < std::numeric_limits<double>::infinity()) {
        if (current_error_norm > previous_error_norm) {
            // Error increased - reduce step size
            step_size *= 0.5;
        } else if (current_error_norm < 0.8 * previous_error_norm) {
            // Good progress - might increase step size slightly
            step_size = std::min(step_size * 1.1, 1.0);
        }
    }
    
    // Ensure minimum step size
    step_size = std::max(step_size, min_step_size);
    
    return step_size;
}

double NewtonRaphsonIK::computeAdaptiveDamping(const Eigen::MatrixXd& J, double base_damping) {
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
