#pragma once

#include <Eigen/Dense>
#include "TrajectoryLib/Robot/RobotArm.h"
#include <vector>
#include <limits>

struct NewtonRaphsonIKResult {
    bool success = false;
    double solve_time = 0.0; // milliseconds
    double position_error = 0.0;
    double orientation_error = 0.0;
    double final_error = 0.0;
    int iterations = 0;
    Eigen::Matrix<double, 7, 1> joint_angles;
    std::vector<double> error_history;
    std::vector<double> step_sizes;
};

class NewtonRaphsonIK {
public:
    explicit NewtonRaphsonIK(RobotArm& robot_arm) 
        : robot(robot_arm) {}
    
    // Main solve function with multiple initial guess strategies
    NewtonRaphsonIKResult solve(const Eigen::Affine3d& target_pose, 
                               const Eigen::Matrix<double, 7, 1>& initial_guess);
    
    // Single solve attempt with given initial guess
    NewtonRaphsonIKResult solveSingle(const Eigen::Affine3d& target_pose, 
                                     const Eigen::Matrix<double, 7, 1>& initial_guess);
    
    // Legacy interface for compatibility
    std::pair<RobotArm, bool> solveIK(const Eigen::Affine3d& target_pose);
    
    // Configuration methods
    void setPositionTolerance(double tol) { position_tolerance = tol; }
    void setOrientationTolerance(double tol) { orientation_tolerance = tol; }
    void setMaxIterations(int max_iter) { max_iterations = max_iter; }
    void setDampingFactor(double damping) { damping_factor = damping; }
    void setAdaptiveDamping(bool adaptive) { adaptive_damping = adaptive; }
    void setEnforceJointLimits(bool enforce) { enforce_joint_limits = enforce; }
    void setMaxStepSize(double max_step) { max_step_size = max_step; }
    void setMinStepSize(double min_step) { min_step_size = min_step; }
    void setJointLimitMargin(double margin) { joint_limit_margin = margin; }
    void setUseDampedLeastSquares(bool use_dls) { use_damped_least_squares = use_dls; }

private:
    RobotArm& robot;
    
    // Solver parameters - tolerances adaptive to robot physical limits (Franka ±0.1mm reproducibility)
    double position_tolerance = std::numeric_limits<double>::quiet_NaN(); // NA - adaptive to robot reproducibility
    double orientation_tolerance = std::numeric_limits<double>::quiet_NaN(); // NA - adaptive to robot reproducibility
    int max_iterations = 1000;
    double damping_factor = 0.001; // Small damping for numerical stability
    bool adaptive_damping = false;
    bool enforce_joint_limits = true;
    double max_step_size = 0.1;
    double min_step_size = 1e-6;
    double joint_limit_margin = 0.01;
    bool use_damped_least_squares = false; // Default to Newton-Raphson method
    
    // Helper methods
    Eigen::Vector3d rotationMatrixToAxisAngle(const Eigen::Matrix3d& R);
    Eigen::Matrix<double, 6, 1> computePoseError(const Eigen::Affine3d& current_pose, 
                                                 const Eigen::Affine3d& target_pose);
    Eigen::Matrix<double, 7, 1> enforceJointLimits(const Eigen::Matrix<double, 7, 1>& joint_angles);
    double computeAdaptiveStepSize(const Eigen::Matrix<double, 6, 1>& error, 
                                   const Eigen::Matrix<double, 7, 1>& delta_q,
                                   double current_error_norm,
                                   double previous_error_norm);
    double computeAdaptiveDamping(const Eigen::MatrixXd& J, double base_damping);
};
