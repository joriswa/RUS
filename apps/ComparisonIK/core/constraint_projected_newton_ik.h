#pragma once

#include <Eigen/Dense>
#include "TrajectoryLib/Robot/RobotArm.h"
#include "TrajectoryLib/Planning/PathPlanner.h"
#include <vector>
#include <limits>

struct ConstraintProjectedNewtonIKResult {
    bool success = false;
    double solve_time = 0.0; // milliseconds
    double position_error = 0.0;
    double orientation_error = 0.0;
    double final_error = 0.0;
    double final_cost = 0.0;
    int iterations = 0;
    int projection_iterations = 0;
    Eigen::Matrix<double, 7, 1> joint_angles;
    std::vector<double> error_history;
    std::vector<double> cost_history;
    std::vector<double> step_sizes;
    std::vector<int> constraint_violations;
    std::vector<double> gradient_norms;  // Track gradient magnitudes
    std::vector<double> step_norms;      // Track actual step sizes taken
};

class ConstraintProjectedNewtonIK {
public:
    explicit ConstraintProjectedNewtonIK(RobotArm& robot_arm, PathPlanner* planner = nullptr) 
        : robot(robot_arm), path_planner(planner) {}
    
    // Main solve function with constraint projection
    ConstraintProjectedNewtonIKResult solve(const Eigen::Affine3d& target_pose, 
                                           const Eigen::Matrix<double, 7, 1>& initial_guess);
    
    // Legacy interface for compatibility with other IK solvers
    std::pair<RobotArm, bool> solveIK(const Eigen::Affine3d& target_pose);
    
    // Configuration methods
    void setPositionTolerance(double tol) { position_tolerance = tol; }
    void setOrientationTolerance(double tol) { orientation_tolerance = tol; }
    void setMaxIterations(int max_iter) { max_iterations = max_iter; }
    void setMaxProjectionIterations(int max_proj_iter) { max_projection_iterations = max_proj_iter; }
    void setDampingFactor(double damping) { damping_factor = damping; }
    void setJointLimitMargin(double margin) { joint_limit_margin = margin; }
    void setLineSearchEnabled(bool enabled) { use_line_search = enabled; }
    void setMaxStepSize(double max_step) { max_step_size = max_step; }
    void setMinStepSize(double min_step) { min_step_size = min_step; }
    void setProjectionTolerance(double tol) { projection_tolerance = tol; }
    void setKinematicOnlyMode(bool enabled) { kinematic_only_mode = enabled; } // New option
    void setCollisionAvoidanceOnlyMode(bool enabled) { collision_avoidance_only_mode = enabled; } // Focus only on collision avoidance
    void setUseAdaptiveDamping(bool enable) { use_adaptive_damping = enable; }

private:
    RobotArm& robot;
    PathPlanner* path_planner;
    
    // Solver parameters - aligned with Franka ±0.1mm repeatability
    double position_tolerance = 0.0001;     // 0.1mm tolerance (matches Franka repeatability)
    double orientation_tolerance = 0.001;   // ~0.057 degree tolerance (matches newton_raphson_ik.h)
    int max_iterations = 500;
    int max_projection_iterations = 10;
    double damping_factor = 0.001;
    double joint_limit_margin = 0.01;          // Margin from joint limits (rad)
    bool use_line_search = true;
    double max_step_size = 0.1;
    double min_step_size = 1e-6;
    double projection_tolerance = 1e-6;
    bool kinematic_only_mode = false; // New parameter for kinematic-only mode
    bool collision_avoidance_only_mode = false; // New parameter for collision-avoidance-only mode
    bool use_adaptive_damping = false; // Enable Levenberg-Marquardt adaptive damping
    double initial_damping = 0.1; // Initial damping factor for adaptive damping
    
    // Adaptive damping parameters (Levenberg-Marquardt style)
    double damping_increase_factor = 10.0;    // Factor to increase damping when cost increases
    double damping_decrease_factor = 0.1;     // Factor to decrease damping when cost decreases
    double min_damping = 1e-6;               // Minimum damping factor
    double max_damping = 1e3;                // Maximum damping factor
    
    // Core algorithm methods
    Eigen::Matrix<double, 7, 1> computeNewtonStep(const Eigen::Affine3d& target_pose,
                                                  const Eigen::Matrix<double, 7, 1>& current_q);
    
    // Compute gradient of total cost function using finite differences
    Eigen::Matrix<double, 7, 1> computeTotalCostGradient(const Eigen::Affine3d& target_pose,
                                                         const Eigen::Matrix<double, 7, 1>& current_q);
    
    // Compute Newton step based on total cost function (pose + collision + manipulability)
    Eigen::Matrix<double, 7, 1> computeTotalCostNewtonStep(const Eigen::Affine3d& target_pose,
                                                           const Eigen::Matrix<double, 7, 1>& current_q);
    
    // Compute hybrid Newton step (pose error primary + collision correction)
    Eigen::Matrix<double, 7, 1> computeHybridNewtonStep(const Eigen::Affine3d& target_pose,
                                                        const Eigen::Matrix<double, 7, 1>& current_q);
    
    // Adaptive damping methods (Levenberg-Marquardt style)
    Eigen::Matrix<double, 7, 1> computeAdaptiveDampedStep(const Eigen::Affine3d& target_pose,
                                                          const Eigen::Matrix<double, 7, 1>& current_q,
                                                          double& current_damping);
    void updateDamping(double& current_damping, double cost_improvement);
    
    Eigen::Matrix<double, 7, 1> projectOntoConstraints(const Eigen::Matrix<double, 7, 1>& q);
    
    double performLineSearch(const Eigen::Affine3d& target_pose,
                            const Eigen::Matrix<double, 7, 1>& current_q,
                            const Eigen::Matrix<double, 7, 1>& direction,
                            double initial_cost);
    
    double computeTotalCost(const Eigen::Affine3d& target_pose,
                           const Eigen::Matrix<double, 7, 1>& q);
    
    // Helper methods
    Eigen::Vector3d rotationMatrixToAxisAngle(const Eigen::Matrix3d& R);
    
    Eigen::Matrix<double, 6, 1> computePoseError(const Eigen::Affine3d& current_pose, 
                                                 const Eigen::Affine3d& target_pose);
    
    Eigen::Matrix<double, 7, 1> enforceJointLimits(const Eigen::Matrix<double, 7, 1>& joint_angles);
    
    double computeJointLimitDistance(const Eigen::Matrix<double, 7, 1>& q);
    
    // Constraint violation checking
    int countConstraintViolations(const Eigen::Matrix<double, 7, 1>& q);
};
