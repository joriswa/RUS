#pragma once

#include <Eigen/Dense>
#include "TrajectoryLib/Robot/RobotArm.h"
#include "ik_cost_functions.h"
#include <vector>
#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>
#include <future>
#include <thread>

// Forward declaration for PathPlanner
class PathPlanner;

struct GridSearchIKResult {
    bool success = false;
    double solve_time = 0.0; // milliseconds
    double position_error = 0.0;
    double orientation_error = 0.0;
    double final_error = 0.0;
    int iterations = 0; // Number of q7 values tested
    Eigen::Matrix<double, 7, 1> joint_angles;
    double best_q7 = 0.0; // The q7 value that gave the best solution
    double final_cost = 0.0; // Final cost of the best solution
    std::vector<double> error_history;
};

class GridSearchIK {
public:
    explicit GridSearchIK(RobotArm& robot_arm) 
        : robot(robot_arm), path_planner(nullptr) {}
    
    // Constructor with PathPlanner for collision checking
    GridSearchIK(RobotArm& robot_arm, PathPlanner* planner) 
        : robot(robot_arm), path_planner(planner) {}
    
    // Main solve function using grid search over q7
    GridSearchIKResult solve(const Eigen::Affine3d& target_pose, 
                            const Eigen::Matrix<double, 7, 1>& initial_guess);
    
    // Legacy interface for compatibility
    std::pair<RobotArm, bool> solveIK(const Eigen::Affine3d& target_pose);
    
    // Configuration methods
    void setPositionTolerance(double tol) { position_tolerance = tol; }
    void setOrientationTolerance(double tol) { orientation_tolerance = tol; }
    void setJointLimitMargin(double margin) { joint_limit_margin = margin; }
    void setUseEncoderPrecision(bool use_encoder) { use_encoder_precision = use_encoder; }
    void setPathPlanner(PathPlanner* planner) { path_planner = planner; }
    void setUseParallel(bool use_parallel) { use_parallel_processing = use_parallel; }
    void setNumThreads(size_t num_threads) { max_threads = num_threads; }

private:
    RobotArm& robot;
    PathPlanner* path_planner; // For collision checking and clearance computation
    
    // Solver parameters
    double position_tolerance = 1e-4;
    double orientation_tolerance = 1e-3;
    double joint_limit_margin = 0.01;
    bool use_encoder_precision = true; // Use 14-bit encoder precision for q7 grid
    
    // Parallel processing parameters
    bool use_parallel_processing = true; // Enable parallel grid search by default
    size_t max_threads = 0; // 0 means use hardware_concurrency()
    
    // Franka robot q7 joint limits
    static constexpr double q7_min = -2.8973;
    static constexpr double q7_max = 2.8973;
    static constexpr int encoder_bits = 14; // 14-bit encoder
    static constexpr int num_encoder_steps = (1 << encoder_bits); // 2^14 = 16384 steps
    
    // Helper methods
    Eigen::Vector3d rotationMatrixToAxisAngle(const Eigen::Matrix3d& R);
    Eigen::Matrix<double, 6, 1> computePoseError(const Eigen::Affine3d& current_pose, 
                                                 const Eigen::Affine3d& target_pose);
    Eigen::Matrix<double, 7, 1> enforceJointLimits(const Eigen::Matrix<double, 7, 1>& joint_angles);
    std::vector<double> generateQ7Grid();
    bool armHasCollision(RobotArm& arm);
    
    // Parallel processing helper structures and methods
    struct LocalSearchResult {
        bool has_valid_solution = false;
        Eigen::Matrix<double, 7, 1> best_solution;
        double best_cost = std::numeric_limits<double>::infinity();
        double best_q7 = 0.0;
        int iterations = 0;
    };
    
    // Process a chunk of q7 values (thread worker function)
    LocalSearchResult processQ7Chunk(const std::vector<double>& q7_chunk,
                                     const Eigen::Affine3d& total_transform,
                                     const Eigen::Affine3d& target_pose,
                                     const std::array<double, 7>& joint_angles_array,
                                     double max_position_error,
                                     double max_orientation_error) const;
};
