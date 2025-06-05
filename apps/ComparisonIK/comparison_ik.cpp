#include "USLib/USTrajectoryPlanner.h"
#include <Eigen/Dense>
#include <TinyURDFParser/KDLRobot.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <sstream>
#include <string>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <random>

// Timer class for measuring execution time
class Timer
{
private:
    using clock_t = std::chrono::high_resolution_clock;
    using second_t = std::chrono::duration<double, std::ratio<1>>;
    std::chrono::time_point<clock_t> m_start;

public:
    Timer()
        : m_start(clock_t::now())
    {}
    void reset() { m_start = clock_t::now(); }
    double elapsed() const
    {
        return std::chrono::duration_cast<second_t>(clock_t::now() - m_start).count();
    }
};

std::vector<Eigen::Matrix4d> readPosesFromCSV(const std::string &filename)
{
    std::vector<Eigen::Matrix4d> poses;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        // Skip empty lines and comment lines
        if (line.empty() || line[0] == '#' || line[0] == '/' || line.find("//") != std::string::npos) {
            continue;
        }
        
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;

        while (std::getline(ss, token, ','))
            values.push_back(std::stod(token));

        if (values.size() < 7)
            continue;

        double x = values[0], y = values[1], z = values[2];
        Eigen::Quaterniond q(values[3], values[4], values[5], values[6]);
        q.normalize();

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

        const Eigen::Vector3d local_move(0.0, 0.0, -0.02);
        T.block<3, 1>(0, 3) += T.block<3, 3>(0, 0) * local_move;

        poses.push_back(T);
    }
    return poses;
}

bool hasCollision(const Eigen::VectorXd &joint_positions, PathPlanner *planner)
{
    auto arm = planner->getArm();
    arm.setJointAngles(joint_positions);

    return planner->armHasCollision(arm);
}

// Structure to hold multiple trial results for variance analysis
struct VarianceAnalysisResult {
    std::vector<double> execution_times;
    std::vector<double> collision_check_times;
    std::vector<double> clearance_compute_times;
    std::vector<bool> successes;
    std::vector<bool> collision_free_results;
    std::vector<double> position_errors;
    std::vector<double> orientation_errors;
    std::vector<double> min_clearances;
    std::vector<double> avg_clearances;
    std::vector<double> weighted_clearances;
    std::vector<int> num_links_checked_results;
    
    double mean_time = 0.0;
    double std_dev_time = 0.0;
    double min_time = 0.0;
    double max_time = 0.0;
    double median_time = 0.0;
    double coefficient_of_variation = 0.0;  // std_dev / mean
    
    // Additional statistics fields for CSV export
    double mean_execution_time = 0.0;
    double std_dev_execution_time = 0.0;
    double median_execution_time = 0.0;
    double min_execution_time = 0.0;
    double max_execution_time = 0.0;
    
    int num_trials = 0;
    int successful_trials = 0;
    double success_rate = 0.0;
};

// Structure to hold results for one IK method on one pose
struct IKMethodResult {
    bool success = false;
    bool collision_free = false;
    double solve_time = 0.0;
    double collision_check_time = 0.0;
    double position_error = -1.0;
    double orientation_error = -1.0;
    Eigen::VectorXd solution;
    int return_code = -1;
    std::string method_name;
    int pose_index = -1;  // Added pose index field
    
    // Enhanced clearance metrics
    double min_clearance = -1.0;          // Minimum distance to obstacles
    double avg_clearance = -1.0;          // Average clearance across all links
    double weighted_clearance = -1.0;     // Volume-weighted clearance
    double max_clearance = -1.0;          // Maximum clearance across all links
    double std_dev_clearance = -1.0;      // Standard deviation of clearances
    double clearance_range = -1.0;        // Range (max - min) of clearances
    std::vector<double> individual_link_clearances;  // Per-link clearance values
    std::vector<std::string> link_names;             // Names of links checked
    bool has_clearance_violation = false; // Whether any link has clearance below threshold
    double clearance_violation_threshold = 0.05;    // 5cm default threshold
    int num_links_checked = 0;            // Number of links that were analyzed
    double clearance_compute_time = 0.0;  // Time spent computing clearance
    
    // Orientation noise metrics (for robustness testing)
    double orientation_noise_magnitude = 0.0;  // Applied noise magnitude (radians)
    Eigen::Vector3d applied_noise_vector;       // Actual noise vector applied
    double noise_robustness_score = 1.0;        // Success rate under noise
    
    // Variance analysis data
    VarianceAnalysisResult variance_analysis;
};

// Structure to hold comparison results for both methods on one pose
struct PoseComparisonResult {
    Eigen::Matrix4d target_pose;
    IKMethodResult kdl_result;
    IKMethodResult selectgoalpose_result;
    int pose_index;
};

// Forward declarations
struct IKMethodResult;
struct PoseComparisonResult;
void printEnhancedStatistics(const std::vector<double>& values, const std::string& metric_name);
void exportResultsToCSV(const std::vector<PoseComparisonResult>& results, const std::string& filename);
void exportVarianceResultsToCSV(const std::vector<PoseComparisonResult>& results, const std::string& filename);
void calculateVarianceAnalysis(VarianceAnalysisResult& analysis);

// Helper function to compute enhanced clearance metrics for a joint configuration
void computeClearanceMetrics(const Eigen::VectorXd &joint_positions, 
                           PathPlanner *planner, 
                           IKMethodResult &result)
{
    Timer clearance_timer;
    
    auto arm = planner->getArm();
    arm.setJointAngles(joint_positions);
    
    auto clearance_metrics = planner->computeArmClearance(arm);
    
    // Basic clearance metrics
    result.min_clearance = clearance_metrics.min_clearance;
    result.avg_clearance = clearance_metrics.avg_clearance;
    result.weighted_clearance = clearance_metrics.weighted_clearance;
    result.num_links_checked = clearance_metrics.num_links_checked;
    
    // Enhanced clearance metrics from individual link data
    result.individual_link_clearances = clearance_metrics.link_clearances;
    
    if (!clearance_metrics.link_clearances.empty()) {
        // Compute additional statistical measures
        double sum = 0.0;
        double min_val = std::numeric_limits<double>::max();
        double max_val = std::numeric_limits<double>::lowest();
        
        for (double clearance : clearance_metrics.link_clearances) {
            sum += clearance;
            min_val = std::min(min_val, clearance);
            max_val = std::max(max_val, clearance);
            
            // Check for clearance violations
            if (clearance < result.clearance_violation_threshold) {
                result.has_clearance_violation = true;
            }
        }
        
        result.min_clearance = min_val;
        result.max_clearance = max_val;
        result.clearance_range = max_val - min_val;
        
        // Compute standard deviation
        double mean = sum / clearance_metrics.link_clearances.size();
        double variance = 0.0;
        for (double clearance : clearance_metrics.link_clearances) {
            variance += (clearance - mean) * (clearance - mean);
        }
        result.std_dev_clearance = std::sqrt(variance / clearance_metrics.link_clearances.size());
        
        // Generate link names (simplified - could be enhanced with actual link names)
        result.link_names.clear();
        for (int i = 0; i < static_cast<int>(clearance_metrics.link_clearances.size()); ++i) {
            result.link_names.push_back("link_" + std::to_string(i + 2)); // Starting from link 2
        }
    } else {
        result.max_clearance = -1.0;
        result.std_dev_clearance = -1.0;
        result.clearance_range = -1.0;
        result.has_clearance_violation = false;
    }
    
    result.clearance_compute_time = clearance_timer.elapsed();
}

// Helper function to apply orientation noise to a target pose
Eigen::Matrix4d applyOrientationNoise(const Eigen::Matrix4d& original_pose, 
                                     double noise_magnitude, 
                                     Eigen::Vector3d& applied_noise_vector) {
    if (noise_magnitude <= 0.0) {
        applied_noise_vector = Eigen::Vector3d::Zero();
        return original_pose;
    }
    
    // Generate random noise vector with specified magnitude
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0.0, 1.0);
    
    // Generate random unit vector
    Eigen::Vector3d noise_direction;
    noise_direction << dist(gen), dist(gen), dist(gen);
    noise_direction.normalize();
    
    // Scale to desired magnitude
    applied_noise_vector = noise_direction * noise_magnitude;
    
    // Create rotation matrix from noise vector (axis-angle representation)
    double angle = applied_noise_vector.norm();
    Eigen::Vector3d axis = applied_noise_vector.normalized();
    
    Eigen::Matrix3d noise_rotation;
    if (angle > 1e-6) {
        noise_rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    } else {
        noise_rotation = Eigen::Matrix3d::Identity();
    }
    
    // Apply noise to original orientation
    Eigen::Matrix4d noisy_pose = original_pose;
    noisy_pose.block<3,3>(0,0) = noise_rotation * original_pose.block<3,3>(0,0);
    
    return noisy_pose;
}

double quaternionDistance(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2)
{
    Eigen::Quaterniond q1_norm = q1.normalized();
    Eigen::Quaterniond q2_norm = q2.normalized();
    double dot = q1_norm.w() * q2_norm.w() + q1_norm.x() * q2_norm.x() + q1_norm.y() * q2_norm.y()
                 + q1_norm.z() * q2_norm.z();
    dot = std::max(-1.0, std::min(1.0, dot));
    return 2.0 * std::acos(std::abs(dot));
}

// Numerical IK fallback using Jacobian pseudoinverse
bool numericalIK(tup::sim::KDLRobot &robot,
                 const Eigen::Vector3d &target_position,
                 const Eigen::Quaterniond &target_orientation,
                 Eigen::VectorXd &solution,
                 int max_iterations = 100,
                 double tolerance = 1e-6,
                 double lambda = 0.01)
{
    Eigen::VectorXd q = robot.getJointsPos();
    solution = q;

    for (int iter = 0; iter < max_iterations; ++iter) {
        Eigen::Vector3d current_pos = robot.getEEPosition();
        Eigen::Quaterniond current_orn(robot.getEEOrnQuat()[0],
                                       robot.getEEOrnQuat()[1],
                                       robot.getEEOrnQuat()[2],
                                       robot.getEEOrnQuat()[3]);

        Eigen::Vector3d pos_error = target_position - current_pos;
        double orn_error = quaternionDistance(target_orientation, current_orn);

        if (pos_error.norm() < tolerance && orn_error < tolerance)
            return true;

        // NOTE: You need to implement robot.getJacobian(J) if not available!
        Eigen::MatrixXd J;
        // robot.getJacobian(J); // Uncomment and implement if available

        // If Jacobian is not available, skip this fallback
        if (J.rows() == 0 || J.cols() == 0) {
            std::cerr << "Jacobian not available; skipping numerical IK fallback." << std::endl;
            return false;
        }

        // Damped pseudoinverse: (J^T * J + lambda^2 * I)^(-1) * J^T
        Eigen::MatrixXd J_pinv;
        if (J.rows() >= J.cols()) {
            J_pinv = (J.transpose() * J
                      + lambda * lambda * Eigen::MatrixXd::Identity(J.cols(), J.cols()))
                         .inverse()
                     * J.transpose();
        } else {
            J_pinv = J.transpose()
                     * (J * J.transpose()
                        + lambda * lambda * Eigen::MatrixXd::Identity(J.rows(), J.rows()))
                           .inverse();
        }

        // Task-space error (position only for simplicity)
        Eigen::VectorXd error(3);
        error << pos_error.x(), pos_error.y(), pos_error.z();
        Eigen::VectorXd dq = J_pinv * error;
        q += dq;
        robot.setConfiguration(q, Eigen::VectorXd::Zero(q.size()), false);
    }
    return false;
}

// Forward declaration for selectGoalPose method wrapper
IKMethodResult solveWithSelectGoalPose(const Eigen::Matrix4d& target_pose, 
                                     PathPlanner* planner, 
                                     tup::sim::KDLRobot& kdl_robot);

// Implementation of selectGoalPose wrapper
IKMethodResult solveWithSelectGoalPose(const Eigen::Matrix4d& target_pose, 
                                     PathPlanner* planner, 
                                     tup::sim::KDLRobot& kdl_robot) {
    IKMethodResult result;
    result.method_name = "selectGoalPose";
    
    Timer solve_timer;
    auto selectgoal_result = planner->selectGoalPose(Eigen::Affine3d(target_pose));
    result.solve_time = solve_timer.elapsed();
    
    result.success = selectgoal_result.second;  // second element is the success boolean
    
    std::cout << "selectGoalPose solve time: " << std::fixed << std::setprecision(6) << result.solve_time << " seconds" << std::endl;
    std::cout << "selectGoalPose result: " << (result.success ? "SUCCESS" : "FAILED") << std::endl;
    
    if (result.success) {
        RobotArm solution_arm = selectgoal_result.first;  // first element is the RobotArm
        Eigen::VectorXd sgp_solution = solution_arm.getJointAngles();
        
        Timer collision_timer;
        bool sgp_collision_free = !hasCollision(sgp_solution, planner);
        result.collision_check_time = collision_timer.elapsed();
        result.collision_free = sgp_collision_free;
        
        std::cout << "selectGoalPose collision check time: " << result.collision_check_time << " seconds" << std::endl;
        std::cout << "selectGoalPose collision status: " << (sgp_collision_free ? "COLLISION-FREE" : "HAS COLLISIONS") << std::endl;
        
        if (sgp_collision_free) {
            result.solution = sgp_solution;
            std::cout << "selectGoalPose solution: " << sgp_solution.transpose() << std::endl;
            
            // Compute clearance metrics
            computeClearanceMetrics(sgp_solution, planner, result);
            std::cout << "selectGoalPose clearance metrics:" << std::endl;
            std::cout << "  - Min clearance: " << result.min_clearance << " meters" << std::endl;
            std::cout << "  - Avg clearance: " << result.avg_clearance << " meters" << std::endl;
            std::cout << "  - Weighted clearance: " << result.weighted_clearance << " meters" << std::endl;
            std::cout << "  - Links checked: " << result.num_links_checked << std::endl;
            std::cout << "  - Clearance compute time: " << result.clearance_compute_time << " seconds" << std::endl;
            
            // Calculate accuracy by setting the robot configuration and measuring error
            kdl_robot.setConfiguration(sgp_solution, Eigen::VectorXd::Zero(sgp_solution.size()), false);
            
            Eigen::Vector3d actual_position = kdl_robot.getEEPosition();
            Eigen::Quaterniond actual_orientation(kdl_robot.getEEOrnQuat()[0],
                                                  kdl_robot.getEEOrnQuat()[1],
                                                  kdl_robot.getEEOrnQuat()[2],
                                                  kdl_robot.getEEOrnQuat()[3]);
            
            Eigen::Vector3d target_position(target_pose.block<3, 1>(0, 3));
            Eigen::Quaterniond target_orientation(target_pose.block<3, 3>(0, 0));
            target_orientation.normalize();
            
            result.position_error = (target_position - actual_position).norm();
            result.orientation_error = quaternionDistance(target_orientation, actual_orientation);
            
            std::cout << "selectGoalPose position error: " << result.position_error << " meters" << std::endl;
            std::cout << "selectGoalPose orientation error: " << result.orientation_error << " radians ("
                      << (result.orientation_error * 180.0 / M_PI) << " degrees)" << std::endl;
        } else {
            std::cout << "selectGoalPose solution found but has collisions" << std::endl;
        }
    } else {
        std::cout << "selectGoalPose failed to find IK solution" << std::endl;
    }
    
    return result;
}

// Enhanced selectGoalPose wrapper with multiple trials for variance analysis and orientation noise testing
IKMethodResult solveWithSelectGoalPoseVariance(const Eigen::Matrix4d& target_pose, 
                                              PathPlanner* planner, 
                                              tup::sim::KDLRobot& kdl_robot,
                                              int pose_index,
                                              int num_trials = 10,
                                              double orientation_noise_magnitude = 0.0) {
    IKMethodResult result;
    result.method_name = "selectGoalPose";
    result.pose_index = pose_index;
    result.orientation_noise_magnitude = orientation_noise_magnitude;
    
    std::string analysis_type = (orientation_noise_magnitude > 0.0) ? 
                               "with orientation noise" : "variance analysis";
    
    std::cout << "Running selectGoalPose with " << num_trials << " trials for " << analysis_type;
    if (orientation_noise_magnitude > 0.0) {
        std::cout << " (noise magnitude: " << orientation_noise_magnitude << " radians)";
    }
    std::cout << "..." << std::endl;
    
    // Collect timing and result data from multiple trials
    std::vector<double> execution_times, collision_check_times, clearance_compute_times;
    std::vector<bool> successes, collision_free_results;
    std::vector<double> position_errors, orientation_errors;
    std::vector<double> min_clearances, avg_clearances, weighted_clearances;
    std::vector<double> max_clearances, std_dev_clearances, clearance_ranges;
    std::vector<int> num_links_checked_results;
    std::vector<bool> clearance_violations;
    
    int successful_attempts = 0;
    int noise_robust_successes = 0;
    bool first_success = false;
    
    for (int trial = 0; trial < num_trials; ++trial) {
        // Apply orientation noise if specified
        Eigen::Matrix4d test_pose = target_pose;
        Eigen::Vector3d applied_noise;
        
        if (orientation_noise_magnitude > 0.0) {
            test_pose = applyOrientationNoise(target_pose, orientation_noise_magnitude, applied_noise);
            
            if (trial == 0) {
                result.applied_noise_vector = applied_noise;
            }
        }
        
        Timer solve_timer;
        auto selectgoal_result = planner->selectGoalPose(Eigen::Affine3d(test_pose));
        double trial_time = solve_timer.elapsed();
        
        // Always record trial data (success or failure)
        execution_times.push_back(trial_time);
        successes.push_back(selectgoal_result.second);
        
        if (selectgoal_result.second) { // Success
            successful_attempts++;
            if (orientation_noise_magnitude > 0.0) {
                noise_robust_successes++;
            }
            
            RobotArm solution_arm = selectgoal_result.first;
            Eigen::VectorXd sgp_solution = solution_arm.getJointAngles();
            
            Timer collision_timer;
            bool sgp_collision_free = !hasCollision(sgp_solution, planner);
            double trial_collision_time = collision_timer.elapsed();
            collision_check_times.push_back(trial_collision_time);
            collision_free_results.push_back(sgp_collision_free);
            
            if (sgp_collision_free) {
                // Compute enhanced clearance metrics for this trial
                Timer clearance_timer;
                auto arm = planner->getArm();
                arm.setJointAngles(sgp_solution);
                auto clearance_metrics = planner->computeArmClearance(arm);
                double trial_clearance_time = clearance_timer.elapsed();
                
                clearance_compute_times.push_back(trial_clearance_time);
                min_clearances.push_back(clearance_metrics.min_clearance);
                avg_clearances.push_back(clearance_metrics.avg_clearance);
                weighted_clearances.push_back(clearance_metrics.weighted_clearance);
                num_links_checked_results.push_back(clearance_metrics.num_links_checked);
                
                // Compute enhanced clearance statistics
                if (!clearance_metrics.link_clearances.empty()) {
                    double min_val = *std::min_element(clearance_metrics.link_clearances.begin(), 
                                                      clearance_metrics.link_clearances.end());
                    double max_val = *std::max_element(clearance_metrics.link_clearances.begin(), 
                                                      clearance_metrics.link_clearances.end());
                    
                    max_clearances.push_back(max_val);
                    clearance_ranges.push_back(max_val - min_val);
                    
                    // Compute standard deviation of clearances
                    double mean_clearance = clearance_metrics.avg_clearance;
                    double variance = 0.0;
                    for (double clearance : clearance_metrics.link_clearances) {
                        variance += (clearance - mean_clearance) * (clearance - mean_clearance);
                    }
                    std_dev_clearances.push_back(std::sqrt(variance / clearance_metrics.link_clearances.size()));
                    
                    // Check for clearance violations
                    bool has_violation = false;
                    for (double clearance : clearance_metrics.link_clearances) {
                        if (clearance < 0.05) { // 5cm threshold
                            has_violation = true;
                            break;
                        }
                    }
                    clearance_violations.push_back(has_violation);
                } else {
                    max_clearances.push_back(-1.0);
                    clearance_ranges.push_back(-1.0);
                    std_dev_clearances.push_back(-1.0);
                    clearance_violations.push_back(false);
                }
                
                // Calculate accuracy by setting the robot configuration and measuring error
                kdl_robot.setConfiguration(sgp_solution, Eigen::VectorXd::Zero(sgp_solution.size()), false);
                
                Eigen::Vector3d actual_position = kdl_robot.getEEPosition();
                Eigen::Quaterniond actual_orientation(kdl_robot.getEEOrnQuat()[0],
                                                      kdl_robot.getEEOrnQuat()[1],
                                                      kdl_robot.getEEOrnQuat()[2],
                                                      kdl_robot.getEEOrnQuat()[3]);
                
                // Calculate error relative to the original target pose (not the noisy one)
                Eigen::Vector3d target_position(target_pose.block<3, 1>(0, 3));
                Eigen::Quaterniond target_orientation(target_pose.block<3, 3>(0, 0));
                target_orientation.normalize();
                
                double pos_error = (target_position - actual_position).norm();
                double orn_error = quaternionDistance(target_orientation, actual_orientation);
                position_errors.push_back(pos_error);
                orientation_errors.push_back(orn_error);
                
                // Store first successful result for summary data
                if (!first_success) {
                    first_success = true;
                    result.success = true;
                    result.solve_time = trial_time;
                    result.collision_check_time = trial_collision_time;
                    result.collision_free = sgp_collision_free;
                    result.solution = sgp_solution;
                    result.min_clearance = clearance_metrics.min_clearance;
                    result.avg_clearance = clearance_metrics.avg_clearance;
                    result.weighted_clearance = clearance_metrics.weighted_clearance;
                    result.num_links_checked = clearance_metrics.num_links_checked;
                    result.clearance_compute_time = trial_clearance_time;
                    result.position_error = pos_error;
                    result.orientation_error = orn_error;
                    
                    // Enhanced clearance metrics for first successful result
                    if (!clearance_metrics.link_clearances.empty()) {
                        result.individual_link_clearances = clearance_metrics.link_clearances;
                        result.max_clearance = *std::max_element(clearance_metrics.link_clearances.begin(), 
                                                                clearance_metrics.link_clearances.end());
                        result.clearance_range = result.max_clearance - result.min_clearance;
                        
                        double variance = 0.0;
                        for (double clearance : clearance_metrics.link_clearances) {
                            variance += (clearance - result.avg_clearance) * (clearance - result.avg_clearance);
                        }
                        result.std_dev_clearance = std::sqrt(variance / clearance_metrics.link_clearances.size());
                        
                        for (double clearance : clearance_metrics.link_clearances) {
                            if (clearance < result.clearance_violation_threshold) {
                                result.has_clearance_violation = true;
                                break;
                            }
                        }
                    }
                }
            } else {
                // No collision-free solution for this trial
                clearance_compute_times.push_back(0.0);
                min_clearances.push_back(-1.0);
                avg_clearances.push_back(-1.0);
                weighted_clearances.push_back(-1.0);
                max_clearances.push_back(-1.0);
                clearance_ranges.push_back(-1.0);
                std_dev_clearances.push_back(-1.0);
                num_links_checked_results.push_back(0);
                clearance_violations.push_back(false);
                position_errors.push_back(-1.0);
                orientation_errors.push_back(-1.0);
            }
        } else {
            // Failed trial - add default values
            collision_check_times.push_back(0.0);
            collision_free_results.push_back(false);
            clearance_compute_times.push_back(0.0);
            min_clearances.push_back(-1.0);
            avg_clearances.push_back(-1.0);
            weighted_clearances.push_back(-1.0);
            max_clearances.push_back(-1.0);
            clearance_ranges.push_back(-1.0);
            std_dev_clearances.push_back(-1.0);
            num_links_checked_results.push_back(0);
            clearance_violations.push_back(false);
            position_errors.push_back(-1.0);
            orientation_errors.push_back(-1.0);
        }
        
        // Progress indicator
        if ((trial + 1) % 5 == 0 || trial == num_trials - 1) {
            std::cout << "  Completed " << (trial + 1) << "/" << num_trials << " trials (" 
                      << successful_attempts << " successful)" << std::endl;
        }
    }
    
    // Calculate noise robustness score
    if (orientation_noise_magnitude > 0.0) {
        result.noise_robustness_score = static_cast<double>(noise_robust_successes) / num_trials;
    }
    
    // Store all variance analysis trial data
    result.variance_analysis.execution_times = execution_times;
    result.variance_analysis.collision_check_times = collision_check_times;
    result.variance_analysis.clearance_compute_times = clearance_compute_times;
    result.variance_analysis.successes = successes;
    result.variance_analysis.collision_free_results = collision_free_results;
    result.variance_analysis.position_errors = position_errors;
    result.variance_analysis.orientation_errors = orientation_errors;
    result.variance_analysis.min_clearances = min_clearances;
    result.variance_analysis.avg_clearances = avg_clearances;
    result.variance_analysis.weighted_clearances = weighted_clearances;
    result.variance_analysis.num_links_checked_results = num_links_checked_results;
    
    result.variance_analysis.num_trials = num_trials;
    result.variance_analysis.successful_trials = successful_attempts;
    result.variance_analysis.success_rate = (100.0 * successful_attempts) / num_trials;
    
    // Calculate variance statistics
    calculateVarianceAnalysis(result.variance_analysis);
    
    // Print variance analysis summary
    std::cout << "\nVariance Analysis Results:" << std::endl;
    std::cout << "  Total trials: " << result.variance_analysis.num_trials << std::endl;
    std::cout << "  Successful trials: " << result.variance_analysis.successful_trials 
              << " (" << std::fixed << std::setprecision(1) << result.variance_analysis.success_rate << "%)" << std::endl;
    
    if (orientation_noise_magnitude > 0.0) {
        std::cout << "  Noise robustness score: " << std::fixed << std::setprecision(3) 
                  << result.noise_robustness_score << std::endl;
        std::cout << "  Applied noise magnitude: " << orientation_noise_magnitude << " radians ("
                  << (orientation_noise_magnitude * 180.0 / M_PI) << " degrees)" << std::endl;
    }
    
    if (!execution_times.empty()) {
        std::cout << "  Mean execution time: " << std::scientific << std::setprecision(4) 
                  << result.variance_analysis.mean_time << " seconds" << std::endl;
        std::cout << "  Standard deviation: " << std::scientific << std::setprecision(4) 
                  << result.variance_analysis.std_dev_time << " seconds" << std::endl;
        std::cout << "  Coefficient of variation: " << std::fixed << std::setprecision(2) 
                  << (result.variance_analysis.coefficient_of_variation * 100.0) << "%" << std::endl;
    } else {
        std::cout << "  No successful executions for timing analysis" << std::endl;
        result.success = false;
    }
    
    // Enhanced clearance statistics reporting
    if (!min_clearances.empty()) {
        auto valid_clearances = min_clearances;
        valid_clearances.erase(std::remove_if(valid_clearances.begin(), valid_clearances.end(),
                                            [](double c) { return c < 0; }), valid_clearances.end());
        
        if (!valid_clearances.empty()) {
            double mean_clearance = std::accumulate(valid_clearances.begin(), valid_clearances.end(), 0.0) / valid_clearances.size();
            double min_clearance = *std::min_element(valid_clearances.begin(), valid_clearances.end());
            double max_clearance = *std::max_element(valid_clearances.begin(), valid_clearances.end());
            
            std::cout << "  Clearance statistics:" << std::endl;
            std::cout << "    Mean min clearance: " << std::fixed << std::setprecision(4) << mean_clearance << "m" << std::endl;
            std::cout << "    Range: " << min_clearance << "m to " << max_clearance << "m" << std::endl;
            
            // Count clearance violations
            int violations = std::count(clearance_violations.begin(), clearance_violations.end(), true);
            if (violations > 0) {
                std::cout << "    Clearance violations: " << violations << "/" << clearance_violations.size() << " trials" << std::endl;
            }
        }
    }
    
    return result;
}

void calculateVarianceAnalysis(VarianceAnalysisResult& analysis) {
    if (analysis.execution_times.empty()) {
        analysis.num_trials = 0;
        analysis.successful_trials = 0;
        analysis.success_rate = 0.0;
        return;
    }
    
    // Extract only successful execution times for statistics
    std::vector<double> successful_times;
    for (size_t i = 0; i < analysis.execution_times.size() && i < analysis.successes.size(); ++i) {
        if (analysis.successes[i]) {
            successful_times.push_back(analysis.execution_times[i]);
        }
    }
    
    if (successful_times.empty()) {
        analysis.mean_time = 0.0;
        analysis.std_dev_time = 0.0;
        analysis.min_time = 0.0;
        analysis.max_time = 0.0;
        analysis.median_time = 0.0;
        analysis.coefficient_of_variation = 0.0;
        return;
    }
    
    // Calculate basic statistics on successful runs only
    std::vector<double> sorted_times = successful_times;
    std::sort(sorted_times.begin(), sorted_times.end());
    
    analysis.mean_time = std::accumulate(successful_times.begin(), successful_times.end(), 0.0) / successful_times.size();
    analysis.min_time = sorted_times.front();
    analysis.max_time = sorted_times.back();
    analysis.median_time = sorted_times[sorted_times.size() / 2];
    
    // Calculate standard deviation
    double variance = 0.0;
    for (double time : successful_times) {
        variance += (time - analysis.mean_time) * (time - analysis.mean_time);
    }
    variance /= successful_times.size();
    analysis.std_dev_time = std::sqrt(variance);
    
    // Calculate coefficient of variation (relative variability)
    analysis.coefficient_of_variation = analysis.mean_time > 0 ? (analysis.std_dev_time / analysis.mean_time) : 0.0;
    
    // Set the additional fields for CSV export
    analysis.mean_execution_time = analysis.mean_time;
    analysis.std_dev_execution_time = analysis.std_dev_time;
    analysis.median_execution_time = analysis.median_time;
    analysis.min_execution_time = analysis.min_time;
    analysis.max_execution_time = analysis.max_time;
}

// Enhanced statistical analysis function
void printEnhancedStatistics(const std::vector<double>& values, const std::string& metric_name) {
    if (values.empty()) return;
    
    std::vector<double> sorted_values = values;
    std::sort(sorted_values.begin(), sorted_values.end());
    
    double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    double median = sorted_values[sorted_values.size() / 2];
    double min_val = sorted_values.front();
    double max_val = sorted_values.back();
    
    // Calculate standard deviation
    double variance = 0.0;
    for (double val : values) {
        variance += (val - mean) * (val - mean);
    }
    variance /= values.size();
    double std_dev = std::sqrt(variance);
    
    // Calculate percentiles
    double p25 = sorted_values[sorted_values.size() / 4];
    double p75 = sorted_values[3 * sorted_values.size() / 4];
    double p95 = sorted_values[95 * sorted_values.size() / 100];
    
    std::cout << "\n" << metric_name << " Statistics:" << std::endl;
    std::cout << "  Mean:         " << std::scientific << std::setprecision(4) << mean << std::endl;
    std::cout << "  Median:       " << std::scientific << std::setprecision(4) << median << std::endl;
    std::cout << "  Std Dev:      " << std::scientific << std::setprecision(4) << std_dev << std::endl;
    std::cout << "  Min:          " << std::scientific << std::setprecision(4) << min_val << std::endl;
    std::cout << "  Max:          " << std::scientific << std::setprecision(4) << max_val << std::endl;
    std::cout << "  25th percentile: " << std::scientific << std::setprecision(4) << p25 << std::endl;
    std::cout << "  75th percentile: " << std::scientific << std::setprecision(4) << p75 << std::endl;
    std::cout << "  95th percentile: " << std::scientific << std::setprecision(4) << p95 << std::endl;
}

// Function to export results to CSV file
void exportResultsToCSV(const std::vector<PoseComparisonResult>& results, const std::string& filename) {
    std::ofstream csv_file(filename);
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open CSV file for writing: " << filename << std::endl;
        return;
    }
    
    // Write header
    csv_file << "pose_index,kdl_success,kdl_collision_free,kdl_solve_time,kdl_collision_check_time,kdl_clearance_compute_time,"
             << "kdl_position_error,kdl_orientation_error,kdl_min_clearance,kdl_avg_clearance,kdl_weighted_clearance,kdl_num_links_checked,"
             << "sgp_success,sgp_collision_free,sgp_solve_time,sgp_collision_check_time,sgp_clearance_compute_time,"
             << "sgp_position_error,sgp_orientation_error,sgp_min_clearance,sgp_avg_clearance,sgp_weighted_clearance,sgp_num_links_checked,"
             << "speed_ratio,clearance_ratio,winner\n";
    
    // Write data rows
    for (const auto& result : results) {
        csv_file << result.pose_index << ","
                 << (result.kdl_result.success ? 1 : 0) << ","
                 << (result.kdl_result.collision_free ? 1 : 0) << ","
                 << std::scientific << std::setprecision(6) << result.kdl_result.solve_time << ","
                 << result.kdl_result.collision_check_time << ","
                 << result.kdl_result.clearance_compute_time << ","
                 << result.kdl_result.position_error << ","
                 << result.kdl_result.orientation_error << ","
                 << result.kdl_result.min_clearance << ","
                 << result.kdl_result.avg_clearance << ","
                 << result.kdl_result.weighted_clearance << ","
                 << result.kdl_result.num_links_checked << ","
                 << (result.selectgoalpose_result.success ? 1 : 0) << ","
                 << (result.selectgoalpose_result.collision_free ? 1 : 0) << ","
                 << result.selectgoalpose_result.solve_time << ","
                 << result.selectgoalpose_result.collision_check_time << ","
                 << result.selectgoalpose_result.clearance_compute_time << ","
                 << result.selectgoalpose_result.position_error << ","
                 << result.selectgoalpose_result.orientation_error << ","
                 << result.selectgoalpose_result.min_clearance << ","
                 << result.selectgoalpose_result.avg_clearance << ","
                 << result.selectgoalpose_result.weighted_clearance << ","
                 << result.selectgoalpose_result.num_links_checked << ",";
        
        // Calculate speed ratio
        if (result.selectgoalpose_result.solve_time > 0) {
            csv_file << (result.kdl_result.solve_time / result.selectgoalpose_result.solve_time);
        } else {
            csv_file << "inf";
        }
        csv_file << ",";
        
        // Calculate clearance ratio
        if (result.selectgoalpose_result.min_clearance > 0 && result.kdl_result.min_clearance > 0) {
            csv_file << (result.kdl_result.min_clearance / result.selectgoalpose_result.min_clearance);
        } else {
            csv_file << "N/A";
        }
        csv_file << ",";
        
        // Determine winner
        std::string winner = "TIE";
        if (result.kdl_result.collision_free && !result.selectgoalpose_result.collision_free) {
            winner = "KDL";
        } else if (!result.kdl_result.collision_free && result.selectgoalpose_result.collision_free) {
            winner = "SGP";
        } else if (result.kdl_result.collision_free && result.selectgoalpose_result.collision_free) {
            winner = (result.kdl_result.solve_time < result.selectgoalpose_result.solve_time) ? "KDL" : "SGP";
        }
        csv_file << winner << "\n";
    }
    
    csv_file.close();
    std::cout << "Results exported to: " << filename << std::endl;
}

// Function to export variance analysis results to CSV file
void exportVarianceResultsToCSV(const std::vector<PoseComparisonResult>& results, const std::string& filename) {
    std::ofstream csv_file(filename);
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open variance CSV file for writing: " << filename << std::endl;
        return;
    }
    
    // Write header for enhanced variance analysis data
    csv_file << "pose_index,method,trial_number,solve_time,collision_check_time,clearance_compute_time,"
             << "success,collision_free,position_error,orientation_error,min_clearance,avg_clearance,"
             << "weighted_clearance,max_clearance,std_dev_clearance,clearance_range,has_clearance_violation,"
             << "num_links_checked,orientation_noise_magnitude,noise_robustness_score,"
             << "mean_solve_time,std_dev_solve_time,median_solve_time,"
             << "min_solve_time,max_solve_time,coefficient_of_variation,success_rate\n";
    
    // Write data rows for each pose comparison result
    for (const auto& pose_result : results) {
        // Export SelectGoalPose variance data
        const auto& sgp_result = pose_result.selectgoalpose_result;
        const auto& variance = sgp_result.variance_analysis;
        
        for (size_t trial = 0; trial < variance.execution_times.size(); ++trial) {
            csv_file << pose_result.pose_index << ","
                     << "selectGoalPose" << ","
                     << trial + 1 << ","
                     << std::scientific << std::setprecision(6) 
                     << variance.execution_times[trial] << ","
                     << variance.collision_check_times[trial] << ","
                     << variance.clearance_compute_times[trial] << ","
                     << (variance.successes[trial] ? 1 : 0) << ","
                     << (variance.collision_free_results[trial] ? 1 : 0) << ","
                     << variance.position_errors[trial] << ","
                     << variance.orientation_errors[trial] << ","
                     << variance.min_clearances[trial] << ","
                     << variance.avg_clearances[trial] << ","
                     << variance.weighted_clearances[trial] << ",";
            
            // Enhanced clearance metrics (use defaults if not available)
            csv_file << (sgp_result.max_clearance >= 0 ? sgp_result.max_clearance : -1.0) << ","
                     << (sgp_result.std_dev_clearance >= 0 ? sgp_result.std_dev_clearance : -1.0) << ","
                     << (sgp_result.clearance_range >= 0 ? sgp_result.clearance_range : -1.0) << ","
                     << (sgp_result.has_clearance_violation ? 1 : 0) << ","
                     << variance.num_links_checked_results[trial] << ","
                     
                     // Orientation noise metrics
                     << sgp_result.orientation_noise_magnitude << ","
                     << sgp_result.noise_robustness_score << ","
                     
                     // Statistical summary
                     << variance.mean_execution_time << ","
                     << variance.std_dev_execution_time << ","
                     << variance.median_execution_time << ","
                     << variance.min_execution_time << ","
                     << variance.max_execution_time << ","
                     << variance.coefficient_of_variation << ","
                     << variance.success_rate << "\n";
        }
    }
    
    csv_file.close();
    std::cout << "Enhanced variance analysis results exported to: " << filename << std::endl;
}

int main(int argc, char *argv[])
{
    Timer total_timer;

    std::string urdf_file = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/panda.urdf";
    std::string scan_poses_file = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/test_poses.csv";
    std::string env_file = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/obstacles.xml";

    // Configuration for enhanced variance analysis
    int num_trials_per_pose = 20;  // Number of trials for variance analysis
    double orientation_noise_magnitude = 0.0;  // Radians (0.0 = no noise, 0.1 = ~5.7 degrees)
    bool test_orientation_noise = false;  // Set to true to test robustness
    
    // Allow command line arguments to configure testing
    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);
        if (arg == "--test-noise" || arg == "-n") {
            test_orientation_noise = true;
            orientation_noise_magnitude = 0.1;  // Default 5.7 degrees of noise
            std::cout << "Orientation noise testing enabled with magnitude: " 
                      << orientation_noise_magnitude << " radians (" 
                      << (orientation_noise_magnitude * 180.0 / M_PI) << " degrees)" << std::endl;
        } else if (arg == "--noise-magnitude" || arg == "-m") {
            if (i + 1 < argc) {
                double custom_magnitude = std::atof(argv[i + 1]);
                if (custom_magnitude > 0.0) {
                    test_orientation_noise = true;
                    orientation_noise_magnitude = custom_magnitude;
                    std::cout << "Custom orientation noise testing enabled with magnitude: " 
                              << orientation_noise_magnitude << " radians (" 
                              << (orientation_noise_magnitude * 180.0 / M_PI) << " degrees)" << std::endl;
                    i++; // Skip the next argument since we used it as the magnitude
                } else {
                    std::cerr << "Invalid noise magnitude: " << argv[i + 1] << std::endl;
                    return -1;
                }
            } else {
                std::cerr << "Missing noise magnitude value after " << arg << std::endl;
                return -1;
            }
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [OPTIONS]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --test-noise, -n           Enable orientation noise testing (default: 0.1 rad)" << std::endl;
            std::cout << "  --noise-magnitude, -m VAL  Set custom noise magnitude in radians" << std::endl;
            std::cout << "  --help, -h                 Show this help message" << std::endl;
            return 0;
        }
    }
    
    auto poses = readPosesFromCSV(scan_poses_file);
    std::cout << "Read " << poses.size() << " poses from CSV file" << std::endl;
    
    if (poses.empty()) {
        std::cerr << "Error: No poses found in CSV file: " << scan_poses_file << std::endl;
        return -1;
    }
    
    // Limit to 5 poses for testing with variance analysis
    if (poses.size() > 5) {
        poses.resize(5);
        std::cout << "Limited to " << poses.size() << " poses for enhanced variance analysis testing" << std::endl;
    }

    std::string analysis_description = test_orientation_noise ? 
                                     "enhanced variance analysis with orientation noise robustness testing" :
                                     "enhanced variance analysis with individual clearance tracking";
    
    std::cout << "Processing " << poses.size() << " poses with " << num_trials_per_pose 
              << " trials each for " << analysis_description << std::endl;

    auto usPlanner = new UltrasoundScanTrajectoryPlanner(urdf_file);
    usPlanner->setEnvironment(env_file);
    auto planner = usPlanner->getPathPlanner();

    // Store comparison results for all poses
    std::vector<PoseComparisonResult> comparison_results;
    comparison_results.reserve(poses.size());

    // Process all poses with enhanced selectGoalPose variance analysis
    for (size_t pose_idx = 0; pose_idx < poses.size(); ++pose_idx) {
        std::cout << "\n" << std::string(70, '=') << std::endl;
        std::cout << "PROCESSING POSE " << (pose_idx + 1) << "/" << poses.size() << std::endl;
        std::cout << std::string(70, '=') << std::endl;
        
        PoseComparisonResult pose_result;
        pose_result.target_pose = poses[pose_idx];
        pose_result.pose_index = pose_idx;
        
        Eigen::Vector3d target_position(poses[pose_idx].block<3, 1>(0, 3));
        Eigen::Quaterniond target_orientation(poses[pose_idx].block<3, 3>(0, 0));
        target_orientation.normalize();

        std::cout << "Target position: " << target_position.transpose() << std::endl;
        std::cout << "Target orientation (quat): " << target_orientation.w() << " " 
                  << target_orientation.x() << " " << target_orientation.y() << " " 
                  << target_orientation.z() << std::endl;

        // Test selectGoalPose method with enhanced variance analysis
        std::cout << "\n--- Enhanced selectGoalPose Analysis ---" << std::endl;
        
        // Create KDL robot for pose verification
        Timer init_timer;
        tup::sim::KDLRobot kdl_robot(urdf_file,
                                     "panda_link0",
                                     "us_image",
                                     Eigen::VectorXd::Zero(7),
                                     Eigen::VectorXd::Zero(7));
        std::cout << "Robot initialization time: " << init_timer.elapsed() << " seconds" << std::endl;
        
        // Run enhanced variance analysis with optional orientation noise
        pose_result.selectgoalpose_result = solveWithSelectGoalPoseVariance(poses[pose_idx], planner, kdl_robot, pose_idx, num_trials_per_pose, orientation_noise_magnitude);
        
        // Mark KDL as not tested (since we're focusing on selectGoalPose variance analysis)
        pose_result.kdl_result.method_name = "KDL_Newton_Raphson";
        pose_result.kdl_result.pose_index = pose_idx;
        pose_result.kdl_result.success = false;
        pose_result.kdl_result.collision_free = false;
        pose_result.kdl_result.solve_time = 0.0;
        pose_result.kdl_result.return_code = -999; // Not tested
        
        comparison_results.push_back(pose_result);
        
        // Print detailed summary for this pose
        std::cout << "\n--- POSE " << (pose_idx + 1) << " ENHANCED ANALYSIS SUMMARY ---" << std::endl;
        std::cout << "selectGoalPose Success Rate: " << std::fixed << std::setprecision(1) 
                  << pose_result.selectgoalpose_result.variance_analysis.success_rate << "%" << std::endl;
        std::cout << "Mean Execution Time: " << std::scientific << std::setprecision(4) 
                  << pose_result.selectgoalpose_result.variance_analysis.mean_time << " seconds" << std::endl;
        std::cout << "Standard Deviation: " << std::scientific << std::setprecision(4) 
                  << pose_result.selectgoalpose_result.variance_analysis.std_dev_time << " seconds" << std::endl;
        std::cout << "Coefficient of Variation: " << std::fixed << std::setprecision(2) 
                  << (pose_result.selectgoalpose_result.variance_analysis.coefficient_of_variation * 100.0) << "%" << std::endl;
        
        if (test_orientation_noise) {
            std::cout << "Noise Robustness Score: " << std::fixed << std::setprecision(3) 
                      << pose_result.selectgoalpose_result.noise_robustness_score << std::endl;
        }
        
        if (pose_result.selectgoalpose_result.collision_free && pose_result.selectgoalpose_result.min_clearance >= 0) {
            std::cout << "Clearance Summary:" << std::endl;
            std::cout << "  Min clearance: " << std::setprecision(4) << pose_result.selectgoalpose_result.min_clearance << "m" << std::endl;
            std::cout << "  Max clearance: " << std::setprecision(4) << pose_result.selectgoalpose_result.max_clearance << "m" << std::endl;
            std::cout << "  Clearance range: " << std::setprecision(4) << pose_result.selectgoalpose_result.clearance_range << "m" << std::endl;
            std::cout << "  Std dev clearance: " << std::setprecision(4) << pose_result.selectgoalpose_result.std_dev_clearance << "m" << std::endl;
            std::cout << "  Has clearance violations: " << (pose_result.selectgoalpose_result.has_clearance_violation ? "Yes" : "No") << std::endl;
            std::cout << "  Individual clearances: ";
            for (size_t i = 0; i < pose_result.selectgoalpose_result.individual_link_clearances.size() && i < 5; ++i) {
                std::cout << std::setprecision(3) << pose_result.selectgoalpose_result.individual_link_clearances[i] << "m ";
            }
            if (pose_result.selectgoalpose_result.individual_link_clearances.size() > 5) {
                std::cout << "... (" << pose_result.selectgoalpose_result.individual_link_clearances.size() << " total)";
            }
            std::cout << std::endl;
        }
    }

    // Generate comprehensive enhanced analysis report
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "COMPREHENSIVE ENHANCED VARIANCE ANALYSIS REPORT" << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    // Collect enhanced variance statistics
    std::vector<double> all_mean_times, all_std_devs, all_coefficients_of_variation;
    std::vector<double> all_success_rates, all_noise_robustness_scores;
    std::vector<double> all_min_clearances, all_max_clearances, all_clearance_ranges;
    std::vector<double> all_std_dev_clearances;
    int total_clearance_violations = 0;
    int total_successful_poses = 0;
    
    for (const auto& result : comparison_results) {
        if (result.selectgoalpose_result.variance_analysis.successful_trials > 0) {
            all_mean_times.push_back(result.selectgoalpose_result.variance_analysis.mean_time);
            all_std_devs.push_back(result.selectgoalpose_result.variance_analysis.std_dev_time);
            all_coefficients_of_variation.push_back(result.selectgoalpose_result.variance_analysis.coefficient_of_variation);
            
            if (result.selectgoalpose_result.min_clearance >= 0) {
                all_min_clearances.push_back(result.selectgoalpose_result.min_clearance);
                all_max_clearances.push_back(result.selectgoalpose_result.max_clearance);
                all_clearance_ranges.push_back(result.selectgoalpose_result.clearance_range);
                all_std_dev_clearances.push_back(result.selectgoalpose_result.std_dev_clearance);
                
                if (result.selectgoalpose_result.has_clearance_violation) {
                    total_clearance_violations++;
                }
                total_successful_poses++;
            }
        }
        all_success_rates.push_back(result.selectgoalpose_result.variance_analysis.success_rate);
        
        if (test_orientation_noise) {
            all_noise_robustness_scores.push_back(result.selectgoalpose_result.noise_robustness_score);
        }
    }

    // Print aggregated enhanced variance statistics
    std::cout << "\nAGGREGATED ENHANCED ANALYSIS:" << std::endl;
    std::cout << "Total poses analyzed: " << comparison_results.size() << std::endl;
    std::cout << "Total trials performed: " << (comparison_results.size() * num_trials_per_pose) << std::endl;
    std::cout << "Poses with successful solutions: " << total_successful_poses << "/" << comparison_results.size() << std::endl;
    
    if (!all_mean_times.empty()) {
        printEnhancedStatistics(all_mean_times, "Mean Execution Times Across Poses");
        printEnhancedStatistics(all_std_devs, "Standard Deviations Across Poses");
        printEnhancedStatistics(all_coefficients_of_variation, "Coefficients of Variation Across Poses");
    }
    
    if (!all_success_rates.empty()) {
        printEnhancedStatistics(all_success_rates, "Success Rates Across Poses");
    }
    
    if (test_orientation_noise && !all_noise_robustness_scores.empty()) {
        printEnhancedStatistics(all_noise_robustness_scores, "Noise Robustness Scores Across Poses");
    }
    
    if (!all_min_clearances.empty()) {
        std::cout << "\nCLEARANCE ANALYSIS:" << std::endl;
        printEnhancedStatistics(all_min_clearances, "Minimum Clearances Across Poses");
        printEnhancedStatistics(all_max_clearances, "Maximum Clearances Across Poses");
        printEnhancedStatistics(all_clearance_ranges, "Clearance Ranges Across Poses");
        printEnhancedStatistics(all_std_dev_clearances, "Clearance Standard Deviations Across Poses");
        
        std::cout << "\nClearance Violations: " << total_clearance_violations << "/" << total_successful_poses 
                  << " poses (" << std::fixed << std::setprecision(1) 
                  << (100.0 * total_clearance_violations / total_successful_poses) << "%)" << std::endl;
    }

    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "Total execution time: " << total_timer.elapsed() << " seconds" << std::endl;

    // Export enhanced results to CSV for Python plotting
    std::string csv_filename = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ComparisonIK/ik_variance_analysis_results.csv";
    exportVarianceResultsToCSV(comparison_results, csv_filename);

    std::cout << "\nEnhanced variance analysis complete! Check the generated Python plots for visualization." << std::endl;
    std::cout << "Run with '--test-noise' or '-n' to enable orientation noise robustness testing." << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    delete usPlanner;
    return 0;
}
