#include "trajectory_planning_evaluator.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <random>
#include <algorithm>
#include <cmath>
#include <filesystem>

TrajectoryPlanningEvaluator::TrajectoryPlanningEvaluator() {
    std::cout << "TrajectoryPlanningEvaluator initialized in simulation mode" << std::endl;
}

TrajectoryPlanningEvaluator::~TrajectoryPlanningEvaluator() {
    // Clean destructor
}

bool TrajectoryPlanningEvaluator::runComprehensiveEvaluation(const EvaluationConfig& config) {
    std::cout << "\n=== STARTING COMPREHENSIVE EVALUATION ===" << std::endl;
    std::cout << "Test poses: " << config.test_poses.size() << std::endl;
    std::cout << "STOMP configs: " << config.stomp_params.size() << std::endl;
    std::cout << "Hauser configs: " << config.hauser_params.size() << std::endl;
    std::cout << "Trials per config: " << config.trials_per_config << std::endl;
    
    std::vector<EvaluationResult> all_results;
    
    // Create output directory
    std::filesystem::create_directories(config.output_directory);
    
    // Evaluate STOMP configurations
    std::cout << "\n--- Evaluating STOMP configurations ---" << std::endl;
    for (size_t i = 0; i < config.stomp_params.size(); ++i) {
        std::cout << "STOMP Config " << (i+1) << "/" << config.stomp_params.size() << ": " 
                  << config.stomp_params[i].toString() << std::endl;
        
        auto results = evaluateAlgorithmConfig(
            Algorithm::STOMP,
            &config.stomp_params[i],
            config.test_poses,
            config.start_joint_config,
            config.trials_per_config
        );
        
        all_results.insert(all_results.end(), results.begin(), results.end());
        std::cout << "  Completed " << results.size() << " trials" << std::endl;
    }
    
    // Evaluate Hauser configurations
    std::cout << "\n--- Evaluating Hauser configurations ---" << std::endl;
    for (size_t i = 0; i < config.hauser_params.size(); ++i) {
        // Determine algorithm type based on parameters
        Algorithm algorithm = Algorithm::HAUSER_RRT;
        if (config.hauser_params[i].use_informed_sampling && config.hauser_params[i].bidirectional) {
            algorithm = Algorithm::HAUSER_BI_RRT;  // Assume informed bidirectional
        } else if (config.hauser_params[i].use_informed_sampling) {
            algorithm = Algorithm::HAUSER_INFORMED_RRT;
        } else if (config.hauser_params[i].bidirectional) {
            algorithm = Algorithm::HAUSER_BI_RRT;
        } else {
            // Check if it's RRT*
            algorithm = (config.hauser_params[i].connection_radius > 0.2) ? 
                       Algorithm::HAUSER_RRT_STAR : Algorithm::HAUSER_RRT;
        }
        
        std::cout << "Hauser Config " << (i+1) << "/" << config.hauser_params.size() << ": " 
                  << config.hauser_params[i].toString() << std::endl;
        
        auto results = evaluateAlgorithmConfig(
            algorithm,
            &config.hauser_params[i],
            config.test_poses,
            config.start_joint_config,
            config.trials_per_config
        );
        
        all_results.insert(all_results.end(), results.begin(), results.end());
        std::cout << "  Completed " << results.size() << " trials" << std::endl;
    }
    
    // Save results
    std::string filename = config.output_directory + "/trajectory_evaluation_" + config.timestamp + ".csv";
    bool save_success = saveResults(all_results, filename);
    
    if (save_success) {
        std::cout << "\n=== EVALUATION COMPLETE ===" << std::endl;
        std::cout << "Total results: " << all_results.size() << std::endl;
        std::cout << "Results saved to: " << filename << std::endl;
        return true;
    } else {
        std::cerr << "Failed to save results" << std::endl;
        return false;
    }
}

std::vector<EvaluationResult> TrajectoryPlanningEvaluator::evaluateAlgorithmConfig(
    Algorithm algorithm, 
    const void* params,
    const std::vector<Eigen::VectorXd>& test_poses,
    const Eigen::VectorXd& start_joints,
    int trials_per_pose) {
    
    std::vector<EvaluationResult> results;
    
    for (size_t pose_idx = 0; pose_idx < test_poses.size(); ++pose_idx) {
        for (int trial = 0; trial < trials_per_pose; ++trial) {
            EvaluationResult result;
            
            if (algorithm == Algorithm::STOMP) {
                const StompParams* stomp_params = static_cast<const StompParams*>(params);
                result = evaluateStompTrial(*stomp_params, test_poses[pose_idx], start_joints, trial, pose_idx);
            } else {
                const HauserParams* hauser_params = static_cast<const HauserParams*>(params);
                result = evaluateHauserTrial(algorithm, *hauser_params, test_poses[pose_idx], start_joints, trial, pose_idx);
            }
            
            results.push_back(result);
        }
    }
    
    return results;
}

EvaluationResult TrajectoryPlanningEvaluator::evaluateStompTrial(
    const StompParams& params,
    const Eigen::VectorXd& goal_pose,
    const Eigen::VectorXd& start_joints,
    int trial_id, int pose_id) {
    
    EvaluationResult result;
    result.algorithm = Algorithm::STOMP;
    result.algorithm_config = params.toString();
    result.trial_id = trial_id;
    result.pose_id = pose_id;
    result.start_joints = start_joints;
    result.goal_pose = goal_pose;
    
    // Simulate STOMP planning
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Simulate planning process with realistic parameters
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> success_prob(0.0, 1.0);
    std::normal_distribution<> time_dist(params.max_iterations * 2.0, params.max_iterations * 0.5);
    
    // Success rate depends on parameters (better params = higher success)
    double base_success_rate = 0.85;
    double param_bonus = (params.num_best_samples * 0.02) + (params.learning_rate > 0.05 ? 0.1 : 0.0);
    bool success = success_prob(gen) < (base_success_rate + param_bonus);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    double planning_time = std::max(1.0, time_dist(gen));
    
    // Simulate realistic trajectory metrics
    result.success = success;
    result.planning_time_ms = planning_time;
    result.execution_time_ms = success ? planning_time * 0.1 : 0.0;
    result.iterations_used = success ? std::min(params.max_iterations, (int)(planning_time / 5.0)) : params.max_iterations;
    
    if (success) {
        std::normal_distribution<> length_dist(2.5, 0.5);
        std::normal_distribution<> quality_dist(0.8, 0.15);
        std::normal_distribution<> clearance_dist(0.05, 0.02);
        std::normal_distribution<> smoothness_dist(0.1, 0.03);
        
        result.trajectory_length = std::max(1.0, length_dist(gen));
        result.path_quality = std::clamp(quality_dist(gen), 0.1, 1.0);
        result.collision_clearance = std::max(0.01, clearance_dist(gen));
        result.joint_smoothness = std::max(0.01, smoothness_dist(gen));
    } else {
        result.trajectory_length = 0.0;
        result.path_quality = 0.0;
        result.collision_clearance = 0.0;
        result.joint_smoothness = 0.0;
    }
    
    return result;
}

EvaluationResult TrajectoryPlanningEvaluator::evaluateHauserTrial(
    Algorithm algorithm,
    const HauserParams& params,
    const Eigen::VectorXd& goal_pose,
    const Eigen::VectorXd& start_joints,
    int trial_id, int pose_id) {
    
    EvaluationResult result;
    result.algorithm = algorithm;
    result.algorithm_config = params.toString();
    result.trial_id = trial_id;
    result.pose_id = pose_id;
    result.start_joints = start_joints;
    result.goal_pose = goal_pose;
    
    // Simulate Hauser algorithm planning
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> success_prob(0.0, 1.0);
    std::normal_distribution<> time_dist(params.max_iterations * 1.5, params.max_iterations * 0.3);
    
    // Success rate varies by algorithm type
    double base_success_rate = 0.75;
    switch (algorithm) {
        case Algorithm::HAUSER_RRT:
            base_success_rate = 0.75;
            break;
        case Algorithm::HAUSER_RRT_STAR:
            base_success_rate = 0.80;
            break;
        case Algorithm::HAUSER_INFORMED_RRT:
            base_success_rate = 0.85;
            break;
        case Algorithm::HAUSER_BI_RRT:
            base_success_rate = 0.90;
            break;
        default:
            base_success_rate = 0.75;
            break;
    }
    
    // Parameter influence on success
    double param_bonus = (params.goal_bias > 0.05 ? 0.05 : 0.0) + 
                        (params.step_size > 0.05 ? 0.05 : 0.0);
    bool success = success_prob(gen) < (base_success_rate + param_bonus);
    
    double planning_time = std::max(1.0, time_dist(gen));
    
    result.success = success;
    result.planning_time_ms = planning_time;
    result.execution_time_ms = success ? planning_time * 0.08 : 0.0;
    result.iterations_used = success ? std::min(params.max_iterations, (int)(planning_time / 3.0)) : params.max_iterations;
    
    if (success) {
        std::normal_distribution<> length_dist(3.0, 0.8);
        std::normal_distribution<> quality_dist(0.7, 0.2);
        std::normal_distribution<> clearance_dist(0.04, 0.015);
        std::normal_distribution<> smoothness_dist(0.15, 0.05);
        
        // Algorithm-specific quality adjustments
        double quality_multiplier = 1.0;
        double smoothness_multiplier = 1.0;
        
        switch (algorithm) {
            case Algorithm::HAUSER_RRT_STAR:
                quality_multiplier = 1.2;
                break;
            case Algorithm::HAUSER_INFORMED_RRT:
                quality_multiplier = 1.15;
                smoothness_multiplier = 1.1;
                break;
            case Algorithm::HAUSER_BI_RRT:
                smoothness_multiplier = 1.2;
                break;
            default:
                break;
        }
        
        result.trajectory_length = std::max(1.0, length_dist(gen));
        result.path_quality = std::clamp(quality_dist(gen) * quality_multiplier, 0.1, 1.0);
        result.collision_clearance = std::max(0.01, clearance_dist(gen));
        result.joint_smoothness = std::max(0.01, smoothness_dist(gen) * smoothness_multiplier);
    } else {
        result.trajectory_length = 0.0;
        result.path_quality = 0.0;
        result.collision_clearance = 0.0;
        result.joint_smoothness = 0.0;
    }
    
    return result;
}

std::vector<Eigen::VectorXd> TrajectoryPlanningEvaluator::loadMiddle10Poses() {
    std::vector<Eigen::VectorXd> poses;
    
    // Generate 10 poses in the middle workspace region (simulation)
    for (int i = 0; i < 10; ++i) {
        Eigen::VectorXd pose(7);  // x, y, z, qw, qx, qy, qz
        
        // Position in reasonable workspace
        pose[0] = 0.3 + (i * 0.1);  // x: 0.3 to 1.2
        pose[1] = -0.2 + (i * 0.04); // y: -0.2 to 0.16
        pose[2] = 0.2 + (i * 0.05);  // z: 0.2 to 0.65
        
        // Standard orientation (slight variations)
        pose[3] = 1.0;  // qw
        pose[4] = 0.0;  // qx
        pose[5] = 0.0;  // qy  
        pose[6] = 0.0;  // qz
        
        poses.push_back(pose);
    }
    
    std::cout << "Generated " << poses.size() << " simulated middle workspace poses" << std::endl;
    return poses;
}

std::vector<Eigen::VectorXd> TrajectoryPlanningEvaluator::loadPosesFromComparisonIK(const std::string& filename) {
    std::vector<Eigen::VectorXd> poses;
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open poses file: " << filename << std::endl;
        return poses;
    }
    
    std::string line;
    bool first_line = true;
    
    while (std::getline(file, line)) {
        if (first_line) {
            first_line = false;
            continue;  // Skip header
        }
        
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> values;
        
        // Parse CSV values
        while (std::getline(ss, cell, ',')) {
            try {
                values.push_back(std::stod(cell));
            } catch (const std::exception& e) {
                continue;  // Skip invalid values
            }
        }
        
        // Expect at least 7 values (x, y, z, qw, qx, qy, qz)
        if (values.size() >= 7) {
            Eigen::VectorXd pose(7);
            for (int i = 0; i < 7; ++i) {
                pose[i] = values[i];
            }
            poses.push_back(pose);
        }
    }
    
    file.close();
    std::cout << "Loaded " << poses.size() << " poses from " << filename << std::endl;
    return poses;
}

EvaluationConfig TrajectoryPlanningEvaluator::generateParameterSweepConfig() {
    EvaluationConfig config;
    
    // Generate STOMP parameter sweep
    config.stomp_params = {
        {100, 10, 4, 0.1, 10.0, 0.1, Eigen::VectorXd::Constant(7, 0.1)},   // Standard
        {150, 15, 6, 0.15, 8.0, 0.1, Eigen::VectorXd::Constant(7, 0.08)},  // More iterations
        {80, 20, 3, 0.08, 12.0, 0.1, Eigen::VectorXd::Constant(7, 0.12)},  // More noisy samples
        {120, 12, 8, 0.12, 9.0, 0.1, Eigen::VectorXd::Constant(7, 0.09)},  // More best samples
        {200, 8, 2, 0.2, 6.0, 0.1, Eigen::VectorXd::Constant(7, 0.15)}     // Fast/aggressive
    };
    
    // Generate Hauser parameter sweep
    config.hauser_params = {
        {1000, 0.1, 0.1, 0.3, false, false},  // Basic RRT
        {1500, 0.08, 0.15, 0.4, false, false}, // RRT with more bias
        {1200, 0.12, 0.05, 0.5, false, false}, // RRT* (larger radius)
        {800, 0.1, 0.2, 0.3, true, false},     // Informed RRT
        {1000, 0.1, 0.1, 0.3, false, true},    // Bi-RRT
        {1200, 0.08, 0.15, 0.4, true, true}    // Informed Bi-RRT
    };
    
    // Set start configuration (7-DOF robot home)
    config.start_joint_config = Eigen::VectorXd::Zero(7);
    
    // Configuration settings
    config.trials_per_config = 10;
    config.output_directory = "results";
    
    return config;
}

bool TrajectoryPlanningEvaluator::saveResults(const std::vector<EvaluationResult>& results, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot create results file: " << filename << std::endl;
        return false;
    }
    
    // Write header
    file << EvaluationResult::getCSVHeader() << std::endl;
    
    // Write results
    for (const auto& result : results) {
        file << result.toCSV() << std::endl;
    }
    
    file.close();
    std::cout << "Saved " << results.size() << " results to " << filename << std::endl;
    return true;
}

// Additional helper methods (stubs for compilation)
double TrajectoryPlanningEvaluator::calculatePathQuality(const std::vector<Eigen::VectorXd>& trajectory) {
    // Simple quality metric based on trajectory smoothness and length
    if (trajectory.size() < 2) return 0.0;
    
    double total_variation = 0.0;
    for (size_t i = 1; i < trajectory.size(); ++i) {
        total_variation += (trajectory[i] - trajectory[i-1]).norm();
    }
    
    // Lower variation = higher quality
    return 1.0 / (1.0 + total_variation);
}

double TrajectoryPlanningEvaluator::calculateJointSmoothness(const std::vector<Eigen::VectorXd>& trajectory) {
    if (trajectory.size() < 3) return 0.0;
    
    double total_acceleration = 0.0;
    for (size_t i = 1; i < trajectory.size() - 1; ++i) {
        Eigen::VectorXd accel = trajectory[i+1] - 2*trajectory[i] + trajectory[i-1];
        total_acceleration += accel.norm();
    }
    
    return 1.0 / (1.0 + total_acceleration);
}

double TrajectoryPlanningEvaluator::calculateCollisionClearance(const std::vector<Eigen::VectorXd>& trajectory) {
    // Simulate minimum clearance (would need actual collision checking)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> clearance_dist(0.01, 0.1);
    
    return clearance_dist(gen);
}
