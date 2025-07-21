#include "repositioning_evaluator.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <algorithm>
#include <numeric>

RepositioningEvaluator::RepositioningEvaluator(const RepositioningEvalConfig& config)
    : config_(config) {
    
    // Create output directory if it doesn't exist
    std::filesystem::create_directories(config_.output_directory);
    
    // Set up algorithm configurations from legacy parameters for backward compatibility
    setupAlgorithmConfigs();
    
    log("Initializing RepositioningEvaluator...");
    log("Selected algorithm: " + AlgorithmUtils::trajectoryAlgorithmToString(config_.trajectory_algorithm));
    
    if (!initializePlanner()) {
        throw std::runtime_error("Failed to initialize trajectory planner");
    }
    
    log("RepositioningEvaluator initialized successfully");
}

void RepositioningEvaluator::setupAlgorithmConfigs() {
    // Update STOMP config from legacy parameters if they differ from defaults
    if (config_.max_stomp_iterations != 100) {
        config_.stomp_config.max_iterations = config_.max_stomp_iterations;
    }
    if (config_.num_noisy_trajectories != 10) {
        config_.stomp_config.num_noisy_trajectories = config_.num_noisy_trajectories;
    }
    if (config_.stomp_learning_rate != 0.1) {
        config_.stomp_config.learning_rate = config_.stomp_learning_rate;
    }
    if (config_.stomp_temperature != 10.0) {
        config_.stomp_config.temperature = config_.stomp_temperature;
    }
    
    // Update path planning config from legacy parameters
    if (config_.max_rrt_iterations != 5000) {
        config_.path_planning_config.max_iterations = config_.max_rrt_iterations;
    }
    if (config_.rrt_step_size != 0.1) {
        config_.path_planning_config.step_size = config_.rrt_step_size;
    }
    if (config_.rrt_goal_bias != 0.1) {
        config_.path_planning_config.goal_bias = config_.rrt_goal_bias;
    }
}

bool RepositioningEvaluator::initializePlanner() {
    try {
        log("Creating UltrasoundScanTrajectoryPlanner...");
        planner_ = std::make_unique<UltrasoundScanTrajectoryPlanner>(config_.robot_urdf_path, 
                                                                   std::vector<double>(config_.initial_joint_config.begin(), config_.initial_joint_config.end()));
        
        log("Setting environment...");
        planner_->setEnvironment(config_.environment_xml_path);
        
        log("Setting initial joint configuration...");
        planner_->setCurrentJoints(config_.initial_joint_config);
        
        return true;
        
    } catch (const std::exception& e) {
        log("Error initializing planner: " + std::string(e.what()));
        return false;
    }
}

std::vector<Eigen::Affine3d> RepositioningEvaluator::loadScanPoses(const std::string& csv_path) {
    std::vector<Eigen::Affine3d> poses;
    std::ifstream file(csv_path);
    
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open scan poses file: " + csv_path);
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;
        
        while (std::getline(ss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            } catch (const std::exception& e) {
                log("Warning: Could not parse token '" + token + "' in line: " + line);
                continue;
            }
        }
        
        if (values.size() < 7) {
            log("Warning: Line has insufficient values (" + std::to_string(values.size()) + "): " + line);
            continue;
        }
        
        // Parse pose: x, y, z, qw, qx, qy, qz
        Eigen::Vector3d position(values[0], values[1], values[2]);
        Eigen::Quaterniond orientation(values[3], values[4], values[5], values[6]);
        orientation.normalize();
        
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        pose.translation() = position;
        pose.linear() = orientation.toRotationMatrix();
        
        // Move pose back 2cm along local Z-axis (like in ComparisonIK)
        const Eigen::Vector3d local_move(0.0, 0.0, -0.02);
        pose.translation() += pose.linear() * local_move;
        
        poses.push_back(pose);
    }
    
    log("Loaded " + std::to_string(poses.size()) + " scan poses from " + csv_path);
    return poses;
}

bool RepositioningEvaluator::runEvaluation() {
    log("Starting comprehensive repositioning evaluation...");
    
    // Load scan poses
    std::vector<Eigen::Affine3d> scan_poses;
    try {
        scan_poses = loadScanPoses(config_.scan_poses_csv_path);
    } catch (const std::exception& e) {
        log("Error loading scan poses: " + std::string(e.what()));
        return false;
    }
    
    if (scan_poses.size() < 2) {
        log("Error: Need at least 2 scan poses for repositioning evaluation");
        return false;
    }
    
    // We'll evaluate repositioning between the first two poses
    Eigen::Affine3d start_pose = scan_poses[0];
    Eigen::Affine3d target_pose = scan_poses[1];
    
    log("Evaluating repositioning between pose 1 and pose 2...");
    log("Start pose position: " + 
        std::to_string(start_pose.translation().x()) + ", " +
        std::to_string(start_pose.translation().y()) + ", " +
        std::to_string(start_pose.translation().z()));
    log("Target pose position: " + 
        std::to_string(target_pose.translation().x()) + ", " +
        std::to_string(target_pose.translation().y()) + ", " +
        std::to_string(target_pose.translation().z()));
    
    // Calculate cartesian distance between poses
    double cartesian_distance = (target_pose.translation() - start_pose.translation()).norm();
    log("Cartesian distance: " + std::to_string(cartesian_distance) + " meters");
    
    // Run multiple trials
    std::vector<RepositioningResult> results;
    results.reserve(config_.num_trials);
    
    for (int trial = 0; trial < config_.num_trials; ++trial) {
        log("Running trial " + std::to_string(trial + 1) + "/" + std::to_string(config_.num_trials));
        
        RepositioningResult result = evaluateSingleRepositioning(start_pose, target_pose, trial + 1);
        result.cartesian_distance = cartesian_distance;
        results.push_back(result);
        
        if (config_.verbose) {
            log("Trial " + std::to_string(trial + 1) + " - Success: " + 
                (result.planning_success ? "YES" : "NO") + 
                ", Time: " + std::to_string(result.planning_time_ms) + "ms");
        }
    }
    
    // Compute statistics
    RepositioningStatistics stats = computeStatistics(results);
    
    // Export results
    std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    
    std::string results_filename = config_.output_directory + "/repositioning_results_" + timestamp + ".csv";
    std::string stats_filename = config_.output_directory + "/repositioning_statistics_" + timestamp + ".txt";
    
    exportResultsToCSV(results, results_filename);
    exportStatistics(stats, stats_filename);
    
    log("Evaluation completed successfully!");
    log("Results exported to: " + results_filename);
    log("Statistics exported to: " + stats_filename);
    log("Success rate: " + std::to_string(stats.success_rate * 100.0) + "%");
    log("Average planning time: " + std::to_string(stats.avg_planning_time_ms) + "ms");
    
    return true;
}

RepositioningResult RepositioningEvaluator::evaluateSingleRepositioning(
    const Eigen::Affine3d& start_pose,
    const Eigen::Affine3d& target_pose,
    int trial_number) {
    
    RepositioningResult result;
    result.algorithm_used = AlgorithmUtils::trajectoryAlgorithmToString(config_.trajectory_algorithm);
    
    try {
        auto start_time = std::chrono::high_resolution_clock::now();
        bool planning_success = false;
        
        // Execute different algorithms based on configuration
        switch (config_.trajectory_algorithm) {
            case TrajectoryAlgorithm::STOMP:
            case TrajectoryAlgorithm::STOMP_WITH_CHECKPOINTS:
            case TrajectoryAlgorithm::STOMP_WITH_EARLY_TERMINATION:
                planning_success = executeStompVariant(start_pose, target_pose, result);
                break;
                
            case TrajectoryAlgorithm::HAUSER:
                planning_success = executeHauserAlgorithm(start_pose, target_pose, result);
                break;
                
            default:
                log("Unknown algorithm specified, falling back to STOMP");
                planning_success = executeStompVariant(start_pose, target_pose, result);
                break;
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        result.planning_time_ms = duration.count() / 1000.0;
        result.planning_success = planning_success;
        
    } catch (const std::exception& e) {
        log("Error in trial " + std::to_string(trial_number) + ": " + std::string(e.what()));
        result.planning_success = false;
    }
    
    return result;
}

bool RepositioningEvaluator::executeStompVariant(
    const Eigen::Affine3d& start_pose,
    const Eigen::Affine3d& target_pose,
    RepositioningResult& result) {
    
    // Set poses for the planner
    std::vector<Eigen::Affine3d> poses = {start_pose, target_pose};
    planner_->setPoses(poses);
    
    // Reset current joints to initial configuration for each trial
    planner_->setCurrentJoints(config_.initial_joint_config);
    
    // Plan trajectories using the existing UltrasoundScanTrajectoryPlanner
    // which already uses STOMP internally
    bool planning_success = planner_->planTrajectories();
    
    if (planning_success) {
        // Get the planned trajectories
        auto trajectories = planner_->getTrajectories();
        
        if (!trajectories.empty()) {
            // Analyze the first (repositioning) trajectory
            const auto& trajectory = trajectories[0].first;
            analyzeTrajectory(trajectory, result);
        }
    }
    
    return planning_success;
}

bool RepositioningEvaluator::executeHauserAlgorithm(
    const Eigen::Affine3d& start_pose,
    const Eigen::Affine3d& target_pose,
    RepositioningResult& result) {
    
    result.path_planning_algorithm_used = AlgorithmUtils::pathPlanningAlgorithmToString(
        config_.path_planning_config.algorithm);
    
    try {
        // Get path planner from the UltrasoundScanTrajectoryPlanner
        auto path_planner = planner_->getPathPlanner();
        
        // Configure path planner with specified algorithm
        Params params = AlgorithmUtils::createParamsFromConfig(config_.path_planning_config);
        path_planner->setParams(params);
        
        // Set start configuration
        path_planner->setStartPose(createRobotArmFromJoints(config_.initial_joint_config));
        
        // Phase 1: Path planning
        auto path_start_time = std::chrono::high_resolution_clock::now();
        
        // Set goal pose and run path planning
        Eigen::Vector3d goal_translation = target_pose.translation();
        Eigen::Matrix3d goal_rotation = target_pose.linear();
        path_planner->setGoalPose(goal_translation, goal_rotation);
        
        bool path_success = path_planner->runPathFinding();
        
        auto path_end_time = std::chrono::high_resolution_clock::now();
        auto path_duration = std::chrono::duration_cast<std::chrono::microseconds>(path_end_time - path_start_time);
        result.path_planning_time_ms = path_duration.count() / 1000.0;
        
        if (!path_success) {
            log("Path planning failed for Hauser algorithm");
            return false;
        }
        
        // Get the planned path
        auto path = path_planner->getPath();
        result.path_planning_iterations_used = static_cast<int>(path.size());
        
        // Phase 2: Motion generation with Hauser
        auto motion_start_time = std::chrono::high_resolution_clock::now();
        
        // Convert path to waypoints matrix
        Eigen::MatrixXd waypoints;
        waypoints.resize(path.size(), 7);
        for (size_t i = 0; i < path.size(); ++i) {
            for (int j = 0; j < 7; ++j) {
                waypoints(i, j) = std::get<1>(path[i]).getJointAngles()[j];
            }
        }
        
        // Create motion generator
        auto motion_generator = planner_->getMotionGenerator();
        motion_generator->setWaypoints(waypoints);
        
        // Run Hauser algorithm
        motion_generator->performHauser(config_.hauser_config.max_iterations, 
                                       config_.hauser_config.output_file);
        
        auto motion_end_time = std::chrono::high_resolution_clock::now();
        auto motion_duration = std::chrono::duration_cast<std::chrono::microseconds>(motion_end_time - motion_start_time);
        result.motion_generation_time_ms = motion_duration.count() / 1000.0;
        
        // Get the resulting trajectory
        auto trajectory = motion_generator->getPath();
        if (!trajectory.empty()) {
            analyzeTrajectory(trajectory, result);
            result.iterations_used = config_.hauser_config.max_iterations;
            return true;
        }
        
    } catch (const std::exception& e) {
        log("Error in Hauser algorithm execution: " + std::string(e.what()));
    }
    
    return false;
}

void RepositioningEvaluator::analyzeTrajectory(
    const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
    RepositioningResult& result) {
    
    result.num_waypoints = static_cast<int>(trajectory.size());
    result.trajectory_length = calculateJointSpaceDistance(trajectory);
    result.trajectory_smoothness = calculateTrajectorySmootness(trajectory);
    result.collision_free = checkTrajectoryCollisionFree(trajectory);
    result.energy_consumption = calculateEnergyConsumption(trajectory);
    
    // Calculate joint space distance
    if (!trajectory.empty()) {
        Eigen::VectorXd start_joints = Eigen::Map<const Eigen::VectorXd>(
            trajectory.front().position.data(), trajectory.front().position.size());
        Eigen::VectorXd end_joints = Eigen::Map<const Eigen::VectorXd>(
            trajectory.back().position.data(), trajectory.back().position.size());
        result.joint_space_distance = (end_joints - start_joints).norm();
    }
    
    // Calculate maximum velocities and accelerations
    double max_vel = 0.0, max_acc = 0.0;
    for (const auto& point : trajectory) {
        for (double vel : point.velocity) {
            max_vel = std::max(max_vel, std::abs(vel));
        }
        for (double acc : point.acceleration) {
            max_acc = std::max(max_acc, std::abs(acc));
        }
    }
    result.max_joint_velocity = max_vel;
    result.max_joint_acceleration = max_acc;
}

RobotArm RepositioningEvaluator::createRobotArmFromJoints(const Eigen::VectorXd& joints) {
    // This is a helper method to create a RobotArm from joint angles
    // We'll need to use the same robot URDF as the planner
    RobotArm arm(config_.robot_urdf_path);
    arm.setJointAngles(joints);
    return arm;
}

double RepositioningEvaluator::calculateTrajectorySmootness(
    const std::vector<MotionGenerator::TrajectoryPoint>& trajectory) {
    
    if (trajectory.size() < 3) return 0.0;
    
    double total_jerk = 0.0;
    int num_joints = trajectory[0].position.size();
    
    for (int j = 0; j < num_joints; ++j) {
        for (size_t i = 2; i < trajectory.size(); ++i) {
            // Calculate jerk as second derivative of velocity
            double acc_curr = trajectory[i].acceleration[j];
            double acc_prev = trajectory[i-1].acceleration[j];
            double dt = 0.1; // Assume 10Hz trajectory
            double jerk = (acc_curr - acc_prev) / dt;
            total_jerk += jerk * jerk;
        }
    }
    
    return total_jerk / trajectory.size();
}

double RepositioningEvaluator::calculateJointSpaceDistance(
    const std::vector<MotionGenerator::TrajectoryPoint>& trajectory) {
    
    if (trajectory.size() < 2) return 0.0;
    
    double total_distance = 0.0;
    
    for (size_t i = 1; i < trajectory.size(); ++i) {
        double segment_distance = 0.0;
        for (size_t j = 0; j < trajectory[i].position.size(); ++j) {
            double diff = trajectory[i].position[j] - trajectory[i-1].position[j];
            segment_distance += diff * diff;
        }
        total_distance += std::sqrt(segment_distance);
    }
    
    return total_distance;
}

bool RepositioningEvaluator::checkTrajectoryCollisionFree(
    const std::vector<MotionGenerator::TrajectoryPoint>& trajectory) {
    
    // This would ideally check collision for each trajectory point
    // For now, we assume the trajectory is collision-free if planning succeeded
    // In a real implementation, you would check each configuration against obstacles
    return true;
}

double RepositioningEvaluator::calculateEnergyConsumption(
    const std::vector<MotionGenerator::TrajectoryPoint>& trajectory) {
    
    double total_energy = 0.0;
    
    // Simple energy model based on joint accelerations and movements
    for (const auto& point : trajectory) {
        for (size_t j = 0; j < point.acceleration.size(); ++j) {
            // Energy proportional to acceleration squared
            total_energy += point.acceleration[j] * point.acceleration[j];
        }
    }
    
    return total_energy;
}

void RepositioningEvaluator::exportResultsToCSV(
    const std::vector<RepositioningResult>& results,
    const std::string& filename) {
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filename);
    }
    
    // Write header
    file << "trial,success,planning_time_ms,trajectory_length,smoothness,num_waypoints,"
         << "joint_distance,cartesian_distance,collision_free,max_velocity,max_acceleration,"
         << "energy_consumption,algorithm,iterations,path_planning_algorithm,"
         << "path_planning_iterations,path_planning_time_ms,motion_generation_time_ms\n";
    
    // Write data
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        file << (i + 1) << ","
             << (r.planning_success ? 1 : 0) << ","
             << r.planning_time_ms << ","
             << r.trajectory_length << ","
             << r.trajectory_smoothness << ","
             << r.num_waypoints << ","
             << r.joint_space_distance << ","
             << r.cartesian_distance << ","
             << (r.collision_free ? 1 : 0) << ","
             << r.max_joint_velocity << ","
             << r.max_joint_acceleration << ","
             << r.energy_consumption << ","
             << r.algorithm_used << ","
             << r.iterations_used << ","
             << r.path_planning_algorithm_used << ","
             << r.path_planning_iterations_used << ","
             << r.path_planning_time_ms << ","
             << r.motion_generation_time_ms << "\n";
    }
    
    file.close();
}

void RepositioningEvaluator::exportStatistics(
    const RepositioningStatistics& stats,
    const std::string& filename) {
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filename);
    }
    
    file << "Ultrasound Repositioning Evaluation Statistics\n";
    file << "=============================================\n\n";
    
    file << "Overall Performance:\n";
    file << "  Total Trials: " << stats.total_trials << "\n";
    file << "  Successful Trials: " << stats.successful_trials << "\n";
    file << "  Success Rate: " << std::fixed << std::setprecision(2) 
         << (stats.success_rate * 100.0) << "%\n\n";
    
    file << "Planning Time Statistics:\n";
    file << "  Average: " << std::fixed << std::setprecision(2) 
         << stats.avg_planning_time_ms << " ms\n";
    file << "  Std Dev: " << std::fixed << std::setprecision(2) 
         << stats.std_planning_time_ms << " ms\n\n";
    
    file << "Trajectory Quality Statistics:\n";
    file << "  Average Length: " << std::fixed << std::setprecision(4) 
         << stats.avg_trajectory_length << " rad\n";
    file << "  Std Dev Length: " << std::fixed << std::setprecision(4) 
         << stats.std_trajectory_length << " rad\n";
    file << "  Average Smoothness: " << std::fixed << std::setprecision(6) 
         << stats.avg_smoothness << "\n";
    file << "  Std Dev Smoothness: " << std::fixed << std::setprecision(6) 
         << stats.std_smoothness << "\n\n";
    
    file << "Distance Statistics:\n";
    file << "  Average Joint Distance: " << std::fixed << std::setprecision(4) 
         << stats.avg_joint_distance << " rad\n";
    file << "  Average Cartesian Distance: " << std::fixed << std::setprecision(4) 
         << stats.avg_cartesian_distance << " m\n\n";
    
    file.close();
}

RepositioningStatistics RepositioningEvaluator::computeStatistics(
    const std::vector<RepositioningResult>& results) {
    
    RepositioningStatistics stats;
    stats.total_trials = static_cast<int>(results.size());
    
    if (results.empty()) return stats;
    
    // Count successful trials
    stats.successful_trials = std::count_if(results.begin(), results.end(),
        [](const RepositioningResult& r) { return r.planning_success; });
    
    stats.success_rate = static_cast<double>(stats.successful_trials) / stats.total_trials;
    
    // Calculate averages for successful trials only
    std::vector<double> planning_times, trajectory_lengths, smoothness_values;
    std::vector<double> joint_distances, cartesian_distances;
    
    for (const auto& result : results) {
        if (result.planning_success) {
            planning_times.push_back(result.planning_time_ms);
            trajectory_lengths.push_back(result.trajectory_length);
            smoothness_values.push_back(result.trajectory_smoothness);
            joint_distances.push_back(result.joint_space_distance);
        }
        cartesian_distances.push_back(result.cartesian_distance);
    }
    
    if (!planning_times.empty()) {
        stats.avg_planning_time_ms = std::accumulate(planning_times.begin(), planning_times.end(), 0.0) / planning_times.size();
        stats.avg_trajectory_length = std::accumulate(trajectory_lengths.begin(), trajectory_lengths.end(), 0.0) / trajectory_lengths.size();
        stats.avg_smoothness = std::accumulate(smoothness_values.begin(), smoothness_values.end(), 0.0) / smoothness_values.size();
        stats.avg_joint_distance = std::accumulate(joint_distances.begin(), joint_distances.end(), 0.0) / joint_distances.size();
        
        // Calculate standard deviations
        double sum_sq_dev = 0.0;
        for (double time : planning_times) {
            sum_sq_dev += (time - stats.avg_planning_time_ms) * (time - stats.avg_planning_time_ms);
        }
        stats.std_planning_time_ms = std::sqrt(sum_sq_dev / planning_times.size());
        
        sum_sq_dev = 0.0;
        for (double length : trajectory_lengths) {
            sum_sq_dev += (length - stats.avg_trajectory_length) * (length - stats.avg_trajectory_length);
        }
        stats.std_trajectory_length = std::sqrt(sum_sq_dev / trajectory_lengths.size());
        
        sum_sq_dev = 0.0;
        for (double smoothness : smoothness_values) {
            sum_sq_dev += (smoothness - stats.avg_smoothness) * (smoothness - stats.avg_smoothness);
        }
        stats.std_smoothness = std::sqrt(sum_sq_dev / smoothness_values.size());
    }
    
    if (!cartesian_distances.empty()) {
        stats.avg_cartesian_distance = std::accumulate(cartesian_distances.begin(), cartesian_distances.end(), 0.0) / cartesian_distances.size();
    }
    
    return stats;
}

void RepositioningEvaluator::log(const std::string& message) {
    if (config_.verbose) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto tm = *std::localtime(&time_t);
        
        std::cout << "[" << std::put_time(&tm, "%H:%M:%S") << "] " << message << std::endl;
    }
}
