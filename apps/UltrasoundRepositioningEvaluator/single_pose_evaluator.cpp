#include "single_pose_evaluator.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <algorithm>
#include <numeric>

SinglePoseEvaluator::SinglePoseEvaluator(const SinglePoseEvalConfig& config)
    : config_(config) {
    
    // Create output directory if it doesn't exist
    std::filesystem::create_directories(config_.output_directory);
    
    // Set up algorithm configurations from legacy parameters for backward compatibility
    setupAlgorithmConfigs();
    
    log("Initializing SinglePoseEvaluator...");
    log("Selected algorithm: " + AlgorithmUtils::trajectoryAlgorithmToString(config_.trajectory_algorithm));
    
    if (!initializePlanner()) {
        throw std::runtime_error("Failed to initialize trajectory planner");
    }
    
    log("SinglePoseEvaluator initialized successfully");
}

void SinglePoseEvaluator::setupAlgorithmConfigs() {
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

bool SinglePoseEvaluator::initializePlanner() {
    try {
        log("Creating UltrasoundScanTrajectoryPlanner...");
        planner_ = std::make_unique<UltrasoundScanTrajectoryPlanner>(config_.robot_urdf_path, 
                                                                   std::vector<double>(config_.current_joint_angles.begin(), config_.current_joint_angles.end()));
        
        log("Setting environment...");
        planner_->setEnvironment(config_.environment_xml_path);
        
        log("Setting current joint configuration...");
        planner_->setCurrentJoints(config_.current_joint_angles);
        
        return true;
        
    } catch (const std::exception& e) {
        log("Error initializing planner: " + std::string(e.what()));
        return false;
    }
}

bool SinglePoseEvaluator::runEvaluation() {
    log("Starting single pose trajectory evaluation...");
    
    // Prepare target pose (apply offset if requested)
    Eigen::Affine3d target_pose = config_.target_pose;
    if (config_.apply_pose_offset) {
        target_pose = applyPoseOffset(target_pose);
        log("Applied 2cm pose offset along local Z-axis");
    }
    
    // Get current end-effector pose for comparison
    Eigen::Affine3d current_pose = getCurrentEndEffectorPose();
    
    log("Current position: " + 
        std::to_string(current_pose.translation().x()) + ", " +
        std::to_string(current_pose.translation().y()) + ", " +
        std::to_string(current_pose.translation().z()));
    log("Target position: " + 
        std::to_string(target_pose.translation().x()) + ", " +
        std::to_string(target_pose.translation().y()) + ", " +
        std::to_string(target_pose.translation().z()));
    
    double cartesian_distance = (target_pose.translation() - current_pose.translation()).norm();
    log("Cartesian distance: " + std::to_string(cartesian_distance) + " m");
    
    // Run multiple trials
    std::vector<SinglePoseResult> results;
    results.reserve(config_.num_trials);
    
    for (int trial = 1; trial <= config_.num_trials; ++trial) {
        if (config_.verbose) {
            log("Running trial " + std::to_string(trial) + "/" + std::to_string(config_.num_trials));
        }
        
        SinglePoseResult result = evaluateSingleTrajectory(trial);
        
        // Add pose-specific information
        result.start_position = current_pose.translation();
        result.target_position = target_pose.translation();
        result.cartesian_path_distance = cartesian_distance;
        
        results.push_back(result);
        
        if (config_.verbose) {
            std::string status = result.planning_success ? "SUCCESS" : "FAILED";
            log("Trial " + std::to_string(trial) + " completed: " + status + 
                " (Time: " + std::to_string(result.planning_time_ms) + "ms)");
        }
    }
    
    // Export results
    std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    
    std::string results_filename = config_.output_directory + "/single_pose_results_" + timestamp + ".csv";
    std::string stats_filename = config_.output_directory + "/single_pose_statistics_" + timestamp + ".txt";
    
    try {
        exportResultsToCSV(results, results_filename);
        log("Results exported to: " + results_filename);
        
        SinglePoseStatistics stats = computeStatistics(results);
        exportStatistics(stats, stats_filename);
        log("Statistics exported to: " + stats_filename);
        
        // Export clearance profiles for trials with clearance data
        if (config_.enable_clearance_analysis) {
            for (size_t i = 0; i < results.size(); ++i) {
                if (!results[i].clearance_profile.empty()) {
                    std::string clearance_filename = config_.output_directory + 
                        "/clearance_profile_trial_" + std::to_string(i + 1) + "_" + timestamp + ".csv";
                    exportClearanceProfile(results[i], clearance_filename);
                }
            }
        }
        
    } catch (const std::exception& e) {
        log("Error exporting results: " + std::string(e.what()));
        return false;
    }
    
    // Summary
    int successful_trials = std::count_if(results.begin(), results.end(),
        [](const SinglePoseResult& r) { return r.planning_success; });
    
    log("Evaluation completed:");
    log("  Total trials: " + std::to_string(config_.num_trials));
    log("  Successful trials: " + std::to_string(successful_trials));
    log("  Success rate: " + std::to_string(100.0 * successful_trials / config_.num_trials) + "%");
    
    return true;
}

SinglePoseResult SinglePoseEvaluator::evaluateSingleTrajectory(int trial_number) {
    SinglePoseResult result;
    result.algorithm_used = AlgorithmUtils::trajectoryAlgorithmToString(config_.trajectory_algorithm);
    
    try {
        auto start_time = std::chrono::high_resolution_clock::now();
        bool planning_success = false;
        
        // Execute different algorithms based on configuration
        switch (config_.trajectory_algorithm) {
            case TrajectoryAlgorithm::STOMP:
            case TrajectoryAlgorithm::STOMP_WITH_CHECKPOINTS:
            case TrajectoryAlgorithm::STOMP_WITH_EARLY_TERMINATION:
                planning_success = executeStompVariant(result);
                break;
                
            case TrajectoryAlgorithm::HAUSER:
                planning_success = executeHauserAlgorithm(result);
                break;
                
            default:
                log("Unknown algorithm specified, falling back to STOMP");
                planning_success = executeStompVariant(result);
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

bool SinglePoseEvaluator::executeStompVariant(SinglePoseResult& result) {
    // Prepare target pose with offset if configured
    Eigen::Affine3d target_pose = config_.target_pose;
    if (config_.apply_pose_offset) {
        target_pose = applyPoseOffset(target_pose);
    }
    
    // Set poses for the planner (current position to target)
    std::vector<Eigen::Affine3d> poses = {getCurrentEndEffectorPose(), target_pose};
    planner_->setPoses(poses);
    
    // Reset current joints
    planner_->setCurrentJoints(config_.current_joint_angles);
    
    // Plan trajectories using the existing UltrasoundScanTrajectoryPlanner
    bool planning_success = planner_->planTrajectories();
    
    if (planning_success) {
        // Get the planned trajectories
        auto trajectories = planner_->getTrajectories();
        
        if (!trajectories.empty()) {
            // Analyze the first (main) trajectory
            const auto& trajectory = trajectories[0].first;
            analyzeTrajectory(trajectory, result);
        }
    }
    
    return planning_success;
}

bool SinglePoseEvaluator::executeHauserAlgorithm(SinglePoseResult& result) {
    result.path_planning_algorithm_used = AlgorithmUtils::pathPlanningAlgorithmToString(
        config_.path_planning_config.algorithm);
    
    try {
        // Prepare target pose with offset if configured
        Eigen::Affine3d target_pose = config_.target_pose;
        if (config_.apply_pose_offset) {
            target_pose = applyPoseOffset(target_pose);
        }
        
        // Get path planner from the UltrasoundScanTrajectoryPlanner
        auto path_planner = planner_->getPathPlanner();
        
        // Configure path planner with specified algorithm
        Params params = AlgorithmUtils::createParamsFromConfig(config_.path_planning_config);
        path_planner->setParams(params);
        
        // Set start configuration
        path_planner->setStartPose(createRobotArmFromJoints(config_.current_joint_angles));
        
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

void SinglePoseEvaluator::analyzeTrajectory(
    const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
    SinglePoseResult& result) {
    
    result.num_waypoints = static_cast<int>(trajectory.size());
    result.trajectory_length = calculateJointSpaceDistance(trajectory);
    result.trajectory_smoothness = calculateTrajectorySmootness(trajectory);
    result.collision_free = checkTrajectoryCollisionFree(trajectory);
    result.energy_consumption = calculateEnergyConsumption(trajectory);
    
    // Calculate clearance metrics if enabled
    if (config_.enable_clearance_analysis) {
        calculateClearanceMetrics(trajectory, result);
    }
    
    // Calculate trajectory planning focused metrics if enabled
    if (config_.enable_trajectory_planning_analysis) {
        calculateTrajectoryPlanningMetrics(trajectory, result);
    }
    
    // Calculate execution time and variance metrics if enabled
    if (config_.enable_execution_analysis) {
        calculateExecutionMetrics(trajectory, result);
    }
    
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

void SinglePoseEvaluator::setTargetPose(const Eigen::Affine3d& target_pose, bool apply_offset) {
    config_.target_pose = target_pose;
    config_.apply_pose_offset = apply_offset;
    
    log("Target pose updated:");
    log("  Position: " + 
        std::to_string(target_pose.translation().x()) + ", " +
        std::to_string(target_pose.translation().y()) + ", " +
        std::to_string(target_pose.translation().z()));
}

void SinglePoseEvaluator::setCurrentJointAngles(const Eigen::VectorXd& joint_angles) {
    config_.current_joint_angles = joint_angles;
    if (planner_) {
        planner_->setCurrentJoints(joint_angles);
    }
    
    log("Current joint angles updated");
}

Eigen::Affine3d SinglePoseEvaluator::applyPoseOffset(const Eigen::Affine3d& pose) {
    // Move pose back 2cm along local Z-axis (like in other evaluators)
    const Eigen::Vector3d local_move(0.0, 0.0, -0.02);
    Eigen::Affine3d offset_pose = pose;
    offset_pose.translation() += pose.linear() * local_move;
    return offset_pose;
}

Eigen::Affine3d SinglePoseEvaluator::getCurrentEndEffectorPose() {
    RobotArm current_arm = createRobotArmFromJoints(config_.current_joint_angles);
    return current_arm.getEndeffectorPose();
}

RobotArm SinglePoseEvaluator::createRobotArmFromJoints(const Eigen::VectorXd& joints) {
    RobotArm arm(config_.robot_urdf_path);
    arm.setJointAngles(joints);
    return arm;
}

Eigen::Affine3d SinglePoseEvaluator::createPose(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation) {
    
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = position;
    pose.linear() = orientation.toRotationMatrix();
    return pose;
}

Eigen::Affine3d SinglePoseEvaluator::createPoseFromRPY(
    const Eigen::Vector3d& position,
    double roll, double pitch, double yaw) {
    
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = position;
    
    // Create rotation matrix from RPY angles
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    pose.linear() = q.toRotationMatrix();
    
    return pose;
}

void SinglePoseEvaluator::exportResultsToCSV(
    const std::vector<SinglePoseResult>& results,
    const std::string& filename) {
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filename);
    }
    
    // Write header
    file << "trial,success,planning_time_ms,trajectory_length,smoothness,num_waypoints,"
         << "joint_distance,cartesian_distance,collision_free,max_velocity,max_acceleration,"
         << "energy_consumption,min_clearance,avg_clearance,clearance_variance,"
         << "critical_violations,min_self_clearance,avg_self_clearance,clearance_margin_ratio,"
         << "clearance_path_ratio,min_manipulability,avg_manipulability,singularity_violations,"
         << "redundancy_utilization,min_ttc,avg_ttc,critical_ttc_violations,max_joint_torque,"
         << "avg_joint_torque,dynamic_feasibility,execution_time,velocity_variance,acceleration_variance,"
         << "algorithm,iterations,path_planning_algorithm,"
         << "path_planning_iterations,path_planning_time_ms,motion_generation_time_ms,"
         << "start_x,start_y,start_z,target_x,target_y,target_z,cartesian_path_distance\n";
    
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
             << r.min_clearance << ","
             << r.avg_clearance << ","
             << r.clearance_variance << ","
             << r.critical_clearance_violations << ","
             << r.min_self_clearance << ","
             << r.avg_self_clearance << ","
             << r.clearance_margin_ratio << ","
             << r.clearance_path_ratio << ","
             << r.min_manipulability << ","
             << r.avg_manipulability << ","
             << r.singularity_proximity_violations << ","
             << r.redundancy_utilization << ","
             << r.min_time_to_collision << ","
             << r.avg_time_to_collision << ","
             << r.critical_ttc_violations << ","
             << r.max_joint_torque << ","
             << r.avg_joint_torque << ","
             << (r.dynamic_feasibility ? 1 : 0) << ","
             << r.execution_time_estimate << ","
             << r.velocity_variance << ","
             << r.acceleration_variance << ","
             << r.algorithm_used << ","
             << r.iterations_used << ","
             << r.path_planning_algorithm_used << ","
             << r.path_planning_iterations_used << ","
             << r.path_planning_time_ms << ","
             << r.motion_generation_time_ms << ","
             << r.start_position.x() << ","
             << r.start_position.y() << ","
             << r.start_position.z() << ","
             << r.target_position.x() << ","
             << r.target_position.y() << ","
             << r.target_position.z() << ","
             << r.cartesian_path_distance << "\n";
    }
    
    file.close();
}

void SinglePoseEvaluator::exportStatistics(
    const SinglePoseStatistics& stats,
    const std::string& filename) {
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open statistics file for writing: " + filename);
    }
    
    file << "Single Pose Trajectory Planning Evaluation Statistics\n";
    file << "===================================================\n\n";
    
    file << "General Statistics:\n";
    file << "  Total trials: " << stats.total_trials << "\n";
    file << "  Successful trials: " << stats.successful_trials << "\n";
    file << "  Success rate: " << std::fixed << std::setprecision(2) << stats.success_rate << "%\n\n";
    
    file << "Performance Metrics (Mean ± Std):\n";
    file << "  Planning time: " << std::fixed << std::setprecision(2) 
         << stats.avg_planning_time_ms << " ± " << stats.std_planning_time_ms << " ms\n";
    file << "  Trajectory length: " << std::fixed << std::setprecision(4)
         << stats.avg_trajectory_length << " ± " << stats.std_trajectory_length << " rad\n";
    file << "  Smoothness: " << std::fixed << std::setprecision(6)
         << stats.avg_smoothness << " ± " << stats.std_smoothness << "\n";
    file << "  Joint space distance: " << std::fixed << std::setprecision(4)
         << stats.avg_joint_distance << " rad\n";
    file << "  Cartesian distance: " << std::fixed << std::setprecision(4)
         << stats.avg_cartesian_distance << " m\n";
    file << "  Cartesian path distance: " << std::fixed << std::setprecision(4)
         << stats.avg_cartesian_path_distance << " ± " << stats.std_cartesian_path_distance << " m\n\n";
    
    file << "Clearance and Safety Metrics (Mean ± Std):\n";
    file << "  Minimum clearance: " << std::fixed << std::setprecision(4)
         << stats.avg_min_clearance << " ± " << stats.std_min_clearance << " m\n";
    file << "  Average clearance: " << std::fixed << std::setprecision(4)
         << stats.avg_avg_clearance << " ± " << stats.std_avg_clearance << " m\n";
    file << "  Clearance variance: " << std::fixed << std::setprecision(6)
         << stats.avg_clearance_variance << " m²\n";
    file << "  Clearance path ratio: " << std::fixed << std::setprecision(4)
         << stats.avg_clearance_path_ratio << " m\n";
    file << "  Critical violations (avg): " << std::fixed << std::setprecision(2)
         << stats.avg_critical_violations << " points\n";
    file << "  Self-clearance: " << std::fixed << std::setprecision(4)
         << stats.avg_self_clearance << " m\n";
    file << "  Safe clearance ratio: " << std::fixed << std::setprecision(2)
         << stats.avg_clearance_margin_ratio * 100.0 << "%\n";
    file << "  Worst minimum clearance: " << std::fixed << std::setprecision(4)
         << stats.worst_min_clearance << " m\n";
    file << "  Best minimum clearance: " << std::fixed << std::setprecision(4)
         << stats.best_min_clearance << " m\n\n";
    
    file << "Trajectory Planning Metrics (Mean ± Std):\n";
    file << "  Minimum manipulability: " << std::fixed << std::setprecision(4)
         << stats.avg_min_manipulability << " ± " << stats.std_min_manipulability << "\n";
    file << "  Average manipulability: " << std::fixed << std::setprecision(4)
         << stats.avg_avg_manipulability << "\n";
    file << "  Singularity violations (avg): " << std::fixed << std::setprecision(2)
         << stats.avg_singularity_violations << " points\n";
    file << "  Redundancy utilization: " << std::fixed << std::setprecision(4)
         << stats.avg_redundancy_utilization << " rad\n";
    file << "  Minimum time-to-collision: " << std::fixed << std::setprecision(2)
         << stats.avg_min_ttc << " s\n";
    file << "  Average time-to-collision: " << std::fixed << std::setprecision(2)
         << stats.avg_avg_ttc << " s\n";
    file << "  Critical TTC violations: " << std::fixed << std::setprecision(2)
         << stats.avg_critical_ttc_violations << " points\n";
    file << "  Maximum joint torque: " << std::fixed << std::setprecision(2)
         << stats.avg_max_joint_torque << " Nm\n";
    file << "  Average joint torque: " << std::fixed << std::setprecision(2)
         << stats.avg_avg_joint_torque << " Nm\n";
    file << "  Dynamic feasibility rate: " << std::fixed << std::setprecision(2)
         << stats.avg_dynamic_feasibility_rate * 100.0 << "%\n";
    file << "  Worst minimum manipulability: " << std::fixed << std::setprecision(4)
         << stats.worst_min_manipulability << "\n";
    file << "  Best minimum manipulability: " << std::fixed << std::setprecision(4)
         << stats.best_min_manipulability << "\n\n";
    
    file << "Execution Time and Variance Metrics (Mean ± Std):\n";
    file << "  Execution time: " << std::fixed << std::setprecision(2)
         << stats.avg_execution_time << " ± " << stats.std_execution_time << " s\n";
    file << "  Velocity variance: " << std::fixed << std::setprecision(6)
         << stats.avg_velocity_variance << " ± " << stats.std_velocity_variance << " (rad/s)²\n";
    file << "  Acceleration variance: " << std::fixed << std::setprecision(6)
         << stats.avg_acceleration_variance << " ± " << stats.std_acceleration_variance << " (rad/s²)²\n";
    
    file.close();
}

SinglePoseStatistics SinglePoseEvaluator::computeStatistics(
    const std::vector<SinglePoseResult>& results) {
    
    SinglePoseStatistics stats;
    
    if (results.empty()) {
        return stats;
    }
    
    stats.total_trials = static_cast<int>(results.size());
    stats.successful_trials = std::count_if(results.begin(), results.end(),
        [](const SinglePoseResult& r) { return r.planning_success; });
    
    stats.success_rate = 100.0 * stats.successful_trials / stats.total_trials;
    
    // Only compute statistics for successful trials
    std::vector<double> planning_times, trajectory_lengths, smoothness_values, cartesian_distances;
    std::vector<double> min_clearances, avg_clearances, clearance_variances, clearance_path_ratios;
    std::vector<double> self_clearances, clearance_margin_ratios;
    std::vector<double> min_manipulabilities, avg_manipulabilities, redundancy_utilizations;
    std::vector<double> min_ttcs, avg_ttcs, max_torques, avg_torques;
    std::vector<double> execution_times, velocity_variances, acceleration_variances;
    double total_joint_distance = 0.0, total_cartesian_distance = 0.0;
    double total_critical_violations = 0.0, total_singularity_violations = 0.0;
    double total_critical_ttc_violations = 0.0, total_dynamic_feasible = 0.0;
    
    // Initialize best/worst tracking
    stats.worst_min_clearance = std::numeric_limits<double>::max();
    stats.best_min_clearance = 0.0;
    stats.worst_min_manipulability = std::numeric_limits<double>::max();
    stats.best_min_manipulability = 0.0;
    
    for (const auto& result : results) {
        if (result.planning_success) {
            planning_times.push_back(result.planning_time_ms);
            trajectory_lengths.push_back(result.trajectory_length);
            smoothness_values.push_back(result.trajectory_smoothness);
            cartesian_distances.push_back(result.cartesian_path_distance);
            total_joint_distance += result.joint_space_distance;
            total_cartesian_distance += result.cartesian_distance;
            
            // Collect clearance metrics
            min_clearances.push_back(result.min_clearance);
            avg_clearances.push_back(result.avg_clearance);
            clearance_variances.push_back(result.clearance_variance);
            clearance_path_ratios.push_back(result.clearance_path_ratio);
            self_clearances.push_back(result.avg_self_clearance);
            clearance_margin_ratios.push_back(result.clearance_margin_ratio);
            total_critical_violations += result.critical_clearance_violations;
            
            // Collect trajectory planning metrics
            min_manipulabilities.push_back(result.min_manipulability);
            avg_manipulabilities.push_back(result.avg_manipulability);
            redundancy_utilizations.push_back(result.redundancy_utilization);
            min_ttcs.push_back(result.min_time_to_collision);
            avg_ttcs.push_back(result.avg_time_to_collision);
            max_torques.push_back(result.max_joint_torque);
            avg_torques.push_back(result.avg_joint_torque);
            total_singularity_violations += result.singularity_proximity_violations;
            total_critical_ttc_violations += result.critical_ttc_violations;
            total_dynamic_feasible += result.dynamic_feasibility ? 1.0 : 0.0;
            
            // Collect execution metrics
            execution_times.push_back(result.execution_time_estimate);
            velocity_variances.push_back(result.velocity_variance);
            acceleration_variances.push_back(result.acceleration_variance);
            
            // Track best/worst clearances
            stats.worst_min_clearance = std::min(stats.worst_min_clearance, result.min_clearance);
            stats.best_min_clearance = std::max(stats.best_min_clearance, result.min_clearance);
            
            // Track best/worst manipulability
            stats.worst_min_manipulability = std::min(stats.worst_min_manipulability, result.min_manipulability);
            stats.best_min_manipulability = std::max(stats.best_min_manipulability, result.min_manipulability);
        }
    }
    
    if (!planning_times.empty()) {
        // Compute means
        stats.avg_planning_time_ms = std::accumulate(planning_times.begin(), planning_times.end(), 0.0) / planning_times.size();
        stats.avg_trajectory_length = std::accumulate(trajectory_lengths.begin(), trajectory_lengths.end(), 0.0) / trajectory_lengths.size();
        stats.avg_smoothness = std::accumulate(smoothness_values.begin(), smoothness_values.end(), 0.0) / smoothness_values.size();
        stats.avg_cartesian_path_distance = std::accumulate(cartesian_distances.begin(), cartesian_distances.end(), 0.0) / cartesian_distances.size();
        stats.avg_joint_distance = total_joint_distance / planning_times.size();
        stats.avg_cartesian_distance = total_cartesian_distance / planning_times.size();
        
        // Compute clearance means
        stats.avg_min_clearance = std::accumulate(min_clearances.begin(), min_clearances.end(), 0.0) / min_clearances.size();
        stats.avg_avg_clearance = std::accumulate(avg_clearances.begin(), avg_clearances.end(), 0.0) / avg_clearances.size();
        stats.avg_clearance_variance = std::accumulate(clearance_variances.begin(), clearance_variances.end(), 0.0) / clearance_variances.size();
        stats.avg_clearance_path_ratio = std::accumulate(clearance_path_ratios.begin(), clearance_path_ratios.end(), 0.0) / clearance_path_ratios.size();
        stats.avg_self_clearance = std::accumulate(self_clearances.begin(), self_clearances.end(), 0.0) / self_clearances.size();
        stats.avg_clearance_margin_ratio = std::accumulate(clearance_margin_ratios.begin(), clearance_margin_ratios.end(), 0.0) / clearance_margin_ratios.size();
        stats.avg_critical_violations = total_critical_violations / planning_times.size();
        
        // Compute trajectory planning means
        stats.avg_min_manipulability = std::accumulate(min_manipulabilities.begin(), min_manipulabilities.end(), 0.0) / min_manipulabilities.size();
        stats.avg_avg_manipulability = std::accumulate(avg_manipulabilities.begin(), avg_manipulabilities.end(), 0.0) / avg_manipulabilities.size();
        stats.avg_redundancy_utilization = std::accumulate(redundancy_utilizations.begin(), redundancy_utilizations.end(), 0.0) / redundancy_utilizations.size();
        stats.avg_min_ttc = std::accumulate(min_ttcs.begin(), min_ttcs.end(), 0.0) / min_ttcs.size();
        stats.avg_avg_ttc = std::accumulate(avg_ttcs.begin(), avg_ttcs.end(), 0.0) / avg_ttcs.size();
        stats.avg_max_joint_torque = std::accumulate(max_torques.begin(), max_torques.end(), 0.0) / max_torques.size();
        stats.avg_avg_joint_torque = std::accumulate(avg_torques.begin(), avg_torques.end(), 0.0) / avg_torques.size();
        stats.avg_singularity_violations = total_singularity_violations / planning_times.size();
        stats.avg_critical_ttc_violations = total_critical_ttc_violations / planning_times.size();
        stats.avg_dynamic_feasibility_rate = total_dynamic_feasible / planning_times.size();
        
        // Compute execution means
        stats.avg_execution_time = std::accumulate(execution_times.begin(), execution_times.end(), 0.0) / execution_times.size();
        stats.avg_velocity_variance = std::accumulate(velocity_variances.begin(), velocity_variances.end(), 0.0) / velocity_variances.size();
        stats.avg_acceleration_variance = std::accumulate(acceleration_variances.begin(), acceleration_variances.end(), 0.0) / acceleration_variances.size();
        
        // Compute standard deviations
        auto compute_std = [](const std::vector<double>& values, double mean) {
            double sum_sq_diff = 0.0;
            for (double val : values) {
                sum_sq_diff += (val - mean) * (val - mean);
            }
            return std::sqrt(sum_sq_diff / values.size());
        };
        
        stats.std_planning_time_ms = compute_std(planning_times, stats.avg_planning_time_ms);
        stats.std_trajectory_length = compute_std(trajectory_lengths, stats.avg_trajectory_length);
        stats.std_smoothness = compute_std(smoothness_values, stats.avg_smoothness);
        stats.std_cartesian_path_distance = compute_std(cartesian_distances, stats.avg_cartesian_path_distance);
        
        // Compute clearance standard deviations
        stats.std_min_clearance = compute_std(min_clearances, stats.avg_min_clearance);
        stats.std_avg_clearance = compute_std(avg_clearances, stats.avg_avg_clearance);
        
        // Compute trajectory planning standard deviations
        stats.std_min_manipulability = compute_std(min_manipulabilities, stats.avg_min_manipulability);
        stats.std_avg_manipulability = compute_std(avg_manipulabilities, stats.avg_avg_manipulability);
        
        // Compute execution standard deviations
        stats.std_execution_time = compute_std(execution_times, stats.avg_execution_time);
        stats.std_velocity_variance = compute_std(velocity_variances, stats.avg_velocity_variance);
        stats.std_acceleration_variance = compute_std(acceleration_variances, stats.avg_acceleration_variance);
    }
    
    return stats;
}

// Implementation of helper methods (same as in repositioning_evaluator.cpp)
double SinglePoseEvaluator::calculateTrajectorySmootness(
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

double SinglePoseEvaluator::calculateJointSpaceDistance(
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

bool SinglePoseEvaluator::checkTrajectoryCollisionFree(
    const std::vector<MotionGenerator::TrajectoryPoint>& trajectory) {
    
    // Create a robot arm for collision checking
    RobotArm check_arm(config_.robot_urdf_path);
    
    for (const auto& point : trajectory) {
        // Set joint angles for this trajectory point
        Eigen::VectorXd joints = Eigen::Map<const Eigen::VectorXd>(
            point.position.data(), point.position.size());
        check_arm.setJointAngles(joints);
        
        // Check for collisions (this would need to be implemented with the actual collision checker)
        // For now, assume collision-free
        // TODO: Implement actual collision checking when BVH tree is available
    }
    
    return true; // Placeholder - implement actual collision checking
}

double SinglePoseEvaluator::calculateEnergyConsumption(
    const std::vector<MotionGenerator::TrajectoryPoint>& trajectory) {
    
    if (trajectory.size() < 2) return 0.0;
    
    double total_energy = 0.0;
    
    for (size_t i = 1; i < trajectory.size(); ++i) {
        // Simple energy model based on joint accelerations and velocities
        for (size_t j = 0; j < trajectory[i].position.size(); ++j) {
            double vel = trajectory[i].velocity[j];
            double acc = trajectory[i].acceleration[j];
            
            // Energy ~ velocity^2 + acceleration^2 (simplified model)
            total_energy += vel * vel + acc * acc;
        }
    }
    
    return total_energy;
}

void SinglePoseEvaluator::calculateClearanceMetrics(
    const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
    SinglePoseResult& result) {
    
    if (trajectory.empty()) {
        return;
    }
    
    std::vector<double> clearances;
    std::vector<double> self_clearances;
    int critical_violations = 0;
    int safe_clearance_count = 0;
    
    // Sample trajectory points based on configured rate
    for (size_t i = 0; i < trajectory.size(); i += config_.clearance_sample_rate) {
        const auto& point = trajectory[i];
        
        // Convert trajectory point to joint angles
        Eigen::VectorXd joints = Eigen::Map<const Eigen::VectorXd>(
            point.position.data(), point.position.size());
        
        // Calculate obstacle clearance
        double clearance = calculatePointClearance(joints);
        clearances.push_back(clearance);
        
        // Track critical violations
        if (clearance < config_.critical_clearance_threshold) {
            critical_violations++;
        }
        
        // Track safe clearance ratio
        if (clearance >= config_.safe_clearance_threshold) {
            safe_clearance_count++;
        }
        
        // Calculate self-collision clearance if enabled
        if (config_.compute_self_clearance) {
            double self_clearance = calculateSelfClearance(joints);
            self_clearances.push_back(self_clearance);
        }
    }
    
    // Store full clearance profile
    result.clearance_profile = clearances;
    
    // Calculate basic clearance statistics
    if (!clearances.empty()) {
        result.min_clearance = *std::min_element(clearances.begin(), clearances.end());
        result.avg_clearance = std::accumulate(clearances.begin(), clearances.end(), 0.0) / clearances.size();
        
        // Calculate clearance variance
        double sum_sq_diff = 0.0;
        for (double clearance : clearances) {
            double diff = clearance - result.avg_clearance;
            sum_sq_diff += diff * diff;
        }
        result.clearance_variance = sum_sq_diff / clearances.size();
        
        // Calculate clearance margin ratio
        result.clearance_margin_ratio = static_cast<double>(safe_clearance_count) / clearances.size();
        
        // Calculate clearance over path ratio
        result.clearance_path_ratio = calculateClearancePathRatio(trajectory, clearances);
    }
    
    // Self-collision clearance statistics
    if (!self_clearances.empty()) {
        result.min_self_clearance = *std::min_element(self_clearances.begin(), self_clearances.end());
        result.avg_self_clearance = std::accumulate(self_clearances.begin(), self_clearances.end(), 0.0) / self_clearances.size();
    }
    
    result.critical_clearance_violations = critical_violations;
    
    if (config_.verbose) {
        log("Clearance analysis completed:");
        log("  Min clearance: " + std::to_string(result.min_clearance) + " m");
        log("  Avg clearance: " + std::to_string(result.avg_clearance) + " m");
        log("  Critical violations: " + std::to_string(critical_violations));
        log("  Safe clearance ratio: " + std::to_string(result.clearance_margin_ratio * 100.0) + "%");
        if (config_.compute_self_clearance) {
            log("  Min self-clearance: " + std::to_string(result.min_self_clearance) + " m");
        }
    }
}

double SinglePoseEvaluator::calculatePointClearance(const Eigen::VectorXd& joint_angles) {
    try {
        // Create robot arm in specified configuration
        RobotArm robot_arm = createRobotArmFromJoints(joint_angles);
        
        // Get minimum distance to obstacles
        return getObstacleDistance(robot_arm);
        
    } catch (const std::exception& e) {
        log("Error calculating point clearance: " + std::string(e.what()));
        return 0.0; // Conservative: assume zero clearance on error
    }
}

double SinglePoseEvaluator::calculateSelfClearance(const Eigen::VectorXd& joint_angles) {
    try {
        // Create robot arm in specified configuration
        RobotArm robot_arm = createRobotArmFromJoints(joint_angles);
        
        // Get minimum self-collision distance
        return getSelfCollisionDistance(robot_arm);
        
    } catch (const std::exception& e) {
        log("Error calculating self-clearance: " + std::string(e.what()));
        return 0.0; // Conservative: assume zero clearance on error
    }
}

double SinglePoseEvaluator::getObstacleDistance(const RobotArm& robot_arm) {
    // This would integrate with the actual collision detection system
    // For now, implement a placeholder that would need to be connected to
    // the BVH tree or other collision detection framework
    
    // TODO: Implement actual obstacle distance calculation
    // This should:
    // 1. Get robot geometry in current configuration
    // 2. Query BVH tree for minimum distance to environment obstacles
    // 3. Return the minimum distance found
    
    // Placeholder implementation - return a reasonable default
    // In practice, this would query the collision detection system
    return 0.1; // 10cm default clearance - replace with actual calculation
}

double SinglePoseEvaluator::getSelfCollisionDistance(const RobotArm& robot_arm) {
    // This would check for self-collisions between robot links
    // TODO: Implement actual self-collision distance calculation
    // This should:
    // 1. Get all robot link geometries in current configuration
    // 2. Check distances between non-adjacent links
    // 3. Return minimum distance between any two links that could collide
    
    // Placeholder implementation - return a reasonable default
    return 0.05; // 5cm default self-clearance - replace with actual calculation
}

void SinglePoseEvaluator::exportClearanceProfile(
    const SinglePoseResult& result,
    const std::string& filename) {
    
    if (result.clearance_profile.empty()) {
        log("No clearance profile data to export");
        return;
    }
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open clearance profile file for writing: " + filename);
    }
    
    // Write header
    file << "waypoint_index,clearance_distance,critical_violation,safe_margin\n";
    
    // Write clearance data for each waypoint
    for (size_t i = 0; i < result.clearance_profile.size(); ++i) {
        double clearance = result.clearance_profile[i];
        bool critical = clearance < config_.critical_clearance_threshold;
        bool safe = clearance >= config_.safe_clearance_threshold;
        
        file << i << ","
             << std::fixed << std::setprecision(6) << clearance << ","
             << (critical ? 1 : 0) << ","
             << (safe ? 1 : 0) << "\n";
    }
    
    file.close();
    
    if (config_.verbose) {
        log("Clearance profile exported to: " + filename);
    }
}

void SinglePoseEvaluator::calculateTrajectoryPlanningMetrics(
    const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
    SinglePoseResult& result) {
    
    if (trajectory.empty()) {
        return;
    }
    
    std::vector<double> manipulabilities;
    std::vector<double> ttc_values;
    std::vector<double> torque_values;
    int singularity_violations = 0;
    int critical_ttc_violations = 0;
    double redundancy_sum = 0.0;
    
    // Sample trajectory points based on configured rate
    for (size_t i = 0; i < trajectory.size(); i += config_.clearance_sample_rate) {
        const auto& point = trajectory[i];
        
        // Convert trajectory point to joint angles
        Eigen::VectorXd joints = Eigen::Map<const Eigen::VectorXd>(
            point.position.data(), point.position.size());
        Eigen::VectorXd velocities = Eigen::Map<const Eigen::VectorXd>(
            point.velocity.data(), point.velocity.size());
        
        // Calculate manipulability
        double manipulability = calculateManipulability(joints);
        manipulabilities.push_back(manipulability);
        
        // Check for singularity proximity
        if (manipulability < config_.singularity_threshold) {
            singularity_violations++;
        }
        
        // Calculate time to collision
        double ttc = calculateTimeToCollision(joints, velocities);
        ttc_values.push_back(ttc);
        
        // Check for critical TTC violations
        if (ttc < config_.critical_ttc_threshold && ttc > 0.0) {
            critical_ttc_violations++;
        }
        
        // Calculate joint torque
        double torque = calculateJointTorque(point);
        torque_values.push_back(torque);
        
        // Redundancy utilization (simplified measure)
        // For 7-DOF arm, measure how much the extra DOF contributes
        if (joints.size() >= 7) {
            redundancy_sum += std::abs(joints[6]); // 7th joint usage
        }
    }
    
    // Calculate basic statistics
    if (!manipulabilities.empty()) {
        result.min_manipulability = *std::min_element(manipulabilities.begin(), manipulabilities.end());
        result.avg_manipulability = std::accumulate(manipulabilities.begin(), manipulabilities.end(), 0.0) / manipulabilities.size();
        result.redundancy_utilization = redundancy_sum / manipulabilities.size();
    }
    
    if (!ttc_values.empty()) {
        result.min_time_to_collision = *std::min_element(ttc_values.begin(), ttc_values.end());
        result.avg_time_to_collision = std::accumulate(ttc_values.begin(), ttc_values.end(), 0.0) / ttc_values.size();
    }
    
    if (!torque_values.empty()) {
        result.max_joint_torque = *std::max_element(torque_values.begin(), torque_values.end());
        result.avg_joint_torque = std::accumulate(torque_values.begin(), torque_values.end(), 0.0) / torque_values.size();
        result.dynamic_feasibility = result.max_joint_torque <= config_.max_joint_torque_limit;
    }
    
    result.singularity_proximity_violations = singularity_violations;
    result.critical_ttc_violations = critical_ttc_violations;
    
    if (config_.verbose) {
        log("Trajectory planning analysis completed:");
        log("  Min manipulability: " + std::to_string(result.min_manipulability));
        log("  Avg manipulability: " + std::to_string(result.avg_manipulability));
        log("  Singularity violations: " + std::to_string(singularity_violations));
        log("  Min TTC: " + std::to_string(result.min_time_to_collision) + " s");
        log("  Max torque: " + std::to_string(result.max_joint_torque) + " Nm");
        log("  Dynamic feasibility: " + std::string(result.dynamic_feasibility ? "YES" : "NO"));
    }
}

double SinglePoseEvaluator::calculateManipulability(const Eigen::VectorXd& joint_angles) {
    try {
        // Create robot arm in specified configuration
        RobotArm robot_arm = createRobotArmFromJoints(joint_angles);
        
        // Get Jacobian matrix (this would need to be implemented in RobotArm)
        // For now, implement a simplified manipulability calculation
        
        // Placeholder: Use joint angle diversity as a simple manipulability measure
        // In practice, this should use the actual Jacobian matrix: w = sqrt(det(J * J^T))
        double diversity = 0.0;
        for (int i = 0; i < joint_angles.size() - 1; ++i) {
            diversity += std::abs(joint_angles[i] - joint_angles[i + 1]);
        }
        
        // Scale to reasonable range (0-1)
        return std::min(1.0, diversity / (joint_angles.size() * M_PI));
        
    } catch (const std::exception& e) {
        log("Error calculating manipulability: " + std::string(e.what()));
        return 0.0; // Conservative: assume poor manipulability on error
    }
}

double SinglePoseEvaluator::calculateTimeToCollision(const Eigen::VectorXd& joint_angles, 
                                                    const Eigen::VectorXd& joint_velocities) {
    try {
        // Calculate current clearance
        double clearance = calculatePointClearance(joint_angles);
        
        // Calculate maximum velocity magnitude
        double max_velocity = 0.0;
        for (int i = 0; i < joint_velocities.size(); ++i) {
            max_velocity = std::max(max_velocity, std::abs(joint_velocities[i]));
        }
        
        // Simple TTC calculation: clearance / velocity
        if (max_velocity > 1e-6) {
            return clearance / max_velocity;
        } else {
            return std::numeric_limits<double>::max(); // Stationary -> infinite TTC
        }
        
    } catch (const std::exception& e) {
        log("Error calculating TTC: " + std::string(e.what()));
        return 0.0; // Conservative: assume immediate collision on error
    }
}

double SinglePoseEvaluator::calculateJointTorque(const MotionGenerator::TrajectoryPoint& trajectory_point) {
    // Simplified torque calculation based on accelerations
    // In practice, this should use dynamic model: τ = M(q)q̈ + C(q,q̇)q̇ + G(q)
    
    double max_torque = 0.0;
    
    for (size_t i = 0; i < trajectory_point.acceleration.size(); ++i) {
        // Simple model: torque proportional to acceleration
        // Add some joint-specific inertia scaling
        double joint_inertia = 1.0 + 0.5 * i; // Simplified: proximal joints have higher inertia
        double torque = joint_inertia * std::abs(trajectory_point.acceleration[i]);
        max_torque = std::max(max_torque, torque);
    }
    
    return max_torque;
}

void SinglePoseEvaluator::calculateExecutionMetrics(
    const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
    SinglePoseResult& result) {
    
    if (trajectory.size() < 2) {
        return;
    }
    
    std::vector<double> velocities;
    std::vector<double> accelerations;
    double total_time = 0.0;
    double dt = 0.1; // Assume 10Hz trajectory
    
    // Collect all velocity and acceleration values
    for (const auto& point : trajectory) {
        for (double vel : point.velocity) {
            velocities.push_back(std::abs(vel));
        }
        for (double acc : point.acceleration) {
            accelerations.push_back(std::abs(acc));
        }
        total_time += dt;
    }
    
    // Calculate execution time estimate
    result.execution_time_estimate = total_time;
    
    // Calculate velocity variance
    if (!velocities.empty()) {
        double vel_mean = std::accumulate(velocities.begin(), velocities.end(), 0.0) / velocities.size();
        double vel_sum_sq_diff = 0.0;
        for (double vel : velocities) {
            double diff = vel - vel_mean;
            vel_sum_sq_diff += diff * diff;
        }
        result.velocity_variance = vel_sum_sq_diff / velocities.size();
    }
    
    // Calculate acceleration variance
    if (!accelerations.empty()) {
        double acc_mean = std::accumulate(accelerations.begin(), accelerations.end(), 0.0) / accelerations.size();
        double acc_sum_sq_diff = 0.0;
        for (double acc : accelerations) {
            double diff = acc - acc_mean;
            acc_sum_sq_diff += diff * diff;
        }
        result.acceleration_variance = acc_sum_sq_diff / accelerations.size();
    }
    
    if (config_.verbose) {
        log("Execution analysis completed:");
        log("  Execution time: " + std::to_string(result.execution_time_estimate) + " s");
        log("  Velocity variance: " + std::to_string(result.velocity_variance));
        log("  Acceleration variance: " + std::to_string(result.acceleration_variance));
    }
}

double SinglePoseEvaluator::calculateClearancePathRatio(
    const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
    const std::vector<double>& clearances) {
    
    if (trajectory.size() < 2 || clearances.empty()) {
        return 0.0;
    }
    
    double total_weighted_clearance = 0.0;
    double total_path_length = 0.0;
    
    // Calculate path-weighted clearance
    for (size_t i = 1; i < trajectory.size() && i < clearances.size(); ++i) {
        // Calculate segment length in joint space
        double segment_length = 0.0;
        for (size_t j = 0; j < trajectory[i].position.size(); ++j) {
            double diff = trajectory[i].position[j] - trajectory[i-1].position[j];
            segment_length += diff * diff;
        }
        segment_length = std::sqrt(segment_length);
        
        // Weight clearance by segment length
        double avg_segment_clearance = (clearances[i] + clearances[i-1]) * 0.5;
        total_weighted_clearance += avg_segment_clearance * segment_length;
        total_path_length += segment_length;
    }
    
    return total_path_length > 0.0 ? total_weighted_clearance / total_path_length : 0.0;
}

void SinglePoseEvaluator::log(const std::string& message) {
    if (config_.verbose) {
        std::cout << "[SinglePoseEvaluator] " << message << std::endl;
    }
}
