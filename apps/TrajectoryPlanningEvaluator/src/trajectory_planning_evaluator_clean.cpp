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
    // Initialize home configuration (7-DOF robot arm)
    home_joint_config_ = Eigen::VectorXd::Zero(7);
    home_joint_config_ << 0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.785;  // Default home position
    
    std::cout << "TrajectoryPlanningEvaluator initialized with real trajectory planning algorithms" << std::endl;
}

TrajectoryPlanningEvaluator::~TrajectoryPlanningEvaluator() {
    // Clean destructor
}

bool TrajectoryPlanningEvaluator::initializeEnvironment(const std::string& urdf_path, const std::string& env_path) {
    try {
        // Initialize robot arm
        robot_arm_ = std::make_unique<RobotArm>(urdf_path);
        
        // Initialize environment and obstacle tree
        RobotManager robot_manager;
        robot_manager.parseURDF(env_path);
        obstacle_tree_ = std::make_shared<BVHTree>(robot_manager.getTransformedObstacles());
        
        // Initialize motion generator with robot arm and obstacle tree
        motion_generator_ = std::make_unique<MotionGenerator>(*robot_arm_);
        motion_generator_->setObstacleTree(obstacle_tree_);
        
        // Initialize path planner with robot arm and obstacle tree
        path_planner_ = std::make_unique<PathPlanner>();
        path_planner_->setStartPose(*robot_arm_);
        path_planner_->setObstacleTree(obstacle_tree_);
        
        // Set home configuration
        robot_arm_->setJointAngles(home_joint_config_);
        
        std::cout << "Environment initialized successfully with direct MotionGenerator and PathPlanner" << std::endl;
        std::cout << "Robot URDF: " << urdf_path << std::endl;
        std::cout << "Environment: " << env_path << std::endl;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing environment: " << e.what() << std::endl;
        return false;
    }
}

bool TrajectoryPlanningEvaluator::loadPosesFromCSV(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open poses file: " << filename << std::endl;
        return false;
    }
    
    target_poses_.clear();
    std::string line;
    
    // Read poses line by line (same format as ComparisonIK)
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;
        
        while (std::getline(ss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing pose data: " << e.what() << std::endl;
                continue;
            }
        }
        
        if (values.size() >= 7) {
            // Parse position and quaternion (same as ComparisonIK)
            Eigen::Vector3d position(values[0], values[1], values[2]);
            Eigen::Quaterniond quaternion(values[3], values[4], values[5], values[6]);
            quaternion.normalize();

            Eigen::Affine3d pose = Eigen::Affine3d::Identity();
            pose.linear() = quaternion.toRotationMatrix();
            pose.translation() = position;
            
            // Apply -2cm local z-axis transformation (same as ComparisonIK)
            const Eigen::Vector3d local_move(0.0, 0.0, -0.02);
            pose.translation() += pose.rotation() * local_move;
            
            target_poses_.push_back(pose);
        }
    }
    
    std::cout << "Loaded " << target_poses_.size() << " poses from " << filename << std::endl;
    return !target_poses_.empty();
}

std::vector<Eigen::VectorXd> TrajectoryPlanningEvaluator::loadPosesFromComparisonIK(const std::string& filename) {
    std::vector<Eigen::VectorXd> poses;
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "❌ Could not open ComparisonIK poses file: " << filename << std::endl;
        return poses;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;
        
        std::stringstream ss(line);
        std::string item;
        std::vector<double> values;
        
        while (std::getline(ss, item, ',')) {
            try {
                values.push_back(std::stod(item));
            } catch (const std::exception& e) {
                // Skip invalid values
                continue;
            }
        }
        
        if (values.size() >= 7) {
            // ComparisonIK format: x, y, z, qw, qx, qy, qz, ... (additional values ignored)
            Eigen::VectorXd pose(7);
            pose[0] = values[0];  // x
            pose[1] = values[1];  // y
            pose[2] = values[2];  // z
            pose[3] = values[3];  // qw
            pose[4] = values[4];  // qx
            pose[5] = values[5];  // qy
            pose[6] = values[6];  // qz
            
            // Apply same transformation as ComparisonIK (-2cm local z-axis)
            // For now, just store the pose as-is since we're simulating
            poses.push_back(pose);
        }
    }
    
    file.close();
    std::cout << "📋 Loaded " << poses.size() << " poses from ComparisonIK format" << std::endl;
    return poses;
}

EvaluationResult TrajectoryPlanningEvaluator::evaluateStompTrial(
    const StompConfig& config,
    const Eigen::VectorXd& goal_pose,
    const Eigen::VectorXd& start_joints,
    int trial_id, int pose_id) {
    
    EvaluationResult result;
    result.trial_id = trial_id;
    result.pose_id = pose_id;
    result.algorithm = EvaluatorAlgorithm::STOMP;
    result.start_joints = start_joints;
    result.goal_pose = goal_pose;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // Convert goal pose to joint configuration using inverse kinematics
        auto [goal_joints, ik_success] = poseToJoints(goal_pose);
        
        if (!ik_success) {
            // IK failed
            result.success = false;
            result.planning_time_ms = 0.0;
            result.execution_time_ms = 0.0;
            result.iterations_used = 0;
            result.trajectory_length = -1.0;
            result.path_quality = -1.0;
            result.collision_clearance = -1.0;
            result.joint_smoothness = -1.0;
            
            std::stringstream config_str;
            config_str << "noisy:" << config.numNoisyTrajectories 
                       << ",iter:" << config.maxIterations
                       << ",lr:" << std::fixed << std::setprecision(2) << config.learningRate;
            result.algorithm_config = config_str.str();
            
            return result;
        }
        
        // Run STOMP planning
        auto [trajectory, planning_success] = runStompPlanning(start_joints, goal_joints, config);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        result.planning_time_ms = duration.count() / 1000.0;
        
        result.success = planning_success;
        
        if (planning_success && !trajectory.empty()) {
            // Convert TrajectoryPoint vector to VectorXd vector for analysis
            std::vector<Eigen::VectorXd> joint_trajectory;
            joint_trajectory.reserve(trajectory.size());
            
            for (const auto& point : trajectory) {
                // TrajectoryPoint has std::vector<double> position
                Eigen::VectorXd joint_config(point.position.size());
                for (size_t i = 0; i < point.position.size(); ++i) {
                    joint_config[i] = point.position[i];
                }
                joint_trajectory.push_back(joint_config);
            }
            
            // Calculate metrics
            result.execution_time_ms = trajectory.size() * config.dt * 1000.0; // Convert to ms
            result.iterations_used = config.maxIterations; // STOMP uses fixed iterations
            
            // Calculate trajectory length
            result.trajectory_length = 0.0;
            for (size_t i = 1; i < joint_trajectory.size(); ++i) {
                result.trajectory_length += (joint_trajectory[i] - joint_trajectory[i-1]).norm();
            }
            
            // Calculate quality metrics
            result.path_quality = calculatePathQuality(joint_trajectory);
            result.joint_smoothness = calculateJointSmoothness(joint_trajectory);
            result.collision_clearance = calculateCollisionClearance(joint_trajectory);
            
        } else {
            // Planning failed
            result.execution_time_ms = -1.0;
            result.iterations_used = config.maxIterations;
            result.trajectory_length = -1.0;
            result.path_quality = -1.0;
            result.collision_clearance = -1.0;
            result.joint_smoothness = -1.0;
        }
        
        // Create configuration string
        std::stringstream config_str;
        config_str << "noisy:" << config.numNoisyTrajectories 
                   << ",iter:" << config.maxIterations
                   << ",lr:" << std::fixed << std::setprecision(2) << config.learningRate
                   << ",temp:" << std::fixed << std::setprecision(1) << config.temperature;
        result.algorithm_config = config_str.str();
        
    } catch (const std::exception& e) {
        std::cerr << "Error in STOMP evaluation: " << e.what() << std::endl;
        result.success = false;
        result.planning_time_ms = -1.0;
        result.execution_time_ms = -1.0;
        result.iterations_used = 0;
        result.trajectory_length = -1.0;
        result.path_quality = -1.0;
        result.collision_clearance = -1.0;
        result.joint_smoothness = -1.0;
        result.algorithm_config = "ERROR";
    }
    
    return result;
}

EvaluationResult TrajectoryPlanningEvaluator::evaluateHauserTrial(
    EvaluatorAlgorithm algorithm,
    const Params& params,
    const Eigen::VectorXd& goal_pose,
    const Eigen::VectorXd& start_joints,
    int trial_id, int pose_id) {
    
    EvaluationResult result;
    result.trial_id = trial_id;
    result.pose_id = pose_id;
    result.algorithm = algorithm;
    result.start_joints = start_joints;
    result.goal_pose = goal_pose;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // Convert goal pose to joint configuration using inverse kinematics
        auto [goal_joints, ik_success] = poseToJoints(goal_pose);
        
        if (!ik_success) {
            // IK failed
            result.success = false;
            result.planning_time_ms = 0.0;
            result.execution_time_ms = 0.0;
            result.iterations_used = 0;
            result.trajectory_length = -1.0;
            result.path_quality = -1.0;
            result.collision_clearance = -1.0;
            result.joint_smoothness = -1.0;
            
            // Create configuration string
            std::stringstream config_str;
            config_str << "algo:" << static_cast<int>(params.algo)
                       << ",step:" << std::fixed << std::setprecision(3) << params.stepSize
                       << ",iter:" << params.maxIterations
                       << ",bias:" << std::fixed << std::setprecision(2) << params.goalBiasProbability;
            result.algorithm_config = config_str.str();
            
            return result;
        }
        
        // Create parameters copy with the specific algorithm
        Params hauser_params = params;
        
        // Map our Algorithm enum to the library's Algorithm enum
        switch (algorithm) {
            case EvaluatorAlgorithm::HAUSER_RRT:
                hauser_params.algo = ::Algorithm::RRT;
                break;
            case EvaluatorAlgorithm::HAUSER_RRT_STAR:
                hauser_params.algo = ::Algorithm::RRTStar;
                break;
            case EvaluatorAlgorithm::HAUSER_INFORMED_RRT:
                hauser_params.algo = ::Algorithm::InformedRRTStar;
                break;
            case EvaluatorAlgorithm::HAUSER_BI_RRT:
                hauser_params.algo = ::Algorithm::RRTConnect;
                break;
            default:
                hauser_params.algo = ::Algorithm::RRT;
                break;
        }
        
        // Run Hauser planning
        auto [trajectory, planning_success] = runHauserPlanning(start_joints, goal_joints, hauser_params);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        result.planning_time_ms = duration.count() / 1000.0;
        
        result.success = planning_success;
        
        if (planning_success && !trajectory.empty()) {
            // Calculate metrics
            result.execution_time_ms = trajectory.size() * 0.1 * 1000.0; // Assume 0.1s per waypoint
            result.iterations_used = params.maxIterations; // Hauser may use fewer, but this is max
            
            // Calculate trajectory length
            result.trajectory_length = 0.0;
            for (size_t i = 1; i < trajectory.size(); ++i) {
                result.trajectory_length += (trajectory[i] - trajectory[i-1]).norm();
            }
            
            // Calculate quality metrics
            result.path_quality = calculatePathQuality(trajectory);
            result.joint_smoothness = calculateJointSmoothness(trajectory);
            result.collision_clearance = calculateCollisionClearance(trajectory);
            
        } else {
            // Planning failed
            result.execution_time_ms = -1.0;
            result.iterations_used = params.maxIterations;
            result.trajectory_length = -1.0;
            result.path_quality = -1.0;
            result.collision_clearance = -1.0;
            result.joint_smoothness = -1.0;
        }
        
        // Create configuration string
        std::stringstream config_str;
        config_str << "algo:" << static_cast<int>(params.algo)
                   << ",step:" << std::fixed << std::setprecision(3) << params.stepSize
                   << ",iter:" << params.maxIterations
                   << ",bias:" << std::fixed << std::setprecision(2) << params.goalBiasProbability;
        result.algorithm_config = config_str.str();
        
    } catch (const std::exception& e) {
        std::cerr << "Error in Hauser evaluation: " << e.what() << std::endl;
        result.success = false;
        result.planning_time_ms = -1.0;
        result.execution_time_ms = -1.0;
        result.iterations_used = 0;
        result.trajectory_length = -1.0;
        result.path_quality = -1.0;
        result.collision_clearance = -1.0;
        result.joint_smoothness = -1.0;
        result.algorithm_config = "ERROR";
    }
    
    return result;
}

std::pair<Eigen::VectorXd, bool> TrajectoryPlanningEvaluator::poseToJoints(const Eigen::VectorXd& pose) {
    if (!robot_arm_) {
        return {Eigen::VectorXd::Zero(7), false};
    }
    
    try {
        // Convert pose vector to Affine3d
        Eigen::Affine3d target_pose = Eigen::Affine3d::Identity();
        target_pose.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
        
        // Quaternion: w, x, y, z
        Eigen::Quaterniond quat(pose[3], pose[4], pose[5], pose[6]);
        quat.normalize();
        target_pose.linear() = quat.toRotationMatrix();
        
        // Apply -2cm local z-axis transformation (same as ComparisonIK)
        const Eigen::Vector3d local_move(0.0, 0.0, -0.02);
        target_pose.translation() += target_pose.rotation() * local_move;
        
        // Use IK to solve for joint configuration
        auto joint_result = robot_arm_->inverseKinematics(target_pose);
        if (joint_result.has_value()) {
            return {joint_result.value(), true};
        } else {
            return {Eigen::VectorXd::Zero(7), false};
        }
    } catch (const std::exception& e) {
        std::cerr << "Error in pose to joints conversion: " << e.what() << std::endl;
        return {Eigen::VectorXd::Zero(7), false};
    }
}

std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool> TrajectoryPlanningEvaluator::runStompPlanning(
    const Eigen::VectorXd& start_joints,
    const Eigen::VectorXd& goal_joints,
    const StompConfig& config) {
    
    if (!motion_generator_) {
        return {std::vector<MotionGenerator::TrajectoryPoint>(), false};
    }
    
    try {
        // Set waypoints (start and goal)
        Eigen::MatrixXd waypoints(2, start_joints.size());
        waypoints.row(0) = start_joints.transpose();
        waypoints.row(1) = goal_joints.transpose();
        
        motion_generator_->setWaypoints(waypoints);
        
        // Create thread pool for STOMP
        unsigned int numThreads = std::thread::hardware_concurrency();
        auto threadPool = std::make_shared<boost::asio::thread_pool>(numThreads);
        
        // Run STOMP algorithm
        bool success = motion_generator_->performSTOMP(config, threadPool);
        
        // Wait for completion
        threadPool->join();
        
        if (success) {
            auto trajectory = motion_generator_->getPath();
            return {trajectory, true};
        } else {
            return {std::vector<MotionGenerator::TrajectoryPoint>(), false};
        }
    } catch (const std::exception& e) {
        std::cerr << "Error in STOMP planning: " << e.what() << std::endl;
        return {std::vector<MotionGenerator::TrajectoryPoint>(), false};
    }
}

std::pair<std::vector<Eigen::VectorXd>, bool> TrajectoryPlanningEvaluator::runHauserPlanning(
    const Eigen::VectorXd& start_joints,
    const Eigen::VectorXd& goal_joints,
    const Params& params) {
    
    if (!path_planner_) {
        return {std::vector<Eigen::VectorXd>(), false};
    }
    
    try {
        // Set start configuration
        RobotArm start_arm = *robot_arm_;
        start_arm.setJointAngles(start_joints);
        path_planner_->setStartPose(start_arm);
        
        // Set goal configuration  
        RobotArm goal_arm = *robot_arm_;
        goal_arm.setJointAngles(goal_joints);
        path_planner_->setGoalConfiguration(goal_arm);
        
        // Set parameters
        path_planner_->setParams(params);
        
        // Run path finding
        bool success = path_planner_->runPathFinding();
        
        if (success) {
            // Get the path as joint angles
            Eigen::MatrixXd path_matrix = path_planner_->getAnglesPath();
            
            // Convert matrix to vector of VectorXd
            std::vector<Eigen::VectorXd> trajectory;
            trajectory.reserve(path_matrix.rows());
            
            for (int i = 0; i < path_matrix.rows(); ++i) {
                trajectory.push_back(path_matrix.row(i));
            }
            
            return {trajectory, true};
        } else {
            return {std::vector<Eigen::VectorXd>(), false};
        }
    } catch (const std::exception& e) {
        std::cerr << "Error in Hauser planning: " << e.what() << std::endl;
        return {std::vector<Eigen::VectorXd>(), false};
    }
}

bool TrajectoryPlanningEvaluator::runComprehensiveEvaluation(const EvaluationConfig& config) {
    if (!robot_arm_ || !motion_generator_ || !path_planner_) {
        std::cerr << "Error: Environment not initialized. Call initializeEnvironment() first." << std::endl;
        return false;
    }
    
    std::vector<EvaluationResult> all_results;
    
    std::cout << "Starting comprehensive trajectory planning evaluation..." << std::endl;
    std::cout << "Test poses: " << config.test_poses.size() << std::endl;
    std::cout << "STOMP configs: " << config.stomp_params.size() << std::endl;
    std::cout << "Hauser configs: " << config.hauser_params.size() << std::endl;
    std::cout << "Trials per config: " << config.trials_per_config << std::endl;
    
    // Hauser algorithm variants
    std::vector<EvaluatorAlgorithm> hauser_algorithms = {
        EvaluatorAlgorithm::STOMP,  // We'll handle this separately
        EvaluatorAlgorithm::HAUSER_RRT,
        EvaluatorAlgorithm::HAUSER_RRT_STAR, 
        EvaluatorAlgorithm::HAUSER_INFORMED_RRT,
        EvaluatorAlgorithm::HAUSER_BI_RRT
    };
    
    int total_evaluations = config.test_poses.size() * config.trials_per_config * 
                           (config.stomp_params.size() + config.hauser_params.size() * 4);
    int current_eval = 0;
    
    // Evaluate STOMP configurations
    for (const auto& stomp_config : config.stomp_params) {
        for (int pose_idx = 0; pose_idx < config.test_poses.size(); ++pose_idx) {
            for (int trial = 0; trial < config.trials_per_config; ++trial) {
                current_eval++;
                std::cout << "Progress: " << current_eval << "/" << total_evaluations 
                          << " (" << std::fixed << std::setprecision(1) 
                          << (100.0 * current_eval / total_evaluations) << "%)" << std::endl;
                
                EvaluationResult result = evaluateStompTrial(
                    stomp_config, 
                    config.test_poses[pose_idx], 
                    config.start_joint_config,
                    trial, pose_idx);
                all_results.push_back(result);
            }
        }
    }
    
    // Evaluate Hauser algorithm variants
    for (const auto& hauser_config : config.hauser_params) {
        // Test each Hauser algorithm variant
        for (int algo_idx = 1; algo_idx < hauser_algorithms.size(); ++algo_idx) { // Skip STOMP
            EvaluatorAlgorithm algo = hauser_algorithms[algo_idx];
            
            for (int pose_idx = 0; pose_idx < config.test_poses.size(); ++pose_idx) {
                for (int trial = 0; trial < config.trials_per_config; ++trial) {
                    current_eval++;
                    std::cout << "Progress: " << current_eval << "/" << total_evaluations 
                              << " (" << std::fixed << std::setprecision(1) 
                              << (100.0 * current_eval / total_evaluations) << "%)" << std::endl;
                    
                    EvaluationResult result = evaluateHauserTrial(
                        algo,
                        hauser_config, 
                        config.test_poses[pose_idx], 
                        config.start_joint_config,
                        trial, pose_idx);
                    all_results.push_back(result);
                }
            }
        }
    }
    
    std::cout << "Evaluation complete! Generated " << all_results.size() << " results." << std::endl;
    
    // Save results
    std::string filename = config.output_directory + "/trajectory_evaluation_" + config.timestamp + ".csv";
    return saveResults(all_results, filename);
}

std::vector<Eigen::VectorXd> TrajectoryPlanningEvaluator::loadMiddle10Poses() {
    // This would load your actual test poses
    // For now, return a simple set of test poses
    std::vector<Eigen::VectorXd> poses;
    
    // Generate 10 test poses (x, y, z, qw, qx, qy, qz format)
    for (int i = 0; i < 10; ++i) {
        Eigen::VectorXd pose(7);
        pose << 0.5 + i * 0.05,  // x
                0.3 + i * 0.02,  // y
                0.4 + i * 0.01,  // z
                1.0, 0.0, 0.0, 0.0;  // quaternion (w, x, y, z)
        poses.push_back(pose);
    }
    
    return poses;
}

EvaluationConfig TrajectoryPlanningEvaluator::generateParameterSweepConfig() {
    EvaluationConfig config;
    
    // Set home configuration as start
    config.start_joint_config = Eigen::VectorXd::Zero(7);
    config.start_joint_config << 0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.785;
    
    // Generate STOMP parameter sweep
    std::vector<int> noisy_trajectories = {4, 8, 12, 16};
    std::vector<int> max_iterations = {200, 500, 800, 1000};
    std::vector<double> learning_rates = {0.1, 0.3, 0.5, 0.7};
    
    for (int noisy : noisy_trajectories) {
        for (int iter : max_iterations) {
            for (double lr : learning_rates) {
                // Use optimized parameters as baseline, only override for research sweep
                StompConfig stomp_config = StompConfig::optimized();
                
                // Override only the parameters being swept for research
                stomp_config.numNoisyTrajectories = noisy;
                stomp_config.numBestSamples = std::min(6, noisy); // Use optimized default (6) when possible
                stomp_config.maxIterations = iter;
                stomp_config.learningRate = lr;
                // Keep optimized dt (0.1097) and temperature (15.9079) for better performance
                
                config.stomp_params.push_back(stomp_config);
            }
        }
    }
    
    // Generate Hauser parameter sweep  
    std::vector<double> step_sizes = {0.02, 0.05, 0.1, 0.15};
    std::vector<int> hauser_iterations = {1000, 2500, 5000, 7500};
    std::vector<double> goal_bias_probs = {0.1, 0.3, 0.5, 0.7};
    
    for (double step : step_sizes) {
        for (int iter : hauser_iterations) {
            for (double bias : goal_bias_probs) {
                Params hauser_params;
                hauser_params.stepSize = step;
                hauser_params.maxIterations = iter;
                hauser_params.goalBiasProbability = bias;
                hauser_params.customCost = false;
                
                config.hauser_params.push_back(hauser_params);
            }
        }
    }
    
    // Load test poses (you would replace this with actual pose loading)
    config.test_poses = loadMiddle10Poses();
    
    config.trials_per_config = 5; // 5 trials per configuration
    config.output_directory = "evaluation_results";
    
    return config;
}

bool TrajectoryPlanningEvaluator::saveResults(const std::vector<EvaluationResult>& results, 
                                                      const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot create output file: " << filename << std::endl;
        return false;
    }
    
    // Write CSV header
    file << EvaluationResult::getCSVHeader() << "\n";
    
    // Write results
    for (const auto& result : results) {
        file << result.toCSV() << "\n";
    }
    
    file.close();
    std::cout << "Results saved to: " << filename << std::endl;
    std::cout << "Total results: " << results.size() << std::endl;
    
    return true;
}

double TrajectoryPlanningEvaluator::calculatePathQuality(const std::vector<Eigen::VectorXd>& trajectory) {
    if (trajectory.size() < 2) return 0.0;
    
    double total_length = 0.0;
    double max_joint_change = 0.0;
    
    for (size_t i = 1; i < trajectory.size(); ++i) {
        Eigen::VectorXd diff = trajectory[i] - trajectory[i-1];
        double segment_length = diff.norm();
        total_length += segment_length;
        
        // Track maximum joint change
        for (int j = 0; j < diff.size(); ++j) {
            max_joint_change = std::max(max_joint_change, std::abs(diff[j]));
        }
    }
    
    // Quality metric: lower total length and smoother changes = higher quality
    // Normalize by trajectory length and penalize large joint changes
    double quality = 1.0 / (1.0 + total_length + max_joint_change * 5.0);
    return std::max(0.0, std::min(1.0, quality));
}

double TrajectoryPlanningEvaluator::calculateJointSmoothness(const std::vector<Eigen::VectorXd>& trajectory) {
    if (trajectory.size() < 3) return 1.0;
    
    double total_curvature = 0.0;
    int count = 0;
    
    for (size_t i = 1; i < trajectory.size() - 1; ++i) {
        Eigen::VectorXd vel1 = trajectory[i] - trajectory[i-1];
        Eigen::VectorXd vel2 = trajectory[i+1] - trajectory[i];
        Eigen::VectorXd accel = vel2 - vel1;
        
        total_curvature += accel.norm();
        count++;
    }
    
    if (count == 0) return 1.0;
    
    double avg_curvature = total_curvature / count;
    // Convert to smoothness measure (lower curvature = higher smoothness)
    double smoothness = 1.0 / (1.0 + avg_curvature * 10.0);
    return std::max(0.0, std::min(1.0, smoothness));
}

double TrajectoryPlanningEvaluator::calculateCollisionClearance(const std::vector<Eigen::VectorXd>& trajectory) {
    if (!robot_arm_ || trajectory.empty()) return 0.0;
    
    double min_clearance = std::numeric_limits<double>::max();
    
    for (const auto& joints : trajectory) {
        try {
            robot_arm_->setJointAngles(joints);
            
            // Get robot links positions for collision checking
            // This is a simplified clearance calculation
            // In practice, you'd use the BVHTree for accurate distance computation
            
            // For now, use a heuristic based on joint limits
            double clearance = 1.0;
            for (int i = 0; i < joints.size(); ++i) {
                double joint_range_usage = std::abs(joints[i]) / 3.14159; // Normalize by pi
                clearance = std::min(clearance, 1.0 - joint_range_usage);
            }
            
            min_clearance = std::min(min_clearance, std::max(0.01, clearance));
        } catch (const std::exception& e) {
            // If joint configuration is invalid, assume low clearance
            min_clearance = std::min(min_clearance, 0.01);
        }
    }
    
    return (min_clearance == std::numeric_limits<double>::max()) ? 0.01 : min_clearance;
}
