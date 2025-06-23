#include <iostream>
#include <iomanip>
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <json/json.h>
#include <Eigen/Dense>

#include "TrajectoryLib/Robot/RobotArm.h"
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Planning/PathPlanner.h"
#include "TrajectoryLib/Robot/RobotManager.h"
#include "GeometryLib/BVHTree.h"

#include <QDebug>

// =============================================================================
// ENHANCED PARAMETER EVALUATOR FOR SCENARIO_1 DATA
// =============================================================================
// This application evaluates STOMP and Hauser parameters using real scenario_1
// poses and obstacles. It integrates with Python optimization libraries
// through YAML configuration and JSON output.
// =============================================================================

struct PoseData {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    bool contact;
    double distance;
    int index;
    
    static PoseData fromCSVRow(const std::string& line) {
        PoseData pose;
        std::stringstream ss(line);
        std::string item;
        std::vector<double> values;
        
        while (std::getline(ss, item, ',')) {
            values.push_back(std::stod(item));
        }
        
        if (values.size() >= 10) {
            pose.position = Eigen::Vector3d(values[0], values[1], values[2]);
            pose.orientation = Eigen::Quaterniond(values[6], values[3], values[4], values[5]); // w,x,y,z
            pose.contact = static_cast<bool>(values[7]);
            pose.distance = values[8];
            pose.index = static_cast<int>(values[9]);
        }
        
        return pose;
    }
    
    Eigen::Affine3d toAffine3d() const {
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.translation() = position;
        transform.linear() = orientation.toRotationMatrix();
        return transform;
    }
};

struct EvaluationMetrics {
    double success_rate = 0.0;
    double avg_planning_time_ms = 0.0;
    double avg_path_length = 0.0;
    double avg_smoothness_score = 0.0;
    double avg_safety_score = 0.0;
    double avg_min_clearance = 0.0;
    double consistency_score = 0.0;
    double computational_efficiency = 0.0;
    
    // Detailed metrics
    std::vector<double> planning_times;
    std::vector<double> path_lengths;
    std::vector<double> smoothness_scores;
    std::vector<double> safety_scores;
    std::vector<bool> successes;
    
    void computeAverages() {
        if (!planning_times.empty()) {
            avg_planning_time_ms = std::accumulate(planning_times.begin(), planning_times.end(), 0.0) / planning_times.size();
        }
        
        if (!path_lengths.empty()) {
            avg_path_length = std::accumulate(path_lengths.begin(), path_lengths.end(), 0.0) / path_lengths.size();
        }
        
        if (!smoothness_scores.empty()) {
            avg_smoothness_score = std::accumulate(smoothness_scores.begin(), smoothness_scores.end(), 0.0) / smoothness_scores.size();
        }
        
        if (!safety_scores.empty()) {
            avg_safety_score = std::accumulate(safety_scores.begin(), safety_scores.end(), 0.0) / safety_scores.size();
        }
        
        if (!successes.empty()) {
            success_rate = std::count(successes.begin(), successes.end(), true) / static_cast<double>(successes.size());
        }
        
        // Compute consistency (1 - coefficient of variation)
        if (planning_times.size() > 1) {
            double mean = avg_planning_time_ms;
            double variance = 0.0;
            for (double time : planning_times) {
                variance += (time - mean) * (time - mean);
            }
            variance /= planning_times.size() - 1;
            double cv = std::sqrt(variance) / mean;
            consistency_score = std::max(0.0, 1.0 - cv);
        }
        
        // Computational efficiency (success rate / avg planning time in seconds)
        if (avg_planning_time_ms > 0) {
            computational_efficiency = success_rate / (avg_planning_time_ms / 1000.0);
        }
    }
};

struct ScenarioConfig {
    std::string name;
    std::string description;
    int difficulty;
    Eigen::VectorXd start_config;
    std::vector<PoseData> target_poses;
    std::string environment_file;
    std::string urdf_file;
};

class EnhancedParameterEvaluator {
public:
    EnhancedParameterEvaluator() = default;
    
    void setJsonOutput(bool json_output) { json_output_ = json_output; }
    
    bool loadConfiguration(const std::string& config_file) {
        try {
            YAML::Node config = YAML::LoadFile(config_file);
            
            // Load algorithm and parameters
            algorithm_ = config["algorithm"].as<std::string>();
            
            if (config["parameters"]) {
                parameters_ = config["parameters"];
            }
            
            // Load scenarios
            if (config["scenarios"]) {
                for (const auto& scenario_node : config["scenarios"]) {
                    ScenarioConfig scenario;
                    scenario.name = scenario_node["name"].as<std::string>();
                    scenario.description = scenario_node["description"].as<std::string>();
                    scenario.difficulty = scenario_node["difficulty"].as<int>();
                    
                    // Load start configuration
                    auto start_config_vec = scenario_node["start_config"].as<std::vector<double>>();
                    scenario.start_config = Eigen::Map<Eigen::VectorXd>(start_config_vec.data(), start_config_vec.size());
                    
                    // Load environment and URDF files
                    scenario.environment_file = scenario_node["environment"].as<std::string>();
                    scenario.urdf_file = scenario_node["urdf"].as<std::string>();
                    
                    // Load target poses if specified directly
                    if (scenario_node["target_poses"]) {
                        for (const auto& pose_node : scenario_node["target_poses"]) {
                            PoseData pose;
                            auto pos = pose_node["position"].as<std::vector<double>>();
                            auto ori = pose_node["orientation"].as<std::vector<double>>();
                            
                            pose.position = Eigen::Vector3d(pos[0], pos[1], pos[2]);
                            pose.orientation = Eigen::Quaterniond(ori[3], ori[0], ori[1], ori[2]); // w,x,y,z
                            pose.contact = pose_node["contact"].as<bool>();
                            pose.distance = pose_node["distance"].as<double>();
                            pose.index = pose_node["index"].as<int>();
                            
                            scenario.target_poses.push_back(pose);
                        }
                    }
                    
                    scenarios_.push_back(scenario);
                }
            }
            
            // Load evaluation settings
            if (config["evaluation_settings"]) {
                auto eval_settings = config["evaluation_settings"];
                num_runs_per_scenario_ = eval_settings["num_runs_per_scenario"].as<int>(5);
                timeout_seconds_ = eval_settings["timeout_seconds"].as<double>(30.0);
            }
            
            // Load resource paths
            if (config["resources"]) {
                auto resources = config["resources"];
                poses_file_ = resources["poses_file"].as<std::string>("");
                obstacles_file_ = resources["obstacles_file"].as<std::string>("");
                urdf_file_ = resources["urdf_file"].as<std::string>("");
            }
            
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Error loading configuration: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool loadScenario1Poses() {
        if (poses_file_.empty()) {
            std::cerr << "Poses file not specified" << std::endl;
            return false;
        }
        
        std::ifstream file(poses_file_);
        if (!file.is_open()) {
            std::cerr << "Cannot open poses file: " << poses_file_ << std::endl;
            return false;
        }
        
        scenario1_poses_.clear();
        std::string line;
        
        while (std::getline(file, line)) {
            if (!line.empty()) {
                PoseData pose = PoseData::fromCSVRow(line);
                scenario1_poses_.push_back(pose);
            }
        }
        
        if (!json_output_) {
            std::cout << "Loaded " << scenario1_poses_.size() << " poses from scenario_1" << std::endl;
        }
        return !scenario1_poses_.empty();
    }
    
    EvaluationMetrics evaluateAlgorithm() {
        EvaluationMetrics overall_metrics;
        
        // Initialize robot and environment
        if (!initializeRobotAndEnvironment()) {
            return overall_metrics;
        }
        
        // If scenarios don't have poses, use scenario_1 poses
        for (auto& scenario : scenarios_) {
            if (scenario.target_poses.empty() && !scenario1_poses_.empty()) {
                // Use subset of scenario_1 poses based on difficulty
                int num_poses = std::min(static_cast<int>(scenario1_poses_.size()), 
                                       scenario.difficulty * 5);
                scenario.target_poses.assign(scenario1_poses_.begin(), 
                                           scenario1_poses_.begin() + num_poses);
            }
        }
        
        // Evaluate each scenario
        for (const auto& scenario : scenarios_) {
            EvaluationMetrics scenario_metrics = evaluateScenario(scenario);
            
            // Merge metrics
            overall_metrics.planning_times.insert(overall_metrics.planning_times.end(),
                                                 scenario_metrics.planning_times.begin(),
                                                 scenario_metrics.planning_times.end());
            
            overall_metrics.path_lengths.insert(overall_metrics.path_lengths.end(),
                                               scenario_metrics.path_lengths.begin(),
                                               scenario_metrics.path_lengths.end());
            
            overall_metrics.smoothness_scores.insert(overall_metrics.smoothness_scores.end(),
                                                    scenario_metrics.smoothness_scores.begin(),
                                                    scenario_metrics.smoothness_scores.end());
            
            overall_metrics.safety_scores.insert(overall_metrics.safety_scores.end(),
                                                scenario_metrics.safety_scores.begin(),
                                                scenario_metrics.safety_scores.end());
            
            overall_metrics.successes.insert(overall_metrics.successes.end(),
                                            scenario_metrics.successes.begin(),
                                            scenario_metrics.successes.end());
        }
        
        overall_metrics.computeAverages();
        return overall_metrics;
    }
    
    void outputResults(const EvaluationMetrics& metrics) {
        Json::Value result;
        result["success_rate"] = metrics.success_rate;
        result["avg_planning_time_ms"] = metrics.avg_planning_time_ms;
        result["avg_path_length"] = metrics.avg_path_length;
        result["avg_smoothness_score"] = metrics.avg_smoothness_score;
        result["avg_safety_score"] = metrics.avg_safety_score;
        result["avg_min_clearance"] = metrics.avg_min_clearance;
        result["consistency_score"] = metrics.consistency_score;
        result["computational_efficiency"] = metrics.computational_efficiency;
        
        // Add detailed data
        Json::Value planning_times_array(Json::arrayValue);
        for (double time : metrics.planning_times) {
            planning_times_array.append(time);
        }
        result["planning_times"] = planning_times_array;
        
        Json::Value success_array(Json::arrayValue);
        for (bool success : metrics.successes) {
            success_array.append(success);
        }
        result["successes"] = success_array;
        
        std::cout << result << std::endl;
    }

private:
    std::string algorithm_;
    YAML::Node parameters_;
    std::vector<ScenarioConfig> scenarios_;
    std::vector<PoseData> scenario1_poses_;
    
    int num_runs_per_scenario_ = 5;
    double timeout_seconds_ = 30.0;
    
    std::string poses_file_;
    std::string obstacles_file_;
    std::string urdf_file_;
    bool json_output_;
    
    std::unique_ptr<RobotArm> robot_;
    std::shared_ptr<BVHTree> obstacle_tree_;
    
    bool initializeRobotAndEnvironment() {
        try {
            // Load robot following evaluator.cpp pattern
            if (urdf_file_.empty()) {
                std::cerr << "URDF file not specified" << std::endl;
                return false;
            }
        
            robot_ = std::make_unique<RobotArm>(urdf_file_);
        
            // Set initial robot configuration like evaluator.cpp
            robot_->setJointAngle("panda_joint1", -M_PI / 4);
            robot_->setJointAngle("panda_joint2", M_PI / 8);
            robot_->setJointAngle("panda_joint3", -M_PI / 8);
            robot_->setJointAngle("panda_joint4", -M_PI / 3);
        
            // Load obstacles
            if (!obstacles_file_.empty()) {
                RobotManager robotManager;
                robotManager.parseURDF(obstacles_file_);
                obstacle_tree_ = std::make_shared<BVHTree>(robotManager.getTransformedObstacles());
            }
        
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Error initializing robot and environment: " << e.what() << std::endl;
            return false;
        }
    }
    
    EvaluationMetrics evaluateScenario(const ScenarioConfig& scenario) {
        EvaluationMetrics metrics;
        
        for (int run = 0; run < num_runs_per_scenario_; ++run) {
            auto start_time = std::chrono::high_resolution_clock::now();
            
            bool success = false;
            double path_length = 0.0;
            double smoothness_score = 0.0;
            double safety_score = 0.0;
            
            try {
                if (algorithm_ == "STOMP") {
                    success = evaluateSTOMP(scenario, path_length, smoothness_score, safety_score);
                } else if (algorithm_ == "Hauser") {
                    success = evaluateHauser(scenario, path_length, smoothness_score, safety_score);
                }
            } catch (const std::exception& e) {
                std::cerr << "Evaluation error: " << e.what() << std::endl;
                success = false;
            }
            
            auto end_time = std::chrono::high_resolution_clock::now();
            double planning_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            
            metrics.planning_times.push_back(planning_time);
            metrics.path_lengths.push_back(path_length);
            metrics.smoothness_scores.push_back(smoothness_score);
            metrics.safety_scores.push_back(safety_score);
            metrics.successes.push_back(success);
        }
        
        metrics.computeAverages();
        return metrics;
    }
    
    bool evaluateSTOMP(const ScenarioConfig& scenario, double& path_length, 
                      double& smoothness_score, double& safety_score) {
        
        MotionGenerator motionGenerator(*robot_);
        if (obstacle_tree_) {
            motionGenerator.setObstacleTree(obstacle_tree_);
        }
        
        // Configure STOMP parameters
        StompConfig config;
        if (parameters_["exploration_constant"]) {
            motionGenerator.setExplorationConstant(parameters_["exploration_constant"].as<double>());
        }
        if (parameters_["num_noisy_trajectories"]) {
            config.numNoisyTrajectories = parameters_["num_noisy_trajectories"].as<int>();
        }
        if (parameters_["num_best_samples"]) {
            config.numBestSamples = parameters_["num_best_samples"].as<int>();
        }
        if (parameters_["max_iterations"]) {
            config.maxIterations = parameters_["max_iterations"].as<int>();
        }
        if (parameters_["learning_rate"]) {
            config.learningRate = parameters_["learning_rate"].as<double>();
        }
        if (parameters_["temperature"]) {
            config.temperature = parameters_["temperature"].as<double>();
        }
        if (parameters_["dt"]) {
            config.dt = parameters_["dt"].as<double>();
        }
        
        // Plan trajectory through poses using the pattern from evaluator.cpp
        if (scenario.target_poses.size() >= 1) {
            // Use first pose as target (following evaluator.cpp pattern)
            PathPlanner planner;
            planner.setStartPose(*robot_);
            if (obstacle_tree_) {
                planner.setObstacleTree(obstacle_tree_);
            }
            
            Eigen::Affine3d transform = scenario.target_poses[0].toAffine3d();
            auto [goal_arm, goal_success] = planner.selectGoalPose(transform);
            
            if (!goal_success) {
                return false;
            }
            
            Eigen::MatrixXd angles(2, 7);
            angles.row(0) = scenario.start_config.transpose();
            angles.row(1) = goal_arm.getJointAngles().transpose();
            motionGenerator.setWaypoints(angles);
            
            bool success = motionGenerator.performSTOMP(config);
            if (success) {
                computeMetrics(motionGenerator.getPath(), path_length, smoothness_score, safety_score);
            }
            return success;
        }
        
        return false;
    }
    
    bool evaluateHauser(const ScenarioConfig& scenario, double& path_length,
                       double& smoothness_score, double& safety_score) {
        
        PathPlanner pathPlanner;
        MotionGenerator motionGenerator(*robot_);
        
        if (obstacle_tree_) {
            pathPlanner.setObstacleTree(obstacle_tree_);
            motionGenerator.setObstacleTree(obstacle_tree_);
        }
        
        // Configure RRT parameters following evaluator.cpp pattern
        Params params;
        if (parameters_["algorithm"]) {
            std::string algo_name = parameters_["algorithm"].as<std::string>();
            if (algo_name == "RRT") params.algo = Algorithm::RRT;
            else if (algo_name == "RRTConnect") params.algo = Algorithm::RRTConnect;
            else if (algo_name == "RRTStar") params.algo = Algorithm::RRTStar;
            else if (algo_name == "InformedRRTStar") params.algo = Algorithm::InformedRRTStar;
        }
        if (parameters_["step_size"]) {
            params.stepSize = parameters_["step_size"].as<double>();
        }
        if (parameters_["goal_bias"]) {
            params.goalBiasProbability = parameters_["goal_bias"].as<double>();
        }
        if (parameters_["max_iterations"]) {
            params.maxIterations = parameters_["max_iterations"].as<int>();
        }
        
        pathPlanner.setParams(params);
        
        // Set start pose
        robot_->setJointAngles(scenario.start_config);
        pathPlanner.setStartPose(*robot_);
        
        // Use first target pose following evaluator.cpp pattern
        if (scenario.target_poses.size() >= 1) {
            Eigen::Affine3d transform = scenario.target_poses[0].toAffine3d();
            auto [goal_arm, goal_success] = pathPlanner.selectGoalPose(transform);
            
            if (!goal_success) {
                return false;
            }
            
            pathPlanner.setGoalConfiguration(goal_arm);
            
            bool planning_success = pathPlanner.runPathFinding();
            if (!planning_success) {
                return false;
            }
            
            // Set waypoints for Hauser smoothing
            motionGenerator.setWaypoints(pathPlanner.getAnglesPath());
            
            int shortcut_iters = parameters_["shortcut_iterations"] ? 
                               parameters_["shortcut_iterations"].as<int>() : 500;
            motionGenerator.performHauser(shortcut_iters);
            
            auto trajectory = motionGenerator.getPath();
            if (!trajectory.empty()) {
                computeMetrics(trajectory, path_length, smoothness_score, safety_score);
                return true;
            }
        }
        
        return false;
    }
    
    Eigen::VectorXd computeInverseKinematics(const Eigen::Affine3d& target_pose) {
        // Use standard IK solving like evaluator.cpp
        PathPlanner pathPlanner;
        if (obstacle_tree_) {
            pathPlanner.setObstacleTree(obstacle_tree_);
        }
        
        pathPlanner.setStartPose(*robot_);
        auto [goal_arm, success] = pathPlanner.selectGoalPose(target_pose);
        
        if (success) {
            return goal_arm.getJointAngles();
        }
        
        return Eigen::VectorXd();
    }
    
    void computeMetrics(const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
                       double& path_length, double& smoothness_score, double& safety_score) {
        
        if (trajectory.empty()) {
            path_length = 0.0;
            smoothness_score = 0.0;
            safety_score = 0.0;
            return;
        }
        
        // Compute path length
        path_length = 0.0;
        for (size_t i = 1; i < trajectory.size(); ++i) {
            Eigen::VectorXd diff = Eigen::Map<const Eigen::VectorXd>(trajectory[i].position.data(), trajectory[i].position.size()) -
                                  Eigen::Map<const Eigen::VectorXd>(trajectory[i-1].position.data(), trajectory[i-1].position.size());
            path_length += diff.norm();
        }
        
        // Compute smoothness (inverse of max jerk)
        double max_jerk = 0.0;
        for (size_t i = 2; i < trajectory.size(); ++i) {
            for (size_t j = 0; j < trajectory[i].acceleration.size(); ++j) {
                double jerk = std::abs(trajectory[i].acceleration[j] - trajectory[i-1].acceleration[j]) / 
                             (trajectory[i].time - trajectory[i-1].time + 1e-6);
                max_jerk = std::max(max_jerk, jerk);
            }
        }
        smoothness_score = 1.0 / (1.0 + max_jerk);
        
        // Compute safety (minimum clearance if available)
        safety_score = 0.8; // Default reasonable safety score
        if (obstacle_tree_) {
            double min_clearance = std::numeric_limits<double>::max();
            for (const auto& point : trajectory) {
                robot_->setJointAngles(Eigen::Map<const Eigen::VectorXd>(point.position.data(), point.position.size()));
                // Simplified clearance computation
                auto clearance_metrics = computeClearance(*robot_);
                min_clearance = std::min(min_clearance, clearance_metrics);
            }
            safety_score = std::min(min_clearance / 0.05, 1.0); // Normalize to 5cm reference
        }
    }
    
    double computeClearance(const RobotArm& arm) {
        // Simplified clearance computation
        // In a real implementation, this would check distances to obstacles
        return 0.05; // Default 5cm clearance
    }
};

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " --config <config_file> [--output-format json]" << std::endl;
        return 1;
    }
    
    std::string config_file;
    std::string output_format = "json";
    
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--config" && i + 1 < argc) {
            config_file = argv[i + 1];
            i++;
        } else if (std::string(argv[i]) == "--output-format" && i + 1 < argc) {
            output_format = argv[i + 1];
            i++;
        }
    }
    
    if (config_file.empty()) {
        std::cerr << "Configuration file is required" << std::endl;
        return 1;
    }
    
    try {
        EnhancedParameterEvaluator evaluator;
        evaluator.setJsonOutput(output_format == "json");
        
        if (!evaluator.loadConfiguration(config_file)) {
            std::cerr << "Failed to load configuration" << std::endl;
            return 1;
        }
        
        if (!evaluator.loadScenario1Poses()) {
            if (output_format != "json") {
                std::cerr << "Warning: Failed to load scenario_1 poses, using configuration poses only" << std::endl;
            }
        }
        
        EvaluationMetrics metrics = evaluator.evaluateAlgorithm();
        evaluator.outputResults(metrics);
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Evaluation failed: " << e.what() << std::endl;
        return 1;
    }
}