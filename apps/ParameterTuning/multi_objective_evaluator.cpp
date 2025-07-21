#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <json/json.h>
#include <Eigen/Dense>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <limits>

// Include the trajectory planning headers
#include "TrajectoryLib/Robot/RobotArm.h"
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Planning/PathPlanner.h"
#include "TrajectoryLib/Robot/RobotManager.h"
#include "GeometryLib/BVHTree.h"

/**
 * Multi-Objective Parameter Evaluator
 * ===================================
 * 
 * Evaluates STOMP parameters across multiple objectives:
 * 1. Planning time (efficiency)
 * 2. Trajectory quality (smoothness, feasibility)
 * 3. Energy consumption (estimated from acceleration)
 * 4. Collision margin (safety)
 * 5. Success rate (reliability)
 */

struct MultiObjectiveResults {
    double planning_time;        // seconds
    double trajectory_quality;   // 1/cost (higher is better)
    double energy_consumption;   // estimated energy units
    double collision_margin;     // minimum obstacle distance
    double success_rate;         // 0.0 to 1.0
    bool success;
};

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
            try {
                values.push_back(std::stod(item));
            } catch (...) {
                // Skip invalid values
            }
        }
        
        if (values.size() >= 10) {
            pose.position = Eigen::Vector3d(values[0], values[1], values[2]);
            pose.orientation = Eigen::Quaterniond(values[6], values[3], values[4], values[5]); // w,x,y,z
            pose.contact = values[7] > 0.5;
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

class MultiObjectiveEvaluator {
private:
    std::shared_ptr<RobotArm> robot_;
    std::shared_ptr<BVHTree> obstacle_tree_;
    std::vector<PoseData> scan_poses_;
    
public:
    MultiObjectiveEvaluator() = default;
    
    bool initialize(const std::string& urdf_file, const std::string& env_file, const std::string& poses_file) {
        // Initialize robot
        robot_ = std::make_shared<RobotArm>();
        if (!robot_->loadRobotFromURDF(urdf_file, "panda_link8")) {
            std::cerr << "Failed to load robot from URDF: " << urdf_file << std::endl;
            return false;
        }
        
        // Load environment obstacles
        obstacle_tree_ = std::make_shared<BVHTree>();
        if (!loadObstacles(env_file)) {
            std::cerr << "Failed to load obstacles from: " << env_file << std::endl;
            return false;
        }
        
        // Load scan poses
        if (!loadScanPoses(poses_file)) {
            std::cerr << "Failed to load scan poses from: " << poses_file << std::endl;
            return false;
        }
        
        return true;
    }
    
    MultiObjectiveResults evaluate(const YAML::Node& config) {
        MultiObjectiveResults results{};
        
        try {
            std::string algorithm = config["algorithm"].as<std::string>();
            const auto& params = config["parameters"][algorithm];
            
            // Setup motion generator with parameters
            MotionGenerator motion_generator;
            if (algorithm == "STOMP") {
                configureSTOMP(motion_generator, params);
            } else if (algorithm == "Hauser") {
                configureHauser(motion_generator, params);
            }
            
            // Run evaluation across multiple trajectory segments
            std::vector<double> planning_times;
            std::vector<double> trajectory_costs;
            std::vector<double> energy_values;
            std::vector<double> collision_margins;
            int successful_plans = 0;
            int total_plans = 0;
            
            // Test on 10 random pose pairs
            for (int i = 0; i < 10 && i + 1 < scan_poses_.size(); ++i) {
                auto start_pose = scan_poses_[i].toAffine3d();
                auto goal_pose = scan_poses_[i + 1].toAffine3d();
                
                auto start_time = std::chrono::high_resolution_clock::now();
                
                // Plan trajectory
                std::vector<Eigen::VectorXd> trajectory;
                bool success = false;
                
                if (algorithm == "STOMP") {
                    success = planWithSTOMP(motion_generator, start_pose, goal_pose, trajectory);
                } else if (algorithm == "Hauser") {
                    success = planWithHauser(motion_generator, start_pose, goal_pose, trajectory);
                }
                
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                
                total_plans++;
                planning_times.push_back(duration.count() / 1000.0);
                
                if (success) {
                    successful_plans++;
                    
                    // Compute trajectory metrics
                    double cost = computeTrajectoryQuality(trajectory);
                    double energy = estimateEnergyConsumption(trajectory);
                    double margin = computeCollisionMargin(trajectory);
                    
                    trajectory_costs.push_back(cost);
                    energy_values.push_back(energy);
                    collision_margins.push_back(margin);
                } else {
                    // Penalty values for failed trajectories
                    trajectory_costs.push_back(1000.0);
                    energy_values.push_back(1000.0);
                    collision_margins.push_back(0.0);
                }
            }
            
            // Aggregate results
            results.planning_time = std::accumulate(planning_times.begin(), planning_times.end(), 0.0) / planning_times.size();
            results.success_rate = static_cast<double>(successful_plans) / total_plans;
            
            if (successful_plans > 0) {
                double avg_cost = std::accumulate(trajectory_costs.begin(), trajectory_costs.end(), 0.0) / trajectory_costs.size();
                results.trajectory_quality = 1.0 / std::max(avg_cost, 1e-6);
                results.energy_consumption = std::accumulate(energy_values.begin(), energy_values.end(), 0.0) / energy_values.size();
                results.collision_margin = *std::min_element(collision_margins.begin(), collision_margins.end());
            } else {
                results.trajectory_quality = 0.0;
                results.energy_consumption = 1000.0;
                results.collision_margin = 0.0;
            }
            
            results.success = true;
            
        } catch (const std::exception& e) {
            std::cerr << "Evaluation failed: " << e.what() << std::endl;
            results.success = false;
        }
        
        return results;
    }

private:
    bool loadObstacles(const std::string& env_file) {
        // Simplified obstacle loading - in practice would parse XML/collision meshes
        return true;
    }
    
    bool loadScanPoses(const std::string& poses_file) {
        std::ifstream file(poses_file);
        if (!file.is_open()) return false;
        
        std::string line;
        std::getline(file, line); // Skip header
        
        while (std::getline(file, line)) {
            if (!line.empty()) {
                scan_poses_.push_back(PoseData::fromCSVRow(line));
            }
        }
        
        return !scan_poses_.empty();
    }
    
    void configureSTOMP(MotionGenerator& generator, const YAML::Node& params) {
        // Configure STOMP parameters
        int num_noisy = params["num_noisy_trajectories"].as<int>();
        int num_best = params["num_best_samples"].as<int>();
        int max_iter = params["max_iterations"].as<int>();
        double learning_rate = params["learning_rate"].as<double>();
        double temperature = params["temperature"].as<double>();
        double dt = params["dt"].as<double>();
        
        // Set up STOMP configuration
        generator.setSTOMPConfig(num_noisy, num_best, max_iter, learning_rate, temperature, dt);
        
        // Set dynamic limits if provided
        if (params["velocity_limit"] && params["acceleration_limit"]) {
            double vel_limit = params["velocity_limit"].as<double>();
            double acc_limit = params["acceleration_limit"].as<double>();
            generator.setDynamicLimits(vel_limit, acc_limit);
        }
    }
    
    void configureHauser(MotionGenerator& generator, const YAML::Node& params) {
        // Configure Hauser algorithm parameters
        if (params["velocity_limit"] && params["acceleration_limit"]) {
            double vel_limit = params["velocity_limit"].as<double>();
            double acc_limit = params["acceleration_limit"].as<double>();
            generator.setDynamicLimits(vel_limit, acc_limit);
        }
    }
    
    bool planWithSTOMP(MotionGenerator& generator, const Eigen::Affine3d& start, 
                      const Eigen::Affine3d& goal, std::vector<Eigen::VectorXd>& trajectory) {
        try {
            // Convert poses to joint configurations
            Eigen::VectorXd start_config, goal_config;
            if (!robot_->inverseKinematics(start, start_config) ||
                !robot_->inverseKinematics(goal, goal_config)) {
                return false;
            }
            
            // Plan with STOMP
            return generator.planSTOMP(robot_, start_config, goal_config, trajectory);
        } catch (...) {
            return false;
        }
    }
    
    bool planWithHauser(MotionGenerator& generator, const Eigen::Affine3d& start,
                       const Eigen::Affine3d& goal, std::vector<Eigen::VectorXd>& trajectory) {
        try {
            // Convert poses to joint configurations
            Eigen::VectorXd start_config, goal_config;
            if (!robot_->inverseKinematics(start, start_config) ||
                !robot_->inverseKinematics(goal, goal_config)) {
                return false;
            }
            
            // Plan with Hauser
            return generator.planHauser(robot_, start_config, goal_config, trajectory);
        } catch (...) {
            return false;
        }
    }
    
    double computeTrajectoryQuality(const std::vector<Eigen::VectorXd>& trajectory) {
        if (trajectory.size() < 2) return 1000.0;
        
        double total_cost = 0.0;
        
        // Smoothness cost
        for (size_t i = 1; i < trajectory.size() - 1; ++i) {
            Eigen::VectorXd accel = trajectory[i+1] - 2*trajectory[i] + trajectory[i-1];
            total_cost += accel.squaredNorm();
        }
        
        // Path length cost
        for (size_t i = 1; i < trajectory.size(); ++i) {
            Eigen::VectorXd diff = trajectory[i] - trajectory[i-1];
            total_cost += diff.norm();
        }
        
        return total_cost;
    }
    
    double estimateEnergyConsumption(const std::vector<Eigen::VectorXd>& trajectory) {
        if (trajectory.size() < 3) return 1000.0;
        
        double total_energy = 0.0;
        double dt = 0.1; // Assume 0.1s between waypoints
        
        for (size_t i = 1; i < trajectory.size() - 1; ++i) {
            // Compute velocities and accelerations
            Eigen::VectorXd velocity = (trajectory[i+1] - trajectory[i-1]) / (2*dt);
            Eigen::VectorXd acceleration = (trajectory[i+1] - 2*trajectory[i] + trajectory[i-1]) / (dt*dt);
            
            // Energy proportional to velocity^2 + acceleration^2
            total_energy += velocity.squaredNorm() + acceleration.squaredNorm();
        }
        
        return total_energy;
    }
    
    double computeCollisionMargin(const std::vector<Eigen::VectorXd>& trajectory) {
        // Simplified collision margin computation
        // In practice, would check actual robot geometry against obstacles
        
        double min_margin = 1.0; // Assume 1m default margin
        
        // Use trajectory smoothness as proxy for collision avoidance
        double smoothness_penalty = 0.0;
        for (size_t i = 1; i < trajectory.size() - 1; ++i) {
            Eigen::VectorXd curvature = trajectory[i+1] - 2*trajectory[i] + trajectory[i-1];
            smoothness_penalty += curvature.norm();
        }
        
        // Higher smoothness penalty indicates more aggressive maneuvering (lower margin)
        min_margin = std::max(0.1, 1.0 - smoothness_penalty * 0.01);
        
        return min_margin;
    }
};

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file.yaml>" << std::endl;
        return 1;
    }
    
    try {
        // Load configuration
        YAML::Node config = YAML::LoadFile(argv[1]);
        
        // Extract scenario information
        const auto& scenario = config["scenario"];
        std::string urdf_file = scenario["urdf_file"].as<std::string>();
        std::string env_file = scenario["environment_file"].as<std::string>();
        std::string poses_file = scenario["poses_file"].as<std::string>();
        
        // Initialize evaluator
        MultiObjectiveEvaluator evaluator;
        if (!evaluator.initialize(urdf_file, env_file, poses_file)) {
            std::cerr << "Failed to initialize evaluator" << std::endl;
            return 1;
        }
        
        // Run evaluation
        auto results = evaluator.evaluate(config);
        
        // Output results as JSON
        Json::Value output;
        output["success"] = results.success;
        
        if (results.success) {
            output["planning_time"] = results.planning_time;
            output["trajectory_quality"] = results.trajectory_quality;
            output["energy_consumption"] = results.energy_consumption;
            output["collision_margin"] = results.collision_margin;
            output["success_rate"] = results.success_rate;
            
            // Legacy format for backwards compatibility
            output["trajectory_cost"] = 1.0 / std::max(results.trajectory_quality, 1e-6);
        } else {
            output["planning_time"] = 999.0;
            output["trajectory_quality"] = 0.0;
            output["energy_consumption"] = 1000.0;
            output["collision_margin"] = 0.0;
            output["success_rate"] = 0.0;
            output["trajectory_cost"] = 1000.0;
        }
        
        Json::StreamWriterBuilder builder;
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(output, &std::cout);
        std::cout << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        
        // Output failed result
        Json::Value output;
        output["success"] = false;
        output["planning_time"] = 999.0;
        output["trajectory_quality"] = 0.0;
        output["energy_consumption"] = 1000.0;
        output["collision_margin"] = 0.0;
        output["success_rate"] = 0.0;
        output["trajectory_cost"] = 1000.0;
        
        Json::StreamWriterBuilder builder;
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(output, &std::cout);
        std::cout << std::endl;
        
        return 1;
    }
    
    return 0;
}
