#ifndef TRAJECTORY_PLANNING_EVALUATOR_H
#define TRAJECTORY_PLANNING_EVALUATOR_H

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <fstream>
#include <map>
#include <iomanip>
#include <sstream>
#include <thread>
#include <Eigen/Dense>

// Trajectory planning libraries
#include "USLib/USTrajectoryPlanner.h"
#include "TrajectoryLib/Robot/RobotArm.h"
#include "TrajectoryLib/Robot/RobotManager.h"
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Planning/PathPlanner.h"

/**
 * @brief Trajectory planning algorithms supported
 */
enum class EvaluatorAlgorithm {
    STOMP,
    HAUSER_RRT,
    HAUSER_RRT_STAR,
    HAUSER_INFORMED_RRT,
    HAUSER_BI_RRT
};

/**
 * @brief Evaluation result for a single trial
 */
struct EvaluationResult {
    EvaluatorAlgorithm algorithm;
    std::string algorithm_config;
    int trial_id;
    int pose_id;
    bool success;
    double planning_time_ms;
    double execution_time_ms;
    int iterations_used;
    double trajectory_length;
    double path_quality;
    double collision_clearance;
    double joint_smoothness;
    Eigen::VectorXd start_joints;
    Eigen::VectorXd goal_pose;
    
    // CSV header
    static std::string getCSVHeader() {
        return "algorithm,config,trial_id,pose_id,success,planning_time_ms,"
               "execution_time_ms,iterations_used,trajectory_length,path_quality,"
               "collision_clearance,joint_smoothness,start_j1,start_j2,start_j3,"
               "start_j4,start_j5,start_j6,start_j7,goal_x,goal_y,goal_z,"
               "goal_qw,goal_qx,goal_qy,goal_qz";
    }
    
    // Convert to CSV row
    std::string toCSV() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6);
        
        // Algorithm and config
        oss << static_cast<int>(algorithm) << "," << algorithm_config << ","
            << trial_id << "," << pose_id << "," << (success ? 1 : 0) << ","
            << planning_time_ms << "," << execution_time_ms << ","
            << iterations_used << "," << trajectory_length << ","
            << path_quality << "," << collision_clearance << ","
            << joint_smoothness;
            
        // Start joints
        for (int i = 0; i < start_joints.size(); ++i) {
            oss << "," << start_joints[i];
        }
        
        // Goal pose (x,y,z,qw,qx,qy,qz)
        for (int i = 0; i < goal_pose.size(); ++i) {
            oss << "," << goal_pose[i];
        }
        
        return oss.str();
    }
};

/**
 * @brief Configuration for parameter sweep evaluation
 */
struct EvaluationConfig {
    std::vector<StompConfig> stomp_params;
    std::vector<Params> hauser_params;
    std::vector<Eigen::VectorXd> test_poses;
    Eigen::VectorXd start_joint_config;
    int trials_per_config = 10;
    std::string output_directory = "results";
    std::string timestamp;
    
    EvaluationConfig() {
        // Generate timestamp for unique results
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        timestamp = std::to_string(time_t);
    }
};

/**
 * @brief Main trajectory planning evaluator class
 */
class TrajectoryPlanningEvaluator {
public:
    TrajectoryPlanningEvaluator();
    ~TrajectoryPlanningEvaluator();
    
    /**
     * @brief Initialize the trajectory planning environment
     */
    bool initializeEnvironment(const std::string& robot_urdf, 
                             const std::string& environment_urdf);
    
    /**
     * @brief Run comprehensive evaluation across all parameter configurations
     */
    bool runComprehensiveEvaluation(const EvaluationConfig& config);
    
    /**
     * @brief Load test poses from CSV file
     */
    bool loadPosesFromCSV(const std::string& filename);
    
    /**
     * @brief Load test poses from middle 10 poses dataset
     */
    std::vector<Eigen::VectorXd> loadMiddle10Poses();
    
    /**
     * @brief Load poses from ComparisonIK CSV format
     */
    std::vector<Eigen::VectorXd> loadPosesFromComparisonIK(const std::string& filename);
    
    /**
     * @brief Generate parameter sweep configurations
     */
    EvaluationConfig generateParameterSweepConfig();
    
    /**
     * @brief Save results to CSV file
     */
    bool saveResults(const std::vector<EvaluationResult>& results, 
                    const std::string& filename);

private:
    // Trajectory planning components
    std::unique_ptr<RobotArm> robot_arm_;
    std::unique_ptr<MotionGenerator> motion_generator_;
    std::unique_ptr<PathPlanner> path_planner_;
    std::shared_ptr<BVHTree> obstacle_tree_;
    Eigen::VectorXd home_joint_config_;
    
    // Target poses storage
    std::vector<Eigen::Affine3d> target_poses_;
    
    /**
     * @brief Evaluate single trial with STOMP
     */
    EvaluationResult evaluateStompTrial(
        const StompConfig& params,
        const Eigen::VectorXd& goal_pose,
        const Eigen::VectorXd& start_joints,
        int trial_id, int pose_id);
    
    /**
     * @brief Evaluate single trial with Hauser algorithm
     */
    EvaluationResult evaluateHauserTrial(
        EvaluatorAlgorithm algorithm,
        const Params& params,
        const Eigen::VectorXd& goal_pose,
        const Eigen::VectorXd& start_joints,
        int trial_id, int pose_id);
    
    /**
     * @brief Run STOMP planning with specific configuration
     */
    std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool> runStompPlanning(
        const Eigen::VectorXd& start_joints,
        const Eigen::VectorXd& goal_joints,
        const StompConfig& config);
    
    /**
     * @brief Run Hauser planning with specific configuration
     */
    std::pair<std::vector<Eigen::VectorXd>, bool> runHauserPlanning(
        const Eigen::VectorXd& start_joints,
        const Eigen::VectorXd& goal_joints,
        const Params& params);
    
    /**
     * @brief Convert pose to joint configuration using inverse kinematics
     */
    std::pair<Eigen::VectorXd, bool> poseToJoints(const Eigen::VectorXd& pose);
    
    /**
     * @brief Calculate path quality metrics
     */
    double calculatePathQuality(const std::vector<Eigen::VectorXd>& trajectory);
    
    /**
     * @brief Calculate joint smoothness metric
     */
    double calculateJointSmoothness(const std::vector<Eigen::VectorXd>& trajectory);
    
    /**
     * @brief Calculate minimum collision clearance
     */
    double calculateCollisionClearance(const std::vector<Eigen::VectorXd>& trajectory);
};

#endif // TRAJECTORY_PLANNING_EVALUATOR_H
