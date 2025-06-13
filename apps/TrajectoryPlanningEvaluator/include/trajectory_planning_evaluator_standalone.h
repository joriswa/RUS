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

/**
 * @brief Trajectory planning algorithms supported
 */
enum class Algorithm {
    STOMP,
    HAUSER_RRT,
    HAUSER_RRT_STAR,
    HAUSER_INFORMED_RRT,
    HAUSER_BI_RRT
};

/**
 * @brief STOMP algorithm configuration parameters
 */
struct StompParams {
    int max_iterations = 100;
    int num_noisy_trajectories = 10;
    int num_best_samples = 4;
    double learning_rate = 0.1;
    double temperature = 10.0;
    double dt = 0.1;
    Eigen::VectorXd joint_std_devs = Eigen::VectorXd::Constant(7, 0.1);
    
    std::string toString() const {
        return "STOMP_iter" + std::to_string(max_iterations) + 
               "_noisy" + std::to_string(num_noisy_trajectories) +
               "_lr" + std::to_string(int(learning_rate * 100));
    }
};

/**
 * @brief Hauser algorithm configuration parameters
 */
struct HauserParams {
    int max_iterations = 1000;
    double step_size = 0.1;
    double goal_bias = 0.1;
    double connection_radius = 0.3;
    bool use_informed_sampling = false;
    bool bidirectional = false;
    
    std::string toString() const {
        std::string base = "HAUSER_iter" + std::to_string(max_iterations) + 
                          "_step" + std::to_string(int(step_size * 100)) +
                          "_bias" + std::to_string(int(goal_bias * 100));
        if (use_informed_sampling) base += "_informed";
        if (bidirectional) base += "_bi";
        return base;
    }
};

/**
 * @brief Evaluation result for a single trial
 */
struct EvaluationResult {
    Algorithm algorithm;
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
    std::vector<StompParams> stomp_params;
    std::vector<HauserParams> hauser_params;
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
     * @brief Run comprehensive evaluation across all parameter configurations
     */
    bool runComprehensiveEvaluation(const EvaluationConfig& config);
    
    /**
     * @brief Evaluate a specific algorithm configuration
     */
    std::vector<EvaluationResult> evaluateAlgorithmConfig(
        Algorithm algorithm, 
        const void* params,  // StompParams* or HauserParams*
        const std::vector<Eigen::VectorXd>& test_poses,
        const Eigen::VectorXd& start_joints,
        int trials_per_pose);
    
    /**
     * @brief Load test poses from middle 10 poses dataset
     */
    static std::vector<Eigen::VectorXd> loadMiddle10Poses();
    
    /**
     * @brief Generate parameter sweep configurations
     */
    static EvaluationConfig generateParameterSweepConfig();
    
    /**
     * @brief Save results to CSV file
     */
    bool saveResults(const std::vector<EvaluationResult>& results, 
                    const std::string& filename);

private:
    /**
     * @brief Evaluate single trial with STOMP
     */
    EvaluationResult evaluateStompTrial(
        const StompParams& params,
        const Eigen::VectorXd& goal_pose,
        const Eigen::VectorXd& start_joints,
        int trial_id, int pose_id);
    
    /**
     * @brief Evaluate single trial with Hauser algorithm
     */
    EvaluationResult evaluateHauserTrial(
        Algorithm algorithm,
        const HauserParams& params,
        const Eigen::VectorXd& goal_pose,
        const Eigen::VectorXd& start_joints,
        int trial_id, int pose_id);
    
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
