#ifndef REPOSITIONING_EVALUATOR_H
#define REPOSITIONING_EVALUATOR_H

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <fstream>
#include <Eigen/Dense>

#include "USLib/USTrajectoryPlanner.h"
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Planning/PathPlanner.h"

/**
 * @brief Enum for trajectory planning algorithms
 */
enum class TrajectoryAlgorithm {
    STOMP,
    STOMP_WITH_CHECKPOINTS,
    STOMP_WITH_EARLY_TERMINATION,
    HAUSER
};

/**
 * @brief Enum for path planning algorithms (used with trajectory planning)
 */
enum class PathPlanningAlgorithm {
    RRT,
    RRT_STAR,
    INFORMED_RRT_STAR,
    RRT_CONNECT
};

/**
 * @brief Configuration for STOMP algorithm
 */
struct StompAlgorithmConfig {
    int max_iterations = 100;
    int num_noisy_trajectories = 10;
    int num_best_samples = 4;
    double learning_rate = 0.1;
    double temperature = 10.0;
    double dt = 0.1;
    int num_joints = 7;
    Eigen::VectorXd joint_std_devs = Eigen::VectorXd::Constant(7, 0.1);
    
    // Convert to StompConfig for MotionGenerator
    StompConfig toStompConfig() const {
        StompConfig config;
        config.maxIterations = max_iterations;
        config.numNoisyTrajectories = num_noisy_trajectories;
        config.numBestSamples = num_best_samples;
        config.learningRate = learning_rate;
        config.temperature = temperature;
        config.dt = dt;
        config.numJoints = num_joints;
        config.jointStdDevs = joint_std_devs;
        return config;
    }
};

/**
 * @brief Configuration for Hauser algorithm
 */
struct HauserAlgorithmConfig {
    int max_iterations = 500;
    std::string output_file = ""; // Optional timing output file
};

/**
 * @brief Configuration for RRT-based path planning algorithms
 */
struct PathPlanningConfig {
    PathPlanningAlgorithm algorithm = PathPlanningAlgorithm::RRT;
    int max_iterations = 5000;
    double step_size = 0.1;
    double goal_bias = 0.1;
    bool custom_cost = false;
};

/**
 * @brief Configuration for a single evaluation run
 */
struct RepositioningEvalConfig {
    std::string robot_urdf_path;
    std::string environment_xml_path;
    std::string scan_poses_csv_path;
    std::string output_directory;
    
    // Robot initial configuration
    Eigen::VectorXd initial_joint_config;
    
    // Evaluation parameters
    int num_trials = 10;
    bool verbose = true;
    
    // Algorithm selection
    TrajectoryAlgorithm trajectory_algorithm = TrajectoryAlgorithm::STOMP;
    
    // Algorithm-specific configurations
    StompAlgorithmConfig stomp_config;
    HauserAlgorithmConfig hauser_config;
    PathPlanningConfig path_planning_config; // Used with Hauser for path planning phase
    
    // Legacy parameters (for backward compatibility)
    int max_stomp_iterations = 100;
    int num_noisy_trajectories = 10;
    double stomp_learning_rate = 0.1;
    double stomp_temperature = 10.0;
    int max_rrt_iterations = 5000;
    double rrt_step_size = 0.1;
    double rrt_goal_bias = 0.1;
};

/**
 * @brief Results from a single repositioning evaluation
 */
struct RepositioningResult {
    bool planning_success = false;
    double planning_time_ms = 0.0;
    double trajectory_length = 0.0;
    double trajectory_smoothness = 0.0;
    int num_waypoints = 0;
    double joint_space_distance = 0.0;
    double cartesian_distance = 0.0;
    bool collision_free = true;
    
    // Additional metrics
    double max_joint_velocity = 0.0;
    double max_joint_acceleration = 0.0;
    double energy_consumption = 0.0;
    
    // Algorithm-specific data
    std::string algorithm_used;
    std::string path_planning_algorithm_used = ""; // For Hauser cases
    int iterations_used = 0;
    int path_planning_iterations_used = 0; // For Hauser cases
    double path_planning_time_ms = 0.0; // For Hauser cases
    double motion_generation_time_ms = 0.0; // For Hauser cases
};

/**
 * @brief Aggregate statistics from multiple evaluation runs
 */
struct RepositioningStatistics {
    double success_rate = 0.0;
    double avg_planning_time_ms = 0.0;
    double avg_trajectory_length = 0.0;
    double avg_smoothness = 0.0;
    double avg_joint_distance = 0.0;
    double avg_cartesian_distance = 0.0;
    
    // Standard deviations
    double std_planning_time_ms = 0.0;
    double std_trajectory_length = 0.0;
    double std_smoothness = 0.0;
    
    int total_trials = 0;
    int successful_trials = 0;
};

/**
 * @brief Utility functions for algorithm conversion
 */
namespace AlgorithmUtils {
    /**
     * @brief Convert TrajectoryAlgorithm enum to string
     */
    inline std::string trajectoryAlgorithmToString(TrajectoryAlgorithm algo) {
        switch (algo) {
            case TrajectoryAlgorithm::STOMP: return "STOMP";
            case TrajectoryAlgorithm::STOMP_WITH_CHECKPOINTS: return "STOMP_WITH_CHECKPOINTS";
            case TrajectoryAlgorithm::STOMP_WITH_EARLY_TERMINATION: return "STOMP_WITH_EARLY_TERMINATION";
            case TrajectoryAlgorithm::HAUSER: return "HAUSER";
            default: return "UNKNOWN";
        }
    }

    /**
     * @brief Convert PathPlanningAlgorithm enum to string
     */
    inline std::string pathPlanningAlgorithmToString(PathPlanningAlgorithm algo) {
        switch (algo) {
            case PathPlanningAlgorithm::RRT: return "RRT";
            case PathPlanningAlgorithm::RRT_STAR: return "RRT_STAR";
            case PathPlanningAlgorithm::INFORMED_RRT_STAR: return "INFORMED_RRT_STAR";
            case PathPlanningAlgorithm::RRT_CONNECT: return "RRT_CONNECT";
            default: return "UNKNOWN";
        }
    }

    /**
     * @brief Convert PathPlanningAlgorithm enum to Algorithm enum for PathPlanner
     */
    inline Algorithm pathPlanningAlgorithmToLibraryEnum(PathPlanningAlgorithm algo) {
        switch (algo) {
            case PathPlanningAlgorithm::RRT: return Algorithm::RRT;
            case PathPlanningAlgorithm::RRT_STAR: return Algorithm::RRTStar;
            case PathPlanningAlgorithm::INFORMED_RRT_STAR: return Algorithm::InformedRRTStar;
            case PathPlanningAlgorithm::RRT_CONNECT: return Algorithm::RRTConnect;
            default: return Algorithm::RRT;
        }
    }

    /**
     * @brief Create Params struct for PathPlanner from PathPlanningConfig
     */
    inline Params createParamsFromConfig(const PathPlanningConfig& config) {
        Params params;
        params.algo = pathPlanningAlgorithmToLibraryEnum(config.algorithm);
        params.maxIterations = config.max_iterations;
        params.stepSize = config.step_size;
        params.goalBiasProbability = config.goal_bias;
        params.customCost = config.custom_cost;
        return params;
    }
}

/**
 * @brief Specialized evaluator for ultrasound repositioning trajectories
 * 
 * This evaluator focuses on the specific use case of repositioning between
 * two scan poses for ultrasound applications. It evaluates both path planning
 * and motion generation performance with metrics relevant to medical robotics.
 */
class RepositioningEvaluator {
public:
    /**
     * @brief Constructor
     * @param config Configuration for the evaluation
     */
    explicit RepositioningEvaluator(const RepositioningEvalConfig& config);
    
    /**
     * @brief Destructor
     */
    ~RepositioningEvaluator() = default;
    
    /**
     * @brief Run comprehensive evaluation
     * @return True if evaluation completed successfully
     */
    bool runEvaluation();
    
    /**
     * @brief Load scan poses from CSV file
     * @param csv_path Path to the CSV file containing scan poses
     * @return Vector of poses loaded from the file
     */
    std::vector<Eigen::Affine3d> loadScanPoses(const std::string& csv_path);
    
    /**
     * @brief Evaluate a single repositioning task
     * @param start_pose Starting pose for repositioning
     * @param target_pose Target pose for repositioning
     * @param trial_number Current trial number for logging
     * @return Evaluation results
     */
    RepositioningResult evaluateSingleRepositioning(
        const Eigen::Affine3d& start_pose,
        const Eigen::Affine3d& target_pose,
        int trial_number
    );
    
    /**
     * @brief Export results to CSV format
     * @param results Vector of results to export
     * @param filename Output filename
     */
    void exportResultsToCSV(
        const std::vector<RepositioningResult>& results,
        const std::string& filename
    );
    
    /**
     * @brief Export summary statistics to file
     * @param stats Statistics to export
     * @param filename Output filename
     */
    void exportStatistics(
        const RepositioningStatistics& stats,
        const std::string& filename
    );
    
    /**
     * @brief Compute aggregate statistics from results
     * @param results Vector of individual results
     * @return Computed statistics
     */
    RepositioningStatistics computeStatistics(
        const std::vector<RepositioningResult>& results
    );

private:
    mutable RepositioningEvalConfig config_;
    std::unique_ptr<UltrasoundScanTrajectoryPlanner> planner_;
    
    /**
     * @brief Initialize the trajectory planner
     * @return True if initialization successful
     */
    bool initializePlanner();
    
    /**
     * @brief Setup algorithm configurations from legacy parameters
     */
    void setupAlgorithmConfigs();
    
    /**
     * @brief Execute STOMP algorithm variants
     * @param start_pose Starting pose
     * @param target_pose Target pose
     * @param result Result object to populate
     * @return True if successful
     */
    bool executeStompVariant(
        const Eigen::Affine3d& start_pose,
        const Eigen::Affine3d& target_pose,
        RepositioningResult& result
    );
    
    /**
     * @brief Execute Hauser algorithm with path planning
     * @param start_pose Starting pose
     * @param target_pose Target pose
     * @param result Result object to populate
     * @return True if successful
     */
    bool executeHauserAlgorithm(
        const Eigen::Affine3d& start_pose,
        const Eigen::Affine3d& target_pose,
        RepositioningResult& result
    );
    
    /**
     * @brief Analyze trajectory and populate result metrics
     * @param trajectory Trajectory to analyze
     * @param result Result object to populate
     */
    void analyzeTrajectory(
        const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
        RepositioningResult& result
    );
    
    /**
     * @brief Create RobotArm from joint configuration
     * @param joints Joint angles vector
     * @return RobotArm instance
     */
    RobotArm createRobotArmFromJoints(const Eigen::VectorXd& joints);
    
    /**
     * @brief Calculate trajectory smoothness metric
     * @param trajectory Trajectory to analyze
     * @return Smoothness value (lower is smoother)
     */
    double calculateTrajectorySmootness(
        const std::vector<MotionGenerator::TrajectoryPoint>& trajectory
    );
    
    /**
     * @brief Calculate total joint space distance
     * @param trajectory Trajectory to analyze
     * @return Total distance in joint space
     */
    double calculateJointSpaceDistance(
        const std::vector<MotionGenerator::TrajectoryPoint>& trajectory
    );
    
    /**
     * @brief Check if trajectory is collision-free
     * @param trajectory Trajectory to check
     * @return True if collision-free
     */
    bool checkTrajectoryCollisionFree(
        const std::vector<MotionGenerator::TrajectoryPoint>& trajectory
    );
    
    /**
     * @brief Calculate energy consumption estimate
     * @param trajectory Trajectory to analyze
     * @return Energy consumption estimate
     */
    double calculateEnergyConsumption(
        const std::vector<MotionGenerator::TrajectoryPoint>& trajectory
    );
    
    /**
     * @brief Log progress and results
     * @param message Message to log
     */
    void log(const std::string& message);
};

#endif // REPOSITIONING_EVALUATOR_H
