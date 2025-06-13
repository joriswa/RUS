#ifndef SINGLE_POSE_EVALUATOR_H
#define SINGLE_POSE_EVALUATOR_H

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
 * @brief Configuration for a single pose evaluation run
 */
struct SinglePoseEvalConfig {
    std::string robot_urdf_path;
    std::string environment_xml_path;
    std::string output_directory;
    
    // Robot configuration
    Eigen::VectorXd current_joint_angles;    // Current robot joint configuration
    Eigen::Affine3d target_pose;             // Target pose to reach
    
    // Evaluation parameters
    int num_trials = 10;
    bool verbose = true;
    bool apply_pose_offset = true;            // Apply 2cm pose offset (like in other evaluators)
    
    // Clearance evaluation parameters
    bool enable_clearance_analysis = true;    // Enable clearance metric computation
    double critical_clearance_threshold = 0.05; // Critical clearance threshold (5cm)
    double safe_clearance_threshold = 0.15;   // Safe clearance threshold (15cm)
    bool compute_self_clearance = true;       // Compute self-collision clearance
    int clearance_sample_rate = 1;           // Sample every N points for clearance (1 = all points)
    
    // Trajectory planning focused evaluation parameters
    bool enable_trajectory_planning_analysis = true; // Enable trajectory planning metrics
    double singularity_threshold = 0.01;            // Manipulability threshold for singularity detection
    double critical_ttc_threshold = 1.0;            // Critical time-to-collision threshold (1 second)
    double max_joint_torque_limit = 100.0;          // Maximum allowable joint torque (Nm)
    bool enable_execution_analysis = true;          // Enable execution time and variance analysis
    
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
 * @brief Results from a single pose evaluation
 */
struct SinglePoseResult {
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
    
    // Clearance metrics
    double min_clearance = std::numeric_limits<double>::max();         // Minimum clearance to any obstacle
    double avg_clearance = 0.0;                                        // Average clearance along trajectory
    double clearance_variance = 0.0;                                   // Variance in clearance values
    int critical_clearance_violations = 0;                             // Points below critical threshold
    double min_self_clearance = std::numeric_limits<double>::max();    // Minimum self-collision clearance
    double avg_self_clearance = 0.0;                                   // Average self-collision clearance
    std::vector<double> clearance_profile;                             // Full clearance profile along trajectory
    double clearance_margin_ratio = 0.0;                               // Ratio of safe clearance points
    double clearance_path_ratio = 0.0;                                 // Clearance integrated over path length
    
    // Trajectory planning focused metrics  
    double min_manipulability = std::numeric_limits<double>::max();    // Minimum manipulability index
    double avg_manipulability = 0.0;                                   // Average manipulability index
    int singularity_proximity_violations = 0;                          // Points near singularities
    double redundancy_utilization = 0.0;                               // How effectively 7th DOF is used
    double min_time_to_collision = std::numeric_limits<double>::max(); // Minimum time to collision
    double avg_time_to_collision = 0.0;                                // Average time to collision
    int critical_ttc_violations = 0;                                   // Points with TTC below threshold
    double max_joint_torque = 0.0;                                     // Maximum joint torque required
    double avg_joint_torque = 0.0;                                     // Average joint torque
    bool dynamic_feasibility = true;                                   // Within actuator limits
    
    // Execution time and variance metrics
    double execution_time_estimate = 0.0;                              // Estimated trajectory execution time
    double velocity_variance = 0.0;                                    // Variance in joint velocities
    double acceleration_variance = 0.0;                                // Variance in joint accelerations
    
    // Algorithm-specific data
    std::string algorithm_used;
    std::string path_planning_algorithm_used = ""; // For Hauser cases
    int iterations_used = 0;
    int path_planning_iterations_used = 0; // For Hauser cases
    double path_planning_time_ms = 0.0; // For Hauser cases
    double motion_generation_time_ms = 0.0; // For Hauser cases
    
    // Pose-specific information
    Eigen::Vector3d start_position;
    Eigen::Vector3d target_position;
    double cartesian_path_distance = 0.0;
};

/**
 * @brief Aggregate statistics from multiple single pose evaluation runs
 */
struct SinglePoseStatistics {
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
    
    // Clearance statistics
    double avg_min_clearance = 0.0;
    double avg_avg_clearance = 0.0;
    double avg_clearance_variance = 0.0;
    double avg_critical_violations = 0.0;
    double avg_self_clearance = 0.0;
    double avg_clearance_margin_ratio = 0.0;
    double avg_clearance_path_ratio = 0.0;
    double std_min_clearance = 0.0;
    double std_avg_clearance = 0.0;
    double worst_min_clearance = std::numeric_limits<double>::max();
    double best_min_clearance = 0.0;
    
    // Trajectory planning focused statistics
    double avg_min_manipulability = 0.0;
    double avg_avg_manipulability = 0.0;
    double avg_singularity_violations = 0.0;
    double avg_redundancy_utilization = 0.0;
    double avg_min_ttc = 0.0;
    double avg_avg_ttc = 0.0;
    double avg_critical_ttc_violations = 0.0;
    double avg_max_joint_torque = 0.0;
    double avg_avg_joint_torque = 0.0;
    double avg_dynamic_feasibility_rate = 0.0;
    double std_min_manipulability = 0.0;
    double std_avg_manipulability = 0.0;
    double worst_min_manipulability = std::numeric_limits<double>::max();
    double best_min_manipulability = 0.0;
    
    // Execution time and variance statistics
    double avg_execution_time = 0.0;
    double std_execution_time = 0.0;
    double avg_velocity_variance = 0.0;
    double avg_acceleration_variance = 0.0;
    double std_velocity_variance = 0.0;
    double std_acceleration_variance = 0.0;
    
    int total_trials = 0;
    int successful_trials = 0;
    
    // Pose-specific statistics
    double avg_cartesian_path_distance = 0.0;
    double std_cartesian_path_distance = 0.0;
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
 * @brief Specialized evaluator for single pose trajectory planning
 * 
 * This evaluator focuses on planning trajectories from current joint angles
 * to a single target pose. Useful for real-time trajectory planning evaluation
 * and algorithm comparison for specific pose-to-pose movements.
 */
class SinglePoseEvaluator {
public:
    /**
     * @brief Constructor
     * @param config Configuration for the evaluation
     */
    explicit SinglePoseEvaluator(const SinglePoseEvalConfig& config);
    
    /**
     * @brief Destructor
     */
    ~SinglePoseEvaluator() = default;
    
    /**
     * @brief Run comprehensive evaluation for the configured pose
     * @return True if evaluation completed successfully
     */
    bool runEvaluation();
    
    /**
     * @brief Evaluate a single trajectory planning task
     * @param trial_number Current trial number for logging
     * @return Evaluation results
     */
    SinglePoseResult evaluateSingleTrajectory(int trial_number);
    
    /**
     * @brief Set a new target pose for evaluation
     * @param target_pose New target pose
     * @param apply_offset Whether to apply 2cm pose offset
     */
    void setTargetPose(const Eigen::Affine3d& target_pose, bool apply_offset = true);
    
    /**
     * @brief Set new current joint angles
     * @param joint_angles New current joint configuration
     */
    void setCurrentJointAngles(const Eigen::VectorXd& joint_angles);
    
    /**
     * @brief Export results to CSV format
     * @param results Vector of results to export
     * @param filename Output filename
     */
    void exportResultsToCSV(
        const std::vector<SinglePoseResult>& results,
        const std::string& filename
    );
    
    /**
     * @brief Export summary statistics to file
     * @param stats Statistics to export
     * @param filename Output filename
     */
    void exportStatistics(
        const SinglePoseStatistics& stats,
        const std::string& filename
    );
    
    /**
     * @brief Compute aggregate statistics from results
     * @param results Vector of individual results
     * @return Computed statistics
     */
    SinglePoseStatistics computeStatistics(
        const std::vector<SinglePoseResult>& results
    );
    
    /**
     * @brief Create a pose from position and orientation
     * @param position Target position (x, y, z)
     * @param orientation Target orientation as quaternion (w, x, y, z)
     * @return Eigen::Affine3d pose
     */
    static Eigen::Affine3d createPose(
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation
    );
    
    /**
     * @brief Create a pose from position and RPY angles
     * @param position Target position (x, y, z)
     * @param roll Roll angle in radians
     * @param pitch Pitch angle in radians
     * @param yaw Yaw angle in radians
     * @return Eigen::Affine3d pose
     */
    static Eigen::Affine3d createPoseFromRPY(
        const Eigen::Vector3d& position,
        double roll, double pitch, double yaw
    );

private:
    mutable SinglePoseEvalConfig config_;
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
     * @param result Result object to populate
     * @return True if successful
     */
    bool executeStompVariant(SinglePoseResult& result);
    
    /**
     * @brief Execute Hauser algorithm with path planning
     * @param result Result object to populate
     * @return True if successful
     */
    bool executeHauserAlgorithm(SinglePoseResult& result);
    
    /**
     * @brief Analyze trajectory and populate result metrics
     * @param trajectory Trajectory to analyze
     * @param result Result object to populate
     */
    void analyzeTrajectory(
        const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
        SinglePoseResult& result
    );
    
    /**
     * @brief Create RobotArm from joint configuration
     * @param joints Joint angles vector
     * @return RobotArm instance
     */
    RobotArm createRobotArmFromJoints(const Eigen::VectorXd& joints);
    
    /**
     * @brief Apply 2cm pose offset along local Z-axis
     * @param pose Input pose
     * @return Pose with offset applied
     */
    Eigen::Affine3d applyPoseOffset(const Eigen::Affine3d& pose);
    
    /**
     * @brief Get current end-effector pose from joint angles
     * @return Current end-effector pose
     */
    Eigen::Affine3d getCurrentEndEffectorPose();
    
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
     * @brief Calculate comprehensive clearance metrics for trajectory
     * @param trajectory Trajectory to analyze
     * @param result Result object to populate with clearance data
     */
    void calculateClearanceMetrics(
        const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
        SinglePoseResult& result
    );
    
    /**
     * @brief Calculate clearance at a specific trajectory point
     * @param joint_angles Joint configuration to check
     * @return Minimum clearance distance to obstacles
     */
    double calculatePointClearance(const Eigen::VectorXd& joint_angles);
    
    /**
     * @brief Calculate self-collision clearance at a trajectory point
     * @param joint_angles Joint configuration to check
     * @return Minimum self-collision clearance distance
     */
    double calculateSelfClearance(const Eigen::VectorXd& joint_angles);
    
    /**
     * @brief Get obstacle distance for robot configuration
     * @param robot_arm Robot in specific configuration
     * @return Minimum distance to any obstacle
     */
    double getObstacleDistance(const RobotArm& robot_arm);
    
    /**
     * @brief Get self-collision distance for robot configuration
     * @param robot_arm Robot in specific configuration
     * @return Minimum self-collision distance
     */
    double getSelfCollisionDistance(const RobotArm& robot_arm);
    
    /**
     * @brief Export detailed clearance profile to CSV
     * @param result Result containing clearance profile
     * @param filename Output filename for clearance profile
     */
    void exportClearanceProfile(
        const SinglePoseResult& result,
        const std::string& filename
    );
    
    /**
     * @brief Calculate trajectory planning focused metrics
     * @param trajectory Trajectory to analyze
     * @param result Result object to populate with trajectory planning metrics
     */
    void calculateTrajectoryPlanningMetrics(
        const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
        SinglePoseResult& result
    );
    
    /**
     * @brief Calculate manipulability index for robot configuration
     * @param joint_angles Joint configuration to check
     * @return Manipulability index (Yoshikawa measure)
     */
    double calculateManipulability(const Eigen::VectorXd& joint_angles);
    
    /**
     * @brief Calculate time to collision for trajectory point
     * @param joint_angles Joint configuration
     * @param joint_velocities Joint velocities
     * @return Time to collision estimate
     */
    double calculateTimeToCollision(const Eigen::VectorXd& joint_angles, 
                                   const Eigen::VectorXd& joint_velocities);
    
    /**
     * @brief Calculate joint torque requirements for trajectory point
     * @param trajectory_point Point with position, velocity, acceleration
     * @return Maximum joint torque magnitude
     */
    double calculateJointTorque(const MotionGenerator::TrajectoryPoint& trajectory_point);
    
    /**
     * @brief Calculate execution time and variance metrics
     * @param trajectory Trajectory to analyze
     * @param result Result object to populate with execution metrics
     */
    void calculateExecutionMetrics(
        const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
        SinglePoseResult& result
    );
    
    /**
     * @brief Calculate clearance over path ratio
     * @param trajectory Trajectory to analyze
     * @param clearances Clearance values for each point
     * @return Path-weighted clearance ratio
     */
    double calculateClearancePathRatio(
        const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
        const std::vector<double>& clearances
    );
    
    /**
     * @brief Log progress and results
     * @param message Message to log
     */
    void log(const std::string& message);
};

#endif // SINGLE_POSE_EVALUATOR_H
