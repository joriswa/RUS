#pragma once

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Robot/RobotArm.h"

/**
 * @brief Task-space trajectory generator for ultrasound scanning
 * 
 * This class generates smooth trajectories for ultrasound scanning by:
 * 1. Interpolating between scan poses in Cartesian (task) space
 * 2. Planning based on probe scan speed for consistent scanning velocity
 * 3. Using analytical inverse kinematics to convert to joint trajectories
 * 4. Applying quintic polynomial smoothing for smooth joint motion
 * 
 * The key advantage over joint-space planning is direct control over the
 * probe's Cartesian motion, ensuring consistent scan speed and smooth paths
 * in task space rather than just joint space.
 */
class TaskSpaceScanTrajectoryGenerator
{
public:
    /**
     * @brief Configuration for task-space scan trajectory generation
     */
    struct ScanConfig {
        double scan_speed_ms = 0.005;        ///< Scan speed in m/s (default: 5mm/s)
        double control_frequency = 10.0;     ///< Control frequency in Hz
        bool interpolate_orientation = true; ///< Whether to smoothly interpolate orientation
        bool validate_ik_solutions = true;   ///< Whether to validate IK solutions for reachability
        double max_joint_velocity = 1.0;     ///< Maximum allowed joint velocity (rad/s)
        double max_joint_acceleration = 2.0; ///< Maximum allowed joint acceleration (rad/s²)
        int max_ik_attempts = 5;             ///< Maximum attempts for IK solving per waypoint
        double ik_perturbation_radius = 0.001; ///< Perturbation radius for IK retry (m)
    };

    /**
     * @brief Result of trajectory generation with diagnostics
     */
    struct GenerationResult {
        std::vector<MotionGenerator::TrajectoryPoint> trajectory;
        bool success = false;
        int num_waypoints_generated = 0;
        int num_ik_failures = 0;
        double total_duration = 0.0;
        double actual_average_speed = 0.0;
        std::string error_message;
    };

    /**
     * @brief Constructor
     * @param robot_arm Reference to the robot arm model for IK and validation
     */
    explicit TaskSpaceScanTrajectoryGenerator(const RobotArm& robot_arm);

    /**
     * @brief Generate a task-space trajectory for ultrasound scanning
     * 
     * @param scan_poses Vector of target scan poses in world coordinates
     * @param start_joints Initial joint configuration
     * @param config Configuration parameters for trajectory generation
     * @return Generation result with trajectory and diagnostics
     */
    GenerationResult generateTrajectory(
        const std::vector<Eigen::Affine3d>& scan_poses,
        const Eigen::VectorXd& start_joints,
        const ScanConfig& config);

    /**
     * @brief Generate trajectory between two poses with specified scan speed
     * 
     * @param start_pose Starting pose in world coordinates
     * @param end_pose Ending pose in world coordinates
     * @param start_joints Starting joint configuration
     * @param config Configuration parameters
     * @return Generation result with trajectory and diagnostics
     */
    GenerationResult generateSegmentTrajectory(
        const Eigen::Affine3d& start_pose,
        const Eigen::Affine3d& end_pose,
        const Eigen::VectorXd& start_joints,
        const ScanConfig& config);

    /**
     * @brief Set collision checking callback for trajectory validation
     * @param collision_checker Function that returns true if a joint configuration has collision
     */
    void setCollisionChecker(std::function<bool(const Eigen::VectorXd&)> collision_checker);

private:
    /**
     * @brief Interpolate Cartesian path between two poses
     * 
     * @param start_pose Starting pose
     * @param end_pose Ending pose
     * @param scan_speed Desired scan speed in m/s
     * @param control_frequency Control frequency in Hz
     * @param interpolate_orientation Whether to interpolate orientation
     * @return Vector of interpolated poses with timing information
     */
    std::vector<std::pair<Eigen::Affine3d, double>> interpolateCartesianPath(
        const Eigen::Affine3d& start_pose,
        const Eigen::Affine3d& end_pose,
        double scan_speed,
        double control_frequency,
        bool interpolate_orientation);

    /**
     * @brief Convert Cartesian waypoints to joint configurations using analytical IK
     * 
     * @param cartesian_waypoints Vector of poses with timing
     * @param start_joints Starting joint configuration for continuity
     * @param config Configuration parameters
     * @return Vector of joint configurations with timing, may have fewer points if IK fails
     */
    std::vector<std::pair<Eigen::VectorXd, double>> convertToJointTrajectory(
        const std::vector<std::pair<Eigen::Affine3d, double>>& cartesian_waypoints,
        const Eigen::VectorXd& start_joints,
        const ScanConfig& config);

    /**
     * @brief Apply quintic polynomial smoothing to joint trajectory
     * 
     * @param joint_waypoints Vector of joint configurations with timing
     * @param config Configuration parameters
     * @return Smoothed trajectory points
     */
    std::vector<MotionGenerator::TrajectoryPoint> smoothJointTrajectory(
        const std::vector<std::pair<Eigen::VectorXd, double>>& joint_waypoints,
        const ScanConfig& config);

    /**
     * @brief Select best IK solution for continuity
     * 
     * @param ik_solutions Array of 4 possible IK solutions
     * @param previous_joints Previous joint configuration
     * @return Best joint configuration, or empty vector if all solutions invalid
     */
    Eigen::VectorXd selectBestIKSolution(
        const std::array<std::array<double, 7>, 4>& ik_solutions,
        const Eigen::VectorXd& previous_joints);

    /**
     * @brief Interpolate orientation using SLERP
     * 
     * @param start_rot Starting rotation matrix
     * @param end_rot Ending rotation matrix
     * @param t Interpolation parameter [0, 1]
     * @return Interpolated rotation matrix
     */
    Eigen::Matrix3d interpolateOrientation(
        const Eigen::Matrix3d& start_rot,
        const Eigen::Matrix3d& end_rot,
        double t);

    /**
     * @brief Compute Cartesian distance between two poses
     * 
     * @param pose1 First pose
     * @param pose2 Second pose
     * @param include_rotation Whether to include rotational distance
     * @return Distance in meters (and radians if rotation included)
     */
    double computeCartesianDistance(
        const Eigen::Affine3d& pose1,
        const Eigen::Affine3d& pose2,
        bool include_rotation = false);

    /**
     * @brief Validate joint configuration against limits and collisions
     * 
     * @param joints Joint configuration to validate
     * @return True if configuration is valid
     */
    bool validateJointConfiguration(const Eigen::VectorXd& joints);

    /**
     * @brief Check if array contains NaN values
     * 
     * @param arr Array to check
     * @return True if any element is NaN
     */
    bool hasNaN(const std::array<double, 7>& arr);

    const RobotArm& robot_arm_;
    std::function<bool(const Eigen::VectorXd&)> collision_checker_;
};
