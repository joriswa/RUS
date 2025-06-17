#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <memory>
#include <fstream>
#include <sstream>
#include <thread>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <Eigen/Dense>
#include <boost/asio/thread_pool.hpp>

#include "TrajectoryLib/Robot/RobotArm.h"
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Planning/PathPlanner.h"
#include "TrajectoryLib/Robot/RobotManager.h"
#include "GeometryLib/BVHTree.h"

#include <QDebug>
#include <QCoreApplication>

/**
 * Enhanced STOMP vs Hauser Trajectory Planning Comparison
 * ======================================================
 * This tool generates trajectory data for comprehensive analysis:
 * - Loads all poses and pre-computes goal configurations
 * - Runs multiple trials per pose for statistical analysis
 * - Outputs detailed trajectory data for external analysis
 * - Supports Python post-processing for metrics and visualization
 */

struct ScanPose
{
    std::string name;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    bool valid;
    Eigen::VectorXd goal_config;
    bool goal_config_valid;

    ScanPose(const std::string &n, const Eigen::Vector3d &pos, const Eigen::Quaterniond &quat, bool v)
        : name(n), position(pos), orientation(quat), valid(v), goal_config_valid(false) {}
};

struct TrialResult
{
    std::string algorithm;
    std::string pose_name;
    int pose_index;
    int trial_number;
    bool success;
    double planning_time_ms;
    double trajectory_duration_s;
    int trajectory_points;
    std::vector<double> start_config;
    std::vector<double> end_config;
    std::vector<MotionGenerator::TrajectoryPoint> trajectory;

    TrialResult() : pose_index(-1), trial_number(-1), success(false),
                    planning_time_ms(0.0), trajectory_duration_s(0.0), trajectory_points(0) {}
};

class EnhancedTrajectoryComparison
{
private:
    std::unique_ptr<RobotArm> robot_;
    std::shared_ptr<BVHTree> obstacle_tree_;
    std::unique_ptr<PathPlanner> path_planner_;
    std::vector<ScanPose> scan_poses_;
    
    // Consistent trajectory output frequency for both algorithms
    static constexpr double TRAJECTORY_OUTPUT_FREQUENCY = 100.0; // Hz

    void initializeLogging()
    {
        // Create results directory if it doesn't exist
        std::system("mkdir -p results");

        // Redirect all library logging to stderr, then redirect stderr to a log file
        // This ensures that any logging from STOMP/Hauser libraries goes to the log file
        std::freopen("results/comparison_stderr.log", "w", stderr);

        // Also redirect stdout temporarily to capture library logs that might go to stdout
        // We'll restore stdout later for our own console output
        std::cerr << "✓ Redirecting library logs to stderr file to keep CSV clean" << std::endl;
        std::cerr << "=== Library logs start here ===" << std::endl;
    }

public:
    EnhancedTrajectoryComparison()
    {
        initializeLogging();
        std::cerr << "\n=== Enhanced STOMP vs Hauser Trajectory Comparison ===" << std::endl;
        std::cerr << "Configuration: Using ONLY feasible poses from scan data" << std::endl;
        std::cerr << "No default/demo poses will be used as fallback" << std::endl;
        std::cerr << "Trajectory output frequency: " << TRAJECTORY_OUTPUT_FREQUENCY << " Hz" << std::endl;
        initializeRobot();
        createTestEnvironment();
        setInitialRobotConfiguration();
        loadScanPoses();
        std::cerr << "Initialization complete.\n"
                  << std::endl;
    }

    void initializeRobot()
    {
        std::cerr << "Loading robot from URDF..." << std::endl;

        std::string urdf_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf";
        try
        {
            robot_ = std::make_unique<RobotArm>(urdf_path);
            std::cerr << "✓ Robot loaded successfully from: " << urdf_path << std::endl;

            auto joint_limits = robot_->jointLimits();
            std::cerr << "  - Robot has " << joint_limits.size() << " joints" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "✗ Failed to load robot: " << e.what() << std::endl;
            throw;
        }
    }

    void createTestEnvironment()
    {
        std::cerr << "Creating test environment..." << std::endl;

        std::string environment_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/obstacles.xml";

        try
        {
            RobotManager robot_manager;
            robot_manager.parseURDF(environment_path);
            obstacle_tree_ = std::make_shared<BVHTree>(robot_manager.getTransformedObstacles());

            std::cerr << "✓ Environment loaded from: " << environment_path << std::endl;
            std::cerr << "✓ BVH tree created with " << robot_manager.getTransformedObstacles().size()
                      << " obstacles" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "⚠️  Failed to load environment from XML: " << e.what() << std::endl;
            std::cerr << "   Creating empty obstacle tree for testing..." << std::endl;

            std::vector<std::shared_ptr<Obstacle>> empty_obstacles;
            obstacle_tree_ = std::make_shared<BVHTree>(empty_obstacles);
            std::cerr << "✓ Empty obstacle tree created for basic testing" << std::endl;
        }
    }

    void setInitialRobotConfiguration()
    {
        std::cerr << "Setting initial robot configuration..." << std::endl;

        Eigen::VectorXd initial_joints(7);
        initial_joints << 0.374894, -0.043533, 0.087470, -1.533429, 0.02237, 1.050135, 0.075773;

        robot_->setJointAngles(initial_joints);

        std::cerr << "✓ Robot set to initial configuration from mainwindow" << std::endl;
        std::cerr << "  Initial joints: [";
        for (int i = 0; i < initial_joints.size(); ++i)
        {
            std::cerr << std::fixed << std::setprecision(6) << initial_joints[i];
            if (i < initial_joints.size() - 1)
                std::cerr << ", ";
        }
        std::cerr << "]" << std::endl;
    }

    void loadScanPoses()
    {
        std::cerr << "Loading scan poses from CSV using mainwindow method..." << std::endl;

        std::string csv_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/scan_poses.csv";
        std::ifstream file(csv_path);

        if (!file.is_open())
        {
            std::cerr << "❌ Error: Failed to open scan poses CSV: " << csv_path << std::endl;
            std::cerr << "   This is required - no default poses will be used." << std::endl;
            std::cerr << "   Please ensure the scan poses CSV file exists." << std::endl;
            throw std::runtime_error("Scan poses CSV file not found: " + csv_path);
        }

        std::string line;
        int pose_count = 0;

        while (std::getline(file, line) && pose_count < 10)
        {
            if (line.empty())
                continue;
            
            std::cerr << "Debug: CSV line " << pose_count + 1 << ": " << line << std::endl;

            std::stringstream ss(line);
            std::string token;

            std::vector<double> values;
            while (std::getline(ss, token, ',')) {
                values.push_back(std::stod(token));
            }

            if (values.size() < 7)
                continue;

            // Use exact same logic as mainwindow.cpp
            double x = values[0];
            double y = values[1];
            double z = values[2];
            Eigen::Quaterniond q(values[3], values[4], values[5], values[6]);
            q.normalize();

            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) = q.toRotationMatrix();
            T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

            const Eigen::Vector3d local_move(0.0, 0.0, -0.02);
            T.block<3, 1>(0, 3) += T.block<3, 3>(0, 0) * local_move;

            // Extract position and orientation from the computed transform
            Eigen::Vector3d position = T.block<3, 1>(0, 3);
            Eigen::Quaterniond orientation(T.block<3, 3>(0, 0));
            
            // Check validity from CSV (assuming last column is validity if available)
            bool valid = true;  // Default to valid since mainwindow doesn't check this
            if (values.size() >= 10) {
                int valid_int = static_cast<int>(values[9]);
                valid = (valid_int == 1);
            }

            std::string name = "ScanPose_" + std::to_string(pose_count + 1);
            scan_poses_.emplace_back(name, position, orientation, valid);
            pose_count++;
        }

        file.close();
        std::cerr << "✓ Loaded " << scan_poses_.size() << " scan poses from CSV using mainwindow method" << std::endl;
        std::cerr << "  Note: Only poses marked as valid (last column = 1) will be used" << std::endl;

        // Show first few loaded poses for verification
        for (size_t i = 0; i < std::min(size_t(3), scan_poses_.size()); ++i)
        {
            const auto &pose = scan_poses_[i];
            std::cerr << "  " << pose.name << ": pos=["
                      << std::fixed << std::setprecision(3)
                      << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z()
                      << "] valid=" << (pose.valid ? "YES" : "NO") << std::endl;
        }

        precomputeGoalConfigurations();
    }

    void setupPathPlanner()
    {
        path_planner_ = std::make_unique<PathPlanner>();
        path_planner_->setStartPose(*robot_);
        path_planner_->setObstacleTree(obstacle_tree_);
    }

    void precomputeGoalConfigurations()
    {
        std::cerr << "\nPre-computing goal configurations for all poses..." << std::endl;

        setupPathPlanner();

        // More aggressive IK parameters for difficult scan poses
        double T_max = 500.0;         // Much higher initial temperature
        double T_min = 0.01;          // Lower final temperature
        double alpha = 0.99;          // Very slow cooling
        int max_iterations = 5000;    // Many more iterations
        int max_no_improvement = 500; // Much more patience

        std::cerr << "Using aggressive IK parameters: T_max=" << T_max
                  << ", iterations=" << max_iterations << std::endl;

        int successful_configs = 0;

        for (size_t i = 0; i < scan_poses_.size(); ++i)
        {
            auto &pose = scan_poses_[i];

            if (!pose.valid)
            {
                std::cerr << "  Skipping invalid pose: " << pose.name << std::endl;
                continue;
            }

            std::cerr << "  Computing config for " << pose.name << " at pos=["
                      << std::fixed << std::setprecision(3)
                      << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z()
                      << "] quat=[" << std::setprecision(2)
                      << pose.orientation.w() << "," << pose.orientation.x()
                      << "," << pose.orientation.y() << "," << pose.orientation.z() << "]... ";

            Eigen::Affine3d target_pose = Eigen::Affine3d::Identity();
            target_pose.translation() = pose.position;
            target_pose.linear() = pose.orientation.toRotationMatrix();

            auto ik_result = path_planner_->selectGoalPoseSimulatedAnnealing(
                target_pose, T_max, T_min, alpha, max_iterations, max_no_improvement);

            if (!ik_result.second)
            {
                std::cerr << "IK failed (trying " << max_iterations << " iterations)" << std::endl;
                continue;
            }

            bool has_collision = path_planner_->armHasCollision(ik_result.first);
            if (has_collision)
            {
                std::cerr << "Collision detected" << std::endl;
                continue;
            }

            pose.goal_config = ik_result.first.getJointAngles();
            pose.goal_config_valid = true;
            successful_configs++;

            std::cerr << "✓ Success" << std::endl;
        }

        std::cerr << "✓ Pre-computed " << successful_configs << "/" << scan_poses_.size() << " goal configurations" << std::endl;

        // Only use feasible poses from scan data - no fallback to default poses
        if (successful_configs == 0)
        {
            std::cerr << "❌ Error: No poses have valid goal configurations from the scan data." << std::endl;
            std::cerr << "   All scan poses either failed IK or have collisions." << std::endl;
            std::cerr << "   Please check the scan poses file or robot configuration." << std::endl;
            throw std::runtime_error("No feasible goal configurations found in scan poses");
        }

        // Filter out poses without valid goal configurations
        auto original_size = scan_poses_.size();
        scan_poses_.erase(
            std::remove_if(scan_poses_.begin(), scan_poses_.end(),
                           [](const ScanPose &pose)
                           { return !pose.valid || !pose.goal_config_valid; }),
            scan_poses_.end());

        std::cerr << "✓ Using " << scan_poses_.size() << " feasible poses from scan data" << std::endl;
        if (scan_poses_.size() < original_size)
        {
            std::cerr << "  (Filtered out " << (original_size - scan_poses_.size()) << " infeasible poses)" << std::endl;
        }
    }

    TrialResult runSTOMPTrial(int pose_index, int trial_number, const Eigen::VectorXd &initial_config, const Eigen::VectorXd &target_config)
    {
        TrialResult result;
        result.algorithm = "STOMP";
        result.pose_name = scan_poses_[pose_index].name;
        result.pose_index = pose_index;
        result.trial_number = trial_number;

        std::cerr << "  Running STOMP trial " << trial_number << " for " << result.pose_name << "... ";

        auto stomp_motion_generator = std::make_unique<MotionGenerator>(*robot_);
        stomp_motion_generator->setObstacleTree(obstacle_tree_);

        StompConfig config;
        config.numNoisyTrajectories = 4;
        config.maxIterations = 30;
        config.numJoints = 7;
        config.outputFrequency = TRAJECTORY_OUTPUT_FREQUENCY; // Use consistent frequency

        Eigen::MatrixXd waypoints(2, initial_config.size());
        waypoints.row(0) = initial_config.transpose();
        waypoints.row(1) = target_config.transpose();
        stomp_motion_generator->setWaypoints(waypoints);
        stomp_motion_generator->createSDF();

        unsigned int numThreads = std::thread::hardware_concurrency();
        auto threadPool = std::make_shared<boost::asio::thread_pool>(numThreads);

        auto start_time = std::chrono::high_resolution_clock::now();
        bool success = stomp_motion_generator->performSTOMP(config, threadPool);
        threadPool->join();
        auto end_time = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        result.planning_time_ms = duration.count();

        auto trajectory = stomp_motion_generator->getPath();

        if (success && !trajectory.empty())
        {
            result.success = true;
            result.trajectory_duration_s = trajectory.back().time;
            result.trajectory_points = trajectory.size();
            result.trajectory = trajectory;

            result.start_config.resize(initial_config.size());
            result.end_config.resize(target_config.size());
            for (int i = 0; i < initial_config.size(); ++i)
            {
                result.start_config[i] = initial_config[i];
                result.end_config[i] = target_config[i];
            }

            std::cerr << "✓ " << result.planning_time_ms << "ms, " << result.trajectory_points << "pts" << std::endl;
        }
        else
        {
            std::cerr << "✗ Failed" << std::endl;
            result.success = false;
        }

        return result;
    }

    TrialResult runHauserTrial(int pose_index, int trial_number, const Eigen::VectorXd &initial_config, const Eigen::VectorXd &target_config)
    {
        TrialResult result;
        result.algorithm = "Hauser";
        result.pose_name = scan_poses_[pose_index].name;
        result.pose_index = pose_index;
        result.trial_number = trial_number;

        std::cerr << "  Running Hauser trial " << trial_number << " for " << result.pose_name << "... ";

        RobotArm goal_arm = *robot_;
        goal_arm.setJointAngles(target_config);
        path_planner_->setGoalConfiguration(goal_arm);

        auto total_start_time = std::chrono::high_resolution_clock::now();

        bool path_found = path_planner_->runPathFinding();

        if (!path_found)
        {
            std::cerr << "✗ BiRRT failed" << std::endl;
            result.success = false;
            return result;
        }

        Eigen::MatrixXd path_waypoints = path_planner_->getAnglesPath();

        auto hauser_motion_generator = std::make_unique<MotionGenerator>(*robot_);
        hauser_motion_generator->setObstacleTree(obstacle_tree_);
        hauser_motion_generator->setWaypoints(path_waypoints);

        unsigned int maxIterations = 300;
        hauser_motion_generator->performHauser(maxIterations, "", TRAJECTORY_OUTPUT_FREQUENCY);

        auto total_end_time = std::chrono::high_resolution_clock::now();
        auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(total_end_time - total_start_time);

        result.planning_time_ms = total_duration.count();

        auto trajectory = hauser_motion_generator->getPath();

        if (!trajectory.empty())
        {
            result.success = true;
            result.trajectory_duration_s = trajectory.back().time;
            result.trajectory_points = trajectory.size();
            result.trajectory = trajectory;

            result.start_config.resize(initial_config.size());
            result.end_config.resize(target_config.size());
            for (int i = 0; i < initial_config.size(); ++i)
            {
                result.start_config[i] = initial_config[i];
                result.end_config[i] = target_config[i];
            }

            std::cerr << "✓ " << result.planning_time_ms << "ms, " << result.trajectory_points << "pts" << std::endl;
        }
        else
        {
            std::cerr << "✗ Failed" << std::endl;
            result.success = false;
        }

        return result;
    }

    void exportTrialResults(const std::vector<TrialResult> &results, const std::string &filename)
    {
        std::ofstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "Failed to open output file: " << filename << std::endl;
            return;
        }

        file << "algorithm,pose_name,pose_index,trial_number,success,planning_time_ms,trajectory_duration_s,trajectory_points";
        file << ",start_config_0,start_config_1,start_config_2,start_config_3,start_config_4,start_config_5,start_config_6";
        file << ",end_config_0,end_config_1,end_config_2,end_config_3,end_config_4,end_config_5,end_config_6" << std::endl;

        for (const auto &result : results)
        {
            file << result.algorithm << ","
                 << result.pose_name << ","
                 << result.pose_index << ","
                 << result.trial_number << ","
                 << (result.success ? 1 : 0) << ","
                 << std::fixed << std::setprecision(1) << result.planning_time_ms << ","
                 << std::fixed << std::setprecision(3) << result.trajectory_duration_s << ","
                 << result.trajectory_points;

            for (size_t i = 0; i < 7; ++i)
            {
                file << ",";
                if (i < result.start_config.size())
                {
                    file << std::fixed << std::setprecision(6) << result.start_config[i];
                }
                else
                {
                    file << "0.0";
                }
            }

            for (size_t i = 0; i < 7; ++i)
            {
                file << ",";
                if (i < result.end_config.size())
                {
                    file << std::fixed << std::setprecision(6) << result.end_config[i];
                }
                else
                {
                    file << "0.0";
                }
            }

            file << std::endl;
        }

        file.close();
        std::cerr << "✓ Exported trial results to: " << filename << std::endl;
    }

    void exportTrajectories(const std::vector<TrialResult> &results, const std::string &directory)
    {
        std::string mkdir_cmd = "mkdir -p " + directory;
        system(mkdir_cmd.c_str());

        for (const auto &result : results)
        {
            if (!result.success || result.trajectory.empty())
                continue;

            std::string filename = directory + "/" + result.algorithm + "_" + result.pose_name + "_trial" + std::to_string(result.trial_number) + "_trajectory.csv";

            std::ofstream file(filename);
            if (!file.is_open())
            {
                std::cerr << "Failed to open trajectory file: " << filename << std::endl;
                continue;
            }

            file << "time,j0,j1,j2,j3,j4,j5,j6,vel0,vel1,vel2,vel3,vel4,vel5,vel6,acc0,acc1,acc2,acc3,acc4,acc5,acc6" << std::endl;

            for (const auto &point : result.trajectory)
            {
                file << std::fixed << std::setprecision(3) << point.time;

                for (size_t i = 0; i < 7; ++i)
                {
                    file << ",";
                    if (i < point.position.size())
                    {
                        file << std::fixed << std::setprecision(6) << point.position[i];
                    }
                    else
                    {
                        file << "0.0";
                    }
                }

                for (size_t i = 0; i < 7; ++i)
                {
                    file << ",";
                    if (i < point.velocity.size())
                    {
                        file << std::fixed << std::setprecision(6) << point.velocity[i];
                    }
                    else
                    {
                        file << "0.0";
                    }
                }

                for (size_t i = 0; i < 7; ++i)
                {
                    file << ",";
                    if (i < point.acceleration.size())
                    {
                        file << std::fixed << std::setprecision(6) << point.acceleration[i];
                    }
                    else
                    {
                        file << "0.0";
                    }
                }

                file << std::endl;
            }

            file.close();
        }

        std::cerr << "✓ Exported " << results.size() << " trajectories to: " << directory << "/" << std::endl;
    }

    std::vector<TrialResult> runFullComparison(int num_poses = 5, int trials_per_pose = 3)
    {
        std::cerr << "\n=== Running Full STOMP vs Hauser Comparison ===" << std::endl;
        std::cerr << "Poses to test: " << num_poses << std::endl;
        std::cerr << "Trials per pose per algorithm: " << trials_per_pose << std::endl;

        std::vector<TrialResult> all_results;
        all_results.reserve(num_poses * trials_per_pose * 2);

        Eigen::VectorXd initial_config = robot_->getJointAngles();

        int valid_poses_tested = 0;
        for (size_t pose_idx = 0; pose_idx < scan_poses_.size() && valid_poses_tested < num_poses; ++pose_idx)
        {
            const auto &pose = scan_poses_[pose_idx];

            if (!pose.valid || !pose.goal_config_valid)
            {
                std::cerr << "Skipping pose " << pose_idx << " (" << pose.name << ") - invalid or no goal config" << std::endl;
                continue;
            }

            std::cerr << "\n--- Testing Pose " << valid_poses_tested + 1 << "/" << num_poses << ": " << pose.name << " ---" << std::endl;

            for (int trial = 1; trial <= trials_per_pose; ++trial)
            {
                robot_->setJointAngles(initial_config);

                auto stomp_result = runSTOMPTrial(pose_idx, trial, initial_config, pose.goal_config);
                all_results.push_back(stomp_result);

                robot_->setJointAngles(initial_config);

                auto hauser_result = runHauserTrial(pose_idx, trial, initial_config, pose.goal_config);
                all_results.push_back(hauser_result);
            }

            valid_poses_tested++;
        }

        std::cerr << "\n=== COMPARISON SUMMARY ===" << std::endl;
        int stomp_successes = 0, hauser_successes = 0;
        double stomp_total_time = 0, hauser_total_time = 0;
        int stomp_trials = 0, hauser_trials = 0;

        for (const auto &result : all_results)
        {
            if (result.algorithm == "STOMP")
            {
                stomp_trials++;
                if (result.success)
                {
                    stomp_successes++;
                    stomp_total_time += result.planning_time_ms;
                }
            }
            else if (result.algorithm == "Hauser")
            {
                hauser_trials++;
                if (result.success)
                {
                    hauser_successes++;
                    hauser_total_time += result.planning_time_ms;
                }
            }
        }

        std::cerr << "STOMP: " << stomp_successes << "/" << stomp_trials << " successes";
        if (stomp_successes > 0)
        {
            std::cerr << " (avg: " << std::fixed << std::setprecision(1) << (stomp_total_time / stomp_successes) << "ms)";
        }
        std::cerr << std::endl;

        std::cerr << "Hauser: " << hauser_successes << "/" << hauser_trials << " successes";
        if (hauser_successes > 0)
        {
            std::cerr << " (avg: " << std::fixed << std::setprecision(1) << (hauser_total_time / hauser_successes) << "ms)";
        }
        std::cerr << std::endl;

        return all_results;
    }
};

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    std::cerr << "=== ENHANCED STOMP vs HAUSER TRAJECTORY COMPARISON ===" << std::endl;
    std::cerr << "Build Mode: Release" << std::endl;
    std::cerr << "Logging: std::cerr (boost logging disabled)" << std::endl;

    int num_poses = 5;
    int trials_per_pose = 3;

    if (argc >= 2)
    {
        num_poses = std::atoi(argv[1]);
        if (num_poses <= 0 || num_poses > 10)
        {
            std::cerr << "Error: num_poses must be between 1 and 10. Using default: 5" << std::endl;
            num_poses = 5;
        }
    }

    if (argc >= 3)
    {
        trials_per_pose = std::atoi(argv[2]);
        if (trials_per_pose <= 0 || trials_per_pose > 10)
        {
            std::cerr << "Error: trials_per_pose must be between 1 and 10. Using default: 3" << std::endl;
            trials_per_pose = 3;
        }
    }

    std::cerr << "Configuration: " << num_poses << " poses, " << trials_per_pose << " trials per pose per algorithm" << std::endl;

    try
    {
        EnhancedTrajectoryComparison comparison;

        auto results = comparison.runFullComparison(num_poses, trials_per_pose);

        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream timestamp_stream;
        timestamp_stream << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        std::string timestamp = timestamp_stream.str();

        std::string results_file = "results/comparison_results_" + timestamp + ".csv";
        comparison.exportTrialResults(results, results_file);

        std::string trajectories_dir = "results/trajectories_" + timestamp;
        comparison.exportTrajectories(results, trajectories_dir);

        std::cerr << "\n=== EXPERIMENT COMPLETED ===" << std::endl;
        std::cerr << "Total trials conducted: " << results.size() << std::endl;

        int total_successes = 0;
        for (const auto &result : results)
        {
            if (result.success)
                total_successes++;
        }

        std::cerr << "Overall success rate: " << std::fixed << std::setprecision(1)
                  << (100.0 * total_successes / results.size()) << "%" << std::endl;
        std::cerr << "Results exported to: " << results_file << std::endl;
        std::cerr << "Trajectories exported to: " << trajectories_dir << "/" << std::endl;

        std::cerr << "\n📊 Run Python analysis:" << std::endl;
        std::cerr << "   python3 enhanced_analysis.py " << results_file << " " << trajectories_dir << std::endl;

        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "\n✗ Fatal Error: " << e.what() << std::endl;
        std::cerr << "\n=== COMPARISON FAILED ===" << std::endl;
        return 1;
    }
}
