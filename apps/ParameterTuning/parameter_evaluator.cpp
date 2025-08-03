#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <random>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <json/json.h>
#include <Eigen/Dense>
#include <boost/asio/thread_pool.hpp>

// Real TrajectoryLib headers - NO simulation!
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Robot/RobotArm.h"
#include "TrajectoryLib/Robot/RobotManager.h"
#include "TrajectoryLib/Planning/PathPlanner.h"
#include "GeometryLib/BVHTree.h"
#include "TrajectoryLib/Planning/PathPlanner.h"
#include "USLib/USTrajectoryPlanner.h"

/**
 * @brief Pose data structure for CSV loading
 */
struct PoseData
{
    Eigen::Vector3d position;
    Eigen::Quaterniond quaternion;
    bool contact;
    double distance;
    int validity;

    static PoseData fromCSVRow(const std::vector<std::string> &row)
    {
        PoseData pose;
        if (row.size() >= 10)
        {
            pose.position = Eigen::Vector3d(
                std::stod(row[0]), std::stod(row[1]), std::stod(row[2]));
            // CSV format: x, y, z, qw, qx, qy, qz, contact, distance, validity (same as PathPlanner mainwindow)
            // Quaternion constructor expects (w, x, y, z)
            pose.quaternion = Eigen::Quaterniond(
                std::stod(row[3]), std::stod(row[4]), std::stod(row[5]), std::stod(row[6]));
            pose.quaternion.normalize(); // Always normalize quaternions like PathPlanner does
            pose.contact = static_cast<bool>(std::stoi(row[7]));
            pose.distance = std::stod(row[8]);
            pose.validity = std::stoi(row[9]);
        }
        return pose;
    }

    bool isValid() const { return validity == 1; }
};

/**
 * @brief Parameter evaluation results
 */
struct EvaluationResult
{
    double successRate = 0.0;
    double avgPlanningTime = 0.0;
    double avgPathLength = 0.0;
    double avgSmoothness = 0.0;
    double avgClearance = 0.0; // New metric for average path clearance
    double compositeScore = 0.0;
    int totalTrajectories = 0;
    std::string algorithm = "";
};

/**
 * @brief STOMP Parameters from YAML
 */
struct StompParameters
{
    double temperature = 15.0;
    double learningRate = 0.3;
    int maxIterations = 500;                                                // Increased from 100 to 500 for better convergence
    double trajectoryDuration = 7.5;                                       // Trajectory duration in seconds (N calculated from this)
    int numNoisyTrajectories = 32;                                          // Increased from 16 to 32 for more exploration
    int numBestSamples = 16;                                                // Increased from 8 to 16 for better selection
    double obstacleCostWeight = 1.0;                                        // Weight for obstacle avoidance cost
    double constraintCostWeight = 1.0;                                      // Weight for constraint violation cost
    double controlCostWeight = 0.1;                                         // Weight for control/smoothness cost
    std::vector<double> jointStdDevs = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}; // Standard deviations for each joint

    static StompParameters fromYAML(const YAML::Node &node)
    {
        StompParameters params;
        if (node["stomp"])
        {
            const auto &stomp = node["stomp"];
            params.temperature = stomp["temperature"].as<double>(15.0);
            params.learningRate = stomp["learning_rate"].as<double>(0.3);
            params.maxIterations = stomp["max_iterations"].as<int>(500);               // Increased default
            params.trajectoryDuration = stomp["trajectory_duration"].as<double>(7.5);  // Trajectory duration in seconds
            params.numNoisyTrajectories = stomp["num_noisy_trajectories"].as<int>(32); // Increased default
            params.numBestSamples = stomp["num_best_samples"].as<int>(16);             // Increased default
            params.obstacleCostWeight = stomp["obstacle_cost_weight"].as<double>(1.0);
            params.constraintCostWeight = stomp["constraint_cost_weight"].as<double>(1.0);
            params.controlCostWeight = stomp["control_cost_weight"].as<double>(0.1);

            // Parse joint standard deviations
            if (stomp["joint_std_devs"])
            {
                params.jointStdDevs.clear();
                for (const auto &stdDev : stomp["joint_std_devs"])
                {
                    params.jointStdDevs.push_back(stdDev.as<double>());
                }
                // Ensure we have exactly 7 values for the 7-DOF robot
                if (params.jointStdDevs.size() != 7)
                {
                    std::cerr << "Warning: Expected 7 joint std devs, got " << params.jointStdDevs.size()
                              << ". Using default values." << std::endl;
                    params.jointStdDevs = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
                }
            }
        }
        return params;
    }
};

/**
 * @brief Load poses from CSV file
 */
std::vector<PoseData> loadPosesFromCSV(const std::string &filename)
{
    std::vector<PoseData> poses;
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Error: Cannot open CSV file: " << filename << std::endl;
        return poses;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::vector<std::string> row;
        std::string cell;

        while (std::getline(ss, cell, ','))
        {
            row.push_back(cell);
        }

        if (row.size() >= 10)
        {
            PoseData pose = PoseData::fromCSVRow(row);
            if (pose.isValid())
            {
                poses.push_back(pose);
            }
        }
    }

    std::cout << "Loaded " << poses.size() << " valid poses from " << filename << std::endl;
    return poses;
}

/**
 * @brief Generate random trajectory pairs for evaluation
 */
std::vector<std::pair<int, int>> generateRandomPairs(int numPoses, int numPairs)
{
    std::vector<std::pair<int, int>> pairs;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, numPoses - 1);

    for (int i = 0; i < numPairs; ++i)
    {
        int start = dis(gen);
        int goal = dis(gen);
        while (goal == start)
        {
            goal = dis(gen);
        }
        pairs.emplace_back(start, goal);
    }

    return pairs;
}

/**
 * @brief Clearance metrics for comprehensive trajectory analysis
 */
struct ClearanceMetrics {
    double minClearance;        // Minimum clearance across all links over entire path
    double avgClearance;        // Average clearance of all links over entire path  
    double avgMinClearance;     // Average of minimum clearance per configuration
};

/**
 * @brief Compute comprehensive clearance metrics over a trajectory path following PathPlanner methodology
 */
ClearanceMetrics computeAverageClearance(const std::vector<MotionGenerator::TrajectoryPoint> &path,
                                        RobotArm *arm,
                                        std::shared_ptr<BVHTree> obstacleTree,
                                        int interpolationPoints = 5)
{
    ClearanceMetrics metrics = {std::numeric_limits<double>::max(), 0.0, 0.0};
    
    if (path.size() < 2 || !obstacleTree)
    {
        metrics.minClearance = 0.0;
        return metrics;
    }

    std::vector<double> allClearanceValues;      // All clearance values for avgClearance
    std::vector<double> minClearancePerConfig;   // Min clearance per configuration for avgMinClearance
    double globalMinClearance = std::numeric_limits<double>::max();

    // For each segment in the path, interpolate and compute clearance
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        // Convert std::vector<double> to Eigen::VectorXd for interpolation
        Eigen::VectorXd startJoints = Eigen::Map<const Eigen::VectorXd>(path[i].position.data(), path[i].position.size());
        Eigen::VectorXd endJoints = Eigen::Map<const Eigen::VectorXd>(path[i + 1].position.data(), path[i + 1].position.size());

        // Interpolate between waypoints
        for (int j = 0; j <= interpolationPoints; ++j)
        {
            double t = static_cast<double>(j) / interpolationPoints;
            Eigen::VectorXd interpJoints = (1.0 - t) * startJoints + t * endJoints;

            // Set joint configuration and compute forward kinematics
            arm->setJointAngles(interpJoints);

            // Get all robot link bounding boxes following PathPlanner methodology
            auto bboxes = arm->getLinkBoundingBoxes();
            
            double minClearanceThisConfig = std::numeric_limits<double>::max();
            
            // Check clearance for all links starting from index 2 (skip base links)
            for (size_t linkIdx = 2; linkIdx < bboxes.size(); ++linkIdx)
            {
                auto bboxCenter = std::get<0>(bboxes[linkIdx]);
                
                // Query distance to obstacles using BVHTree
                auto [distance, closestPoint, closestPointOnObstacle] = obstacleTree->getDistanceAndClosestPoint(bboxCenter);
                
                // Store all clearance values for averaging
                allClearanceValues.push_back(distance);
                
                // Track minimum clearance for this configuration
                minClearanceThisConfig = std::min(minClearanceThisConfig, distance);
                
                // Track global minimum clearance
                globalMinClearance = std::min(globalMinClearance, distance);
            }
            
            // Store minimum clearance for this configuration
            if (minClearanceThisConfig != std::numeric_limits<double>::max())
            {
                minClearancePerConfig.push_back(minClearanceThisConfig);
            }
        }
    }

    // Compute final metrics
    metrics.minClearance = (globalMinClearance != std::numeric_limits<double>::max()) ? globalMinClearance : 0.0;
    
    if (!allClearanceValues.empty())
    {
        double totalClearance = 0.0;
        for (double clearance : allClearanceValues)
        {
            totalClearance += clearance;
        }
        metrics.avgClearance = totalClearance / allClearanceValues.size();
    }
    
    if (!minClearancePerConfig.empty())
    {
        double totalMinClearance = 0.0;
        for (double minClearance : minClearancePerConfig)
        {
            totalMinClearance += minClearance;
        }
        metrics.avgMinClearance = totalMinClearance / minClearancePerConfig.size();
    }

    return metrics;
}

/**
 * @brief Real STOMP trajectory evaluation using TrajectoryLib
 */
EvaluationResult evaluateSTOMPTrajectories(const StompParameters &params,
                                           const std::vector<PoseData> &poses,
                                           const std::string &urdfPath,
                                           const std::string &obstaclesPath,
                                           int numTrajectoryPairs = 20)
{
    EvaluationResult result;
    result.algorithm = "STOMP";
    result.totalTrajectories = numTrajectoryPairs;

    try
    {
        // Initialize robot arm and obstacles from config paths
        RobotArm *arm = new RobotArm(urdfPath);

        // Initialize path planner for pose validation
        PathPlanner *pathPlanner = new PathPlanner();
        pathPlanner->setStartPose(*arm);

        // Load obstacles and create BVHTree for collision checking
        std::cout << "Loading obstacles from: " << obstaclesPath << std::endl;
        RobotManager robotManager;
        robotManager.parseURDF(obstaclesPath);

        std::cout << "Loaded " << robotManager.getTransformedObstacles().size()
                  << " obstacles from " << obstaclesPath << std::endl;

        // Get the actual transformed obstacles from the RobotManager
        auto transformedObstacles = robotManager.getTransformedObstacles();
        std::shared_ptr<BVHTree> obstacleTree = std::make_shared<BVHTree>(transformedObstacles);

        // Set obstacle tree for path planner
        if (obstacleTree)
        {
            pathPlanner->setObstacleTree(obstacleTree);
        }

        // Initialize motion generator
        RobotArm temp = *arm;

        MotionGenerator *motionGenerator = new MotionGenerator(temp);

        // Set obstacle tree for motion generator too (like USTrajectoryPlanner does)
        if (obstacleTree)
        {
            motionGenerator->setObstacleTree(obstacleTree);
        }

        // Set up the current joint configuration - use a more reasonable starting pose for Franka robot
        // Instead of all zeros, use a configuration that's more likely to reach the workspace
        Eigen::VectorXd currentJoints(7);
        currentJoints << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785; // Common ready pose for Franka

        // Set current joints for PathPlanner (like USTrajectoryPlanner does in setCurrentJoints)
        RobotArm startArm = *arm;
        startArm.setJointAngles(currentJoints);
        pathPlanner->setStartPose(startArm);

        // Create STOMP configuration with parameters to optimize - use optimized config
        StompConfig stompConfig;
        stompConfig.temperature = params.temperature;
        stompConfig.learningRate = params.learningRate;
        stompConfig.maxIterations = params.maxIterations;
        stompConfig.trajectoryDuration = params.trajectoryDuration;
        stompConfig.numNoisyTrajectories = params.numNoisyTrajectories;
        stompConfig.numBestSamples = params.numBestSamples;
        stompConfig.obstacleCostWeight = params.obstacleCostWeight;
        stompConfig.constraintCostWeight = params.constraintCostWeight;
        stompConfig.controlCostWeight = params.controlCostWeight;

        // Set joint standard deviations from optimized parameters
        stompConfig.jointStdDevs = Eigen::VectorXd(7);
        for (int i = 0; i < 7; ++i)
        {
            stompConfig.jointStdDevs(i) = params.jointStdDevs[i];
        }

        // First pass: Trust CSV validity and create poses for STOMP evaluation
        std::vector<std::pair<int, RobotArm>> validPoses;

        int validCount = 0;
        int invalidCount = 0;

        std::cout << "Phase 1: Loading poses from CSV (with IK validation)..." << std::endl;

        for (int poseIdx = 0; poseIdx < poses.size(); poseIdx++)
        {
            const auto &pose = poses[poseIdx];

            // Trust the CSV validity field since user confirmed poses are correct
            if (pose.validity == 1)
            {
                // Convert pose to Eigen::Affine3d
                Eigen::Affine3d transform = Eigen::Affine3d::Identity();
                transform.translation() = pose.position;
                transform.linear() = pose.quaternion.toRotationMatrix();

                // Create a properly initialized RobotArm with URDF
                RobotArm robotArm(urdfPath);

                // Use PathPlanner to solve IK for this pose (like mainwindow does)
                auto [solvedArm, ikSolved] = pathPlanner->selectGoalPoseSimulatedAnnealing(transform);

                if (ikSolved)
                {
                    validPoses.push_back({poseIdx, solvedArm});
                    validCount++;

                    std::cout << "Pose " << (poseIdx + 1) << ": ✅ Loaded (IK solved)" << std::endl;
                }
                else
                {
                    invalidCount++;
                    std::cout << "Pose " << (poseIdx + 1) << ": ❌ Failed (IK failed)" << std::endl;
                }
            }
            else
            {
                invalidCount++;
            }
        }

        std::cout << "Phase 1 Complete: Found " << validCount << " valid poses out of " << poses.size() << " total poses" << std::endl;
        std::cout << "Valid poses: " << validCount << ", Invalid poses: " << invalidCount << std::endl;

        if (validPoses.size() < 2)
        {
            std::cout << "ERROR: Need at least 2 valid poses for trajectory evaluation, found " << validPoses.size() << std::endl;
            result.compositeScore = 100.0; // Massive penalty for insufficient poses to run STOMP
            return result;
        }

        // Second pass: Generate random trajectory pairs from valid poses only
        std::cout << "\nPhase 2: Generating trajectory pairs from valid poses..." << std::endl;

        std::vector<std::pair<std::pair<int, int>, std::pair<RobotArm, RobotArm>>> validPairs;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, validPoses.size() - 1);

        for (int pairIdx = 0; pairIdx < numTrajectoryPairs && pairIdx < (validPoses.size() * (validPoses.size() - 1) / 2); pairIdx++)
        {
            int startIdx = dis(gen);
            int goalIdx = dis(gen);
            while (goalIdx == startIdx)
            {
                goalIdx = dis(gen);
            }

            const auto &startValid = validPoses[startIdx];
            const auto &goalValid = validPoses[goalIdx];

            validPairs.push_back({{startValid.first, goalValid.first}, {startValid.second, goalValid.second}});
            std::cout << "  Generated pair " << (pairIdx + 1) << ": pose " << startValid.first
                      << " -> pose " << goalValid.first << std::endl;
        }

        std::cout << "Phase 2 Complete: Generated " << validPairs.size() << " trajectory pairs from " << validPoses.size() << " valid poses" << std::endl;

        // Phase 3: Run STOMP evaluation on all valid pairs
        std::cout << "\nPhase 3: Running STOMP evaluation on valid pairs..." << std::endl;
        std::cout << "Total pairs to evaluate: " << validPairs.size() << std::endl;

        int successCount = 0;
        double totalPlanningTime = 0.0;
        double totalPathLength = 0.0;
        double totalSmoothness = 0.0;
        double totalClearance = 0.0; // Track total clearance for averaging

        // Initialize shared SDF cache following USTrajectoryPlanner pattern
        std::vector<std::vector<std::vector<double>>> sharedSdf;
        Eigen::Vector3d sharedSdfMinPoint, sharedSdfMaxPoint;
        double sharedSdfResolution;
        bool sdfCacheInitialized = false;

        std::cout << "Initializing shared SDF cache for all evaluations..." << std::endl;

        // Create temporary MotionGenerator to compute SDF once (like USTrajectoryPlanner::initializeSharedSdf)
        MotionGenerator tempGenerator(*arm);
        tempGenerator.setObstacleTree(obstacleTree);
        tempGenerator.createSDF();

        if (tempGenerator.isSdfInitialized())
        {
            sharedSdf = tempGenerator.getSdf();
            sharedSdfMinPoint = tempGenerator.getSdfMinPoint();
            sharedSdfMaxPoint = tempGenerator.getSdfMaxPoint();
            sharedSdfResolution = tempGenerator.getSdfResolution();
            sdfCacheInitialized = true;

            std::cout << "✅ SDF cache initialized: "
                      << sharedSdf.size() << "x" << sharedSdf[0].size() << "x" << sharedSdf[0][0].size()
                      << " voxels at " << sharedSdfResolution << "m resolution" << std::endl;
        }
        else
        {
            std::cout << "⚠️  SDF cache initialization failed - will use standard obstacle tree" << std::endl;
        }

        for (size_t i = 0; i < validPairs.size(); ++i)
        {
            const auto &validPair = validPairs[i];
            const auto &poseIndices = validPair.first;
            const auto &armConfigs = validPair.second;

            std::cout << "Evaluating pair " << (i + 1) << "/" << validPairs.size()
                      << " [" << std::fixed << std::setprecision(1)
                      << (100.0 * (i + 1) / validPairs.size()) << "%] "
                      << "(pose " << poseIndices.first << " → pose " << poseIndices.second << ")..." << std::flush;

            // Create motion generator with shared SDF (like USTrajectoryPlanner::createMotionGeneratorWithSharedSdf)
            std::unique_ptr<MotionGenerator> motionGenerator;
            if (sdfCacheInitialized)
            {
                // Use shared SDF constructor for maximum performance
                motionGenerator = std::make_unique<MotionGenerator>(*arm, sharedSdf, sharedSdfMinPoint,
                                                                    sharedSdfMaxPoint, sharedSdfResolution, obstacleTree);
            }
            else
            {
                // Fallback to standard constructor with obstacle tree
                motionGenerator = std::make_unique<MotionGenerator>(*arm);
                if (obstacleTree)
                {
                    motionGenerator->setObstacleTree(obstacleTree);
                }
            }

            // Set start and goal configurations for pure STOMP evaluation (not using full trajectory planning pipeline)
            Eigen::MatrixXd waypoints(2, 7);
            waypoints.row(0) = armConfigs.first.getJointAngles();  // Pre-validated start configuration
            waypoints.row(1) = armConfigs.second.getJointAngles(); // Pre-validated goal configuration
            motionGenerator->setWaypoints(waypoints);              // Use motion generator with shared SDF

            // Create thread pool for STOMP execution (like USTrajectoryPlanner does)
            unsigned int numThreads = std::thread::hardware_concurrency();
            auto threadPool = std::make_shared<boost::asio::thread_pool>(numThreads);

            // Measure planning time
            auto startTime = std::chrono::high_resolution_clock::now();

            bool success = false;
            try
            {
                success = motionGenerator->performSTOMP(stompConfig, threadPool, static_cast<int>(i)); // Use shared SDF motion generator
            }
            catch (const std::exception &)
            {
                success = false;
            }
            threadPool->join(); // Wait for completion

            auto endTime = std::chrono::high_resolution_clock::now();
            double planningTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();

            if (success)
            {
                successCount++;
                totalPlanningTime += planningTime;

                // Get the trajectory path from the motion generator with shared SDF
                auto path = motionGenerator->getPath(); // Use shared SDF motion generator

                // Calculate path length (joint space distance)
                double pathLength = 0.0;
                if (path.size() > 1)
                {
                    for (size_t j = 1; j < path.size(); ++j)
                    {
                        // Convert std::vector<double> to Eigen::VectorXd
                        Eigen::VectorXd curr = Eigen::Map<Eigen::VectorXd>(path[j].position.data(), path[j].position.size());
                        Eigen::VectorXd prev = Eigen::Map<Eigen::VectorXd>(path[j - 1].position.data(), path[j - 1].position.size());
                        Eigen::VectorXd diff = curr - prev;
                        pathLength += diff.norm();
                    }
                }
                totalPathLength += pathLength;

                // Calculate smoothness (acceleration-based metric)
                double smoothness = 0.0;
                if (path.size() > 2)
                {
                    for (size_t j = 1; j < path.size() - 1; ++j)
                    {
                        // Convert std::vector<double> to Eigen::VectorXd
                        Eigen::VectorXd next = Eigen::Map<Eigen::VectorXd>(path[j + 1].position.data(), path[j + 1].position.size());
                        Eigen::VectorXd curr = Eigen::Map<Eigen::VectorXd>(path[j].position.data(), path[j].position.size());
                        Eigen::VectorXd prev = Eigen::Map<Eigen::VectorXd>(path[j - 1].position.data(), path[j - 1].position.size());
                        Eigen::VectorXd accel = next - 2 * curr + prev;
                        smoothness += accel.squaredNorm();
                    }
                    smoothness = 1.0 / (1.0 + smoothness); // Higher is smoother
                }
                totalSmoothness += smoothness;

                // Calculate comprehensive clearance metrics over the path
                ClearanceMetrics clearanceMetrics = computeAverageClearance(path, arm, obstacleTree, 3); // 3 interpolation points per segment
                totalClearance += clearanceMetrics.avgClearance; // Use average clearance for aggregate scoring

                std::cout << " ✅ SUCCESS (" << std::fixed << std::setprecision(0) << planningTime
                          << "ms, " << path.size() << " waypoints, length=" << std::setprecision(2)
                          << pathLength << ", clearance=" << std::setprecision(3) << clearanceMetrics.avgClearance 
                          << "m, min=" << std::setprecision(3) << clearanceMetrics.minClearance
                          << "m, avgMin=" << std::setprecision(3) << clearanceMetrics.avgMinClearance << "m)" << std::endl;
            }
            else
            {
                std::cout << " ❌ FAILED (" << std::fixed << std::setprecision(0) << planningTime << "ms)" << std::endl;
            }

            // Progress summary every 5 pairs
            if ((i + 1) % 5 == 0)
            {
                double currentSuccessRate = (double)successCount / (i + 1) * 100.0;
                std::cout << "  === STOMP Progress: " << (i + 1) << "/" << validPairs.size()
                          << " | Success: " << successCount << "/" << (i + 1)
                          << " (" << std::fixed << std::setprecision(1) << currentSuccessRate << "%) ===" << std::endl;
            }
        } // End of for loop over valid trajectory pairs

        // Print evaluation summary
        std::cout << std::endl
                  << "=== EVALUATION SUMMARY ===" << std::endl;
        std::cout << "Total poses loaded: " << poses.size() << std::endl;
        std::cout << "Valid poses found: " << validPoses.size() << std::endl;
        std::cout << "Trajectory pairs requested: " << numTrajectoryPairs << std::endl;
        std::cout << "Trajectory pairs evaluated: " << validPairs.size() << std::endl;
        std::cout << "Successful STOMP trajectories: " << successCount << std::endl;

        // Clean up memory
        delete arm;
        delete pathPlanner;
        delete motionGenerator;

        // Calculate results based only on valid pose pairs that were actually evaluated
        if (validPairs.size() > 0)
        {
            result.successRate = static_cast<double>(successCount) / validPairs.size(); // Success rate of attempted evaluations
            result.totalTrajectories = validPairs.size();                               // Update to reflect actual evaluated pairs

            if (successCount > 0)
            {
                result.avgPlanningTime = totalPlanningTime / successCount;
                result.avgPathLength = totalPathLength / successCount;
                result.avgSmoothness = totalSmoothness / successCount;
                result.avgClearance = totalClearance / successCount;
            }

            // Composite objective (lower is better) - only meaningful if we had valid poses to evaluate
            double timeScore = (successCount > 0) ? result.avgPlanningTime / 1000.0 : 5.0;                        // Normalize to ~0.1-1.0, high penalty if no success
            double successScore = (1.0 - result.successRate) * 50.0;                                              // EXTREMELY HEAVY penalty for STOMP trajectory failures (50x multiplier)
            double clearanceScore = (successCount > 0) ? std::max(0.0, (0.1 - result.avgClearance) * 10.0) : 2.0; // Penalty for low clearance (target: >0.1m)

            // Heavily weighted composite score to prioritize STOMP trajectory success above all else
            // Focus on: STOMP success rate (80%), planning time (15%), clearance (5%)
            result.compositeScore = 0.15 * timeScore + 0.8 * successScore + 0.05 * clearanceScore;
        }
        else
        {
            // No valid pose pairs found - extremely high penalty for complete STOMP failure
            result.compositeScore = 100.0; // Massive penalty for no valid STOMP trajectories
            result.successRate = 0.0;
            result.totalTrajectories = 0;
            std::cout << "WARNING: No valid pose pairs found for evaluation!" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in STOMP evaluation: " << e.what() << std::endl;
        std::cerr << "DEBUG: Full exception details: " << typeid(e).name() << std::endl;
        result.compositeScore = 200.0; // EXTREME penalty for STOMP exceptions/crashes
    }

    return result;
}

/**
 * @brief Save results to JSON
 */
void saveResultsToJSON(const EvaluationResult &result, const std::string &filename)
{
    Json::Value jsonResult;
    jsonResult["algorithm"] = result.algorithm;
    jsonResult["success_rate"] = result.successRate;
    jsonResult["avg_planning_time_ms"] = result.avgPlanningTime;
    jsonResult["avg_path_length"] = result.avgPathLength;
    jsonResult["avg_smoothness"] = result.avgSmoothness;
    jsonResult["avg_clearance"] = result.avgClearance; // Include clearance in JSON output
    jsonResult["composite_score"] = result.compositeScore;
    jsonResult["total_trajectories"] = result.totalTrajectories;

    // Multi-objective optimization metrics with HEAVY STOMP failure penalties
    jsonResult["avg_constraint_violations"] = (1.0 - result.avgSmoothness) * 5.0;          // Increased constraint violation penalty
    jsonResult["collision_risk_score"] = std::max(0.5, (1.0 - result.successRate) * 50.0); // EXTREME penalty for STOMP trajectory failures

    jsonResult["timestamp"] = std::chrono::duration_cast<std::chrono::seconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count();

    std::ofstream file(filename);
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(jsonResult, &file);

    std::cout << "Results saved to: " << filename << std::endl;
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <config.yaml>" << std::endl;
        return 1;
    }

    std::string configFile = argv[1];

    try
    {
        // Load configuration
        YAML::Node config = YAML::LoadFile(configFile);

        // Extract file paths from config
        std::string csvFile = config["csv_file"].as<std::string>();
        std::string urdfFile = config["urdf_file"].as<std::string>();
        std::string obstaclesFile = config["obstacles_file"].as<std::string>();

        // Load poses
        auto poses = loadPosesFromCSV(csvFile);
        if (poses.empty())
        {
            std::cerr << "Error: No valid poses loaded from CSV" << std::endl;
            return 1;
        }

        // Extract STOMP parameters
        StompParameters stompParams = StompParameters::fromYAML(config);

        std::cout << "\\n=== PARAMETER EVALUATION ===" << std::endl;
        std::cout << "Parameters:" << std::endl;
        std::cout << "  Temperature: " << stompParams.temperature << std::endl;
        std::cout << "  Learning Rate: " << stompParams.learningRate << std::endl;
        std::cout << "  Max Iterations: " << stompParams.maxIterations << std::endl;
        std::cout << "  Trajectory Duration: " << stompParams.trajectoryDuration << "s (N computed dynamically)" << std::endl;
        std::cout << "  Obstacle Cost Weight: " << stompParams.obstacleCostWeight << std::endl;
        std::cout << "  Constraint Cost Weight: " << stompParams.constraintCostWeight << std::endl;
        std::cout << "  Control Cost Weight: " << stompParams.controlCostWeight << std::endl;
        std::cout << "  Joint Std Devs: [";
        for (size_t i = 0; i < stompParams.jointStdDevs.size(); ++i)
        {
            std::cout << stompParams.jointStdDevs[i];
            if (i < stompParams.jointStdDevs.size() - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        // Evaluate trajectories
        int numTrajectoryPairs = config["evaluation"]["trajectory_pairs"].as<int>(20);
        std::cout << "DEBUG: About to call evaluateSTOMPTrajectories with " << numTrajectoryPairs << " pairs" << std::endl;
        auto result = evaluateSTOMPTrajectories(stompParams, poses, urdfFile, obstaclesFile, numTrajectoryPairs);
        std::cout << "DEBUG: evaluateSTOMPTrajectories returned" << std::endl;

        std::cout << "\\nResults:" << std::endl;
        std::cout << "  Success Rate: " << std::fixed << std::setprecision(6) << (result.successRate * 100.0) << "%" << std::endl;
        std::cout << "  Avg Planning Time: " << std::fixed << std::setprecision(3) << result.avgPlanningTime << "ms" << std::endl;
        std::cout << "  Avg Path Length: " << std::fixed << std::setprecision(6) << result.avgPathLength << std::endl;
        std::cout << "  Avg Smoothness: " << std::fixed << std::setprecision(6) << result.avgSmoothness << std::endl;
        std::cout << "  Avg Clearance: " << std::fixed << std::setprecision(6) << result.avgClearance << "m" << std::endl;
        std::cout << "  Composite Score: " << result.compositeScore << std::endl;

        // Save results
        std::string outputFile = config["output_file"].as<std::string>("results.json");
        saveResultsToJSON(result, outputFile);

        // Also output score to stdout for Python integration
        std::cout << "OBJECTIVE_VALUE: " << result.compositeScore << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
