#include "USTrajectoryPlanner.h"
#include <QDebug>
#include "TrajectoryLib/Logger.h"
#include "TrajectoryLib/Planning/PathPlanner.h"
#include <algorithm>
#include <atomic>
#include <boost/asio/post.hpp>
#include <boost/asio/use_future.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>

UltrasoundScanTrajectoryPlanner::UltrasoundScanTrajectoryPlanner(const std::string &environmentString)
    : _hardwareConfig(HardwareConfig::detect())
    , _environment(environmentString)
{
    _arm = new RobotArm(environmentString);
    _pathPlanner = new PathPlanner();
    _pathPlanner->setStartPose(*_arm);
    _motionGenerator = new MotionGenerator(*_arm);

    initializeThreadPool();

    LOG_INFO << "Hardware: " << _hardwareConfig.physicalCores
             << " cores, batch size: " << _hardwareConfig.batchSize;
}

UltrasoundScanTrajectoryPlanner::~UltrasoundScanTrajectoryPlanner()
{
    if (_sharedThreadPool) {
        _sharedThreadPool->join();
        _sharedThreadPool.reset();
    }

    if (_arm) {
        delete _arm;
        _arm = nullptr;
    }

    if (_motionGenerator) {
        delete _motionGenerator;
        _motionGenerator = nullptr;
    }
}

void UltrasoundScanTrajectoryPlanner::setCurrentJoints(const Eigen::VectorXd &joints)
{
    _currentJoints = joints;

    RobotArm startArm = *_arm;
    startArm.setJointAngles(joints);
    _pathPlanner->setStartPose(startArm);
}

void UltrasoundScanTrajectoryPlanner::setEnvironment(const std::string &environment)
{
    if (!environment.empty()) {
        _environment = environment;

        _robotManager.parseURDF(_environment);

        _obstacleTree = std::make_shared<BVHTree>(_robotManager.getTransformedObstacles());

        _pathPlanner->setObstacleTree(_obstacleTree);
        _motionGenerator->setObstacleTree(_obstacleTree);
    }
}

void UltrasoundScanTrajectoryPlanner::setPoses(const std::vector<Eigen::Affine3d> &poses)
{
    _poses.clear();
    _poses.reserve(poses.size());

    for (const auto &pose : poses) {
        _poses.push_back(pose);
    }
}

bool UltrasoundScanTrajectoryPlanner::planTrajectories(bool useHauserForRepositioning,
                                                       bool enableShortcutting)
{
    if (_environment.empty()) {
        throw std::runtime_error("Environment string is not populated.");
    }

    if (_currentJoints.size() != 7) {
        throw std::runtime_error("Current joints are not populated or are not = 7.");
    }

    if (_poses.empty()) {
        throw std::runtime_error("Target poses are not populated.");
    }

    _trajectories.clear();

    // Special case: single pose request - only return free movement trajectory
    if (_poses.size() == 1) {
        LOG_INFO << "Single pose request: planning direct movement trajectory";

        auto checkpointResult = _pathPlanner->planCheckpoints(_poses, _currentJoints);
        auto &checkpoints = checkpointResult.checkpoints;
        size_t firstValidIndex = checkpointResult.firstValidIndex;

        if (firstValidIndex >= checkpoints.size()) {
            throw std::runtime_error("No valid checkpoint found for single pose request.");
        }

        // Extract target arm configuration
        RobotArm targetArm = checkpoints[firstValidIndex].first;

        // Plan single trajectory from current position to target pose
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> singleRequest;
        std::vector<std::string> singleDescription;

        singleRequest.emplace_back(_currentJoints, targetArm.getJointAngles());
        singleDescription.emplace_back("Single pose free movement: current -> pose 0");

        std::vector<Trajectory> result;
        if (useHauserForRepositioning) {
            result = planTrajectoryBatchHauser(singleRequest, singleDescription, enableShortcutting);
        } else {
            result = planTrajectoryBatch(singleRequest, singleDescription, enableShortcutting);
        }

        if (!result.empty() && !result[0].empty()) {
            std::vector<MotionGenerator::TrajectoryPoint> trajectoryPoints;
            trajectoryPoints.reserve(result[0].size());

            for (const auto &point : result[0]) {
                trajectoryPoints.push_back(point);
            }

            _trajectories.emplace_back(trajectoryPoints, false); // Free movement = NOT contact force
            LOG_INFO << "Single pose trajectory: " << trajectoryPoints.size() << " points";
            return true;
        } else {
            LOG_ERROR << "Failed to plan single pose trajectory";
            return false;
        }
    }

    auto checkpointResult = _pathPlanner->planCheckpoints(_poses, _currentJoints);
    auto &checkpoints = checkpointResult.checkpoints;
    auto &validSegments = checkpointResult.validSegments;
    size_t firstValidIndex = checkpointResult.firstValidIndex;

    // Log the indices of the valid segments
    LOG_INFO << "Valid segments: " << validSegments.size();
    for (const auto &segment : validSegments) {
        LOG_DEBUG << "Segment " << segment.first << "-" << segment.second;
    }

    // Validate that we have a valid starting point
    if (firstValidIndex >= checkpoints.size()) {
        throw std::runtime_error("No valid checkpoint found to start trajectory planning.");
    }
    if (firstValidIndex > 0) {
        LOG_INFO << "Using first valid checkpoint: " << firstValidIndex;
    }

    // Extract arm configurations
    std::vector<RobotArm> arms;
    for (const auto &[arm, valid] : checkpoints) {
        arms.push_back(arm);
    }

    // Build repositioning trajectory requests for batch planning
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> repositioningRequests;
    std::vector<std::string> repositioningDescriptions; // Track what each repositioning is for

    // Add initial repositioning trajectory (current joints -> first valid checkpoint)
    repositioningRequests.emplace_back(_currentJoints, arms[firstValidIndex].getJointAngles());
    repositioningDescriptions.emplace_back("Initial repositioning: current -> pose "
                                           + std::to_string(firstValidIndex));

    // Add inter-segment repositioning trajectories
    for (size_t segIdx = 0; segIdx + 1 < validSegments.size(); segIdx++) {
        size_t currentEnd = validSegments[segIdx].second;
        size_t nextStart = validSegments[segIdx + 1].first;

        repositioningRequests.emplace_back(arms[currentEnd].getJointAngles(),
                                           arms[nextStart].getJointAngles());
        repositioningDescriptions.emplace_back("Inter-segment repositioning: pose "
                                               + std::to_string(currentEnd) + " -> pose "
                                               + std::to_string(nextStart));
    }

    LOG_INFO << "Planning " << repositioningRequests.size() << " repositioning trajectories using "
             << (useHauserForRepositioning ? "RRT+Hauser" : "STOMP");

    // Use appropriate planning method for repositioning trajectories
    std::vector<Trajectory> repositioningResults;
    if (useHauserForRepositioning) {
        repositioningResults = planTrajectoryBatchHauser(repositioningRequests,
                                                         repositioningDescriptions,
                                                         enableShortcutting);
    } else {
        repositioningResults = planTrajectoryBatch(repositioningRequests,
                                                   repositioningDescriptions,
                                                   enableShortcutting);
    }

    // Now build the final trajectory sequence in the correct order
    size_t repositioningIndex = 0;

    // 1. Add initial repositioning
    if (repositioningIndex < repositioningResults.size()
        && !repositioningResults[repositioningIndex].empty()) {
        std::vector<MotionGenerator::TrajectoryPoint> trajectoryPoints;
        trajectoryPoints.reserve(repositioningResults[repositioningIndex].size());

        for (const auto &point : repositioningResults[repositioningIndex]) {
            trajectoryPoints.push_back(point);
        }

        _trajectories.emplace_back(trajectoryPoints, false); // repositioning = NOT contact force
        LOG_DEBUG << "Initial repositioning: " << trajectoryPoints.size() << " points";
    } else {
        LOG_ERROR << "Initial repositioning failed";
        return false;
    }
    repositioningIndex++;

    // 2. Add contact force trajectories for each valid segment, with inter-segment repositioning in between
    for (size_t segIdx = 0; segIdx < validSegments.size(); segIdx++) {
        size_t start = validSegments[segIdx].first;
        size_t end = validSegments[segIdx].second;

        // Add contact force trajectory for this segment
        if (start == end) {
            // Single point - add a minimal trajectory for contact force
            std::vector<MotionGenerator::TrajectoryPoint> singlePoint;
            MotionGenerator::TrajectoryPoint point;
            point.time = 0.0;

            // Convert Eigen::VectorXd to std::vector<double>
            auto jointAngles = arms[start].getJointAngles();
            point.position.resize(jointAngles.size());
            point.velocity.resize(jointAngles.size());
            point.acceleration.resize(jointAngles.size());

            for (int i = 0; i < jointAngles.size(); ++i) {
                point.position[i] = jointAngles[i];
                point.velocity[i] = 0.0;
                point.acceleration[i] = 0.0;
            }

            singlePoint.push_back(point);

            _trajectories.emplace_back(singlePoint, true); // contact force = TRUE
            LOG_DEBUG << "Contact force point at checkpoint " << start;
        } else {
            // Multi-point segment - create contact force trajectory through all waypoints
            std::vector<Eigen::VectorXd> segmentWaypoints;
            for (size_t i = start; i <= end; i++) {
                segmentWaypoints.push_back(arms[i].getJointAngles());
            }

            // Generate time-optimal trajectory through all waypoints
            MotionGenerator tempGenerator(*_arm);
            if (_obstacleTree) {
                tempGenerator.setObstacleTree(_obstacleTree);
            }

            auto trajectoryPoints = tempGenerator.generateTrajectoryFromCheckpoints(
                segmentWaypoints);
            
            _trajectories.emplace_back(trajectoryPoints, true); // contact force = TRUE
            LOG_DEBUG << "Contact force trajectory: " << start << "-" << end
                     << " (" << trajectoryPoints.size() << " points)";
        }

        // Add inter-segment repositioning if not the last segment
        if (segIdx + 1 < validSegments.size()) {
            if (repositioningIndex < repositioningResults.size()
                && !repositioningResults[repositioningIndex].empty()) {
                std::vector<MotionGenerator::TrajectoryPoint> trajectoryPoints;
                trajectoryPoints.reserve(repositioningResults[repositioningIndex].size());

                // Copy repositioning trajectory points
                for (const auto &point : repositioningResults[repositioningIndex]) {
                    trajectoryPoints.push_back(point);
                }

                _trajectories.emplace_back(trajectoryPoints,
                                           false); // repositioning = NOT contact force
                size_t currentEnd = validSegments[segIdx].second;
                size_t nextStart = validSegments[segIdx + 1].first;
                LOG_DEBUG << "Inter-segment repositioning " << currentEnd << "-" << nextStart
                         << ": " << trajectoryPoints.size() << " points";
            } else {
                LOG_ERROR << "Inter-segment repositioning failed for segment " << segIdx;
                return false;
            }
            repositioningIndex++;
        }
    }

    LOG_INFO << "Generated " << _trajectories.size() << " trajectories";
    
    // Check and fix discontinuities between trajectory segments
    bool discontinuitiesFixed = fixTrajectoryDiscontinuities();
    if (discontinuitiesFixed) {
        LOG_INFO << "Trajectory discontinuities corrected";
    }
    
    printSegmentTimes();
    
    return true;
}

std::vector<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>>
UltrasoundScanTrajectoryPlanner::getTrajectories()
{
    return _trajectories;
}

std::vector<Eigen::Affine3d> UltrasoundScanTrajectoryPlanner::getScanPoses() const
{
    return _poses;
}

void UltrasoundScanTrajectoryPlanner::initializeThreadPool()
{
    if (!_sharedThreadPool) {
        const auto &config = _hardwareConfig;
        size_t optimalThreads = config.optimalThreadsForFlatParallelization;

        _sharedThreadPool = std::make_shared<boost::asio::thread_pool>(optimalThreads);

        // Pre-warm the thread pool
        for (size_t i = 0; i < optimalThreads; ++i) {
            boost::asio::post(*_sharedThreadPool, []() {
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            });
        }
        
        LOG_INFO << "Thread pool initialized with " << optimalThreads 
                 << " threads for flat parallelization";
    }
}

std::vector<Trajectory> UltrasoundScanTrajectoryPlanner::planTrajectoryBatch(
    const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> &requests,
    const std::vector<std::string> &descriptions,
    bool enableShortcutting)
{
    if (requests.empty()) {
        LOG_WARNING << "Empty trajectory batch requested";
        return {};
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    LOG_INFO << "Batch planning: " << requests.size() << " trajectories";

    initializeThreadPool();
    
    // Initialize shared SDF before parallel processing to avoid race conditions
    initializeSharedSdf();
    
    // Determine optimal parallelization strategy based on workload
    bool useFlatParallelization = shouldUseFlatParallelization(requests.size());

    std::vector<Trajectory> results(requests.size());
    std::vector<std::future<void>> futures;
    futures.reserve(requests.size());

    const size_t batchSize = _hardwareConfig.batchSize;
    std::atomic<size_t> completedTrajectories{0};
    std::atomic<size_t> successfulTrajectories{0};

    for (size_t i = 0; i < requests.size(); i += batchSize) {
        size_t endIdx = std::min(i + batchSize, requests.size());

        // Process batch
        for (size_t j = i; j < endIdx; ++j) {
            auto future = boost::asio::post(
                *_sharedThreadPool,
                boost::asio::use_future([this,
                                         j,
                                         &requests,
                                         &descriptions,
                                         &results,
                                         &completedTrajectories,
                                         &successfulTrajectories,
                                         enableShortcutting,
                                         useFlatParallelization]() {
                    try {
                        const auto &startJoints = requests[j].first;
                        const auto &targetJoints = requests[j].second;
                        
                        auto localGenerator = createMotionGeneratorWithSharedSdf(*_arm);
                        StompConfig config;
                        // Set parallelization strategy based on workload analysis
                        config.disableInternalParallelization = useFlatParallelization;

                        Eigen::MatrixXd waypoints(2, startJoints.size());
                        waypoints.row(0) = startJoints.transpose();
                        waypoints.row(1) = targetJoints.transpose();

                        localGenerator->setWaypoints(waypoints);

                        bool success = false;
                        try {
                            success = localGenerator->performSTOMP(config,
                                                                   _sharedThreadPool,
                                                                   static_cast<int>(j));
                        } catch (const StompFailedException &) {
                            success = false;
                        }

                        if (success) {
                            LOG_DEBUG << "STOMP success: " << descriptions[j];
                            results[j] = localGenerator->getPath();
                            successfulTrajectories++;
                        } else {
                            LOG_WARNING << "STOMP failed, trying two-step fallback: " << descriptions[j];

                            // Fallback: Two separate STOMP trajectories
                            // Step 1: startJoints -> _currentJoints (stopping at rest)
                            auto step1Generator = createMotionGeneratorWithSharedSdf(*_arm);

                            Eigen::MatrixXd step1Waypoints(2, startJoints.size());
                            step1Waypoints.row(0) = startJoints.transpose();
                            step1Waypoints.row(1) = _currentJoints.transpose();
                            step1Generator->setWaypoints(step1Waypoints);

                            StompConfig fallBackConfig;
                            // Use same parallelization strategy for fallback
                            fallBackConfig.disableInternalParallelization = useFlatParallelization;

                            bool step1Success = false;
                            try {
                                step1Success = step1Generator->performSTOMP(fallBackConfig,
                                                                            _sharedThreadPool);
                            } catch (const StompFailedException &) {
                                step1Success = false;
                            }

                            // Step 2: _currentJoints -> targetJoints (starting from rest)
                            auto step2Generator = createMotionGeneratorWithSharedSdf(*_arm);

                            Eigen::MatrixXd step2Waypoints(2, _currentJoints.size());
                            step2Waypoints.row(0) = _currentJoints.transpose();
                            step2Waypoints.row(1) = targetJoints.transpose();
                            step2Generator->setWaypoints(step2Waypoints);

                            bool step2Success = false;
                            try {
                                step2Success = step2Generator->performSTOMP(fallBackConfig, _sharedThreadPool);
                            } catch (const StompFailedException &) {
                                step2Success = false;
                            }

                            if (step1Success && step2Success) {

                                Trajectory step1Traj = step1Generator->getPath();
                                Trajectory step2Traj = step2Generator->getPath();

                                Trajectory combinedTrajectory;
                                combinedTrajectory.reserve(step1Traj.size() + step2Traj.size() - 1);
                                
                                for (const auto &point : step1Traj) {
                                    combinedTrajectory.push_back(point);
                                }
                                for (size_t i = 1; i < step2Traj.size(); ++i) {
                                    combinedTrajectory.push_back(step2Traj[i]);
                                }

                                results[j] = combinedTrajectory;
                                successfulTrajectories++;
                                LOG_DEBUG << "Two-step STOMP success: " << descriptions[j]
                                         << " (" << step1Traj.size() << "+" << step2Traj.size()
                                         << "=" << combinedTrajectory.size() << " points)";
                            } else {
                                LOG_ERROR << "Two-step STOMP failed: " << descriptions[j]
                                          << " (Step 1: " << (step1Success ? "OK" : "FAIL")
                                          << ", Step 2: " << (step2Success ? "OK" : "FAIL") << ")";
                            }
                        }
                        completedTrajectories++;

                    } catch (const std::exception &e) {
                        LOG_ERROR << "Batch planning exception (" << descriptions[j]
                                  << "): " << e.what();
                        completedTrajectories++;
                    }
                }));

            futures.push_back(std::move(future));
        }

        if (endIdx < requests.size()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    for (auto &future : futures) {
        try {
            future.get();
        } catch (const std::exception &e) {
            LOG_ERROR << "Future completion error: " << e.what();
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    // Performance metrics logging
    double trajectoryThroughput = (double)successfulTrajectories / (duration.count() / 1000.0);
    double avgTimePerTrajectory = (double)duration.count() / requests.size();
    
    LOG_INFO << "Batch planning complete (" << (useFlatParallelization ? "FLAT" : "HIERARCHICAL") << "): " 
             << successfulTrajectories << "/" << requests.size()
             << " (" << std::fixed << std::setprecision(1) 
             << (100.0 * successfulTrajectories / requests.size()) << "%) in "
             << duration.count() << "ms";
    LOG_INFO << "Performance: " << std::fixed << std::setprecision(2) 
             << trajectoryThroughput << " trajectories/sec, " 
             << avgTimePerTrajectory << "ms avg/trajectory";

    return results;
}

std::vector<Trajectory> UltrasoundScanTrajectoryPlanner::planTrajectoryBatchHauser(
    const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> &requests,
    const std::vector<std::string> &descriptions,
    bool enableShortcutting)
{
    if (requests.empty()) {
        LOG_WARNING << "Empty Hauser batch requested";
        return {};
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    LOG_INFO << "Hauser batch planning: " << requests.size() << " trajectories";

    initializeThreadPool();
    
    // Determine optimal parallelization strategy (Hauser doesn't use internal parallelization currently)
    bool useFlatParallelization = shouldUseFlatParallelization(requests.size());

    std::vector<Trajectory> results(requests.size());
    std::vector<std::future<void>> futures;
    futures.reserve(requests.size());

    // Batch size for memory efficiency and cache optimization
    const size_t batchSize = _hardwareConfig.batchSize;
    std::atomic<size_t> completedTrajectories{0};
    std::atomic<size_t> successfulTrajectories{0};

    for (size_t i = 0; i < requests.size(); i += batchSize) {
        size_t endIdx = std::min(i + batchSize, requests.size());

        // Process batch
        for (size_t j = i; j < endIdx; ++j) {
            auto future = boost::asio::post(
                *_sharedThreadPool,
                boost::asio::use_future([this,
                                         j,
                                         &requests,
                                         &descriptions,
                                         &results,
                                         &completedTrajectories,
                                         &successfulTrajectories,
                                         enableShortcutting,
                                         useFlatParallelization]() {
                    try {
                        // Each thread gets its own motion generator to avoid contention
                        MotionGenerator localGenerator(*_arm);

                        // Set obstacle tree if available
                        if (_obstacleTree) {
                            localGenerator.setObstacleTree(_obstacleTree);
                        }

                        // Set start and target joints for the trajectory
                        const auto &startJoints = requests[j].first;
                        const auto &targetJoints = requests[j].second;

                        // Use RRT Connect to generate waypoints for Hauser optimization
                        auto hauserPathPlanner = std::make_unique<PathPlanner>();

                        // Configure PathPlanner to use RRT Connect for Hauser waypoints
                        Params rrtConnectParams;
                        rrtConnectParams.algo = RRTConnect;
                        rrtConnectParams.stepSize = 0.2;
                        rrtConnectParams.goalBiasProbability = 0.2;
                        rrtConnectParams.maxIterations = 10000;

                        hauserPathPlanner->setParams(rrtConnectParams);

                        // Set start and goal configurations
                        RobotArm startArm(*_arm);
                        startArm.setJointAngles(startJoints);
                        hauserPathPlanner->setStartPose(startArm);
                        hauserPathPlanner->setObstacleTree(_obstacleTree);

                        RobotArm goalArm(*_arm);
                        goalArm.setJointAngles(targetJoints);
                        hauserPathPlanner->setGoalConfiguration(goalArm);

                        bool rrtPathFound = hauserPathPlanner->runPathFinding();

                        // Retry RRT Connect once if it failed
                        if (!rrtPathFound) {
                            LOG_DEBUG << "RRT Connect retry for Hauser...";
                            rrtPathFound = hauserPathPlanner->runPathFinding();
                        }

                        bool success = false;

                        if (rrtPathFound) {
                            // Apply shortcutting to optimize RRT Connect waypoints before Hauser
                            if (enableShortcutting) {
                                hauserPathPlanner->shortcutPath(100, 20);
                            }

                            // Get optimized RRT Connect path and convert to waypoints for Hauser
                            Eigen::MatrixXd rrtWaypoints = hauserPathPlanner->getAnglesPath();
                            localGenerator.setWaypoints(rrtWaypoints);

                            // Use Hauser optimization with RRT Connect waypoints
                            try {
                                localGenerator.performHauser(100); // maxIterations = 1000
                                success = true;
                            } catch (const std::exception &e) {
                                LOG_WARNING << "Hauser optimization failed: " << descriptions[j];
                                success = false;
                            }
                        } else {
                            LOG_WARNING << "RRT Connect failed for Hauser: " << descriptions[j];
                        }

                        if (success) {
                            LOG_DEBUG << "Hauser success: " << descriptions[j];
                            results[j] = localGenerator.getPath();
                            successfulTrajectories++;
                        } else {
                            LOG_DEBUG << "Hauser failed: " << descriptions[j];
                        }
                        completedTrajectories++;

                    } catch (const std::exception &e) {
                        // Log error but continue with other trajectories
                        LOG_ERROR << "Hauser batch planning exception (" << descriptions[j]
                                  << "): " << e.what();
                        completedTrajectories++;
                        // results[j] remains default-constructed (empty)
                    }
                }));

            futures.push_back(std::move(future));
        }

        // Add small delay between batches to prevent CPU saturation
        if (endIdx < requests.size()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    // Wait for all trajectories to complete
    for (auto &future : futures) {
        try {
            future.get();
        } catch (const std::exception &e) {
            LOG_ERROR << "Future completion error: " << e.what();
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    // Performance metrics logging for Hauser
    double trajectoryThroughput = (double)successfulTrajectories / (duration.count() / 1000.0);
    double avgTimePerTrajectory = (double)duration.count() / requests.size();
    
    LOG_INFO << "Hauser batch complete (" << (useFlatParallelization ? "FLAT" : "HIERARCHICAL") << "): " 
             << successfulTrajectories << "/" << requests.size()
             << " (" << std::fixed << std::setprecision(1) 
             << (100.0 * successfulTrajectories / requests.size()) << "%) in "
             << duration.count() << "ms";
    LOG_INFO << "Performance: " << std::fixed << std::setprecision(2) 
             << trajectoryThroughput << " trajectories/sec, " 
             << avgTimePerTrajectory << "ms avg/trajectory";

    return results;
}

void UltrasoundScanTrajectoryPlanner::validateTrajectoryDiscontinuities() const
{
    if (_trajectories.empty()) {
        LOG_WARNING << "No trajectories to validate";
        return;
    }

    LOG_DEBUG << "Validating trajectory continuity";
    
    double maxPositionDiscontinuity = 0.0;
    int discontinuityCount = 0;
    
    const double POSITION_THRESHOLD = 0.01; // 0.01 radians (~0.57 degrees)
    
    for (size_t segIdx = 0; segIdx < _trajectories.size() - 1; ++segIdx) {
        const auto& currentSegment = _trajectories[segIdx].first;
        const auto& nextSegment = _trajectories[segIdx + 1].first;
        
        if (currentSegment.empty() || nextSegment.empty()) {
            LOG_WARNING << "Trajectory segment " << segIdx << " or " << (segIdx + 1) << " is empty";
            continue;
        }
        
        const auto& currentEnd = currentSegment.back();
        const auto& nextStart = nextSegment.front();
        
        // Check position continuity
        if (currentEnd.position.size() != nextStart.position.size()) {
            LOG_ERROR << "Position vector size mismatch between segments " << segIdx << " and " << (segIdx + 1);
            continue;
        }
        
        double maxJointDiff = 0.0;
        int worstJoint = -1;
        for (size_t j = 0; j < currentEnd.position.size(); ++j) {
            double jointDiff = std::abs(nextStart.position[j] - currentEnd.position[j]);
            if (jointDiff > maxJointDiff) {
                maxJointDiff = jointDiff;
                worstJoint = static_cast<int>(j);
            }
        }
        
        if (maxJointDiff > POSITION_THRESHOLD) {
            LOG_WARNING << "Position discontinuity segments " << segIdx << "-" << (segIdx + 1)
                       << ": joint " << worstJoint << " differs by " << std::fixed << std::setprecision(3) 
                       << (maxJointDiff * 180.0 / M_PI) << " deg";
            maxPositionDiscontinuity = std::max(maxPositionDiscontinuity, maxJointDiff);
            discontinuityCount++;
        }
    }
    
    // Summary
    if (discontinuityCount == 0) {
        LOG_DEBUG << "No significant discontinuities detected";
    } else if (maxPositionDiscontinuity > 0.1) { // > 5.7 degrees
        LOG_WARNING << "Large position discontinuities detected: " << discontinuityCount << " found";
    } else {
        LOG_DEBUG << "Minor discontinuities: " << discontinuityCount << " found";
    }
}

bool UltrasoundScanTrajectoryPlanner::fixTrajectoryDiscontinuities()
{
    if (_trajectories.size() < 2) {
        return false; // No discontinuities possible with less than 2 segments
    }

    LOG_DEBUG << "Checking trajectory discontinuities";
    
    const double POSITION_THRESHOLD = 0.01; // 0.01 radians (~0.57 degrees) 
    const double CORRECTION_TIME = 0.1;     // 100ms for correction segments
    bool discontinuitiesFound = false;
    
    for (size_t segIdx = 0; segIdx < _trajectories.size() - 1; ++segIdx) {
        auto& currentSegment = _trajectories[segIdx].first;
        const auto& nextSegment = _trajectories[segIdx + 1].first;
        bool currentIsContactForce = _trajectories[segIdx].second;
        
        if (currentSegment.empty() || nextSegment.empty()) {
            continue;
        }
        
        const auto& currentEnd = currentSegment.back();
        const auto& nextStart = nextSegment.front();
        
        // Check for position discontinuity
        double maxJointDiff = 0.0;
        int worstJoint = -1;
        for (size_t j = 0; j < currentEnd.position.size() && j < nextStart.position.size(); ++j) {
            double jointDiff = std::abs(nextStart.position[j] - currentEnd.position[j]);
            if (jointDiff > maxJointDiff) {
                maxJointDiff = jointDiff;
                worstJoint = static_cast<int>(j);
            }
        }
        
        if (maxJointDiff > POSITION_THRESHOLD) {
            LOG_DEBUG << "Discontinuity at segments " << segIdx << "-" << (segIdx + 1)
                    << ": joint " << worstJoint << " differs by " << std::fixed << std::setprecision(2) 
                    << (maxJointDiff * 180.0 / M_PI) << " deg";
            
            // Fix discontinuity by appending correction segment to the current trajectory
            bool correctionAdded = addCorrectionSegment(segIdx, currentEnd, nextStart);
            if (correctionAdded) {
                LOG_DEBUG << "Added correction segment to trajectory " << segIdx;
                discontinuitiesFound = true;
            } else {
                LOG_WARNING << "Failed to add correction segment for segment " << segIdx;
            }
        }
    }
    
    if (!discontinuitiesFound) {
        LOG_DEBUG << "No discontinuities requiring correction";
    }
    
    return discontinuitiesFound;
}

bool UltrasoundScanTrajectoryPlanner::addCorrectionSegment(
    size_t segmentIndex,
    const MotionGenerator::TrajectoryPoint& fromPoint, 
    const MotionGenerator::TrajectoryPoint& toPoint)
{
    try {
        auto motionGenerator = createMotionGeneratorWithSharedSdf(*_arm);
        
        LOG_DEBUG << "Planning correction segment";
        
        // Create a correction trajectory that starts at t=0 (as intended behavior requires)
        MotionGenerator::TrajectoryPoint correctionStart = fromPoint;
        MotionGenerator::TrajectoryPoint correctionEnd = toPoint;
        
        // Reset times to 0 to ensure correction trajectory starts at t=0
        correctionStart.time = 0.0;
        correctionEnd.time = 0.0; // Will be calculated by computeTimeOptimalSegment
        
        auto correctionTrajectory = motionGenerator->computeTimeOptimalSegment(
            correctionStart,  // start at t=0
            correctionEnd,    // end (time will be calculated)
            fromPoint.time              // startTime = 0 (every trajectory should start at t=0)
        );
        
        if (correctionTrajectory.empty()) {
            LOG_ERROR << "Failed to generate correction trajectory";
            return false;
        }
        
        // Append the correction trajectory points to the current segment (skip first point to avoid duplication)
        auto& currentSegment = _trajectories[segmentIndex].first;
        double currentSegmentEndTime = currentSegment.empty() ? 0.0 : currentSegment.back().time;
        
        for (size_t i = 1; i < correctionTrajectory.size(); ++i) { // Skip first point
            auto point = correctionTrajectory[i];
            // Adjust time to continue from where current segment ends
            point.time += currentSegmentEndTime;
            currentSegment.push_back(point);
        }
        
        // Ensure the correction segment ends exactly at the expected position
        if (!currentSegment.empty()) {
            auto& lastPoint = currentSegment.back();
            lastPoint.position = toPoint.position;
            lastPoint.velocity = toPoint.velocity;
            lastPoint.acceleration = toPoint.acceleration;
        }
        
        // DO NOT adjust next segment timing - each segment should start at t=0 as intended
        // The next segment will naturally start at t=0 as designed
        
        double correctionDuration = correctionTrajectory.back().time - correctionTrajectory.front().time;
        LOG_DEBUG << "Correction segment added: " << (correctionTrajectory.size() - 1) 
                << " points, duration: " << std::fixed << std::setprecision(3) 
                << correctionDuration << "s";
        
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR << "Exception while generating correction segment: " << e.what();
        return false;
    }
}

void UltrasoundScanTrajectoryPlanner::initializeSharedSdf()
{
    if (_sdfCacheInitialized || !_obstacleTree) {
        return;
    }
    
    LOG_DEBUG << "Initializing shared SDF cache";
    
    MotionGenerator tempGenerator(*_arm);
    tempGenerator.setObstacleTree(_obstacleTree);
    tempGenerator.createSDF();
    
    if (tempGenerator.isSdfInitialized()) {
        _sharedSdf = tempGenerator.getSdf();
        _sharedSdfMinPoint = tempGenerator.getSdfMinPoint();
        _sharedSdfMaxPoint = tempGenerator.getSdfMaxPoint();
        _sharedSdfResolution = tempGenerator.getSdfResolution();
        _sdfCacheInitialized = true;
        
        LOG_DEBUG << "SDF cache initialized: " 
                 << _sharedSdf.size() << "x" << _sharedSdf[0].size() << "x" << _sharedSdf[0][0].size() 
                 << " voxels at " << _sharedSdfResolution << "m resolution";
    } else {
        LOG_WARNING << "Failed to initialize SDF cache";
    }
}

std::unique_ptr<MotionGenerator> UltrasoundScanTrajectoryPlanner::createMotionGeneratorWithSharedSdf(const RobotArm& arm)
{
    if (!_sdfCacheInitialized) {
        initializeSharedSdf();
    }
    
    if (_sdfCacheInitialized) {
        return std::make_unique<MotionGenerator>(arm, _sharedSdf, _sharedSdfMinPoint, 
                                               _sharedSdfMaxPoint, _sharedSdfResolution, _obstacleTree);
    } else {
        auto generator = std::make_unique<MotionGenerator>(arm);
        if (_obstacleTree) {
            generator->setObstacleTree(_obstacleTree);
        }
        return generator;
    }
}

bool UltrasoundScanTrajectoryPlanner::shouldUseFlatParallelization(size_t numTrajectories) const
{
    // Workload-aware parallelization strategy:
    // - Small batches (≤ physicalCores): Use hierarchical (internal STOMP parallelization)
    // - Large batches (> physicalCores): Use flat (trajectory-level parallelization)
    const size_t threshold = _hardwareConfig.physicalCores;
    bool useFlatParallelization = numTrajectories > threshold;
    
    LOG_DEBUG << "Parallelization strategy for " << numTrajectories << " trajectories: "
              << (useFlatParallelization ? "FLAT" : "HIERARCHICAL")
              << " (threshold: " << threshold << " cores)";
    
    return useFlatParallelization;
}

void UltrasoundScanTrajectoryPlanner::printSegmentTimes() const
{
    if (_trajectories.empty()) {
        LOG_INFO << "No trajectory segments to display";
        return;
    }
    
    LOG_INFO << "Trajectory segment timing analysis:";
    LOG_INFO << "=================================";
    
    double totalDuration = 0.0;
    
    for (size_t segIdx = 0; segIdx < _trajectories.size(); ++segIdx) {
        const auto& segment = _trajectories[segIdx].first;
        bool isContactForce = _trajectories[segIdx].second;
        
        if (segment.empty()) {
            LOG_INFO << "Segment " << segIdx << ": EMPTY ("
                    << (isContactForce ? "Contact" : "Repositioning") << ")";
            continue;
        }
        
        double startTime = segment.front().time;
        double endTime = segment.back().time;
        double duration = endTime - startTime;
        
        LOG_INFO << "Segment " << segIdx << ": " 
                << std::fixed << std::setprecision(3)
                << startTime << "s -> " << endTime << "s"
                << " (duration: " << duration << "s, "
                << segment.size() << " points, "
                << (isContactForce ? "Contact" : "Repositioning") << ")";
        
        totalDuration = std::max(totalDuration, endTime);
    }
    
    LOG_INFO << "=================================";
    LOG_INFO << "Total trajectory duration: " << std::fixed << std::setprecision(3) 
            << totalDuration << "s across " << _trajectories.size() << " segments";
}
