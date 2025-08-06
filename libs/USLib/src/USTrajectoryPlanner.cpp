#include "USLib/USTrajectoryPlanner.h"
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

    // Track planning success/failure status for robust partial planning
    PlanningStatus status;

    // Special case: single pose request - only return free movement trajectory
    if (_poses.size() == 1) {
        LOG_INFO << "Single pose request: planning direct movement trajectory";

        auto checkpointResult = _pathPlanner->planCheckpoints(_poses, _currentJoints);
        auto &validArms = checkpointResult.validArms;

        if (validArms.empty()) {
            LOG_ERROR << "No valid checkpoint found for single pose request";
            return false;
        }

        // Extract target arm configuration
        RobotArm targetArm = validArms[0];

        // Plan single trajectory from current position to target pose
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> singleRequest;
        std::vector<std::string> singleDescription;

        singleRequest.emplace_back(_currentJoints, targetArm.getJointAngles());
        singleDescription.emplace_back("Single pose free movement: current -> pose 0");

        std::vector<Trajectory> result;
        try {
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
                LOG_INFO << "Single pose trajectory successful: " << trajectoryPoints.size() << " points";
                return true;
            } else {
                LOG_ERROR << "Failed to plan single pose trajectory - trajectory planning returned empty result";
                return false;
            }
        } catch (const std::exception& e) {
            LOG_ERROR << "Failed to plan single pose trajectory - exception: " << e.what();
            return false;
        }
    }

    auto checkpointResult = _pathPlanner->planCheckpoints(_poses, _currentJoints);
    auto &validArms = checkpointResult.validArms;
    auto &validSegments = checkpointResult.validSegments;
    auto &validPoseIndices = checkpointResult.validPoseIndices;

    // Log the indices of the valid segments
    LOG_INFO << "Valid segments: " << validSegments.size();
    for (const auto &segment : validSegments) {
        LOG_DEBUG << "Segment " << segment.first << "-" << segment.second;
    }

    // Validate that we have valid poses to work with
    if (validArms.empty()) {
        throw std::runtime_error("No valid checkpoint found to start trajectory planning.");
    }
    
    LOG_INFO << "Found " << validArms.size() << " valid poses out of " << checkpointResult.totalOriginalPoses << " total poses";

    // Extract arm configurations - now we only have valid arms
    std::vector<RobotArm> arms = validArms;

    // Build repositioning trajectory requests for batch planning (only inter-segment repositioning)
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> repositioningRequests;
    std::vector<std::string> repositioningDescriptions; // Track what each repositioning is for

    // Add inter-segment repositioning trajectories (initial repositioning handled separately)
    for (size_t segIdx = 0; segIdx + 1 < validSegments.size(); segIdx++) {
        size_t currentEnd = validSegments[segIdx].second;
        size_t nextStart = validSegments[segIdx + 1].first;

        repositioningRequests.emplace_back(arms[currentEnd].getJointAngles(),
                                           arms[nextStart].getJointAngles());
        repositioningDescriptions.emplace_back("Inter-segment repositioning: valid pose " 
                                               + std::to_string(currentEnd) + " -> valid pose "
                                               + std::to_string(nextStart) + " (original poses "
                                               + std::to_string(validPoseIndices[currentEnd]) + " -> "
                                               + std::to_string(validPoseIndices[nextStart]) + ")");
    }

    LOG_INFO << "Planning " << repositioningRequests.size() << " inter-segment repositioning trajectories using "
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

    // Find the longest connected chain of segments with successful repositioning
    auto [chainStartIdx, chainEndIdx] = findLongestConnectedChain(validSegments, repositioningResults);
    
    // Filter segments to only include the selected chain
    std::vector<std::pair<size_t, size_t>> selectedSegments;
    std::vector<Trajectory> selectedRepositioningResults;
    
    for (size_t segIdx = chainStartIdx; segIdx <= chainEndIdx; segIdx++) {
        selectedSegments.push_back(validSegments[segIdx]);
        
        // Add corresponding repositioning result if not the last segment in chain
        if (segIdx < chainEndIdx) {
            // The repositioning index is relative to the original validSegments
            if (segIdx < repositioningResults.size()) {
                selectedRepositioningResults.push_back(repositioningResults[segIdx]);
            }
        }
    }
    
    LOG_INFO << "Using selected chain: segments " << chainStartIdx << "-" << chainEndIdx 
             << " (" << selectedSegments.size() << " segments, " 
             << selectedRepositioningResults.size() << " repositioning trajectories)";

    // Now build the final trajectory sequence in the correct order
    
    // Find the first reachable segment by trying initial repositioning with fallback
    size_t startingSegmentIdx = 0;
    bool initialRepositioningSuccessful = false;
    
    std::tie(startingSegmentIdx, initialRepositioningSuccessful) = 
        tryInitialRepositioning(selectedSegments, arms, validPoseIndices, useHauserForRepositioning, enableShortcutting, status);

    // Log information about which segments will be processed
    if (startingSegmentIdx > 0) {
        LOG_INFO << "Starting trajectory planning from selected segment " << startingSegmentIdx 
                 << " (skipping first " << startingSegmentIdx << " segments in chain for maximum reachability)";
        size_t totalPosesInChain = 0;
        for (size_t segIdx = startingSegmentIdx; segIdx < selectedSegments.size(); segIdx++) {
            totalPosesInChain += selectedSegments[segIdx].second - selectedSegments[segIdx].first + 1;
        }
        LOG_INFO << "Final trajectory chain contains " << totalPosesInChain << " poses across " 
                 << (selectedSegments.size() - startingSegmentIdx) << " segments";
    } else {
        LOG_INFO << "Starting trajectory planning from first segment in selected chain (all segments reachable)";
        size_t totalPosesInChain = 0;
        for (const auto& segment : selectedSegments) {
            totalPosesInChain += segment.second - segment.first + 1;
        }
        LOG_INFO << "Full selected chain contains " << totalPosesInChain << " poses across " 
                 << selectedSegments.size() << " segments";
    }

    size_t repositioningIndex = 0;

    // 2. Add contact force trajectories for each selected segment starting from the reachable segment, with inter-segment repositioning in between
    for (size_t segIdx = startingSegmentIdx; segIdx < selectedSegments.size(); segIdx++) {
        size_t start = selectedSegments[segIdx].first;
        size_t end = selectedSegments[segIdx].second;
        status.totalSegments++;

        try {
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
                LOG_DEBUG << "Contact force point at valid checkpoint " << start 
                          << " (original pose " << validPoseIndices[start] << ")";
                status.successfulSegments++;
                status.successes.push_back("Contact force segment " + std::to_string(segIdx) + " (single point)");
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
                
                LOG_DEBUG << "Contact force trajectory: valid poses " << start << "-" << end 
                          << " (original poses " << validPoseIndices[start] << "-" << validPoseIndices[end] << ") ("
                          << trajectoryPoints.size() << " points, " << (end - start + 1) << " poses)";
                status.successfulSegments++;
                status.successes.push_back("Contact force segment " + std::to_string(segIdx) + 
                                         " (" + std::to_string(end - start + 1) + " poses)");
            }
        } catch (const std::exception& e) {
            LOG_WARNING << "Contact force trajectory generation failed for segment " << segIdx 
                        << " (poses " << start << "-" << end << "): " << e.what() << " - skipping segment";
            status.failures.push_back("Contact force segment " + std::to_string(segIdx) + " failed: " + e.what());
            // Continue with next segment instead of failing completely
        }

        // Add inter-segment repositioning if not the last segment (handle failure gracefully)
        if (segIdx + 1 < selectedSegments.size()) {
            status.totalRepositioning++;
            // Calculate the correct repositioning index: segIdx - startingSegmentIdx gives us the offset from our starting segment
            size_t repositioningIdx = segIdx - startingSegmentIdx;
            if (repositioningIdx < selectedRepositioningResults.size()
                && !selectedRepositioningResults[repositioningIdx].empty()) {
                std::vector<MotionGenerator::TrajectoryPoint> trajectoryPoints;
                trajectoryPoints.reserve(selectedRepositioningResults[repositioningIdx].size());

                // Copy repositioning trajectory points
                for (const auto &point : selectedRepositioningResults[repositioningIdx]) {
                    trajectoryPoints.push_back(point);
                }

                _trajectories.emplace_back(trajectoryPoints,
                                           false); // repositioning = NOT contact force
                size_t currentEnd = selectedSegments[segIdx].second;
                size_t nextStart = selectedSegments[segIdx + 1].first;
                LOG_DEBUG << "Inter-segment repositioning " << currentEnd << "-" << nextStart
                          << " (original poses " << validPoseIndices[currentEnd] << "-" << validPoseIndices[nextStart] << ")"
                          << ": " << trajectoryPoints.size() << " points";
                status.successfulRepositioning++;
                status.successes.push_back("Inter-segment repositioning " + std::to_string(segIdx) + " successful");
            } else {
                LOG_WARNING << "Inter-segment repositioning failed for segment " << segIdx << " - skipping repositioning";
                status.failures.push_back("Inter-segment repositioning " + std::to_string(segIdx) + " failed");
                // Continue without this repositioning trajectory instead of failing completely
            }
        }
    }

    // Log detailed planning status
    LOG_INFO << "Planning Status Summary:";
    LOG_INFO << "  Contact force segments: " << status.successfulSegments << "/" << status.totalSegments;
    LOG_INFO << "  Repositioning trajectories: " << status.successfulRepositioning << "/" << status.totalRepositioning;
    
    if (!status.successes.empty()) {
        LOG_INFO << "  Successes:";
        for (const auto& success : status.successes) {
            LOG_INFO << "    - " << success;
        }
    }
    
    if (!status.failures.empty()) {
        LOG_WARNING << "  Failures:";
        for (const auto& failure : status.failures) {
            LOG_WARNING << "    - " << failure;
        }
    }

    // Only fail if NO contact force segments could be planned (no useful scanning work possible)
    if (status.successfulSegments == 0) {
        LOG_ERROR << "Critical failure: No contact force segments could be planned - no scanning work possible";
        return false;
    }

    LOG_INFO << "Generated " << _trajectories.size() << " trajectories"
             << " (" << status.successfulSegments << " contact force segments + "
             << status.successfulRepositioning << " repositioning trajectories)";

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
            boost::asio::post(*_sharedThreadPool,
                              []() { std::this_thread::sleep_for(std::chrono::microseconds(1)); });
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
                            LOG_WARNING << "STOMP failed, trying two-step fallback: "
                                        << descriptions[j];

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
                                step2Success = step2Generator->performSTOMP(fallBackConfig,
                                                                            _sharedThreadPool);
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
                                LOG_DEBUG << "Two-step STOMP success: " << descriptions[j] << " ("
                                          << step1Traj.size() << "+" << step2Traj.size() << "="
                                          << combinedTrajectory.size() << " points)";
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
    double trajectoryThroughput = (double) successfulTrajectories / (duration.count() / 1000.0);
    double avgTimePerTrajectory = (double) duration.count() / requests.size();

    LOG_INFO << "Batch planning complete (" << (useFlatParallelization ? "FLAT" : "HIERARCHICAL")
             << "): " << successfulTrajectories << "/" << requests.size() << " (" << std::fixed
             << std::setprecision(1) << (100.0 * successfulTrajectories / requests.size())
             << "%) in " << duration.count() << "ms";
    LOG_INFO << "Performance: " << std::fixed << std::setprecision(2) << trajectoryThroughput
             << " trajectories/sec, " << avgTimePerTrajectory << "ms avg/trajectory";

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
                        rrtConnectParams.stepSize = 0.1;
                        rrtConnectParams.goalBiasProbability = 0.5;
                        rrtConnectParams.maxIterations = 20000;
                        rrtConnectParams.maxRestarts = 25;

                        hauserPathPlanner->setParams(rrtConnectParams);

                        LOG_DEBUG << "Set RRT Connect maxRestarts to "
                                  << rrtConnectParams.maxRestarts;

                        // Set start and goal configurations
                        RobotArm startArm(*_arm);
                        startArm.setJointAngles(startJoints);
                        hauserPathPlanner->setStartPose(startArm);
                        hauserPathPlanner->setObstacleTree(_obstacleTree);

                        RobotArm goalArm(*_arm);
                        goalArm.setJointAngles(targetJoints);
                        hauserPathPlanner->setGoalConfiguration(goalArm);

                        bool rrtPathFound = hauserPathPlanner->runPathFinding();

                        bool success = false;

                        if (rrtPathFound) {
                            // Get optimized RRT Connect path and convert to waypoints for Hauser
                            Eigen::MatrixXd rrtWaypoints = hauserPathPlanner->getAnglesPath();
                            localGenerator.setWaypoints(rrtWaypoints);

                            // Use Hauser optimization with RRT Connect waypoints
                            try {
                                localGenerator.performHauser(100);

                                // Validate the Hauser trajectory actually reaches the goal
                                auto hauserPath = localGenerator.getPath();
                                if (!hauserPath.empty()) {
                                    // Check if final trajectory point is close to target
                                    const auto &finalPoint = hauserPath.back();
                                    bool goalReached = true;
                                    const double goalTolerance = 0.1; // 0.1 radians tolerance

                                    for (size_t k = 0;
                                         k < std::min((size_t) finalPoint.position.size(),
                                                      (size_t) targetJoints.size());
                                         ++k) {
                                        double deviation = std::abs(finalPoint.position[k]
                                                                    - targetJoints[k]);
                                        if (deviation > goalTolerance) {
                                            goalReached = false;
                                            break;
                                        }
                                    }

                                    if (goalReached) {
                                        success = true;
                                    } else {
                                        LOG_WARNING << "Hauser trajectory doesn't reach goal: "
                                                    << descriptions[j];
                                        success = false;
                                    }
                                } else {
                                    LOG_WARNING << "Hauser generated empty trajectory: "
                                                << descriptions[j];
                                    success = false;
                                }
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
                            LOG_WARNING << "Hauser failed, trying two-step fallback: "
                                        << descriptions[j];

                            // Fallback: Two separate RRT Connect + Hauser trajectories
                            // Step 1: startJoints -> _currentJoints (stopping at rest)
                            bool step1Success = false;
                            Trajectory step1Trajectory;

                            try {
                                auto step1PathPlanner = std::make_unique<PathPlanner>();
                                step1PathPlanner->setParams(rrtConnectParams);

                                RobotArm step1StartArm(*_arm);
                                step1StartArm.setJointAngles(startJoints);
                                step1PathPlanner->setStartPose(step1StartArm);
                                step1PathPlanner->setObstacleTree(_obstacleTree);

                                RobotArm step1GoalArm(*_arm);
                                step1GoalArm.setJointAngles(_currentJoints);
                                step1PathPlanner->setGoalConfiguration(step1GoalArm);

                                bool step1RrtFound = step1PathPlanner->runPathFinding();
                                if (step1RrtFound) {
                                    MotionGenerator step1Generator(*_arm);
                                    if (_obstacleTree) {
                                        step1Generator.setObstacleTree(_obstacleTree);
                                    }

                                    Eigen::MatrixXd step1RrtWaypoints = step1PathPlanner
                                                                            ->getAnglesPath();
                                    step1Generator.setWaypoints(step1RrtWaypoints);
                                    step1Generator.performHauser(100);
                                    step1Trajectory = step1Generator.getPath();
                                    step1Success = !step1Trajectory.empty();
                                }
                            } catch (const std::exception &e) {
                                step1Success = false;
                            }

                            // Step 2: _currentJoints -> targetJoints (starting from rest)
                            bool step2Success = false;
                            Trajectory step2Trajectory;

                            try {
                                auto step2PathPlanner = std::make_unique<PathPlanner>();
                                step2PathPlanner->setParams(rrtConnectParams);

                                RobotArm step2StartArm(*_arm);
                                step2StartArm.setJointAngles(_currentJoints);
                                step2PathPlanner->setStartPose(step2StartArm);
                                step2PathPlanner->setObstacleTree(_obstacleTree);

                                RobotArm step2GoalArm(*_arm);
                                step2GoalArm.setJointAngles(targetJoints);
                                step2PathPlanner->setGoalConfiguration(step2GoalArm);

                                bool step2RrtFound = step2PathPlanner->runPathFinding();
                                if (step2RrtFound) {
                                    MotionGenerator step2Generator(*_arm);
                                    if (_obstacleTree) {
                                        step2Generator.setObstacleTree(_obstacleTree);
                                    }

                                    Eigen::MatrixXd step2RrtWaypoints = step2PathPlanner
                                                                            ->getAnglesPath();
                                    step2Generator.setWaypoints(step2RrtWaypoints);
                                    step2Generator.performHauser(100);
                                    step2Trajectory = step2Generator.getPath();
                                    step2Success = !step2Trajectory.empty();
                                }
                            } catch (const std::exception &e) {
                                step2Success = false;
                            }

                            if (step1Success && step2Success) {
                                // Combine trajectories, skipping duplicate middle point
                                Trajectory combinedTrajectory;
                                combinedTrajectory.reserve(step1Trajectory.size()
                                                           + step2Trajectory.size() - 1);

                                for (const auto &point : step1Trajectory) {
                                    combinedTrajectory.push_back(point);
                                }
                                for (size_t i = 1; i < step2Trajectory.size(); ++i) {
                                    combinedTrajectory.push_back(step2Trajectory[i]);
                                }

                                results[j] = combinedTrajectory;
                                successfulTrajectories++;
                                LOG_DEBUG << "Two-step Hauser success: " << descriptions[j] << " ("
                                          << step1Trajectory.size() << "+" << step2Trajectory.size()
                                          << "=" << combinedTrajectory.size() << " points)";
                            } else {
                                LOG_ERROR << "Two-step Hauser failed: " << descriptions[j]
                                          << " (Step 1: " << (step1Success ? "OK" : "FAIL")
                                          << ", Step 2: " << (step2Success ? "OK" : "FAIL") << ")";
                            }
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
    double trajectoryThroughput = (double) successfulTrajectories / (duration.count() / 1000.0);
    double avgTimePerTrajectory = (double) duration.count() / requests.size();

    LOG_INFO << "Hauser batch complete (" << (useFlatParallelization ? "FLAT" : "HIERARCHICAL")
             << "): " << successfulTrajectories << "/" << requests.size() << " (" << std::fixed
             << std::setprecision(1) << (100.0 * successfulTrajectories / requests.size())
             << "%) in " << duration.count() << "ms";
    LOG_INFO << "Performance: " << std::fixed << std::setprecision(2) << trajectoryThroughput
             << " trajectories/sec, " << avgTimePerTrajectory << "ms avg/trajectory";

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
        const auto &currentSegment = _trajectories[segIdx].first;
        const auto &nextSegment = _trajectories[segIdx + 1].first;

        if (currentSegment.empty() || nextSegment.empty()) {
            LOG_WARNING << "Trajectory segment " << segIdx << " or " << (segIdx + 1) << " is empty";
            continue;
        }

        const auto &currentEnd = currentSegment.back();
        const auto &nextStart = nextSegment.front();

        // Check position continuity
        if (currentEnd.position.size() != nextStart.position.size()) {
            LOG_ERROR << "Position vector size mismatch between segments " << segIdx << " and "
                      << (segIdx + 1);
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
                        << ": joint " << worstJoint << " differs by " << std::fixed
                        << std::setprecision(3) << (maxJointDiff * 180.0 / M_PI) << " deg";
            maxPositionDiscontinuity = std::max(maxPositionDiscontinuity, maxJointDiff);
            discontinuityCount++;
        }
    }

    // Summary
    if (discontinuityCount == 0) {
        LOG_DEBUG << "No significant discontinuities detected";
    } else if (maxPositionDiscontinuity > 0.1) { // > 5.7 degrees
        LOG_WARNING << "Large position discontinuities detected: " << discontinuityCount
                    << " found";
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
        auto &currentSegment = _trajectories[segIdx].first;
        const auto &nextSegment = _trajectories[segIdx + 1].first;
        bool currentIsContactForce = _trajectories[segIdx].second;

        if (currentSegment.empty() || nextSegment.empty()) {
            continue;
        }

        const auto &currentEnd = currentSegment.back();
        const auto &nextStart = nextSegment.front();

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
            LOG_DEBUG << "Discontinuity at segments " << segIdx << "-" << (segIdx + 1) << ": joint "
                      << worstJoint << " differs by " << std::fixed << std::setprecision(2)
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
    const MotionGenerator::TrajectoryPoint &fromPoint,
    const MotionGenerator::TrajectoryPoint &toPoint)
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
            correctionStart, // start at t=0
            correctionEnd,   // end (time will be calculated)
            fromPoint.time   // startTime = 0 (every trajectory should start at t=0)
        );

        if (correctionTrajectory.empty()) {
            LOG_ERROR << "Failed to generate correction trajectory";
            return false;
        }

        // Append the correction trajectory points to the current segment (skip first point to avoid duplication)
        auto &currentSegment = _trajectories[segmentIndex].first;
        double currentSegmentEndTime = currentSegment.empty() ? 0.0 : currentSegment.back().time;

        for (size_t i = 1; i < correctionTrajectory.size(); ++i) { // Skip first point
            auto point = correctionTrajectory[i];
            // Adjust time to continue from where current segment ends
            point.time += currentSegmentEndTime;
            currentSegment.push_back(point);
        }

        // Ensure the correction segment ends exactly at the expected position
        if (!currentSegment.empty()) {
            auto &lastPoint = currentSegment.back();
            lastPoint.position = toPoint.position;
            lastPoint.velocity = toPoint.velocity;
            lastPoint.acceleration = toPoint.acceleration;
        }

        // DO NOT adjust next segment timing - each segment should start at t=0 as intended
        // The next segment will naturally start at t=0 as designed

        double correctionDuration = correctionTrajectory.back().time
                                    - correctionTrajectory.front().time;
        LOG_DEBUG << "Correction segment added: " << (correctionTrajectory.size() - 1)
                  << " points, duration: " << std::fixed << std::setprecision(3)
                  << correctionDuration << "s";

        return true;

    } catch (const std::exception &e) {
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

        LOG_DEBUG << "SDF cache initialized: " << _sharedSdf.size() << "x" << _sharedSdf[0].size()
                  << "x" << _sharedSdf[0][0].size() << " voxels at " << _sharedSdfResolution
                  << "m resolution";
    } else {
        LOG_WARNING << "Failed to initialize SDF cache";
    }
}

std::unique_ptr<MotionGenerator> UltrasoundScanTrajectoryPlanner::createMotionGeneratorWithSharedSdf(
    const RobotArm &arm)
{
    if (!_sdfCacheInitialized) {
        initializeSharedSdf();
    }

    if (_sdfCacheInitialized) {
        return std::make_unique<MotionGenerator>(arm,
                                                 _sharedSdf,
                                                 _sharedSdfMinPoint,
                                                 _sharedSdfMaxPoint,
                                                 _sharedSdfResolution,
                                                 _obstacleTree);
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

    LOG_DEBUG << "Parallelization strategy for " << numTrajectories
              << " trajectories: " << (useFlatParallelization ? "FLAT" : "HIERARCHICAL")
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
        const auto &segment = _trajectories[segIdx].first;
        bool isContactForce = _trajectories[segIdx].second;

        if (segment.empty()) {
            LOG_INFO << "Segment " << segIdx << ": EMPTY ("
                     << (isContactForce ? "Contact" : "Repositioning") << ")";
            continue;
        }

        double startTime = segment.front().time;
        double endTime = segment.back().time;
        double duration = endTime - startTime;

        LOG_INFO << "Segment " << segIdx << ": " << std::fixed << std::setprecision(3) << startTime
                 << "s -> " << endTime << "s"
                 << " (duration: " << duration << "s, " << segment.size() << " points, "
                 << (isContactForce ? "Contact" : "Repositioning") << ")";

        totalDuration = std::max(totalDuration, endTime);
    }

    LOG_INFO << "=================================";
    LOG_INFO << "Total trajectory duration: " << std::fixed << std::setprecision(3) << totalDuration
             << "s across " << _trajectories.size() << " segments";
}

std::pair<size_t, bool> UltrasoundScanTrajectoryPlanner::tryInitialRepositioning(
    const std::vector<std::pair<size_t, size_t>>& validSegments,
    const std::vector<RobotArm>& arms,
    const std::vector<size_t>& validPoseIndices,
    bool useHauserForRepositioning,
    bool enableShortcutting,
    PlanningStatus& status)
{
    for (size_t segIdx = 0; segIdx < validSegments.size(); segIdx++) {
        size_t segmentStart = validSegments[segIdx].first;
        
        // Try to plan initial repositioning to this segment
        std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> initialRequest;
        std::vector<std::string> initialDescription;
        
        initialRequest.emplace_back(_currentJoints, arms[segmentStart].getJointAngles());
        initialDescription.emplace_back("Initial repositioning: current -> valid pose " 
                                       + std::to_string(segmentStart) + " (original pose " 
                                       + std::to_string(validPoseIndices[segmentStart]) + ")");
        
        std::vector<Trajectory> initialResult;
        try {
            // For initial repositioning, use direct planning without two-step fallback
            if (useHauserForRepositioning) {
                // Use Hauser but without two-step fallback for initial repositioning
                MotionGenerator localGenerator(*_arm);
                if (_obstacleTree) {
                    localGenerator.setObstacleTree(_obstacleTree);
                }

                // Use RRT Connect to generate waypoints for Hauser optimization
                auto hauserPathPlanner = std::make_unique<PathPlanner>();
                Params rrtConnectParams;
                rrtConnectParams.algo = RRTConnect;
                rrtConnectParams.stepSize = 0.1;
                rrtConnectParams.goalBiasProbability = 0.5;
                rrtConnectParams.maxIterations = 20000;
                rrtConnectParams.maxRestarts = 25;
                hauserPathPlanner->setParams(rrtConnectParams);

                RobotArm startArm(*_arm);
                startArm.setJointAngles(_currentJoints);
                hauserPathPlanner->setStartPose(startArm);
                hauserPathPlanner->setObstacleTree(_obstacleTree);

                RobotArm goalArm(*_arm);
                goalArm.setJointAngles(arms[segmentStart].getJointAngles());
                hauserPathPlanner->setGoalConfiguration(goalArm);

                bool rrtPathFound = hauserPathPlanner->runPathFinding();
                bool success = false;

                if (rrtPathFound) {
                    Eigen::MatrixXd rrtWaypoints = hauserPathPlanner->getAnglesPath();
                    localGenerator.setWaypoints(rrtWaypoints);

                    try {
                        localGenerator.performHauser(100);
                        auto hauserPath = localGenerator.getPath();
                        
                        if (!hauserPath.empty()) {
                            // Validate goal reaching
                            const auto &finalPoint = hauserPath.back();
                            const auto &targetJoints = arms[segmentStart].getJointAngles();
                            bool goalReached = true;
                            const double goalTolerance = 0.1;

                            for (size_t k = 0; k < std::min((size_t)finalPoint.position.size(), (size_t)targetJoints.size()); ++k) {
                                double deviation = std::abs(finalPoint.position[k] - targetJoints[k]);
                                if (deviation > goalTolerance) {
                                    goalReached = false;
                                    break;
                                }
                            }

                            if (goalReached) {
                                initialResult.push_back(hauserPath);
                                success = true;
                            }
                        }
                    } catch (const std::exception &e) {
                        success = false;
                    }
                }

                if (!success) {
                    LOG_DEBUG << "Direct Hauser failed for initial repositioning to segment " << segIdx;
                }
            } else {
                // Use STOMP for initial repositioning (direct planning without two-step fallback)
                auto localGenerator = createMotionGeneratorWithSharedSdf(*_arm);
                StompConfig config;
                config.disableInternalParallelization = shouldUseFlatParallelization(1);

                Eigen::MatrixXd waypoints(2, _currentJoints.size());
                waypoints.row(0) = _currentJoints.transpose();
                waypoints.row(1) = arms[segmentStart].getJointAngles().transpose();

                localGenerator->setWaypoints(waypoints);

                bool success = false;
                try {
                    success = localGenerator->performSTOMP(config, _sharedThreadPool, static_cast<int>(segIdx));
                    
                    if (success) {
                        auto stompPath = localGenerator->getPath();
                        if (!stompPath.empty()) {
                            initialResult.push_back(stompPath);
                        } else {
                            success = false;
                        }
                    }
                } catch (const StompFailedException &) {
                    success = false;
                } catch (const std::exception &e) {
                    LOG_DEBUG << "STOMP exception for initial repositioning to segment " << segIdx << ": " << e.what();
                    success = false;
                }

                if (!success) {
                    LOG_DEBUG << "Direct STOMP failed for initial repositioning to segment " << segIdx;
                }
            }
            
            if (!initialResult.empty() && !initialResult[0].empty()) {
                // Success! Use this segment as starting point
                std::vector<MotionGenerator::TrajectoryPoint> trajectoryPoints;
                trajectoryPoints.reserve(initialResult[0].size());
                
                for (const auto &point : initialResult[0]) {
                    trajectoryPoints.push_back(point);
                }
                
                _trajectories.emplace_back(trajectoryPoints, false); // repositioning = NOT contact force
                LOG_DEBUG << "Initial repositioning successful to segment " << segIdx 
                          << " (valid pose " << segmentStart << ", original pose " << validPoseIndices[segmentStart] << "): " 
                          << trajectoryPoints.size() << " points";
                
                status.totalRepositioning++;
                status.successfulRepositioning++;
                status.successes.push_back("Initial repositioning to segment " + std::to_string(segIdx) + " successful");
                
                return std::make_pair(segIdx, true);
            }
        } catch (const std::exception& e) {
            LOG_DEBUG << "Initial repositioning failed to segment " << segIdx << ": " << e.what();
        }
        
        LOG_DEBUG << "Initial repositioning failed to segment " << segIdx << " - trying next segment";
    }
    
    // All attempts failed
    LOG_WARNING << "All initial repositioning attempts failed - starting from current position without repositioning";
    status.totalRepositioning++;
    status.failures.push_back("All initial repositioning attempts failed");
    
    return std::make_pair(0, false);
}

std::pair<size_t, size_t> UltrasoundScanTrajectoryPlanner::findLongestConnectedChain(
    const std::vector<std::pair<size_t, size_t>>& validSegments,
    const std::vector<Trajectory>& repositioningResults)
{
    if (validSegments.empty()) {
        return std::make_pair(0, 0);
    }
    
    // If we only have one segment, return it
    if (validSegments.size() == 1) {
        return std::make_pair(0, 0);
    }
    
    // Find all connected chains
    std::vector<std::pair<size_t, size_t>> chains;
    std::vector<size_t> chainPoseCounts;
    
    size_t chainStart = 0;
    
    for (size_t segIdx = 0; segIdx < validSegments.size(); segIdx++) {
        bool isLastSegment = (segIdx == validSegments.size() - 1);
        bool repositioningFailed = false;
        
        // Check if repositioning from this segment to next fails (if not last segment)
        if (!isLastSegment) {
            if (segIdx >= repositioningResults.size() || repositioningResults[segIdx].empty()) {
                repositioningFailed = true;
            }
        }
        
        // If repositioning failed or we're at the last segment, end the current chain
        if (repositioningFailed || isLastSegment) {
            size_t chainEnd = segIdx;
            
            // Calculate total poses in this chain
            size_t totalPoses = 0;
            for (size_t chainSegIdx = chainStart; chainSegIdx <= chainEnd; chainSegIdx++) {
                totalPoses += validSegments[chainSegIdx].second - validSegments[chainSegIdx].first + 1;
            }
            
            chains.emplace_back(chainStart, chainEnd);
            chainPoseCounts.push_back(totalPoses);
            
            LOG_DEBUG << "Found chain: segments " << chainStart << "-" << chainEnd 
                      << " with " << totalPoses << " poses";
            
            // Start new chain from next segment (if repositioning failed)
            if (repositioningFailed && segIdx + 1 < validSegments.size()) {
                chainStart = segIdx + 1;
            }
        }
    }
    
    // Find the chain with maximum poses
    size_t maxPoses = 0;
    size_t bestChainIndex = 0;
    
    for (size_t i = 0; i < chainPoseCounts.size(); i++) {
        if (chainPoseCounts[i] > maxPoses) {
            maxPoses = chainPoseCounts[i];
            bestChainIndex = i;
        }
    }
    
    auto selectedChain = chains[bestChainIndex];
    
    LOG_INFO << "Chain selection analysis:";
    LOG_INFO << "  Found " << chains.size() << " connected chains";
    for (size_t i = 0; i < chains.size(); i++) {
        LOG_INFO << "    Chain " << i << ": segments " << chains[i].first << "-" << chains[i].second 
                 << " (" << chainPoseCounts[i] << " poses)" << (i == bestChainIndex ? " [SELECTED]" : "");
    }
    
    if (chains.size() > 1) {
        LOG_INFO << "  Selected longest chain with " << maxPoses << " poses";
        
        // Log which segments are being skipped
        if (selectedChain.first > 0) {
            size_t skippedAtStart = 0;
            for (size_t i = 0; i < selectedChain.first; i++) {
                skippedAtStart += validSegments[i].second - validSegments[i].first + 1;
            }
            LOG_INFO << "  Skipping " << skippedAtStart << " poses at start (segments 0-" << (selectedChain.first - 1) << ")";
        }
        
        if (selectedChain.second < validSegments.size() - 1) {
            size_t skippedAtEnd = 0;
            for (size_t i = selectedChain.second + 1; i < validSegments.size(); i++) {
                skippedAtEnd += validSegments[i].second - validSegments[i].first + 1;
            }
            LOG_INFO << "  Skipping " << skippedAtEnd << " poses at end (segments " << (selectedChain.second + 1) << "-" << (validSegments.size() - 1) << ")";
        }
    } else {
        LOG_INFO << "  All segments form one connected chain - no repositioning failures";
    }
    
    return selectedChain;
}
