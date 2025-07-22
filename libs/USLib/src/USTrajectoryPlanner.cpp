#include "USLib/USTrajectoryPlanner.h"
#include "TrajectoryLib/Logger.h"
#include "TrajectoryLib/Planning/PathPlanner.h"
#include <QDebug>
#include <fstream>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <boost/asio/post.hpp>
#include <boost/asio/use_future.hpp>

UltrasoundScanTrajectoryPlanner::UltrasoundScanTrajectoryPlanner(const std::string &environmentString, 
                                                               const std::vector<double> &currentJoints)
    : _hardwareConfig(HardwareConfig::detect()), _environment(environmentString)
{
    // Convert vector<double> to Eigen::VectorXd
    _currentJoints = Eigen::Map<const Eigen::VectorXd>(currentJoints.data(), currentJoints.size());
    
    // Initialize the arm from the environment string (assumed to be URDF)
    _arm = new RobotArm(environmentString);
    _pathPlanner = new PathPlanner();
    _pathPlanner->setStartPose(*_arm);
    _motionGenerator = new MotionGenerator(*_arm);
    
    // Initialize hardware-optimized thread pool
    initializeThreadPool();
    
    LOG_INFO << "Hardware Config - Physical cores:" << _hardwareConfig.physicalCores 
             << " Logical cores:" << _hardwareConfig.logicalCores
             << " Optimal batch size:" << _hardwareConfig.batchSize;
}

UltrasoundScanTrajectoryPlanner::~UltrasoundScanTrajectoryPlanner()
{
    // Ensure thread pool is properly shut down
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

std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>
UltrasoundScanTrajectoryPlanner::planSingleStompTrajectory(const Eigen::VectorXd &startJoints,
                                                           const Eigen::VectorXd &targetJoints,
                                                           const StompConfig &config)
{
    try {
        auto motionGen = new MotionGenerator(*_arm);
        motionGen->setObstacleTree(_obstacleTree);

        unsigned int numThreads = std::thread::hardware_concurrency();
        auto threadPool = std::make_shared<boost::asio::thread_pool>(numThreads);

        // Simple straight-line STOMP optimization
        Eigen::MatrixXd waypoints(2, startJoints.size());
        waypoints.row(0) = startJoints.transpose();
        waypoints.row(1) = targetJoints.transpose();
        motionGen->setWaypoints(waypoints);
        
        // Run STOMP optimization with straight-line initialization
        motionGen->performSTOMP(config, threadPool);
        LOG_INFO << "STOMP optimization succeeded for single trajectory";
        
        threadPool->join();

        auto result = std::make_pair(motionGen->getPath(), false);
        delete motionGen;
        return result;
    } catch (const std::exception &e) {
        LOG_ERROR << "Exception in planSingleStompTrajectory: " << e.what();
        throw;
    }
}

bool UltrasoundScanTrajectoryPlanner::planTrajectories()
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

    unsigned int numThreads = std::thread::hardware_concurrency();
    LOG_INFO << "Hardware threads available: " << numThreads;
    std::shared_ptr<boost::asio::thread_pool> threadPool
        = std::make_shared<boost::asio::thread_pool>(numThreads);

    auto checkpointResult = _pathPlanner->planCheckpoints(_poses, _currentJoints);
    auto &checkpoints = checkpointResult.checkpoints;
    auto &validSegments = checkpointResult.validSegments;
    auto &jumpPairs = checkpointResult.jumpPairs;
    size_t firstValidIndex = checkpointResult.firstValidIndex;

    // Log the indices of the valid segments
    LOG_INFO << "Valid segments identified:";
    for (const auto &segment : validSegments) {
        LOG_INFO << "  Segment: start=" << segment.first << " end=" << segment.second;
    }

    if (checkpoints.size() == 1) {
        auto [arm, valid] = checkpoints[0];

        if (!valid) {
            throw std::runtime_error("Single checkpoint is invalid.");
        }

        StompConfig config = StompConfig::optimized(); // Use optimized STOMP configuration
        config.maxComputeTimeMs = 0.0; // Remove time limit
        // Simple straight-line STOMP optimization
        
        auto trajectory = planSingleStompTrajectory(_currentJoints, arm.getJointAngles(), config);
        _trajectories.push_back(trajectory);
        return true;
    }

    // Validate that we have a valid starting point
    if (firstValidIndex >= checkpoints.size()) {
        throw std::runtime_error("No valid checkpoint found to start trajectory planning.");
    }
    if (firstValidIndex > 0) {
        LOG_INFO << "First " << firstValidIndex 
                 << " checkpoint(s) are invalid. Using first valid checkpoint at index: "
                 << firstValidIndex;
    }

    std::vector<RobotArm> arms;
    for (const auto &[arm, valid] : checkpoints) {
        arms.push_back(arm);
    }

    std::vector<std::promise<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>>>
        promises;
    std::vector<std::future<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>>> futures;
    size_t taskIndex = 0;

    size_t numTasks = 1 + validSegments.size()
                      + (validSegments.size() > 1 ? validSegments.size() - 1 : 0);
    promises.resize(numTasks);
    futures.reserve(numTasks);

    for (auto &promise : promises) {
        futures.push_back(promise.get_future());
    }

    StompConfig config = StompConfig::optimized(); // Use optimized STOMP configuration
    config.maxComputeTimeMs = 0.0; // Remove time limit
    // Simple straight-line STOMP optimization

    boost::asio::dispatch(
        *threadPool,
        [this,
         &arms,
         firstValidIndex,
         &promises,
         taskIndex = taskIndex++,
         threadPool,
         config]() {
            try {
                auto motionGen = new MotionGenerator(*_arm);
                motionGen->setObstacleTree(_obstacleTree);

                // Simple straight-line STOMP optimization
                Eigen::MatrixXd waypoints(2, _currentJoints.size());
                waypoints.row(0) = _currentJoints.transpose();
                waypoints.row(1) = arms[firstValidIndex].getJointAngles().transpose();
                motionGen->setWaypoints(waypoints);
                
                // Run STOMP optimization with straight-line initialization
                motionGen->performSTOMP(config, threadPool);
                LOG_INFO << "STOMP optimization succeeded for initial repositioning";

                auto result = std::make_pair(motionGen->getPath(), false);
                delete motionGen;
                promises[taskIndex].set_value(result);
            } catch (...) {
                promises[taskIndex].set_exception(std::current_exception());
            }
        });

    for (size_t segIdx = 0; segIdx < validSegments.size(); segIdx++) {
        size_t start = validSegments[segIdx].first;
        size_t end = validSegments[segIdx].second;

        boost::asio::post(
            *threadPool,
            [this, &arms, start, end, &promises, taskIndex = taskIndex++, threadPool, config]() {
                try {
                    auto motionGen = new MotionGenerator(*_arm);
                    motionGen->setObstacleTree(_obstacleTree);

                    std::vector<Eigen::VectorXd> jointCheckpoints;
                    for (size_t i = start; i <= end; i++) {
                        jointCheckpoints.push_back(arms[i].getJointAngles());
                    }

                    auto trajectoryPoints = motionGen->generateTrajectoryFromCheckpoints(
                        jointCheckpoints);

                    auto result = std::make_pair(trajectoryPoints, true);
                    delete motionGen;
                    promises[taskIndex].set_value(result);
                } catch (...) {
                    promises[taskIndex].set_exception(std::current_exception());
                }
            });

        if (segIdx + 1 < validSegments.size()) {
            size_t nextStart = validSegments[segIdx + 1].first;

            boost::asio::post(
                *threadPool,
                [this,
                 &arms,
                 end,
                 nextStart,
                 &promises,
                 taskIndex = taskIndex++,
                 threadPool,
                 config]() {
                    try {
                        auto motionGen = new MotionGenerator(*_arm);
                        motionGen->setObstacleTree(_obstacleTree);

                        // Simple straight-line STOMP optimization
                        Eigen::MatrixXd waypoints(2, _currentJoints.size());
                        waypoints.row(0) = arms[end].getJointAngles().transpose();
                        waypoints.row(1) = arms[nextStart].getJointAngles().transpose();
                        motionGen->setWaypoints(waypoints);
                        
                        // Run STOMP optimization with straight-line initialization
                        motionGen->performSTOMP(config, threadPool);
                        LOG_INFO << "STOMP optimization succeeded for inter-segment repositioning";

                        auto result = std::make_pair(motionGen->getPath(), false);
                        delete motionGen;
                        promises[taskIndex].set_value(result);
                    } catch (...) {
                        promises[taskIndex].set_exception(std::current_exception());
                    }
                });
        }
    }

    _trajectories.reserve(futures.size());
    bool repositioningFailed = false;

    // Collect trajectories in order and enforce continuity
    std::vector<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>> sequentialTrajectories;
    Eigen::VectorXd lastEndPosition;
    bool hasLastEndPosition = false;

    for (size_t i = 0; i < futures.size(); ++i) {
        try {
            auto result = futures[i].get();
            auto trajectory = result.first;
            bool isContactForce = result.second;

            sequentialTrajectories.push_back(result);
        } catch (const std::exception &e) {
            std::string errorMsg = e.what();
            if (errorMsg.find("STOMP") != std::string::npos) {
                LOG_WARNING << "STOMP planning failed at trajectory " << i << ": " << e.what();
            } else {
                LOG_WARNING << "Unknown planning failure at trajectory " << i << ": " << e.what();
            }
            LOG_INFO << "Discarding " << (futures.size() - i) << " remaining trajectories";
            repositioningFailed = true;
            break;
        }
    }

    _trajectories = sequentialTrajectories;

    threadPool->join();
 
    return !repositioningFailed;
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
        const auto& config = _hardwareConfig;
        
        // Create thread pool with optimal core usage
        // Use physical cores for computational work, reserve logical cores for I/O
        size_t computeThreads = std::max(1u, config.physicalCores);
        
        // For STOMP's parallel trajectory generation, we want sufficient threads
        // but not so many that we overwhelm the scheduler
        size_t optimalThreads = std::min(computeThreads, 
                                       static_cast<size_t>(config.batchSize));
        
        _sharedThreadPool = std::make_shared<boost::asio::thread_pool>(optimalThreads);
        
        // Pre-warm the thread pool
        for (size_t i = 0; i < optimalThreads; ++i) {
            boost::asio::post(*_sharedThreadPool, []() {
                // Warm-up task - helps with thread initialization overhead
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            });
        }
    }
}

std::vector<Trajectory> UltrasoundScanTrajectoryPlanner::planTrajectoryBatch(
    const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& requests)
{
    if (requests.empty()) {
        LOG_WARNING << "Batch planning requested with empty trajectory list";
        return {};
    }
    
    // Log batch start with configuration details
    auto startTime = std::chrono::high_resolution_clock::now();
    LOG_INFO << "Starting trajectory batch planning - " << requests.size() << " trajectories";
    LOG_INFO << "Hardware config: " << _hardwareConfig.physicalCores << " physical cores, " 
             << _hardwareConfig.logicalCores << " logical cores, batch size: " 
             << _hardwareConfig.batchSize;
    
    // Ensure thread pool is initialized
    initializeThreadPool();
    
    std::vector<Trajectory> results(requests.size());
    std::vector<std::future<void>> futures;
    futures.reserve(requests.size());
    
    // Batch size for memory efficiency and cache optimization
    const size_t batchSize = _hardwareConfig.batchSize;
    std::atomic<size_t> completedTrajectories{0};
    std::atomic<size_t> successfulTrajectories{0};
    
    for (size_t i = 0; i < requests.size(); i += batchSize) {
        size_t endIdx = std::min(i + batchSize, requests.size());
        LOG_DEBUG << "Processing batch " << (i / batchSize + 1) << " (trajectories " 
                  << i << "-" << (endIdx - 1) << ")";
        
        // Process batch
        for (size_t j = i; j < endIdx; ++j) {
            auto future = boost::asio::post(*_sharedThreadPool, 
                boost::asio::use_future([this, j, &requests, &results, &completedTrajectories, &successfulTrajectories]() {
                    try {
                        // Each thread gets its own motion generator to avoid contention
                        MotionGenerator localGenerator(*_arm);
                        
                        // Use optimized STOMP configuration
                        auto config = StompConfig::optimized();
                        config.maxComputeTimeMs = 0.0; // Remove time limit
                        
                        // Set obstacle tree if available
                        if (_obstacleTree) {
                            localGenerator.setObstacleTree(_obstacleTree);
                        }
                        
                        // Set start and target joints for the trajectory
                        const auto& startJoints = requests[j].first;
                        const auto& targetJoints = requests[j].second;
                        
                        // Create waypoints matrix as expected by MotionGenerator
                        Eigen::MatrixXd waypoints(2, startJoints.size());
                        waypoints.row(0) = startJoints.transpose();
                        waypoints.row(1) = targetJoints.transpose();
                        
                        localGenerator.setWaypoints(waypoints);
                        
                        // Create a thread pool with optimal cores for STOMP's internal parallelization
                        // STOMP benefits from multiple threads for its noisy trajectory generation
                        size_t stompThreads = std::max(2u, _hardwareConfig.physicalCores / 2);
                        auto localThreadPool = std::make_shared<boost::asio::thread_pool>(stompThreads);
                        
                        // Perform STOMP planning with trajectory index for context
                        bool success = localGenerator.performSTOMP(config, localThreadPool, static_cast<int>(j));
                        localThreadPool->join();
                        
                        if (success) {
                            results[j] = localGenerator.getPath();
                            successfulTrajectories++;
                        } else {
                            LOG_WARNING << "[Traj " << j << "] STOMP planning failed";
                        }
                        completedTrajectories++;
                        
                    } catch (const std::exception& e) {
                        // Log error but continue with other trajectories
                        LOG_ERROR << "[Traj " << j << "] Batch planning exception: " << e.what();
                        completedTrajectories++;
                        // results[j] remains default-constructed (empty)
                    }
                }));
            
            futures.push_back(std::move(future));
        }
        
        // Optional: Add small delay between batches to prevent CPU saturation
        if (endIdx < requests.size()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
    
    // Wait for all trajectories to complete
    for (auto& future : futures) {
        try {
            future.get();
        } catch (const std::exception& e) {
            LOG_ERROR << "Future completion error: " << e.what();
        }
    }
    
    // Calculate timing and log completion statistics
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    LOG_INFO << "Trajectory batch planning completed:";
    LOG_INFO << "  Total trajectories: " << requests.size();
    LOG_INFO << "  Successful: " << successfulTrajectories << " (" 
             << std::fixed << std::setprecision(1) 
             << (100.0 * successfulTrajectories / requests.size()) << "%)";
    LOG_INFO << "  Failed: " << (requests.size() - successfulTrajectories);
    LOG_INFO << "  Duration: " << duration.count() << "ms";
    LOG_INFO << "  Average: " << std::fixed << std::setprecision(2) 
             << (duration.count() / static_cast<double>(requests.size())) << "ms per trajectory";

    return results;
}
