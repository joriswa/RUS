#include "USLib/USTrajectoryPlanner.h"

UltrasoundScanTrajectoryPlanner::UltrasoundScanTrajectoryPlanner(const std::string &robot_urdf)
{
    // Check if the URDF file exists
    std::ifstream urdf_file(robot_urdf.c_str());
    if (!urdf_file.good()) {
        throw std::runtime_error("Robot URDF file not found: " + robot_urdf);
    }
    urdf_file.close();

    _arm = new RobotArm(robot_urdf);
    _pathPlanner = new PathPlanner();
    _pathPlanner->setStartPose(*_arm);
    _motionGenerator = new MotionGenerator(*_arm);
}

UltrasoundScanTrajectoryPlanner::~UltrasoundScanTrajectoryPlanner()
{
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

        Eigen::MatrixXd waypoints(2, startJoints.size());
        waypoints.row(0) = startJoints.transpose();
        waypoints.row(1) = targetJoints.transpose();

        motionGen->setWaypoints(waypoints);

        unsigned int numThreads = std::thread::hardware_concurrency();
        auto threadPool = std::make_shared<boost::asio::thread_pool>(numThreads);

        motionGen->performSTOMP(config, threadPool);
        threadPool->join();

        auto result = std::make_pair(motionGen->getPath(), false);
        delete motionGen;
        return result;
    } catch (const std::exception &e) {
        qDebug() << "Exception in planSingleStompTrajectory: " << e.what();
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
    qDebug() << "num threads " << numThreads;
    std::shared_ptr<boost::asio::thread_pool> threadPool
        = std::make_shared<boost::asio::thread_pool>(2 * numThreads);

    auto checkpointResult = _pathPlanner->planCheckpoints(_poses, _currentJoints);
    auto &checkpoints = checkpointResult.checkpoints;
    auto &validSegments = checkpointResult.validSegments;
    auto &jumpPairs = checkpointResult.jumpPairs;
    size_t firstValidIndex = checkpointResult.firstValidIndex;

    // print the indices of the valid segments
    qDebug() << "Valid segments:";
    for (const auto &segment : validSegments) {
        qDebug() << "Start:" << segment.first << "End:" << segment.second;
    }

    if (checkpoints.size() == 1) {
        auto [arm, valid] = checkpoints[0];

        if (!valid) {
            throw std::runtime_error("Single checkpoint is invalid.");
        }

        StompConfig config;
        auto trajectory = planSingleStompTrajectory(_currentJoints, arm.getJointAngles(), config);
        _trajectories.push_back(trajectory);
        return true;
    }

    // Validate that we have a valid starting point
    if (firstValidIndex >= checkpoints.size()) {
        throw std::runtime_error("No valid checkpoint found to start trajectory planning.");
    }
    if (firstValidIndex > 0) {
        qDebug() << "First" << firstValidIndex
                 << "checkpoint(s) are invalid. Using first valid checkpoint at index:"
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

    StompConfig config;
    bool useHauserForRepositioning = false; // TODO: Make this configurable

    boost::asio::post(
        *threadPool,
        [this,
         &arms,
         firstValidIndex,
         &promises,
         taskIndex = taskIndex++,
         threadPool,
         config,
         useHauserForRepositioning]() {
            try {
                auto motionGen = new MotionGenerator(*_arm);
                motionGen->setObstacleTree(_obstacleTree);

                Eigen::MatrixXd waypoints(2, _currentJoints.size());
                waypoints.row(0) = _currentJoints.transpose();
                waypoints.row(1) = arms[firstValidIndex].getJointAngles().transpose();

                motionGen->setWaypoints(waypoints);

                if (useHauserForRepositioning) {
                    auto threadPathPlanner = std::make_unique<PathPlanner>();
                    RobotArm startArm = *_arm;
                    startArm.setJointAngles(_currentJoints);
                    threadPathPlanner->setStartPose(startArm);
                    threadPathPlanner->setObstacleTree(_obstacleTree);

                    RobotArm goalArm = *_arm;
                    goalArm.setJointAngles(arms[firstValidIndex].getJointAngles());
                    threadPathPlanner->setGoalConfiguration(goalArm);

                    bool pathFound = threadPathPlanner->runPathFinding();

                    if (pathFound) {
                        Eigen::MatrixXd geometricPath = threadPathPlanner->getAnglesPath();
                        motionGen->setWaypoints(geometricPath);
                        motionGen->performHauser(300, "", 100);
                    } else {
                        qDebug()
                            << "BiRRT failed for repositioning, terminating trajectory planning";
                        delete motionGen;
                        promises[taskIndex].set_exception(std::make_exception_ptr(std::runtime_error(
                            "Hauser path planning failed for initial repositioning")));
                        return;
                    }
                } else {
                    try {
                        motionGen->performSTOMP(config, threadPool);
                    } catch (const std::exception &e) {
                        qDebug() << "STOMP failed for repositioning:" << e.what();
                        delete motionGen;
                        promises[taskIndex].set_exception(std::make_exception_ptr(std::runtime_error(
                            "STOMP trajectory planning failed for repositioning: "
                            + std::string(e.what()))));
                        return;
                    }
                }

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
                 config,
                 useHauserForRepositioning]() {
                    try {
                        auto motionGen = new MotionGenerator(*_arm);
                        motionGen->setObstacleTree(_obstacleTree);

                        Eigen::MatrixXd waypoints(2, _currentJoints.size());
                        waypoints.row(0) = arms[end].getJointAngles().transpose();
                        waypoints.row(1) = arms[nextStart].getJointAngles().transpose();

                        motionGen->setWaypoints(waypoints);

                        if (useHauserForRepositioning) {
                            
                            auto threadPathPlanner = std::make_unique<PathPlanner>();
                            RobotArm startArm = *_arm;
                            startArm.setJointAngles(arms[end].getJointAngles());
                            threadPathPlanner->setStartPose(startArm);
                            threadPathPlanner->setObstacleTree(_obstacleTree);

                            RobotArm goalArm = *_arm;
                            goalArm.setJointAngles(arms[nextStart].getJointAngles());
                            threadPathPlanner->setGoalConfiguration(goalArm);

                            bool pathFound = threadPathPlanner->runPathFinding();

                            if (pathFound) {
                                Eigen::MatrixXd geometricPath = threadPathPlanner->getAnglesPath();
                                motionGen->setWaypoints(geometricPath);
                                motionGen->performHauser(300, "", 100);
                            } else {
                                qDebug() << "BiRRT failed for inter-segment repositioning, "
                                            "terminating trajectory planning";
                                delete motionGen;
                                promises[taskIndex].set_exception(std::make_exception_ptr(
                                    std::runtime_error("Hauser path planning failed for "
                                                       "inter-segment repositioning")));
                                return;
                            }
                        } else {
                            motionGen->performSTOMP(config, threadPool);
                        }

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

            // Check for trajectory continuity if we have a previous endpoint
            if (hasLastEndPosition && !trajectory.empty()) {
                // Calculate the discrepancy between expected start and actual start
                Eigen::VectorXd actualStart(trajectory[0].position.size());
                for (size_t j = 0; j < trajectory[0].position.size(); ++j) {
                    actualStart[j] = trajectory[0].position[j];
                }
                
                double maxError = 0.0;
                for (int j = 0; j < lastEndPosition.size(); ++j) {
                    double error = std::abs(actualStart[j] - lastEndPosition[j]);
                    maxError = std::max(maxError, error);
                }
                
                if (maxError > 1e-4) {
                    qDebug() << "TRAJECTORY CONTINUITY ERROR detected at trajectory" << i;
                    qDebug() << "Maximum joint position error:" << maxError;
                    qDebug() << "This confirms the trajectory chaining problem you identified!";
                    
                    // Log per-joint errors for debugging
                    for (int j = 0; j < lastEndPosition.size(); ++j) {
                        double error = std::abs(actualStart[j] - lastEndPosition[j]);
                        if (error > 1e-6) {
                            qDebug() << "Joint" << j << "error:" << error;
                        }
                    }
                }
            }

            // Update the last end position for next trajectory
            if (!trajectory.empty()) {
                lastEndPosition.resize(trajectory.back().position.size());
                for (size_t j = 0; j < trajectory.back().position.size(); ++j) {
                    lastEndPosition[j] = trajectory.back().position[j];
                }
                hasLastEndPosition = true;
            }

            sequentialTrajectories.push_back(result);
        } catch (const std::exception &e) {
            std::string errorMsg = e.what();
            if (errorMsg.find("Hauser") != std::string::npos) {
                qDebug() << "Hauser planning failed at trajectory" << i;
            } else if (errorMsg.find("STOMP") != std::string::npos) {
                qDebug() << "STOMP planning failed at trajectory" << i;
            } else {
                qDebug() << "Unknown planning failure at trajectory" << i;
            }
            qDebug() << "Error:" << e.what();
            qDebug() << "Discarding" << (futures.size() - i) << "remaining trajectories";
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
