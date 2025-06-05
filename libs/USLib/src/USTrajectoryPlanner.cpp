#include "USLib/USTrajectoryPlanner.h"

UltrasoundScanTrajectoryPlanner::UltrasoundScanTrajectoryPlanner(const std::string &robot_urdf)
{
    // Check if the URDF file exists
    std::ifstream urdf_file(robot_urdf.c_str());
    if (!urdf_file.good())
    {
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
    if (_arm)
    {
        delete _arm;
        _arm = nullptr;
    }

    if (_motionGenerator)
    {
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
    if (!environment.empty())
    {
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

    for (const auto &pose : poses)
    {
        Eigen::Affine3d modified_pose = pose;

        const Eigen::Vector3d
            local_translation(0.0,
                              0.0,
                              // -0.02); // Derived emperically from the error in the realsense camera
                              0.0);
        const Eigen::Vector3d world_translation =
            modified_pose.rotation() * local_translation;

        modified_pose.translation() += world_translation;

        _poses.push_back(modified_pose);
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
    auto& checkpoints = checkpointResult.checkpoints;
    auto& validSegments = checkpointResult.validSegments; 
    auto& jumpPairs = checkpointResult.jumpPairs;
    size_t firstValidIndex = checkpointResult.firstValidIndex;

    if (checkpoints.size() == 1) {
        auto [arm, valid] = checkpoints[0];

        if (!valid) {
            throw std::runtime_error("Single checkpoint is invalid.");
        }

        StompConfig config;
        // Plan a direct trajectory from current joints to the single pose
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

    // Extract arms from checkpoints for trajectory planning
    std::vector<RobotArm> arms;
    for (const auto& [arm, valid] : checkpoints) {
        arms.push_back(arm);
    }

    // Prepare for parallel execution
    std::vector<std::promise<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>>>
        promises;
    std::vector<std::future<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>>> futures;
    size_t taskIndex = 0;

    // Reserve space for all tasks
    size_t numTasks = 1 + validSegments.size()
                      + (validSegments.size() > 1 ? validSegments.size() - 1 : 0);
    promises.resize(numTasks);
    futures.reserve(numTasks);

    for (auto &promise : promises) {
        futures.push_back(promise.get_future());
    }

    // Initial trajectory: current position to first valid checkpoint
    StompConfig config;
    boost::asio::post(
        *threadPool,
        [this, &arms, firstValidIndex, &promises, taskIndex = taskIndex++, threadPool, config]() {
            try {
                auto motionGen = new MotionGenerator(*_arm);
                motionGen->setObstacleTree(_obstacleTree);

                Eigen::MatrixXd waypoints(2, _currentJoints.size());
                waypoints.row(0) = _currentJoints.transpose();
                waypoints.row(1) = arms[firstValidIndex].getJointAngles().transpose();

                motionGen->setWaypoints(waypoints);
                motionGen->performSTOMP(config, threadPool);

                auto result = std::make_pair(motionGen->getPath(), false);
                delete motionGen;
                promises[taskIndex].set_value(result);
            } catch (...) {
                promises[taskIndex].set_exception(std::current_exception());
            }
        });

    // Process segments
    for (size_t segIdx = 0; segIdx < validSegments.size(); segIdx++) {
        size_t start = validSegments[segIdx].first;
        size_t end = validSegments[segIdx].second;

        boost::asio::post(
            *threadPool,
            [this, &arms, start, end, &promises, taskIndex = taskIndex++, threadPool, config]() {
                try {
                    auto motionGen = new MotionGenerator(*_arm);
                    motionGen->setObstacleTree(_obstacleTree);

                    std::vector<Eigen::VectorXd> segment;
                    for (size_t i = start; i <= end; i++) {
                        segment.push_back(arms[i].getJointAngles());
                    }

                    motionGen->performSTOMPWithCheckpoints(segment, {}, config, threadPool);
                    auto result = std::make_pair(motionGen->getPath(), true);
                    delete motionGen;
                    promises[taskIndex].set_value(result);
                } catch (...) {
                    promises[taskIndex].set_exception(std::current_exception());
                }
            });

        if (segIdx + 1 < validSegments.size()) {
            size_t nextStart = validSegments[segIdx + 1].first;

            boost::asio::post(*threadPool,
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

                                      Eigen::MatrixXd waypoints(2, _currentJoints.size());
                                      waypoints.row(0) = arms[end].getJointAngles().transpose();
                                      waypoints.row(1) = arms[nextStart].getJointAngles().transpose();

                                      motionGen->setWaypoints(waypoints);
                                      motionGen->performSTOMP(config, threadPool);

                                      auto result = std::make_pair(motionGen->getPath(), false);
                                      delete motionGen;
                                      promises[taskIndex].set_value(result);
                                  } catch (...) {
                                      promises[taskIndex].set_exception(std::current_exception());
                                  }
                              });
        }
    }

    // Wait for all tasks to complete and collect results
    _trajectories.reserve(futures.size());
    for (auto &future : futures) {
        _trajectories.push_back(future.get());
    }

    // Ensure all tasks are completed before destroying the thread pool
    threadPool->join();

    return true;
}

std::vector<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>>
UltrasoundScanTrajectoryPlanner::getTrajectories()
{

    return _trajectories;
}

// write a getter for the scanPoses
std::vector<Eigen::Affine3d> UltrasoundScanTrajectoryPlanner::getScanPoses() const
{
    return _poses;
}
