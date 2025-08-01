#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Robot/franka_ik_He.h"
#include <atomic>
#include <cmath>
#include <future>
bool MotionGenerator::armHasCollision(std::vector<double> jointPositions)
{
    RobotArm temp = _arm;
    Eigen::Matrix<double, 7, 1> angles = Eigen::Map<Eigen::Matrix<double, 7, 1>>(
        jointPositions.data());
    temp.setJointAngles(angles);
    return armHasCollision(temp);
}
bool MotionGenerator::armHasCollision(RobotArm &arm)
{
    auto start = std::chrono::high_resolution_clock::now();
    auto bBoxes = arm.getCollisionBoxes();
    int i = 0;
    for (const auto &bBox : bBoxes) {
        ++i;
        if (i < 2)
            continue;
        auto [center, halfDims, axes] = bBox;
        if (_obstacleTree->isBoxIntersecting(center, halfDims, axes)) {
            return true;
        }
    }
    return false;
}
bool MotionGenerator::isSdfInitialized() const
{
    return _sdfInitialized;
}
const std::vector<std::vector<std::vector<double>>> &MotionGenerator::getSdf() const
{
    return _sdf;
}
const Eigen::Vector3d &MotionGenerator::getSdfMinPoint() const
{
    return _sdfMinPoint;
}
const Eigen::Vector3d &MotionGenerator::getSdfMaxPoint() const
{
    return _sdfMaxPoint;
}
double MotionGenerator::getSdfResolution() const
{
    return _sdfResolution;
}
ArmFeasibilityChecker::ArmFeasibilityChecker(RobotArm arm,
                                             const std::shared_ptr<BVHTree> &obstacleTree)
    : _arm(arm)
    , _obstacleTree(obstacleTree)
{}
bool ArmFeasibilityChecker::ConfigFeasible(const ParabolicRamp::Vector &x)
{
    Eigen::Matrix<double, 7, 1> jointAngles;
    for (int i = 0; i < 7; ++i) {
        jointAngles[i] = x[i];
    }
    RobotArm temp = _arm;
    temp.setJointAngles(jointAngles);
    auto bBoxes = temp.getCollisionBoxes();
    int i = 0;
    for (const auto &bBox : bBoxes) {
        ++i;
        if (i < 2)
            continue;
        auto [center, halfDims, axes] = bBox;
        if (_obstacleTree->isBoxIntersecting(center, halfDims, axes)) {
            return false;
        }
    }
    return true;
}
bool ArmFeasibilityChecker::SegmentFeasible(const ParabolicRamp::Vector &a,
                                            const ParabolicRamp::Vector &b)
{
    if (!ConfigFeasible(a) || !ConfigFeasible(b)) {
        return false;
    }
    const int numSteps = 100;
    ParabolicRamp::Vector q(a.size());
    for (int step = 1; step < numSteps; ++step) {
        double t = static_cast<double>(step) / numSteps;
        for (size_t i = 0; i < a.size(); ++i) {
            q[i] = a[i] + t * (b[i] - a[i]);
        }
        if (!ConfigFeasible(q)) {
            return false;
        }
    }
    return true;
}
MotionGenerator::MotionGenerator(RobotArm arm)
{
    _arm = arm;
}
MotionGenerator::MotionGenerator(RobotArm arm,
                                 const std::vector<std::vector<std::vector<double>>> &sdf,
                                 const Eigen::Vector3d &sdfMinPoint,
                                 const Eigen::Vector3d &sdfMaxPoint,
                                 double sdfResolution,
                                 std::shared_ptr<BVHTree> obstacleTree)
{
    _arm = arm;
    _sdf = sdf;
    _sdfMinPoint = sdfMinPoint;
    _sdfMaxPoint = sdfMaxPoint;
    _sdfResolution = sdfResolution;
    _obstacleTree = obstacleTree;
    _sdfInitialized = true;
}
void MotionGenerator::setWaypoints(const Eigen::MatrixXd &waypoints)
{
    _waypoints = waypoints;
    _arm.setJointAngles(waypoints.row(0));
}
void MotionGenerator::convertToTimeOptimal()
{
    std::vector<TrajectoryPoint> timeOptimalPath;
    double curTime = 0.0;
    for (size_t i = 0; i < _path.size() - 1; ++i) {
        auto &start = _path[i];
        auto &end = _path[i + 1];
        std::vector<TrajectoryPoint> segmentPath = computeTimeOptimalSegment(start, end, curTime);
        timeOptimalPath.insert(timeOptimalPath.end(), segmentPath.begin(), segmentPath.end() - 1);
        curTime = segmentPath.back().time;
    }
    _path = std::move(timeOptimalPath);
}
std::vector<MotionGenerator::TrajectoryPoint> MotionGenerator::computeTimeOptimalSegment(
    const TrajectoryPoint &start, const TrajectoryPoint &end, double startTime)
{
    std::vector<TrajectoryPoint> segment;
    double v_max = std::numeric_limits<double>::max();
    double a_max = std::numeric_limits<double>::max();
    for (size_t k = 0; k < _numJoints; ++k) {
        double diff = std::abs(end.position[k] - start.position[k]);
        if (diff > 0) {
            v_max = std::min(v_max, _maxJointVelocities[k]);
            a_max = std::min(a_max, _maxJointAccelerations[k]);
        }
    }
    double total_distance = 0;
    for (size_t k = 0; k < _numJoints; ++k) {
        total_distance = std::max(total_distance, std::abs(end.position[k] - start.position[k]));
    }
    double t_acc = v_max / a_max;
    double d_acc = 0.5 * a_max * t_acc * t_acc;
    double d_const = total_distance - 2 * d_acc;
    double total_time;
    if (d_const > 0) {
        double t_const = d_const / v_max;
        total_time = 2 * t_acc + t_const;
    } else {
        t_acc = std::sqrt(total_distance / a_max);
        total_time = 2 * t_acc;
    }
    int numPoints = static_cast<int>(total_time / 0.05);
    for (int i = 0; i <= numPoints; ++i) {
        double t = static_cast<double>(i) / numPoints * total_time;
        TrajectoryPoint point;
        point.time = startTime + t;
        point.position.resize(_numJoints);
        point.velocity.resize(_numJoints);
        point.acceleration.resize(_numJoints);
        for (size_t k = 0; k < _numJoints; ++k) {
            double diff = end.position[k] - start.position[k];
            double direction = (diff > 0) ? 1.0 : -1.0;
            double abs_diff = std::abs(diff);
            double s, v, a;
            if (t <= t_acc) {
                s = 0.5 * a_max * t * t;
                v = a_max * t;
                a = a_max;
            } else if (t <= total_time - t_acc) {
                s = d_acc + v_max * (t - t_acc);
                v = v_max;
            } else {
                double t_dec = total_time - t;
                s = total_distance - 0.5 * a_max * t_dec * t_dec;
                v = a_max * t_dec;
                a = a_max;
            }
            s = (s / total_distance) * abs_diff;
            v = (v / total_distance) * abs_diff;
            a = (a / total_distance) * abs_diff;
            point.position[k] = start.position[k] + direction * s;
            point.velocity[k] = direction * v;
            point.acceleration[k] = direction * a;
        }
        segment.push_back(point);
    }
    return segment;
}
void MotionGenerator::generateInitialTrajectory()
{
    _path.clear();
    for (int i = 0; i < _waypoints.rows(); ++i) {
        TrajectoryPoint point;
        point.position = std::vector<double>(_numJoints, 0.0);
        for (int k = 0; k < _numJoints; k++) {
            point.position[k] = _waypoints.row(i)[k];
        }
        point.velocity = std::vector<double>(_numJoints, 0.0);
        point.acceleration = std::vector<double>(_numJoints, 0.0);
        point.time = i;
        _path.push_back(point);
    }
}
void MotionGenerator::performHauser(unsigned int maxIterations,
                                    const std::string &out,
                                    unsigned int outputFrequency)
{
    _path.clear();
    ParabolicRamp::DynamicPath traj;
    traj.Init(_maxJointVelocities, _maxJointAccelerations);
    std::vector<std::vector<double>> milestones;
    for (int i = 0; i < _waypoints.rows(); ++i) {
        Eigen::Matrix<double, 7, 1> jointAngles = _waypoints.row(i);
        std::vector<double> jointAnglesVector(jointAngles.data(),
                                              jointAngles.data() + jointAngles.size());
        milestones.push_back(jointAnglesVector);
    }
    traj.SetMilestones(milestones);
    ArmFeasibilityChecker checker(_arm, _obstacleTree);
    ParabolicRamp::RampFeasibilityChecker feasibilityChecker(&checker, 1e-3);
    int res = traj.Shortcut(maxIterations, feasibilityChecker, out);
    double time = 0.0;
    for (double t = 0.0; t <= traj.GetTotalTime(); t += 0.01) {
        std::vector<double> out;
        traj.Evaluate(t, out);
        for (int i = 0; i < out.size(); ++i) {
        }
        TrajectoryPoint newPoint;
        newPoint.position = out;
        newPoint.velocity = std::vector<double>(_numJoints, 0.0);
        newPoint.time = t;
        newPoint.acceleration = std::vector<double>(_numJoints, 0.0);
        _path.push_back(newPoint);
        time += 0.01;
    }
}
void MotionGenerator::setObstacleTree(const std::shared_ptr<BVHTree> &newObstacleTree)
{
    _obstacleTree = newObstacleTree;
}
void MotionGenerator::createSDF()
{
    if (_sdfInitialized) {
        return;
    }
    Eigen::Vector3d minPoint(-1.4, -1.4, -0.5);
    Eigen::Vector3d maxPoint(1.4, 1.4, 1.4);
    double resolution = 0.005;
    _sdfMinPoint = minPoint;
    _sdfMaxPoint = maxPoint;
    _sdfResolution = resolution;
    _sdf = _obstacleTree->toSDF(minPoint, maxPoint, resolution);
    _sdfInitialized = true;
}

void MotionGenerator::logInitialTrajectoryState(const Eigen::MatrixXd &theta,
                                                int N,
                                                double dt,
                                                const StompConfig &config)
{
    // Simplified initial state logging - only log if there are issues
    bool initialCollisionFree = !checkCollisions(theta, N);
    if (!initialCollisionFree) {
        LOG_DEBUG << "Initial trajectory has collisions";
    }
}

void MotionGenerator::updateBestValidSolution(const std::vector<Eigen::MatrixXd> &noisyTrajectories,
                                              const std::vector<double> &costs,
                                              STOMPConvergenceState &convergenceState,
                                              int iteration,
                                              int N,
                                              double dt)
{
    int validSamples = 0;
    double bestCost = std::numeric_limits<double>::max();
    int bestIndex = -1;

    for (size_t k = 0; k < noisyTrajectories.size(); ++k) {
        bool isCollisionFree = !checkCollisions(noisyTrajectories[k], N);
        bool isConstraintValid = isTrajectoryValidWithMargin(noisyTrajectories[k], dt, 1., 1.);
        bool isValid = isCollisionFree && isConstraintValid;

        if (isValid) {
            validSamples++;
            if (costs[k] < bestCost) {
                bestCost = costs[k];
                bestIndex = k;
            }
        }
    }

    if (bestIndex >= 0 && bestCost < convergenceState.bestValidCost) {
        convergenceState.success = true;
        convergenceState.bestValidCost = bestCost;
        convergenceState.bestValidTheta = noisyTrajectories[bestIndex];
        LOG_DEBUG << "New best valid solution found (cost: " << std::fixed << std::setprecision(4)
                  << bestCost << ")";
    }
}

void MotionGenerator::updateOptimizationState(const Eigen::MatrixXd &theta,
                                              STOMPConvergenceState &convergenceState,
                                              const StompConfig &config,
                                              int iteration,
                                              double dt,
                                              const Eigen::MatrixXd &previousTheta)
{
    int N = theta.rows();
    bool collisionFree = !checkCollisions(theta, N);
    bool constraintsValid = isTrajectoryValidWithMargin(theta, dt, 1., 1.);
    bool trajectoryValid = collisionFree && constraintsValid;

    // Check if weighted-average theta is better than our best valid sample
    if (trajectoryValid) {
        double thetaCost = _costCalculator->computeCost(theta, dt);
        if (thetaCost < convergenceState.bestValidCost) {
            convergenceState.success = true;
            convergenceState.bestValidCost = thetaCost;
            convergenceState.bestValidTheta = theta;
            LOG_DEBUG << "Weighted-average theta is new best (cost: " << std::fixed
                      << std::setprecision(4) << thetaCost << ")";
        }
    }

    // Update convergence counters
    double trajectoryCost = _costCalculator->computeCost(theta, dt);
    double costDiff = std::abs(trajectoryCost - convergenceState.prevTrajectoryCost);
    convergenceState.prevTrajectoryCost = trajectoryCost;

    if (trajectoryValid && config.enableEarlyStopping) {
        convergenceState.earlyStoppingCounter++;
    } else if (config.enableEarlyStopping) {
        convergenceState.earlyStoppingCounter = 0;
    }

    if (costDiff < STOMPConvergenceState::costConvergenceThreshold) {
        convergenceState.noChangeCounter++;
    } else {
        convergenceState.noChangeCounter = 0;
    }

    double trajectoryChange = (theta - previousTheta).norm();
}

bool MotionGenerator::performSTOMP(const StompConfig &config,
                                   std::shared_ptr<boost::asio::thread_pool> sharedPool,
                                   int trajectoryIndex)
{
    bool wasDebugEnabled = Logger::instance().isLevelEnabled(LogLevel::DEBUG);
    Logger::instance().setDebugEnabled(true);

    QElapsedTimer timer;
    timer.start();
    LOG_INFO << "STOMP optimization: " << config.numNoisyTrajectories << " samples, "
             << config.maxIterations << " max iterations";

    auto initData = initializeSTOMPExecution(config, sharedPool);
    auto &theta = initData.theta;
    auto *pool = initData.pool;
    int N = initData.N;
    double dt = initData.dt;
    auto &limits = initData.limits;

    logInitialTrajectoryState(theta, N, dt, config);

    STOMPConvergenceState convergenceState;
    std::vector<std::pair<Eigen::MatrixXd, double>> bestSamples;

    // Main optimization loop - clean and focused
    for (int iteration = 0; iteration < config.maxIterations; iteration++) {
        if (checkTimeLimit(config,
                           timer,
                           iteration,
                           convergenceState.success,
                           convergenceState.bestValidCost)) {
            break;
        }

        // Core STOMP algorithm
        auto noisyTrajectories = generateNoisySamples(config, theta, limits, bestSamples, pool);
        auto [costs, weights] = evaluateTrajectories(noisyTrajectories, config, dt, pool);

        // Track best valid solution found in this iteration
        updateBestValidSolution(noisyTrajectories, costs, convergenceState, iteration, N, dt);

        // Update trajectory using weighted samples
        updateBestSamples(noisyTrajectories, costs, bestSamples, config.numBestSamples);
        Eigen::MatrixXd previousTheta = theta;
        theta = applyTrajectoryUpdate(theta, noisyTrajectories, weights, config, N, limits);

        // Update optimization state and check convergence
        updateOptimizationState(theta, convergenceState, config, iteration, dt, previousTheta);

        if (checkConvergence(convergenceState, config, iteration)) {
            break;
        }
    }

    bool result = finalizeSTOMPResult(theta, convergenceState, config, N, dt, timer);

    // Restore previous debug state
    Logger::instance().setDebugEnabled(wasDebugEnabled);

    return result;
}
MotionGenerator::STOMPInitData MotionGenerator::initializeSTOMPExecution(
    const StompConfig &config, std::shared_ptr<boost::asio::thread_pool> sharedPool)
{
    _path.clear();
    createSDF();

    STOMPInitData initData;
    if (sharedPool) {
        initData.pool = sharedPool.get();
    } else {
        unsigned int hwThreads = std::thread::hardware_concurrency();
        unsigned int numThreads = hwThreads > 0 ? std::max(2u, hwThreads / 2) : 4;
        initData.localPool = std::make_unique<boost::asio::thread_pool>(numThreads);
        initData.pool = initData.localPool.get();
    }

    Eigen::Matrix<double, 1, Eigen::Dynamic> startVec = _waypoints.row(0);
    Eigen::Matrix<double, 1, Eigen::Dynamic> goalVec = _waypoints.row(_waypoints.rows() - 1);

    // Estimate trajectory time using conservative joint limits
    TrajectoryPoint start, end;
    for (int i = 0; i < config.numJoints; ++i) {
        start.position.push_back(startVec[i]);
        end.position.push_back(goalVec[i]);
        start.velocity.push_back(0.0);
        end.velocity.push_back(0.0);
        start.acceleration.push_back(0.0);
        end.acceleration.push_back(0.0);
    }

    // Temporarily reduce limits for trajectory time estimation
    std::vector<double> originalMaxVel = _maxJointVelocities;
    std::vector<double> originalMaxAcc = _maxJointAccelerations;
    const double conservativeFactor = 0.5;

    for (size_t i = 0; i < _maxJointVelocities.size(); ++i) {
        _maxJointVelocities[i] *= conservativeFactor;
        _maxJointAccelerations[i] *= conservativeFactor;
    }

    auto segment = computeTimeOptimalSegment(start, end, 0.0);
    double estimatedTime = segment.back().time;

    // Restore original limits
    _maxJointVelocities = originalMaxVel;
    _maxJointAccelerations = originalMaxAcc;

    initData.N = config.N;
    initData.dt = estimatedTime / (initData.N - 1);
    initData.theta = initializeTrajectory(goalVec, startVec, initData.N, config.numJoints);
    initializeMatrices(initData.N, initData.dt);
    initializeCostCalculator(config); // Moved here AFTER initializeMatrices
    initData.limits = _arm.jointLimits();

    LOG_DEBUG << "STOMP initialized: N=" << initData.N << ", dt=" << initData.dt
              << "s, estimated_time=" << estimatedTime << "s";

    return initData;
}
std::vector<Eigen::MatrixXd> MotionGenerator::generateNoisySamples(
    const StompConfig &config,
    const Eigen::MatrixXd &theta,
    const std::vector<std::pair<double, double>> &limits,
    const std::vector<std::pair<Eigen::MatrixXd, double>> &bestSamples,
    boost::asio::thread_pool *pool)
{
    int actualBestSamples = std::min(config.numBestSamples, static_cast<int>(bestSamples.size()));
    std::vector<Eigen::MatrixXd> noisyTrajectories(config.numNoisyTrajectories + actualBestSamples);

    if (config.disableInternalParallelization) {
        // Sequential generation
        for (int k = 0; k < config.numNoisyTrajectories; ++k) {
            noisyTrajectories[k] = generateNoisyTrajectory(theta, config.jointStdDevs, limits);
        }
    } else {
        // Parallel generation
        std::vector<std::promise<void>> promises(config.numNoisyTrajectories);
        std::vector<std::future<void>> futures;
        for (auto &promise : promises) {
            futures.push_back(promise.get_future());
        }
        for (int k = 0; k < config.numNoisyTrajectories; ++k) {
            boost::asio::dispatch(
                *pool, [this, k, &noisyTrajectories, &theta, &config, &limits, &promises]() {
                    try {
                        noisyTrajectories[k] = generateNoisyTrajectory(theta,
                                                                       config.jointStdDevs,
                                                                       limits);
                        promises[k].set_value();
                    } catch (...) {
                        promises[k].set_exception(std::current_exception());
                    }
                });
        }
        for (auto &future : futures) {
            future.wait();
        }
    }

    // Add best samples from previous iterations
    for (int b = 0; b < actualBestSamples; b++) {
        noisyTrajectories[config.numNoisyTrajectories + b] = bestSamples[b].first;
    }
    return noisyTrajectories;
}
std::pair<std::vector<double>, Eigen::VectorXd> MotionGenerator::evaluateTrajectories(
    const std::vector<Eigen::MatrixXd> &trajectories,
    const StompConfig &config,
    double dt,
    boost::asio::thread_pool *pool)
{
    int totalSamples = static_cast<int>(trajectories.size());
    std::vector<double> costs(totalSamples);
    Eigen::VectorXd costVector(totalSamples);

    if (config.disableInternalParallelization) {
        // Sequential evaluation
        for (int k = 0; k < totalSamples; k++) {
            double trajectoryCost = _costCalculator->computeCost(trajectories[k], dt);
            costs[k] = trajectoryCost;
            costVector(k) = trajectoryCost;
        }
    } else {
        // Parallel evaluation
        std::vector<std::promise<void>> promises(totalSamples);
        std::vector<std::future<void>> futures;
        for (auto &promise : promises) {
            futures.push_back(promise.get_future());
        }
        for (int k = 0; k < totalSamples; k++) {
            boost::asio::dispatch(*pool,
                                  [this, k, &trajectories, &costs, &costVector, dt, &promises]() {
                                      try {
                                          double trajectoryCost
                                              = _costCalculator->computeCost(trajectories[k], dt);
                                          costs[k] = trajectoryCost;
                                          costVector(k) = trajectoryCost;
                                          promises[k].set_value();
                                      } catch (...) {
                                          promises[k].set_exception(std::current_exception());
                                      }
                                  });
        }
        for (auto &future : futures) {
            future.wait();
        }
    }

    // Calculate weights using softmax
    double minCost = costVector.minCoeff();
    double maxCost = costVector.maxCoeff();
    double costRange = maxCost - minCost;
    Eigen::VectorXd weights(totalSamples);

    if (costRange < 1e-6) {
        weights.setConstant(1.0 / totalSamples);
    } else {
        Eigen::VectorXd expCosts
            = (-config.temperature * (costVector.array() - minCost) / costRange).exp();
        weights = expCosts / expCosts.sum();
    }

    return {costs, weights};
}
void MotionGenerator::updateBestSamples(const std::vector<Eigen::MatrixXd> &trajectories,
                                        const std::vector<double> &costs,
                                        std::vector<std::pair<Eigen::MatrixXd, double>> &bestSamples,
                                        int numBestSamples)
{
    std::vector<std::pair<Eigen::MatrixXd, double>> iterationSamples;
    for (size_t k = 0; k < trajectories.size() && k < costs.size(); k++) {
        iterationSamples.push_back({trajectories[k], costs[k]});
    }
    std::sort(iterationSamples.begin(), iterationSamples.end(), [](const auto &a, const auto &b) {
        return a.second < b.second;
    });
    bestSamples = std::vector<std::pair<Eigen::MatrixXd, double>>(
        iterationSamples.begin(),
        iterationSamples.begin()
            + std::min(numBestSamples, static_cast<int>(iterationSamples.size())));
}
Eigen::MatrixXd MotionGenerator::applyTrajectoryUpdate(
    const Eigen::MatrixXd &theta,
    const std::vector<Eigen::MatrixXd> &noisyTrajectories,
    const Eigen::VectorXd &weights,
    const StompConfig &config,
    int N,
    const std::vector<std::pair<double, double>> &limits)
{
    Eigen::MatrixXd deltaTheta = Eigen::MatrixXd::Zero(N, config.numJoints);
    for (int d = 0; d < config.numJoints; d++) {
        for (int k = 0; k < weights.size(); k++) {
            deltaTheta.col(d) += weights(k) * (noisyTrajectories[k].col(d) - theta.col(d));
        }
    }
    Eigen::MatrixXd deltaS = smoothTrajectoryUpdate(deltaTheta);
    Eigen::MatrixXd updatedTheta = theta + config.learningRate * deltaS;

    // CRITICAL FIX: Only clamp intermediate waypoints (i=1 to N-2)
    // Start waypoint (i=0) and goal waypoint (i=N-1) must remain fixed
    for (int i = 1; i < N - 1; i++) {
        for (int d = 0; d < config.numJoints; d++) {
            updatedTheta(i, d) = std::clamp(updatedTheta(i, d), limits[d].first, limits[d].second);
        }
    }
    return updatedTheta;
}
bool MotionGenerator::checkCollisions(const Eigen::MatrixXd &theta, int N)
{
    std::atomic<bool> collides{false};
    std::atomic<int> collisionTimestep{-1};
    const int numWaypoints = N - 2;
    const int sparseStep = 4;
    std::vector<std::future<void>> sparseFutures;
    for (int i = 1; i < N - 1; i += sparseStep) {
        sparseFutures.emplace_back(
            std::async(std::launch::async, [this, &theta, &collides, &collisionTimestep, i]() {
                // FIXED: Use memory ordering for better thread safety
                if (!collides.load(std::memory_order_acquire)) {
                    RobotArm checkArm = _arm;
                    checkArm.setJointAngles(theta.row(i));
                    if (armHasCollision(checkArm)) {
                        collides.store(true, std::memory_order_release);
                        collisionTimestep.store(i, std::memory_order_release);
                    }
                }
            }));
    }
    for (auto &future : sparseFutures) {
        future.wait();
    }
    if (!collides.load(std::memory_order_acquire)) {
        const int numThreads = std::min(4, std::max(1, numWaypoints / 8));
        const int waypointsPerThread = std::max(1, numWaypoints / numThreads);
        std::vector<std::future<void>> collisionFutures;
        for (int t = 0; t < numThreads && !collides.load(std::memory_order_acquire); ++t) {
            int startIdx = t * waypointsPerThread + 1;
            int endIdx = (t == numThreads - 1) ? N - 1 : startIdx + waypointsPerThread;
            collisionFutures.emplace_back(
                std::async(std::launch::async,
                           [this, &theta, &collides, &collisionTimestep, startIdx, endIdx]() {
                               RobotArm checkArm = _arm;
                               for (int i = startIdx;
                                    i < endIdx && !collides.load(std::memory_order_acquire);
                                    ++i) {
                                   checkArm.setJointAngles(theta.row(i));
                                   if (armHasCollision(checkArm)) {
                                       collides.store(true, std::memory_order_release);
                                       collisionTimestep.store(i, std::memory_order_release);
                                       return;
                                   }
                               }
                           }));
        }
        for (auto &future : collisionFutures) {
            future.wait();
        }
    }

    return collides.load(std::memory_order_acquire);
}
void MotionGenerator::updateConvergenceState(STOMPConvergenceState &state,
                                             const Eigen::MatrixXd &theta,
                                             const StompConfig &config,
                                             int iteration,
                                             double dt,
                                             bool collisionFree)
{
    double trajectoryCost = _costCalculator->computeCost(theta, dt);
    double trajectoryCostDiff = std::abs(trajectoryCost - state.prevTrajectoryCost);
    state.prevTrajectoryCost = trajectoryCost;
    if (collisionFree) {
        state.success = true;
        if (trajectoryCost < state.bestValidCost) {
            state.bestValidCost = trajectoryCost;
            state.bestValidTheta = theta;
            LOG_DEBUG << "Better valid solution found at iteration " << iteration
                      << " (cost: " << std::fixed << std::setprecision(4) << trajectoryCost << ")";
        }
        if (config.enableEarlyStopping) {
            state.earlyStoppingCounter++;
            LOG_DEBUG << "Valid trajectory found (early stopping: " << state.earlyStoppingCounter
                      << "/" << config.earlyStoppingPatience << ")";
        }
    } else {
        if (config.enableEarlyStopping) {
            state.earlyStoppingCounter = 0;
        }
    }
    if (trajectoryCostDiff < STOMPConvergenceState::costConvergenceThreshold) {
        state.noChangeCounter++;
    } else {
        state.noChangeCounter = 0;
    }
}
bool MotionGenerator::checkTimeLimit(const StompConfig &config,
                                     const QElapsedTimer &timer,
                                     int iteration,
                                     bool success,
                                     double bestCost)
{
    if (config.maxComputeTimeMs > 0.0 && timer.elapsed() > config.maxComputeTimeMs) {
        LOG_INFO << "Time limit exceeded (" << config.maxComputeTimeMs << "ms) after " << iteration
                 << " iterations (elapsed: " << timer.elapsed() << "ms)";
        if (!success) {
            throw StompTimeoutException("STOMP exceeded time limit without finding valid solution");
        } else {
            LOG_INFO << "Using best valid solution found (cost: " << std::fixed
                     << std::setprecision(4) << bestCost << ")";
            return true;
        }
    }
    return false;
}
bool MotionGenerator::checkConvergence(const STOMPConvergenceState &state,
                                       const StompConfig &config,
                                       int iteration)
{
    if (config.enableEarlyStopping && state.earlyStoppingCounter >= config.earlyStoppingPatience
        && state.success) {
        LOG_INFO << "Early stopping after " << iteration + 1 << " iterations (cost: " << std::fixed
                 << std::setprecision(4) << state.bestValidCost << ")";
        return true;
    }
    if (state.noChangeCounter >= STOMPConvergenceState::convergencePatience && state.success) {
        LOG_INFO << "Cost converged after " << iteration + 1 << " iterations (cost: " << std::fixed
                 << std::setprecision(4) << state.bestValidCost << ")";
        return true;
    }
    return false;
}
bool MotionGenerator::finalizeSTOMPResult(const Eigen::MatrixXd &theta,
                                          const STOMPConvergenceState &state,
                                          const StompConfig &config,
                                          int N,
                                          double dt,
                                          const QElapsedTimer &timer)
{
    LOG_INFO << "STOMP optimization complete: " << timer.elapsed() << "ms";

    if (!state.success || state.bestValidTheta.rows() == 0) {
        LOG_WARNING << "STOMP failed - no valid solution found";
        throw StompFailedException(
            "STOMP failed to find valid solution (collision-free + constraints)");
    }

    LOG_INFO << "STOMP success (cost: " << std::fixed << std::setprecision(4) << state.bestValidCost
             << ", " << timer.elapsed() << "ms)";

    Eigen::MatrixXd finalTheta = state.bestValidTheta;

    // Validate final trajectory constraints
    if (!isTrajectoryValid(finalTheta, dt)) {
        LOG_WARNING << "Final trajectory violates kinematic constraints";
        throw StompFailedException("STOMP result violates kinematic constraints");
    }

    // Convert to trajectory points
    _path.clear();
    for (int i = 0; i < N; ++i) {
        TrajectoryPoint point;
        point.time = i * dt;
        point.position.resize(config.numJoints);
        point.velocity.resize(config.numJoints);
        point.acceleration.resize(config.numJoints);

        for (int d = 0; d < config.numJoints; ++d) {
            point.position[d] = finalTheta(i, d);

            if (i == 0 || i == N - 1) {
                // Zero velocity/acceleration at start and end
                point.velocity[d] = 0.0;
                point.acceleration[d] = 0.0;
            } else {
                // Central difference for velocity and acceleration
                point.velocity[d] = (finalTheta(i + 1, d) - finalTheta(i - 1, d)) / (2 * dt);
                point.acceleration[d] = (finalTheta(i + 1, d) - 2 * finalTheta(i, d)
                                         + finalTheta(i - 1, d))
                                        / (dt * dt);
            }
        }
        _path.push_back(point);
    }

    LOG_DEBUG << "Finalized trajectory with " << _path.size() << " points";
    return true;
}
Eigen::MatrixXd MotionGenerator::smoothTrajectoryUpdate(const Eigen::MatrixXd &dTheta)
{
    Eigen::MatrixXd smoothed = _M * dTheta;
    return smoothed;
}
Eigen::MatrixXd MotionGenerator::initializeTrajectory(Eigen::Matrix<double, 7, 1> goalVec,
                                                      Eigen::Matrix<double, 7, 1> startVec,
                                                      int N,
                                                      const int D)
{
    Eigen::MatrixXd theta = Eigen::MatrixXd::Zero(N, D);
    double start_index = 0;
    double end_index = N - 1;
    double T[6];
    T[0] = 1.0;
    T[1] = (end_index - start_index);
    for (int i = 2; i <= 5; ++i)
        T[i] = T[i - 1] * T[1];
    Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(D, 6);
    for (int d = 0; d < D; ++d) {
        double x0 = startVec[d];
        double x1 = goalVec[d];
        coeff(d, 0) = x0;
        coeff(d, 1) = 0.0;
        coeff(d, 2) = 0.0;
        coeff(d, 3) = (-20 * x0 + 20 * x1) / (2 * T[3]);
        coeff(d, 4) = (30 * x0 - 30 * x1) / (2 * T[4]);
        coeff(d, 5) = (-12 * x0 + 12 * x1) / (2 * T[5]);
    }
    for (int i = 0; i < N; ++i) {
        double t[6];
        t[0] = 1.0;
        t[1] = i - start_index;
        for (int k = 2; k <= 5; ++k)
            t[k] = t[k - 1] * t[1];
        for (int d = 0; d < D; ++d) {
            theta(i, d) = 0.0;
            for (int k = 0; k <= 5; ++k) {
                theta(i, d) += t[k] * coeff(d, k);
            }
        }
    }
    return theta;
}
void MotionGenerator::initializeMatrices(const int &N, const double &dt)
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N + 2, N);
    for (int i = 0; i < N; ++i) {
        A(i, i) = 1. / (dt * dt);
        A(i + 1, i) = -2. / (dt * dt);
        A(i + 2, i) = 1. / (dt * dt);
    }

    // set the start and end to super hi values
    A(0, 0) = 1e6;         // Start position constraint
    A(1, 0) = 0.0;         // Start velocity constraint
    A(2, 0) = 0.0;         // Start acceleration constraint
    A(N, N - 1) = 1e6;     // End position constraint
    A(N + 1, N - 1) = 0.0; // End velocity constraint
    A(N + 2, N - 1) = 0.0; // End acceleration constraint

    _R = (A.transpose() * A) + 1e-6 * Eigen::MatrixXd::Identity(N, N);
    Eigen::MatrixXd Rinv = _R.fullPivLu().inverse();
    Eigen::MatrixXd temp = Rinv;
    Eigen::LLT<Eigen::MatrixXd> llt(temp);
    _L = llt.matrixL();
    _M = Rinv;
    for (int i = 0; i < N; i++) {
        // FIXED: Prevent division by zero when matrix column is all zeros
        double maxCoeff = _M.col(i).cwiseAbs().maxCoeff();
        if (maxCoeff > 1e-12) { // Avoid division by zero with numerical tolerance
            _M.col(i) *= (1.0 / N) / maxCoeff;
        } else {
            // If column is essentially zero, set it to a small uniform value
            _M.col(i).setConstant(1e-6 / N);
        }
    }
    _matricesInitialized = true;
}
void MotionGenerator::saveTrajectoryToCSV(const std::string &filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        LOG_ERROR << "Could not open file " << filename;
        return;
    }
    file << "time,";
    for (int i = 0; i < _numJoints; ++i) {
        file << "position_" << i << ",";
    }
    for (int i = 0; i < _numJoints; ++i) {
        file << "velocity_" << i << ",";
    }
    for (int i = 0; i < _numJoints; ++i) {
        file << "acceleration_" << i << (i == _numJoints - 1 ? "\n" : ",");
    }
    for (const auto &point : _path) {
        file << std::fixed << std::setprecision(6) << point.time << ",";
        for (const auto &pos : point.position) {
            file << pos << ",";
        }
        for (const auto &vel : point.velocity) {
            file << vel << ",";
        }
        for (size_t i = 0; i < point.acceleration.size(); ++i) {
            file << point.acceleration[i] << (i == point.acceleration.size() - 1 ? "\n" : ",");
        }
    }
    file.close();
}
ObstacleCostCalculator::ObstacleCostCalculator(
    RobotArm arm,
    std::shared_ptr<BVHTree> obstacleTree,
    const std::vector<std::vector<std::vector<double>>> &sdf,
    const Eigen::Vector3d &sdfMinPoint,
    const Eigen::Vector3d &sdfMaxPoint,
    double sdfResolution)
    : _arm(arm)
    , _obstacleTree(obstacleTree)
    , _sdf(sdf)
    , _sdfMinPoint(sdfMinPoint)
    , _sdfMaxPoint(sdfMaxPoint)
    , _sdfResolution(sdfResolution)
{}
// Helper function to sample points on oriented bounding box surface
std::vector<Eigen::Vector3d> sampleBoundingBoxPoints(const Eigen::Vector3d &center,
                                                     const Eigen::Vector3d &halfDims,
                                                     const Eigen::Matrix3d &axes)
{
    std::vector<Eigen::Vector3d> points;

    // Sample 8 vertices of the bounding box
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                Eigen::Vector3d localPoint((i == 0 ? -halfDims.x() : halfDims.x()),
                                           (j == 0 ? -halfDims.y() : halfDims.y()),
                                           (k == 0 ? -halfDims.z() : halfDims.z()));
                Eigen::Vector3d worldPoint = center + axes * localPoint;
                points.push_back(worldPoint);
            }
        }
    }

    // Sample face centers
    for (int axis = 0; axis < 3; ++axis) {
        for (int dir = 0; dir < 2; ++dir) {
            Eigen::Vector3d localPoint = Eigen::Vector3d::Zero();
            localPoint[axis] = (dir == 0 ? -halfDims[axis] : halfDims[axis]);
            Eigen::Vector3d worldPoint = center + axes * localPoint;
            points.push_back(worldPoint);
        }
    }

    // Add center point
    points.push_back(center);

    return points;
}

double ObstacleCostCalculator::computeCost(const Eigen::MatrixXd &trajectory, double dt)
{
    double cost = 0.0;
    int N = trajectory.rows();

    const double clearanceRadius = 0.03; // 3cm safety clearance

    RobotArm currentArm = _arm;

    for (int i = 0; i < N; ++i) {
        currentArm.setJointAngles(trajectory.row(i));

        // Get robot link collision boxes for sphere center approximation
        auto bBoxes = currentArm.getCollisionBoxes();
        double timestepCost = 0.0;

        // Process each robot link (skip base links)
        int linkIndex = 0;
        for (const auto &bBox : bBoxes) {
            ++linkIndex;
            if (linkIndex < 2) // Skip base links as in original code
                continue;

            auto [center, halfDims, axes] = bBox;

            // Use maximum bounding box dimension as link radius
            double linkRadius = 0.75 * std::max({halfDims.x(), halfDims.y(), halfDims.z()});

            // Use bounding box center as sphere center approximation
            Eigen::Vector3d sphereCenter = center;

            // Query SDF for distance to nearest obstacle
            Eigen::Vector3d gridCoords = (sphereCenter - _sdfMinPoint) / _sdfResolution;
            int grid_i = static_cast<int>(std::round(gridCoords.x()));
            int grid_j = static_cast<int>(std::round(gridCoords.y()));
            int grid_k = static_cast<int>(std::round(gridCoords.z()));

            double sdfDistance = 0.0;
            bool validSdfQuery = false;

            // Bounds check and SDF lookup
            if (grid_i >= 0 && grid_i < static_cast<int>(_sdf.size()) && !_sdf.empty()
                && grid_j >= 0 && grid_j < static_cast<int>(_sdf[grid_i].size())
                && !_sdf[grid_i].empty() && grid_k >= 0
                && grid_k < static_cast<int>(_sdf[grid_i][grid_j].size())) {
                sdfDistance = _sdf[grid_i][grid_j][grid_k];
                validSdfQuery = true;
            }

            // Calculate cost based on sphere-obstacle distance
            if (validSdfQuery) {
                // Distance from sphere surface to obstacle (negative means penetration)
                double surfaceDistance = sdfDistance - linkRadius;

                if (surfaceDistance < clearanceRadius) {
                    // Calculate cost using STOMP paper formulation
                    // Cost increases quadratically as we approach obstacles
                    double penetration = clearanceRadius - surfaceDistance;
                    double normalizedPenetration = penetration / clearanceRadius;

                    // Quadratic cost function as in original STOMP
                    double sphereCost = normalizedPenetration * normalizedPenetration;

                    // Add velocity scaling (approximate from trajectory derivatives)
                    double velocityScale = 1.0;
                    if (i > 0 && i < N - 1) {
                        // Approximate velocity magnitude using finite differences
                        Eigen::VectorXd positionDiff = trajectory.row(i + 1)
                                                       - trajectory.row(i - 1);
                        double jointVelocityMagnitude = positionDiff.norm() / (2.0 * dt);
                        velocityScale = 1.0
                                        + 0.1 * jointVelocityMagnitude; // Small velocity penalty
                    }

                    timestepCost += sphereCost * velocityScale;
                }

                // Add collision penalty when sphere penetrates obstacle
                if (surfaceDistance < 0) {
                    timestepCost += 1.0; // Collision penalty of 1.0
                }
            }
        }

        // Check for actual arm collision at this timestep and add penalty
        // Use the same logic as MotionGenerator::armHasCollision
        auto allBBoxes = currentArm.getCollisionBoxes();
        bool hasCollision = false;
        int linkIdx = 0;
        for (const auto &bbox : allBBoxes) {
            ++linkIdx;
            if (linkIdx < 2) // Skip base links as in original armHasCollision
                continue;
            auto [bboxCenter, bboxHalfDims, bboxAxes] = bbox;
            if (_obstacleTree->isBoxIntersecting(bboxCenter, bboxHalfDims, bboxAxes)) {
                hasCollision = true;
                break;
            }
        }
        if (hasCollision) {
            timestepCost += 10.0; // Collision penalty when arm actually has collision
        }

        cost += timestepCost;
    }

    return cost;
}
ConstraintCostCalculator::ConstraintCostCalculator(const std::vector<double> &maxVel,
                                                   const std::vector<double> &maxAcc)
    : _maxJointVelocities(maxVel)
    , _maxJointAccelerations(maxAcc)
{}
double ConstraintCostCalculator::computeCost(const Eigen::MatrixXd &trajectory, double dt)
{
    double cost = 0;
    int N = trajectory.rows();
    int D = trajectory.cols();
    double totalTime = (N - 1) * dt;

    // Debug counters
    int velocityViolations = 0;
    int accelerationViolations = 0;
    double maxVelViolation = 0.0;
    double maxAccViolation = 0.0;

    // Use central differences for better numerical accuracy
    Eigen::MatrixXd velocity = Eigen::MatrixXd::Zero(N, D);
    Eigen::MatrixXd acceleration = Eigen::MatrixXd::Zero(N, D);

    // Calculate velocities using central differences where possible
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < D; ++j) {
            if (i == 0 || i == N - 1) {
                // Enforce zero velocity at boundaries (start/end at rest)
                velocity(i, j) = 0.0;
            } else {
                // Central difference for interior points
                velocity(i, j) = (trajectory(i + 1, j) - trajectory(i - 1, j)) / (2.0 * dt);
            }
        }
    }

    // Calculate accelerations using central differences where possible
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < D; ++j) {
            if (i == 0 || i == N - 1) {
                // Enforce zero acceleration at boundaries (start/end at rest)
                acceleration(i, j) = 0.0;
            } else if (i == 1) {
                // Forward difference for point near start
                acceleration(i, j) = (velocity(i + 1, j) - velocity(i, j)) / dt;
            } else if (i == N - 2) {
                // Backward difference for point near end
                acceleration(i, j) = (velocity(i, j) - velocity(i - 1, j)) / dt;
            } else {
                // Central difference for interior points
                acceleration(i, j) = (velocity(i + 1, j) - velocity(i - 1, j)) / (2.0 * dt);
            }
        }
    }

    // Calculate constraint violations with proper normalization
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < D; ++j) {
            // Velocity constraint cost (normalized)
            double normalizedVel = std::abs(velocity(i, j)) / _maxJointVelocities[j];
            if (normalizedVel > 1.0) {
                cost += std::pow(normalizedVel - 1.0, 2); // Quadratic penalty beyond limits
                velocityViolations++;
                maxVelViolation = std::max(maxVelViolation, normalizedVel - 1.0);
            }
            // Small smoothness penalty even within limits
            cost += 0.01 * std::pow(normalizedVel, 2);

            // Acceleration constraint cost (normalized)
            double normalizedAcc = std::abs(acceleration(i, j)) / _maxJointAccelerations[j];
            if (normalizedAcc > 1.0) {
                cost += std::pow(normalizedAcc - 1.0, 2); // Quadratic penalty beyond limits
                accelerationViolations++;
                maxAccViolation = std::max(maxAccViolation, normalizedAcc - 1.0);
            }
            // Small smoothness penalty even within limits
            cost += 0.01 * std::pow(normalizedAcc, 2);
        }
    }

    // Normalize by trajectory duration to make costs comparable across different trajectory lengths
    double normalizedCost = cost / totalTime;

    return normalizedCost;
}

ControlCostCalculator::ControlCostCalculator(const Eigen::MatrixXd &R)
    : _R(R)
{}

double ControlCostCalculator::computeCost(const Eigen::MatrixXd &trajectory, double dt)
{
    // Original STOMP control cost: q_control = (1/2) * θ^T * R * θ
    // This penalizes trajectory acceleration (second-order derivatives) for smoothness

    const int N = trajectory.rows();
    const int D = trajectory.cols();
    double totalCost = 0.0;

    // Apply control cost for each joint separately
    for (int j = 0; j < D; ++j) {
        // Extract joint trajectory for this DOF
        Eigen::VectorXd jointTrajectory = trajectory.col(j);

        // Compute control cost: (1/2) * θ^T * R * θ
        double jointControlCost = 0.5 * jointTrajectory.transpose() * _R * jointTrajectory;
        totalCost += jointControlCost;
    }

    // Scale by dt^4 to compensate for 1/dt^4 scaling in R matrix construction
    // This brings control cost to reasonable magnitude
    double dt4 = dt * dt * dt * dt;

    // Normalize by number of joints and trajectory length for consistent scaling
    return (totalCost * dt4) / (D * N);
}

void CompositeCostCalculator::addCostCalculator(std::unique_ptr<CostCalculator> calculator,
                                                double weight)
{
    _costCalculators.push_back(std::move(calculator));
    _weights.push_back(weight);
}
double CompositeCostCalculator::computeCost(const Eigen::MatrixXd &trajectory, double dt)
{
    double totalCost = 0.0;
    for (size_t i = 0; i < _costCalculators.size(); ++i) {
        totalCost += _weights[i] * _costCalculators[i]->computeCost(trajectory, dt);
    }
    return totalCost;
}
void MotionGenerator::initializeCostCalculator(const StompConfig &config)
{
    _costCalculator = std::make_unique<CompositeCostCalculator>();

    // Add obstacle cost calculator with weight from config
    _costCalculator->addCostCalculator(std::make_unique<ObstacleCostCalculator>(_arm,
                                                                                _obstacleTree,
                                                                                _sdf,
                                                                                _sdfMinPoint,
                                                                                _sdfMaxPoint,
                                                                                _sdfResolution),
                                       config.obstacleCostWeight);

    // Add constraint cost calculator with weight from config
    _costCalculator
        ->addCostCalculator(std::make_unique<ConstraintCostCalculator>(_maxJointVelocities,
                                                                       _maxJointAccelerations),
                            config.constraintCostWeight);

    // Add control (smoothness) cost calculator from original STOMP paper
    _costCalculator->addCostCalculator(std::make_unique<ControlCostCalculator>(_R),
                                       config.controlCostWeight);
}
void MotionGenerator::initializeCostCalculatorCheckpoints(
    const std::vector<Eigen::VectorXd> &checkpoints)
{
    _costCalculator = std::make_unique<CompositeCostCalculator>();
}
Eigen::MatrixXd MotionGenerator::generateNoisyTrajectory(
    const Eigen::MatrixXd &baseTrajectory,
    const Eigen::VectorXd &stdDevs,
    const std::vector<std::pair<double, double>> &limits)
{
    int numPoints = baseTrajectory.rows();
    int numJoints = baseTrajectory.cols();
    Eigen::MatrixXd noisyTrajectory = baseTrajectory;

    // CRITICAL FIX: Only generate noise for intermediate waypoints (exclude start and goal)
    if (numPoints > 2) {
        Eigen::MatrixXd epsilon = Eigen::MatrixXd::Zero(numPoints, numJoints);
        std::random_device rd;
        std::mt19937 gen(rd());
        for (int d = 0; d < numJoints; d++) {
            std::normal_distribution<> dist(0, stdDevs[d]);
            epsilon.col(d) = _L
                             * Eigen::VectorXd::NullaryExpr(numPoints, [&]() { return dist(gen); });
        }

        // FIXED: Safe block operations with proper dimension checking
        int intermediatePoints = numPoints - 2;
        if (intermediatePoints > 0 && intermediatePoints <= epsilon.rows() - 1) {
            noisyTrajectory.block(1, 0, intermediatePoints, numJoints)
                += epsilon.block(1, 0, intermediatePoints, numJoints);
        }

        // Clamp only the intermediate waypoints that received noise
        for (int i = 1; i < numPoints - 1; i++) {
            for (int d = 0; d < numJoints; d++) {
                noisyTrajectory(i, d) = std::clamp(noisyTrajectory(i, d),
                                                   std::get<0>(limits[d]),
                                                   std::get<1>(limits[d]));
            }
        }
    }
    return noisyTrajectory;
}
std::vector<MotionGenerator::TrajectoryPoint> MotionGenerator::generateTrajectoryFromCheckpoints(
    const std::vector<Eigen::VectorXd> &checkpoints)
{
    std::vector<TrajectoryPoint> trajectory;
    if (checkpoints.empty())
        return trajectory;
    const size_t n_joints = _maxJointVelocities.size();
    const size_t n_checkpoints = checkpoints.size();
    if (n_checkpoints < 2) {
        TrajectoryPoint pt;
        pt.position.resize(n_joints);
        pt.velocity.resize(n_joints);
        pt.acceleration.resize(n_joints);
        pt.time = 0.0;
        for (size_t j = 0; j < n_joints; ++j) {
            pt.position[j] = checkpoints[0][j];
            pt.velocity[j] = 0.0;
            pt.acceleration[j] = 0.0;
        }
        trajectory.push_back(pt);
        return trajectory;
    }
    Eigen::VectorXd max_vel = Eigen::Map<const Eigen::VectorXd>(_maxJointVelocities.data(),
                                                                n_joints);
    Eigen::VectorXd max_acc = Eigen::Map<const Eigen::VectorXd>(_maxJointAccelerations.data(),
                                                                n_joints);
    std::vector<double> segment_times;
    std::vector<double> absolute_times{0.0};
    for (size_t i = 1; i < n_checkpoints; ++i) {
        Eigen::VectorXd delta = (checkpoints[i] - checkpoints[i - 1]).cwiseAbs();
        Eigen::VectorXd t_vel = delta.cwiseQuotient(max_vel);
        double t_vel_max = t_vel.maxCoeff();
        Eigen::VectorXd t_acc = (delta * 15.0 / 2.0).cwiseSqrt().cwiseQuotient(max_acc.cwiseSqrt());
        double t_acc_max = t_acc.maxCoeff();
        double segment_time = std::max(t_vel_max, t_acc_max);
        segment_times.push_back(segment_time);
        absolute_times.push_back(absolute_times.back() + segment_time);
    }
    std::vector<Eigen::VectorXd> velocities(n_checkpoints, Eigen::VectorXd::Zero(n_joints));
    std::vector<Eigen::VectorXd> accelerations(n_checkpoints, Eigen::VectorXd::Zero(n_joints));
    for (size_t i = 1; i < n_checkpoints - 1; ++i) {
        const double dt_prev = segment_times[i - 1];
        const double dt_next = segment_times[i];
        const double dt_total = dt_prev + dt_next;
        for (size_t j = 0; j < n_joints; ++j) {
            velocities[i][j] = (checkpoints[i + 1][j] - checkpoints[i - 1][j]) / dt_total;
            velocities[i][j] = std::clamp(velocities[i][j], -max_vel[j], max_vel[j]);
        }
    }
    std::vector<std::vector<Eigen::VectorXd>> coeffs(n_joints);
    for (size_t j = 0; j < n_joints; ++j) {
        coeffs[j].resize(n_checkpoints - 1);
        for (size_t i = 0; i < n_checkpoints - 1; ++i) {
            const double q0 = checkpoints[i][j];
            const double qf = checkpoints[i + 1][j];
            const double v0 = velocities[i][j];
            const double vf = velocities[i + 1][j];
            const double a0 = accelerations[i][j];
            const double af = accelerations[i + 1][j];
            const double T = segment_times[i];
            Eigen::VectorXd c(6);
            c[0] = q0;
            c[1] = v0;
            c[2] = a0 / 2.0;
            const double T2 = T * T;
            const double T3 = T2 * T;
            const double T4 = T3 * T;
            const double T5 = T4 * T;
            Eigen::Matrix3d A;
            A << T3, T4, T5, 3 * T2, 4 * T3, 5 * T4, 6 * T, 12 * T2, 20 * T3;
            Eigen::Vector3d b;
            b << qf - q0 - v0 * T - 0.5 * a0 * T2, vf - v0 - a0 * T, af - a0;
            Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
            c[3] = x[0];
            c[4] = x[1];
            c[5] = x[2];
            coeffs[j][i] = c;
        }
    }
    const double control_rate = 100.0;
    const double dt = 1.0 / control_rate;
    const double total_time = absolute_times.back();
    for (double t = 0.0; t <= total_time + 1e-6; t += dt) {
        TrajectoryPoint pt;
        pt.position.resize(n_joints);
        pt.velocity.resize(n_joints);
        pt.acceleration.resize(n_joints);
        pt.time = t;
        const double clamped_t = std::min(t, total_time);
        size_t segment_idx = 0;
        while (segment_idx < segment_times.size() - 1
               && clamped_t > absolute_times[segment_idx + 1]) {
            ++segment_idx;
        }
        const double t_segment = clamped_t - absolute_times[segment_idx];
        for (size_t j = 0; j < n_joints; ++j) {
            const Eigen::VectorXd &c = coeffs[j][segment_idx];
            pt.position[j] = c[0] + c[1] * t_segment + c[2] * t_segment * t_segment
                             + c[3] * t_segment * t_segment * t_segment
                             + c[4] * t_segment * t_segment * t_segment * t_segment
                             + c[5] * t_segment * t_segment * t_segment * t_segment * t_segment;
            pt.velocity[j] = c[1] + 2 * c[2] * t_segment + 3 * c[3] * t_segment * t_segment
                             + 4 * c[4] * t_segment * t_segment * t_segment
                             + 5 * c[5] * t_segment * t_segment * t_segment * t_segment;
            pt.acceleration[j] = 2 * c[2] + 6 * c[3] * t_segment + 12 * c[4] * t_segment * t_segment
                                 + 20 * c[5] * t_segment * t_segment * t_segment;
        }
        trajectory.push_back(pt);
    }
    return trajectory;
}
bool MotionGenerator::isTrajectoryValid(const Eigen::MatrixXd &trajectory, double dt) const
{
    int N = trajectory.rows();
    int D = trajectory.cols();
    for (int i = 0; i < N - 1; ++i) {
        for (int j = 0; j < D; ++j) {
            double velocity = (trajectory(i + 1, j) - trajectory(i, j)) / dt;
            if (std::abs(velocity) > _maxJointVelocities[j]) {
                return false;
            }
        }
    }
    for (int i = 0; i < N - 2; ++i) {
        for (int j = 0; j < D; ++j) {
            double velocity_curr = (trajectory(i + 1, j) - trajectory(i, j)) / dt;
            double velocity_next = (trajectory(i + 2, j) - trajectory(i + 1, j)) / dt;
            double acceleration = (velocity_next - velocity_curr) / dt;
            if (std::abs(acceleration) > _maxJointAccelerations[j]) {
                return false;
            }
        }
    }
    return true;
}

bool MotionGenerator::isTrajectoryValidWithMargin(const Eigen::MatrixXd &trajectory,
                                                  double dt,
                                                  double velocityMargin,
                                                  double accelerationMargin) const
{
    int N = trajectory.rows();
    int D = trajectory.cols();
    for (int i = 0; i < N - 1; ++i) {
        for (int j = 0; j < D; ++j) {
            double velocity = (trajectory(i + 1, j) - trajectory(i, j)) / dt;
            if (std::abs(velocity) > _maxJointVelocities[j] * velocityMargin) {
                return false;
            }
        }
    }
    for (int i = 0; i < N - 2; ++i) {
        for (int j = 0; j < D; ++j) {
            double velocity_curr = (trajectory(i + 1, j) - trajectory(i, j)) / dt;
            double velocity_next = (trajectory(i + 2, j) - trajectory(i + 1, j)) / dt;
            double acceleration = (velocity_next - velocity_curr) / dt;
            if (std::abs(acceleration) > _maxJointAccelerations[j] * accelerationMargin) {
                return false;
            }
        }
    }
    return true;
}
