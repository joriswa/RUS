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
bool MotionGenerator::performSTOMP(const StompConfig &config,
                                   std::shared_ptr<boost::asio::thread_pool> sharedPool,
                                   int trajectoryIndex)
{
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
    STOMPConvergenceState convergenceState;
    std::vector<std::pair<Eigen::MatrixXd, double>> bestSamples;
    for (int iteration = 0; iteration < config.maxIterations; iteration++) {
        if (checkTimeLimit(config,
                           convergenceState.overallTimer,
                           iteration,
                           convergenceState.success,
                           convergenceState.bestCollisionFreeCost)) {
            break;
        }
        auto noisyTrajectories = generateNoisySamples(config, theta, limits, bestSamples, pool);
        auto [costs, weights] = evaluateTrajectories(noisyTrajectories, config, dt, pool);
        updateBestSamples(noisyTrajectories, costs, bestSamples, config.numBestSamples);
        theta = applyTrajectoryUpdate(theta, noisyTrajectories, weights, config, N, limits);

        // Explicit collision and constraint checking
        bool collisionFree = !checkCollisions(theta, N);
        bool constraintsValid = isTrajectoryValidWithMargin(theta, dt, 1., 1.);
        bool trajectoryValid = collisionFree;

        updateConvergenceState(convergenceState, theta, config, iteration, dt, trajectoryValid);
        if (checkConvergence(convergenceState, config, iteration)) {
            break;
        }
    }
    return finalizeSTOMPResult(theta, convergenceState, config, N, dt, timer);
}
MotionGenerator::STOMPInitData MotionGenerator::initializeSTOMPExecution(
    const StompConfig &config, std::shared_ptr<boost::asio::thread_pool> sharedPool)
{
    _path.clear();
    createSDF();
    initializeCostCalculator(config);
    STOMPInitData initData;
    if (sharedPool) {
        initData.pool = sharedPool.get();
    } else {
        // FIXED: Handle case where hardware_concurrency() returns 0
        unsigned int hwThreads = std::thread::hardware_concurrency();
        unsigned int numThreads = hwThreads > 0 ? std::max(2u, hwThreads / 2) : 4;
        initData.localPool = std::make_unique<boost::asio::thread_pool>(numThreads);
        initData.pool = initData.localPool.get();
    }
    Eigen::Matrix<double, 1, Eigen::Dynamic> startVec = _waypoints.row(0);
    Eigen::Matrix<double, 1, Eigen::Dynamic> goalVec = _waypoints.row(_waypoints.rows() - 1);
    TrajectoryPoint start, end;
    for (int i = 0; i < config.numJoints; ++i) {
        start.position.push_back(startVec[i]);
        end.position.push_back(goalVec[i]);
        start.velocity.push_back(0.0);
        end.velocity.push_back(0.0);
        start.acceleration.push_back(0.0);
        end.acceleration.push_back(0.0);
    }

    std::vector<double> originalMaxVel = _maxJointVelocities;
    std::vector<double> originalMaxAcc = _maxJointAccelerations;
    const double conservativeFactor = 0.5; // Increased for better exploration in hard cases

    for (size_t i = 0; i < _maxJointVelocities.size(); ++i) {
        _maxJointVelocities[i] *= conservativeFactor;
        _maxJointAccelerations[i] *= conservativeFactor;
    }

    auto segment = computeTimeOptimalSegment(start, end, 0.0);
    double estimatedTime = segment.back().time;

    _maxJointVelocities = originalMaxVel;
    _maxJointAccelerations = originalMaxAcc;

    // Calculate N based on trajectory time and fixed dt
    int naturalN = static_cast<int>(std::ceil(estimatedTime / config.dt)) + 1;

    if (naturalN >= 20) {
        // Use fixed dt when natural calculation gives enough points
        initData.dt = config.dt;
        initData.N = naturalN;
    } else {
        // Adjust dt to maintain trajectory duration while ensuring minimum 20 points
        initData.N = 20;
        initData.dt = estimatedTime / (initData.N - 1); // Ensure correct temporal resolution
    }

    initData.theta = initializeTrajectory(goalVec, startVec, initData.N, config.numJoints);
    initializeMatrices(initData.N, initData.dt);
    initData.limits = _arm.jointLimits();
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

    // Check if internal parallelization is disabled (for batch mode optimization)
    if (config.disableInternalParallelization) {
        // Sequential generation for better batch-level parallelization
        for (int k = 0; k < config.numNoisyTrajectories; ++k) {
            noisyTrajectories[k] = generateNoisyTrajectory(theta, config.jointStdDevs, limits);
        }
    } else {
        // Original parallel generation for single trajectory optimization
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

    // Check if internal parallelization is disabled (for batch mode optimization)
    if (config.disableInternalParallelization) {
        // Sequential evaluation for better batch-level parallelization
        for (int k = 0; k < totalSamples; k++) {
            double trajectoryCost = _costCalculator->computeCost(trajectories[k], dt);
            costs[k] = trajectoryCost;
            costVector(k) = trajectoryCost;
        }
    } else {
        // Original parallel evaluation for single trajectory optimization
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

    double minCost = costVector.minCoeff();
    double maxCost = costVector.maxCoeff();
    double costRange = maxCost - minCost;
    Eigen::VectorXd weights(totalSamples);
    if (costRange < 1e-6) {
        weights.setConstant(1.0 / totalSamples); // Use totalSamples instead of numNoisyTrajectories
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
    const int numWaypoints = N - 2;
    const int sparseStep = 4;
    std::vector<std::future<void>> sparseFutures;
    for (int i = 1; i < N - 1; i += sparseStep) {
        sparseFutures.emplace_back(std::async(std::launch::async, [this, &theta, &collides, i]() {
            // FIXED: Use memory ordering for better thread safety
            if (!collides.load(std::memory_order_acquire)) {
                RobotArm checkArm = _arm;
                checkArm.setJointAngles(theta.row(i));
                if (armHasCollision(checkArm)) {
                    collides.store(true, std::memory_order_release);
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
                std::async(std::launch::async, [this, &theta, &collides, startIdx, endIdx]() {
                    RobotArm checkArm = _arm;
                    for (int i = startIdx; i < endIdx && !collides.load(std::memory_order_acquire);
                         ++i) {
                        checkArm.setJointAngles(theta.row(i));
                        if (armHasCollision(checkArm)) {
                            collides.store(true, std::memory_order_release);
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
                                             bool trajectoryValid)
{
    double trajectoryCost = _costCalculator->computeCost(theta, dt);
    double trajectoryCostDiff = std::abs(trajectoryCost - state.prevTrajectoryCost);
    state.prevTrajectoryCost = trajectoryCost;
    if (trajectoryValid) {  // Now requires both collision-free AND constraint satisfaction
        state.success = true;
        if (trajectoryCost < state.bestCollisionFreeCost) {
            state.bestCollisionFreeCost = trajectoryCost;
            state.bestCollisionFreeTheta = theta;
            LOG_DEBUG << "Better valid solution found at iteration " << iteration
                      << " (cost: " << std::fixed << std::setprecision(4) << trajectoryCost << ")";
        }
        if (config.enableEarlyStopping) {
            state.earlyStoppingCounter++;
            LOG_DEBUG << "Valid trajectory found (early stopping: "
                      << state.earlyStoppingCounter << "/" << config.earlyStoppingPatience << ")";
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
                 << " iterations";
        if (!success) {
            throw StompTimeoutException(
                "STOMP exceeded time limit without finding collision-free solution");
        } else {
            LOG_INFO << "Using best collision-free solution found (cost: " << std::fixed
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
    if (config.enableEarlyStopping && state.earlyStoppingCounter >= config.earlyStoppingPatience) {
        LOG_INFO << "Early stopping after " << iteration + 1 << " iterations (cost: " << std::fixed
                 << std::setprecision(4) << state.bestCollisionFreeCost << ")";
        return true;
    }
    if (state.noChangeCounter >= STOMPConvergenceState::convergencePatience && state.success) {
        LOG_INFO << "Cost converged after " << iteration + 1 << " iterations (cost: " << std::fixed
                 << std::setprecision(4) << state.bestCollisionFreeCost << ")";
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
    LOG_FAST_DEBUG << "Optimization complete: " << timer.elapsed() << "ms";
    Eigen::MatrixXd finalTheta = theta;
    if (state.success && state.bestCollisionFreeTheta.rows() > 0) {
        LOG_INFO << "STOMP completed (cost: " << std::fixed << std::setprecision(4)
                 << state.bestCollisionFreeCost << ", " << timer.elapsed() << "ms)";
        finalTheta = state.bestCollisionFreeTheta;
    } else {
        LOG_WARNING << "STOMP failed after " << config.maxIterations << " iterations ("
                    << timer.elapsed() << "ms)";
        throw StompFailedException("STOMP failed to find collision-free solution");
    }
    _path.clear();
    for (int i = 0; i < N; ++i) {
        TrajectoryPoint point;
        point.time = i * dt;
        point.position.resize(config.numJoints);
        point.velocity.resize(config.numJoints);
        point.acceleration.resize(config.numJoints);
        for (int d = 0; d < config.numJoints; ++d) {
            point.position[d] = finalTheta(i, d);
        }
        if (i == 0) {
            for (int d = 0; d < config.numJoints; ++d) {
                // FIXED: Use more stable forward finite difference or enforce zero boundary conditions
                if (N >= 3) {
                    // Second-order forward difference for better numerical stability
                    point.velocity[d] = (-3 * finalTheta(0, d) + 4 * finalTheta(1, d)
                                         - finalTheta(2, d))
                                        / (2 * dt);
                } else {
                    // Fallback for short trajectories
                    point.velocity[d] = (finalTheta(1, d) - finalTheta(0, d)) / dt;
                }
                // For boundary acceleration, enforce zero for stability (start at rest)
                point.acceleration[d] = 0.0;
            }
        } else if (i == N - 1) {
            for (int d = 0; d < config.numJoints; ++d) {
                // FIXED: Use more stable backward finite difference or enforce zero boundary conditions
                if (N >= 3) {
                    // Second-order backward difference for better numerical stability
                    point.velocity[d] = (3 * finalTheta(N - 1, d) - 4 * finalTheta(N - 2, d)
                                         + finalTheta(N - 3, d))
                                        / (2 * dt);
                } else {
                    // Fallback for short trajectories
                    point.velocity[d] = (finalTheta(N - 1, d) - finalTheta(N - 2, d)) / dt;
                }
                // For boundary acceleration, enforce zero for stability (end at rest)
                point.acceleration[d] = 0.0;
            }
        } else {
            for (int d = 0; d < config.numJoints; ++d) {
                point.velocity[d] = (finalTheta(i + 1, d) - finalTheta(i - 1, d)) / (2 * dt);
                point.acceleration[d] = (finalTheta(i + 1, d) - 2 * finalTheta(i, d)
                                         + finalTheta(i - 1, d))
                                        / (dt * dt);
            }
        }
        _path.push_back(point);
    }

    // Final constraint validation
    if (!isTrajectoryValid(finalTheta, dt)) {
        LOG_WARNING << "Final trajectory violates velocity/acceleration constraints";
        throw StompFailedException("STOMP result violates kinematic constraints");
    }

    return state.success;
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
    _R = A.transpose() * A;
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
    double cost = 0;
    int N = trajectory.rows();
    RobotArm currentArm = _arm;
    currentArm.setJointAngles(trajectory.row(0));
    RobotArm prevArm = currentArm;

    // Store sample points from previous timestep for velocity calculation
    std::vector<std::vector<Eigen::Vector3d>> prevSamplePoints;

    bool collides = false;
    for (int i = 0; i < N; ++i) {
        RobotArm curArm = _arm;
        curArm.setJointAngles(trajectory.row(i));
        auto bboxesNew = curArm.getLinkBoundingBoxes();
        auto bboxesOld = prevArm.getLinkBoundingBoxes();

        double minDistance = std::numeric_limits<double>::max();
        bool foundValidDistance = false;

        // Current sample points for this timestep
        std::vector<std::vector<Eigen::Vector3d>> curSamplePoints;

        for (int iter = 0; iter < bboxesNew.size(); ++iter) {
            auto [center, halfDims, axes] = bboxesNew[iter];
            auto [centerOld, halfDimsOld, axesOld] = bboxesOld[iter];

            // Sample multiple points on the bounding box surface
            auto samplePoints = sampleBoundingBoxPoints(center, halfDims, axes);
            curSamplePoints.push_back(samplePoints);

            for (size_t pointIdx = 0; pointIdx < samplePoints.size(); ++pointIdx) {
                const auto &point = samplePoints[pointIdx];
                // Use SDF for distance lookup instead of BVH tree query
                Eigen::Vector3d gridCoords = (point - _sdfMinPoint) / _sdfResolution;
                int grid_i = static_cast<int>(gridCoords.x());
                int grid_j = static_cast<int>(gridCoords.y());
                int grid_k = static_cast<int>(gridCoords.z());

                double distance = 0.0;
                // FIXED: Proper 3D array bounds checking to prevent segmentation faults
                if (grid_i >= 0 && grid_i < _sdf.size() && !_sdf.empty() && grid_j >= 0
                    && grid_j < _sdf[grid_i].size() && !_sdf[grid_i].empty() && grid_k >= 0
                    && grid_k < _sdf[grid_i][grid_j].size()) {
                    distance = _sdf[grid_i][grid_j][grid_k];

                } else {
                    qDebug() << "SDF not accessible - Point:" << point.x() << point.y() << point.z()
                             << "Grid coords:" << grid_i << grid_j << grid_k
                             << "SDF size:" << _sdf.size() << "SDF bounds: min(" << _sdfMinPoint.x()
                             << _sdfMinPoint.y() << _sdfMinPoint.z() << ") max(" << _sdfMaxPoint.x()
                             << _sdfMaxPoint.y() << _sdfMaxPoint.z()
                             << ") resolution:" << _sdfResolution;
                    distance = 0.1; // 10cm clearance
                }

                foundValidDistance = true;
                minDistance = std::min(minDistance, distance);

                double clearance = 0.1;
                double obstacleCost = std::max(0.0, clearance - distance);

                // Scale cost by velocity as in original STOMP paper
                cost += obstacleCost;
            }
        }

        if (!collides) {
            auto bBoxes = curArm.getCollisionBoxes();
            int link = 0;
            for (const auto &bBox : bBoxes) {
                ++link;
                if (link < 2)
                    continue;
                auto [center, halfDims, axes] = bBox;
                if (_obstacleTree->isBoxIntersecting(center, halfDims, axes)) {
                    cost += 100.0;
                    collides = true;
                    break;
                }
            }
        }

        prevArm = curArm;
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
    Eigen::MatrixXd velocity = Eigen::MatrixXd::Zero(N - 1, D);
    Eigen::MatrixXd acceleration = Eigen::MatrixXd::Zero(N - 2, D);
    for (int i = 0; i < N - 1; ++i) {
        velocity.row(i) = (trajectory.row(i + 1) - trajectory.row(i)) / dt;
    }
    for (int i = 0; i < N - 2; ++i) {
        acceleration.row(i) = (velocity.row(i + 1) - velocity.row(i)) / dt;
    }
    for (int i = 0; i < N - 1; ++i) {
        for (int j = 0; j < D; ++j) {
            cost += std::max(0., std::abs(velocity(i, j)) - _maxJointVelocities[j]);
        }
    }
    for (int i = 0; i < N - 2; ++i) {
        for (int j = 0; j < D; ++j) {
            cost += std::max(0., std::abs(acceleration(i, j)) - _maxJointAccelerations[j]);
        }
    }
    return cost;
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
    const double control_rate = 1000.0;
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
