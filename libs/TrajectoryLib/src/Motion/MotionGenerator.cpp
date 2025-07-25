#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Robot/franka_ik_He.h"
#include <cmath> // Required for std::exp
#include <future>
#include <atomic> // Required for atomic collision checking

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
    auto start = std::chrono::high_resolution_clock::now(); // Start timing

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

    const int numSteps = 100; // Number of interpolation steps
    ParabolicRamp::Vector q(a.size());

    for (int step = 1; step < numSteps; ++step) {
        double t = static_cast<double>(step) / numSteps;

        // Linear interpolation between configurations
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

/**
 * @brief Computes a time-optimal trajectory segment between two given points.
 *
 * This function generates a trajectory segment that starts at the given start point,
 * ends at the given end point, and adheres to the maximum joint velocities and accelerations.
 * The trajectory segment is generated using a trapezoidal velocity profile.
 *
 * @param start The starting point of the trajectory segment.
 * @param end The ending point of the trajectory segment.
 * @param startTime The time at which the trajectory segment starts.
 *
 * @return A vector of trajectory points representing the time-optimal trajectory segment.
 */
std::vector<MotionGenerator::TrajectoryPoint> MotionGenerator::computeTimeOptimalSegment(
    const TrajectoryPoint &start, const TrajectoryPoint &end, double startTime)
{
    std::vector<TrajectoryPoint> segment;
    double v_max = std::numeric_limits<double>::max();
    double a_max = std::numeric_limits<double>::max();

    // Find the maximum joint velocities and accelerations
    for (size_t k = 0; k < _numJoints; ++k) {
        double diff = std::abs(end.position[k] - start.position[k]);
        if (diff > 0) {
            v_max = std::min(v_max, _maxJointVelocities[k] * 0.5);
            a_max = std::min(a_max, _maxJointAccelerations[k] * 0.5);
        }
    }

    // Calculate the total distance to be covered
    double total_distance = 0;
    for (size_t k = 0; k < _numJoints; ++k) {
        total_distance = std::max(total_distance, std::abs(end.position[k] - start.position[k]));
    }

    // Calculate the time for acceleration and deceleration phases
    double t_acc = v_max / a_max;
    double d_acc = 0.5 * a_max * t_acc * t_acc;
    double d_const = total_distance - 2 * d_acc;

    // Calculate the total time for the trajectory segment
    double total_time;
    if (d_const > 0) {
        double t_const = d_const / v_max;
        total_time = 2 * t_acc + t_const;
    } else {
        t_acc = std::sqrt(total_distance / a_max);
        total_time = 2 * t_acc;
    }

    int numPoints = static_cast<int>(total_time / 0.05); // At least 100 points or one every 10ms:
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
                // Acceleration phase
                s = 0.5 * a_max * t * t;
                v = a_max * t;
                a = a_max;
            } else if (t <= total_time - t_acc) {
                // Constant velocity phase (if it exists)
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

/**
 * @brief Performs the shortcutting algorithm outlined by Hauser et al. for trajectory optimization.
 *
 * This function implements the Hauser algorithm to generate an optimized trajectory
 * based on the waypoints set in the MotionGenerator. It creates a time-parameterized
 * path that satisfies kinematic constraints and avoids obstacles.
 *
 * @param maxIterations The maximum number of iterations for the shortcut algorithm.
 *                      This parameter is currently unused in the function body.
 * @param out A string parameter that may be used for output purposes. Its exact
 *            usage is not clear from the current implementation.
 *
 * @return This function doesn't return a value, but it populates the internal
 *         _path member with the optimized trajectory points.
 */
void MotionGenerator::performHauser(unsigned int maxIterations,
                                    const std::string &out,
                                    unsigned int outputFrequency)
{
    _path.clear();
    ParabolicRamp::DynamicPath traj;
    // Use optimized velocity and acceleration limits from parameter tuning
    traj.Init({1.93, 1.93, 1.93, 1.93, 1.93, 1.93, 1.93}, {0.523, 0.523, 0.523, 0.523, 0.523, 0.523, 0.523});

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
            // print the values contained in the vector out
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

    Eigen::Vector3d minPoint(-1.5, -1.5, -0.1);
    Eigen::Vector3d maxPoint(1.5, 1.5, 1.5);
    double resolution = 0.1;
    _sdfMinPoint = minPoint;
    _sdfMaxPoint = maxPoint;
    _sdfResolution = resolution;
    _sdf = _obstacleTree->toSDF(minPoint, maxPoint, resolution);
    _sdfInitialized = true;
}

Eigen::MatrixXd MotionGenerator::initializeTrajectory(Eigen::Matrix<double, 7, 1> goalVec,
                                                      Eigen::Matrix<double, 7, 1> startVec,
                                                      int N,
                                                      const int D)
{
    Eigen::MatrixXd theta = Eigen::MatrixXd::Zero(N, D);

    double start_index = 0;   // Starting index (first point)
    double end_index = N - 1; // Ending index (last point)

    double T[6]; // Powers of the time duration
    T[0] = 1.0;
    T[1] = (end_index - start_index); // Use normalized time over the trajectory length

    // Fill in higher powers of T
    for (int i = 2; i <= 5; ++i)
        T[i] = T[i - 1] * T[1];

    // Spline coefficients for each dimension
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

    // Fill in the trajectory points using the spline
    for (int i = 0; i < N; ++i) {
        double t[6]; // Powers of the current time step
        t[0] = 1.0;
        t[1] = i - start_index;

        // Fill in higher powers of t
        for (int k = 2; k <= 5; ++k)
            t[k] = t[k - 1] * t[1];

        // Compute trajectory for each dimension
        for (int d = 0; d < D; ++d) {
            theta(i, d) = 0.0;
            for (int k = 0; k <= 5; ++k) {
                theta(i, d) += t[k] * coeff(d, k);
            }
        }
    }

    return theta;
}

bool MotionGenerator::performSTOMP(const StompConfig &config,
                                   std::shared_ptr<boost::asio::thread_pool> sharedPool,
                                   int trajectoryIndex)
{
    QElapsedTimer timer;
    timer.start();
    
    // Create trajectory context for logging
    std::string context = trajectoryIndex >= 0 ? 
        "[Traj " + std::to_string(trajectoryIndex) + "] " : "";
    
    LOG_INFO << context << "Starting STOMP optimization (config: " 
             << config.numNoisyTrajectories << " samples, " 
             << config.maxIterations << " max iterations)";
    timer.restart();

    _path.clear();
    createSDF();
    initializeCostCalculator();

    // Use shared pool if provided, otherwise create local pool
    std::unique_ptr<boost::asio::thread_pool> localPool;
    boost::asio::thread_pool *pool;

    if (sharedPool) {
        pool = sharedPool.get();
    } else {
        // Create local pool with fewer threads to avoid oversubscription
        unsigned int numThreads = std::max(2u, std::thread::hardware_concurrency() / 2);
        localPool = std::make_unique<boost::asio::thread_pool>(numThreads);
        pool = localPool.get();
    }

    // Extract start and goal
    Eigen::Matrix<double, 1, Eigen::Dynamic> startVec = _waypoints.row(0);
    Eigen::Matrix<double, 1, Eigen::Dynamic> goalVec = _waypoints.row(_waypoints.rows() - 1);

    // Initialize trajectory points
    TrajectoryPoint start, end;
    for (int i = 0; i < config.numJoints; ++i) {
        start.position.push_back(startVec[i]);
        end.position.push_back(goalVec[i]);
        start.velocity.push_back(0.0);
        end.velocity.push_back(0.0);
        start.acceleration.push_back(0.0);
        end.acceleration.push_back(0.0);
    }

    auto segment = computeTimeOptimalSegment(start, end, 0.0);
    double estimatedTime = segment.back().time;
    double dt = config.dt;
    int N = static_cast<int>(estimatedTime / dt) + 1;
    Eigen::MatrixXd theta = initializeTrajectory(goalVec, startVec, N, config.numJoints);

    initializeMatrices(N, dt);

    auto limits = _arm.jointLimits();
    double prevTrajectoryCost = std::numeric_limits<double>::max();
    std::vector<double> prevSmoothnessCost(N, std::numeric_limits<double>::max());

    // Cost convergence parameters - Adjusted for optimized STOMP parameters
    // More lenient convergence to allow full optimization with our high-quality settings
    const double costConvergenceThreshold = 1e-6;  // Stricter threshold for better convergence
    const int convergencePatience = 3;             // More patience for our optimized algorithm
    int noChangeCounter = 0;

    // Early stopping configuration
    const bool enableEarlyStopping = config.enableEarlyStopping;
    const int earlyStoppingPatience = config.earlyStoppingPatience;
    int earlyStoppingCounter = 0;

    std::vector<std::pair<Eigen::MatrixXd, double>> bestSamples;

    timer.restart();
    bool success = false;
    Eigen::MatrixXd bestCollisionFreeTheta;
    double bestCollisionFreeCost = std::numeric_limits<double>::max();

    // Timer for overall STOMP execution time limit
    QElapsedTimer overallTimer;
    overallTimer.start();

    for (int iteration = 0; iteration < config.maxIterations; iteration++) {
        // Check time limit at start of each iteration
        if (config.maxComputeTimeMs > 0.0 && overallTimer.elapsed() > config.maxComputeTimeMs) {
            LOG_INFO << context << "Time limit exceeded (" << config.maxComputeTimeMs 
                     << "ms) after " << iteration << " iterations, terminating STOMP";
            if (!success) {
                throw StompTimeoutException("STOMP exceeded time limit without finding collision-free solution");
            } else {
                LOG_INFO << context << "Using best collision-free solution found before timeout (cost: " 
                         << std::fixed << std::setprecision(6) << bestCollisionFreeCost << ")";
                break;
            }
        }
        std::vector<Eigen::MatrixXd> noisyTrajectories(config.numNoisyTrajectories
                                                       + config.numBestSamples);

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

        for (int b = 0; b < std::min(config.numBestSamples, static_cast<int>(bestSamples.size()));
             b++) {
            noisyTrajectories[config.numNoisyTrajectories + b] = bestSamples[b].first;
        }

        int totalSamples = config.numNoisyTrajectories
                           + std::min(config.numBestSamples, static_cast<int>(bestSamples.size()));
        Eigen::VectorXd S(totalSamples);
        Eigen::VectorXd P(totalSamples);
        std::vector<double> totalCosts(totalSamples);

        promises.clear();
        promises.resize(totalSamples);
        futures.clear();

        for (auto &promise : promises) {
            futures.push_back(promise.get_future());
        }

        for (int k = 0; k < totalSamples; k++) {
            boost::asio::dispatch(*pool,
                                  [this, k, &noisyTrajectories, &totalCosts, &S, dt, &promises]() {
                                      try {
                                          double trajectoryCost
                                              = _costCalculator->computeCost(noisyTrajectories[k],
                                                                             dt);
                                          totalCosts[k] = trajectoryCost;
                                          S(k) = trajectoryCost;
                                          promises[k].set_value();
                                      } catch (...) {
                                          promises[k].set_exception(std::current_exception());
                                      }
                                  });
        }

        for (auto &future : futures) {
            future.wait();
        }

        double minCosts = S.minCoeff();
        double maxCosts = S.maxCoeff();
        double costRanges = maxCosts - minCosts;
        double h = config.temperature;

        if (costRanges < 1e-6) {
            P.setConstant(1.0 / config.numNoisyTrajectories);
        } else {
            Eigen::VectorXd expCosts = (-h * (S.array() - minCosts) / costRanges).exp();
            P = expCosts / expCosts.sum();
        }

        std::vector<std::pair<Eigen::MatrixXd, double>> iterationSamples;
        for (int k = 0; k < totalSamples; k++) {
            iterationSamples.push_back({noisyTrajectories[k], totalCosts[k]});
        }

        std::sort(iterationSamples.begin(),
                  iterationSamples.end(),
                  [](const auto &a, const auto &b) { return a.second < b.second; });

        bestSamples = std::vector<std::pair<Eigen::MatrixXd, double>>(
            iterationSamples.begin(),
            iterationSamples.begin()
                + std::min(config.numBestSamples, static_cast<int>(iterationSamples.size())));

        Eigen::MatrixXd deltaTheta = Eigen::MatrixXd::Zero(N, config.numJoints);
        for (int d = 0; d < config.numJoints; d++) {
            for (int k = 0; k < totalSamples; k++) {
                deltaTheta.col(d) += P(k) * (noisyTrajectories[k].col(d) - theta.col(d));
            }
        }

        Eigen::MatrixXd deltaS = smoothTrajectoryUpdate(deltaTheta);
        theta += 0.1 * deltaS;

        // Apply joint limits
        for (int i = 0; i < N - 1; i++) {
            for (int d = 0; d < config.numJoints; d++) {
                theta(i, d) = std::clamp(theta(i, d),
                                         std::get<0>(limits[d]),
                                         std::get<1>(limits[d]));
            }
        }

        // Parallel collision checking for better performance
        double trajectoryCost = _costCalculator->computeCost(theta, dt);
        double trajectoryCostDiff = std::abs(trajectoryCost - prevTrajectoryCost);
        prevTrajectoryCost = trajectoryCost;

        // Sparse collision checking optimization: check every 4th waypoint first
        std::atomic<bool> collides{false};
        const int numWaypoints = N - 2; // Skip start and end points
        const int sparseStep = 4; // Check every 4th waypoint first
        
        // First pass: Sparse collision checking (much faster)
        std::vector<std::future<void>> sparseFutures;
        
        for (int i = 1; i < N - 1; i += sparseStep) {
            sparseFutures.emplace_back(std::async(std::launch::async, [this, &theta, &collides, i]() {
                if (!collides.load()) {
                    RobotArm checkArm = _arm;
                    checkArm.setJointAngles(theta.row(i));
                    if (armHasCollision(checkArm)) {
                        collides.store(true);
                    }
                }
            }));
        }
        
        // Wait for sparse check
        for (auto& future : sparseFutures) {
            future.wait();
        }
        
        // Second pass: Full collision checking only if sparse check passed
        if (!collides.load()) {
            const int numThreads = std::min(4, std::max(1, numWaypoints / 8));
            const int waypointsPerThread = std::max(1, numWaypoints / numThreads);
            
            std::vector<std::future<void>> collisionFutures;
            
            for (int t = 0; t < numThreads && !collides.load(); ++t) {
                int startIdx = t * waypointsPerThread + 1;
                int endIdx = (t == numThreads - 1) ? N - 1 : startIdx + waypointsPerThread;
                
                collisionFutures.emplace_back(std::async(std::launch::async, [this, &theta, &collides, startIdx, endIdx]() {
                    RobotArm checkArm = _arm;
                    for (int i = startIdx; i < endIdx && !collides.load(); ++i) {
                        checkArm.setJointAngles(theta.row(i));
                        if (armHasCollision(checkArm)) {
                            collides.store(true);
                            return;
                        }
                    }
                }));
            }
            
            // Wait for all threads to complete
            for (auto& future : collisionFutures) {
                future.wait();
            }
        }

        if (!collides.load()) {
            success = true;
            if (trajectoryCost < bestCollisionFreeCost) {
                bestCollisionFreeCost = trajectoryCost;
                bestCollisionFreeTheta = theta;
                LOG_DEBUG << context << "Found better collision-free solution at iteration " << iteration
                         << " with cost " << std::fixed << std::setprecision(6) << trajectoryCost;
            }
            
            // Early stopping: increment counter when collision-free trajectory is found
            if (enableEarlyStopping) {
                earlyStoppingCounter++;
                LOG_DEBUG << context << "Collision-free trajectory found (early stopping: " 
                         << earlyStoppingCounter << "/" << earlyStoppingPatience << ")";
                
                if (earlyStoppingCounter >= earlyStoppingPatience) {
                    LOG_INFO << context << "Early stopping triggered - found collision-free solution after " 
                             << iteration + 1 << " iterations (cost: " 
                             << std::fixed << std::setprecision(6) << bestCollisionFreeCost << ")";
                    break;
                }
            }
        } else {
            // Reset early stopping counter if we hit collision again
            if (enableEarlyStopping) {
                earlyStoppingCounter = 0;
            }
        }

        if (trajectoryCostDiff < costConvergenceThreshold) {
            noChangeCounter++;
            if (noChangeCounter >= convergencePatience && success) {
                LOG_INFO << context << "Cost converged after " << iteration + 1
                         << " iterations with collision-free solution (cost: " 
                         << std::fixed << std::setprecision(6) << bestCollisionFreeCost << ")";
                break;
            }
        } else {
            noChangeCounter = 0; // Reset if cost changed significantly
        }
    }

    LOG_FAST_DEBUG << "Step 6 - Optimization loop: " << timer.elapsed() << "ms";

    if (success && bestCollisionFreeTheta.rows() > 0) {
        LOG_INFO << context << "STOMP completed successfully - cost: " 
                 << std::fixed << std::setprecision(6) << bestCollisionFreeCost
                 << " (duration: " << timer.elapsed() << "ms)";
        theta = bestCollisionFreeTheta;
    } else {
        LOG_WARNING << context << "STOMP failed - no collision-free solution found after " 
                   << config.maxIterations << " iterations (duration: " << timer.elapsed() << "ms)";
        // Throw exception to enable fallback to BiRRT+Hauser in hybrid approach
        throw StompFailedException("STOMP failed to find collision-free solution");
    }

    // Convert theta to trajectory points
    for (int i = 0; i < N; ++i) {
        TrajectoryPoint point;
        point.time = i * dt;
        point.position.resize(config.numJoints);
        point.velocity.resize(config.numJoints);
        point.acceleration.resize(config.numJoints);

        for (int d = 0; d < config.numJoints; ++d) {
            point.position[d] = theta(i, d);
        }

        // Calculate velocities and accelerations using finite differences
        if (i == 0) {
            for (int d = 0; d < config.numJoints; ++d) {
                point.velocity[d] = (theta(1, d) - theta(0, d)) / dt;
                point.acceleration[d] = (theta(2, d) - 2 * theta(1, d) + theta(0, d)) / (dt * dt);
            }
        } else if (i == N - 1) {
            for (int d = 0; d < config.numJoints; ++d) {
                point.velocity[d] = (theta(N - 1, d) - theta(N - 2, d)) / dt;
                point.acceleration[d] = (theta(N - 1, d) - 2 * theta(N - 2, d) + theta(N - 3, d))
                                        / (dt * dt);
            }
        } else {
            for (int d = 0; d < config.numJoints; ++d) {
                point.velocity[d] = (theta(i + 1, d) - theta(i - 1, d)) / (2 * dt);
                point.acceleration[d] = (theta(i + 1, d) - 2 * theta(i, d) + theta(i - 1, d))
                                        / (dt * dt);
            }
        }
        _path.push_back(point);
    }

    // _path = computeTimeOptimalScaling(_path);

    return success;
}

Eigen::MatrixXd MotionGenerator::smoothTrajectoryUpdate(const Eigen::MatrixXd &dTheta)
{
    // Apply the smoothing matrix to the trajectory data
    Eigen::MatrixXd smoothed = _M * dTheta;

    return smoothed;
}

double MotionGenerator::computeObstacleCost(const RobotArm &curArm,
                                            const RobotArm &prevArm,
                                            double dt)
{
    double cost = 0;
    auto bboxesNew = curArm.getLinkBoundingBoxes();
    auto bboxesOld = prevArm.getLinkBoundingBoxes();

    for (int iter = 3; iter <= bboxesNew.size() - 1; ++iter) {
        auto [center, halfDims, axes] = bboxesNew[iter];
        auto [centerOld, halfDimsOld, axesOld] = bboxesOld[iter];

        Eigen::Vector3d gridCoords = (center - _sdfMinPoint) / _sdfResolution;
        int i = static_cast<int>(gridCoords.x());
        int j = static_cast<int>(gridCoords.y());
        int k = static_cast<int>(gridCoords.z());

        double radius = 0;
        for (int j = 0; j < 3; ++j) {
            Eigen::Vector3d corner = halfDims.x() * axes.col(0) + halfDims.y() * axes.col(1)
                                     + halfDims.z() * axes.col(2);
            radius = 0.75 * std::max(radius, corner.norm());
        }

        if (i >= 0 && i < _sdf.size() && j >= 0 && j < _sdf[0].size() && k >= 0
            && k < _sdf[0][0].size()) {
            double dist = _sdf[i][j][k];
            double diff = radius + 0.3 - dist;

            // Modified section: Exponential scaling
            if (diff > 0) {
                cost += diff * ((center - centerOld) / dt).norm();
            }
        }
    }

    return cost;
}

double MotionGenerator::computeTrajectoryCost(const Eigen::MatrixXd &theta, double dt)
{
    double cost = 0;
    int N = theta.rows();
    int D = theta.cols();

    Eigen::MatrixXd velocity = Eigen::MatrixXd::Zero(N - 1, D);
    Eigen::MatrixXd acceleration = Eigen::MatrixXd::Zero(N - 2, D);

    for (int i = 0; i < N - 1; ++i) {
        velocity.row(i) = (theta.row(i + 1) - theta.row(i)) / dt;
    }

    for (int i = 0; i < N - 2; ++i) {
        acceleration.row(i) = (velocity.row(i + 1) - velocity.row(i)) / dt;
    }

    for (int i = 0; i < N - 1; ++i) {
        for (int j = 0; j < D; ++j) {
            cost += 0.1 * std::max(0., std::abs(velocity(i, j)) - _maxJointVelocities[j]);
        }
    }

    for (int i = 0; i < N - 2; ++i) {
        for (int j = 0; j < D; ++j) {
            cost += 0.1 * std::max(0., std::abs(acceleration(i, j)) - _maxJointAccelerations[j]);
        }
    }

    RobotArm startArm = _arm;
    startArm.setJointAngles(theta.row(0));
    RobotArm prevArm = startArm;

    for (int i = 1; i < N; ++i) {
        RobotArm curArm = prevArm;
        curArm.setJointAngles(theta.row(i));
        // Eigen::VectorXd torques = computeTorques(curArm, velocity.row(i), acceleration.row(i));
        cost += computeObstacleCost(curArm, prevArm, dt);
        // qDebug() << "Torques:" << torques(3);
        prevArm = curArm;
    }

    return cost;
}

Eigen::VectorXd MotionGenerator::computeTorques(const RobotArm &arm,
                                                const Eigen::VectorXd &velocities,
                                                const Eigen::VectorXd &accelerations)
{
    int D = velocities.size();
    Eigen::VectorXd torques(D);

    std::vector<double> masses = arm.getLinkMasses();
    std::vector<Eigen::Vector3d> centersOfMass = arm.getLinkCentersOfMass();
    std::vector<Eigen::Matrix3d> inertias = arm.getLinkInertias();

    Eigen::MatrixXd J(6, D);
    arm.computeJacobian(J);

    Eigen::Vector3d g(0, 0, -9.81);

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(D, D);
    Eigen::VectorXd C = Eigen::VectorXd::Zero(D);
    Eigen::VectorXd G = Eigen::VectorXd::Zero(D);

    for (int i = 0; i < D; ++i) {
        Eigen::Vector3d r = centersOfMass[i] - arm.getJointPosition(i);
        Eigen::Matrix3d R = arm.getJointRotation(i);

        M += masses[i] * J.block(0, 0, 3, D).transpose() * J.block(0, 0, 3, D)
             + J.block(3, 0, 3, D).transpose() * R * inertias[i] * R.transpose()
                   * J.block(3, 0, 3, D);

        C += (masses[i] * J.block(0, 0, 3, D).transpose() * J.block(0, 0, 3, D) * velocities)
                 .cwiseProduct(velocities);

        G += masses[i] * J.block(0, 0, 3, D).transpose() * g;
    }

    torques = M * accelerations + C + G;

    return torques;
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
        _M.col(i) *= (1.0 / N) / _M.col(i).cwiseAbs().maxCoeff();
    }

    _matricesInitialized = true;
}

void MotionGenerator::saveTrajectoryToCSV(const std::string &filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }
    // Write header
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

    // Write data
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
    // std::cout << "Trajectory saved to " << filename << std::endl;
}

void MotionGenerator::setExplorationConstant(double newExplorationConstant)
{
    _explorationConstant = newExplorationConstant;
}

std::vector<double> MotionGenerator::computeEndeffectorSpeedOverPath()
{
    std::vector<double> endeffectorSpeed;
    for (int i = 0; i < _path.size(); i++) {
        double speed = 0;
        for (int j = 0; j < 3; j++) {
            speed += _path[i].velocity[j] * _path[i].velocity[j];
        }
        endeffectorSpeed.push_back(std::sqrt(speed));
    }
    return endeffectorSpeed;
}

std::vector<Eigen::VectorXd> MotionGenerator::generateQuinticPolynomialCoeffs(
    const Eigen::VectorXd &start, const Eigen::VectorXd &end, double T)
{
    std::vector<Eigen::VectorXd> coeffs;
    for (int i = 0; i < start.size(); ++i) {
        Eigen::Matrix<double, 6, 6> A;
        A << 0, 0, 0, 0, 0, 1, T * T * T * T * T, T * T * T * T, T * T * T, T * T, T, 1, 0, 0, 0, 0,
            1, 0, 5 * T * T * T * T, 4 * T * T * T, 3 * T * T, 2 * T, 1, 0, 0, 0, 0, 2, 0, 0,
            20 * T * T * T, 12 * T * T, 6 * T, 2, 0, 0;

        Eigen::VectorXd b(6);
        b << start(i), end(i), 0, 0, 0, 0;

        Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
        coeffs.push_back(x);
    }
    return coeffs;
}

Eigen::VectorXd MotionGenerator::evaluateQuinticPolynomial(
    const std::vector<Eigen::VectorXd> &coeffs, double t)
{
    Eigen::VectorXd result(coeffs.size());
    for (size_t i = 0; i < coeffs.size(); ++i) {
        const auto &c = coeffs[i];
        result(i) = c(0) * t * t * t * t * t + c(1) * t * t * t * t + c(2) * t * t * t
                    + c(3) * t * t + c(4) * t + c(5);
    }
    return result;
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

double ObstacleCostCalculator::computeCost(const Eigen::MatrixXd &trajectory, double dt)
{
    double cost = 0;
    int N = trajectory.rows();

    RobotArm currentArm = _arm;
    currentArm.setJointAngles(trajectory.row(0));
    RobotArm prevArm = currentArm;

    bool collides = false;
    for (int i = 0; i < N; ++i) {
        RobotArm curArm = _arm;
        curArm.setJointAngles(trajectory.row(i));

        auto bboxesNew = curArm.getLinkBoundingBoxes();
        auto bboxesOld = prevArm.getLinkBoundingBoxes();

        for (int iter = 0; iter < bboxesNew.size(); ++iter) {
            auto [center, halfDims, axes] = bboxesNew[iter];
            auto [centerOld, halfDimsOld, axesOld] = bboxesOld[iter];

            Eigen::Vector3d gridCoords = (center - _sdfMinPoint) / _sdfResolution;
            int i = static_cast<int>(gridCoords.x());
            int j = static_cast<int>(gridCoords.y());
            int k = static_cast<int>(gridCoords.z());

            double radius = 0;
            for (int j = 0; j < 3; ++j) {
                Eigen::Vector3d corner = halfDims.x() * axes.col(0) + halfDims.y() * axes.col(1)
                                         + halfDims.z() * axes.col(2);
                radius = 0.5 * std::max(radius, corner.norm());
            }

            if (i >= 0 && i < _sdf.size() && j >= 0 && j < _sdf[0].size() && k >= 0
                && k < _sdf[0][0].size()) {
                double dist = _sdf[i][j][k];
                cost += std::max(radius + 0.05 - dist, 0.0) * ((center - centerOld) / dt).norm();
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
                    collides = true;
                    break;
                }
            }
        }

        if (collides) {
            cost += 1.;
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
            cost += 0.1 * std::max(0., std::abs(velocity(i, j)) - _maxJointVelocities[j]);
        }
    }

    for (int i = 0; i < N - 2; ++i) {
        for (int j = 0; j < D; ++j) {
            cost += 0.1 * std::max(0., std::abs(acceleration(i, j)) - _maxJointAccelerations[j]);
        }
    }

    return cost;
}

double TaskSpacePathTrackingCostCalculator::computeCost(const Eigen::MatrixXd &trajectory, double dt)
{
    double cost = 0.0;
    int N = trajectory.rows();

    if (_taskSpacePositions.size() < 2) {
        return 0.0; // Not enough checkpoints for interpolation
    }

    RobotArm checkArm = _arm;

    for (int i = 0; i < N; ++i) {
        checkArm.setJointAngles(trajectory.row(i));
        Eigen::Affine3d currentPose = checkArm.getEndeffectorPose();

        double s = static_cast<double>(i) / (N - 1);

        auto [targetPos, targetOrient] = interpolateTaskSpacePath(s);

        double posDeviation = (currentPose.translation() - targetPos).norm();
        cost += _positionWeight * posDeviation * posDeviation;

        Eigen::Matrix3d orientDiff = currentPose.linear() * targetOrient.transpose();
        Eigen::AngleAxisd angleAxis(orientDiff);
        double orientDeviation = std::abs(angleAxis.angle());
        cost += _orientationWeight * orientDeviation * orientDeviation;
    }
    return cost;
}

std::pair<Eigen::Vector3d, Eigen::Matrix3d>
TaskSpacePathTrackingCostCalculator::interpolateTaskSpacePath(double s)
{
    double targetDist = s * _totalDistance;

    size_t segment = 0;
    while (segment < _cumulativeDistances.size() - 1
           && _cumulativeDistances[segment + 1] < targetDist) {
        segment++;
    }

    if (segment >= _cumulativeDistances.size() - 1) {
        return {_taskSpacePositions.back(), _taskSpaceOrientations.back()};
    }

    double segmentStart = _cumulativeDistances[segment];
    double segmentEnd = _cumulativeDistances[segment + 1];
    double segmentLength = segmentEnd - segmentStart;

    double alpha = 0.0;
    if (segmentLength > 1e-6) {
        alpha = (targetDist - segmentStart) / segmentLength;
    }

    Eigen::Vector3d interpolatedPos = _taskSpacePositions[segment] * (1.0 - alpha)
                                      + _taskSpacePositions[segment + 1] * alpha;

    Eigen::Quaterniond q1(_taskSpaceOrientations[segment]);
    Eigen::Quaterniond q2(_taskSpaceOrientations[segment + 1]);
    Eigen::Quaterniond q = q1.slerp(alpha, q2);

    return {interpolatedPos, q.toRotationMatrix()};
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

void MotionGenerator::initializeCostCalculator()
{
    _costCalculator = std::make_unique<CompositeCostCalculator>();

    _costCalculator->addCostCalculator(std::make_unique<ObstacleCostCalculator>(_arm,
                                                                                _obstacleTree,
                                                                                _sdf,
                                                                                _sdfMinPoint,
                                                                                _sdfMaxPoint,
                                                                                _sdfResolution),
                                       1.0);
}

void MotionGenerator::initializeCostCalculatorCheckpoints(
    const std::vector<Eigen::VectorXd> &checkpoints)
{
    _costCalculator = std::make_unique<CompositeCostCalculator>();

    // _costCalculator->addCostCalculator(std::make_unique<ObstacleCostCalculator>(_arm,
    //                                                                             _obstacleTree,
    //                                                                             _sdf,
    //                                                                             _sdfMinPoint,
    //                                                                             _sdfMaxPoint,
    //                                                                             _sdfResolution),
    //                                    5.);

    // _costCalculator->addCostCalculator(
    //     std::make_unique<TaskSpacePathTrackingCostCalculator>(_arm, checkpoints, 1.0, 1.0), 2.0);
}

Eigen::MatrixXd MotionGenerator::generateNoisyTrajectory(
    const Eigen::MatrixXd &baseTrajectory,
    const Eigen::VectorXd &stdDevs,
    const std::vector<std::pair<double, double>> &limits)
{
    int numPoints = baseTrajectory.rows();
    int numJoints = baseTrajectory.cols();

    Eigen::MatrixXd noisyTrajectory = baseTrajectory;
    Eigen::MatrixXd epsilon = Eigen::MatrixXd::Zero(numPoints, numJoints);

    std::random_device rd;
    std::mt19937 gen(rd());

    // Generate correlated noise for each joint
    for (int d = 0; d < numJoints; d++) {
        std::normal_distribution<> dist(0, stdDevs[d]);
        epsilon.col(d) = _L * Eigen::VectorXd::NullaryExpr(numPoints, [&]() { return dist(gen); });
    }

    // Apply noise
    noisyTrajectory += epsilon;

    // Apply joint limits
    for (int i = 0; i < numPoints; i++) {
        for (int d = 0; d < numJoints; d++) {
            noisyTrajectory(i, d) = std::clamp(noisyTrajectory(i, d),
                                               std::get<0>(limits[d]),
                                               std::get<1>(limits[d]));
        }
    }

    return noisyTrajectory;
}

Eigen::MatrixXd MotionGenerator::applyProbabilisticUpdate(
    const Eigen::MatrixXd &currentTrajectory,
    const std::vector<Eigen::MatrixXd> &sampleTrajectories,
    const Eigen::VectorXd &probabilities,
    double learningRate)
{
    int N = currentTrajectory.rows();
    int D = currentTrajectory.cols();

    Eigen::MatrixXd deltaTheta = Eigen::MatrixXd::Zero(N, D);

    // Apply weighted update from samples
    for (int d = 0; d < D; d++) {
        for (size_t k = 0; k < sampleTrajectories.size(); k++) {
            deltaTheta.col(d) += probabilities(k)
                                 * (sampleTrajectories[k].col(d) - currentTrajectory.col(d));
        }
    }

    // Apply smoothing
    Eigen::MatrixXd deltaS = smoothTrajectoryUpdate(deltaTheta);

    // Return updated trajectory
    return currentTrajectory + learningRate * deltaS;
}

/**
 * @brief Applies time-optimal scaling to the trajectory while respecting joint constraints
 * 
 * This function takes a collision-free trajectory from STOMP and scales it in time
 * to execute as fast as possible while respecting velocity, acceleration, and jerk limits.
 * Uses a phase-plane approach for time-optimal scaling.
 * 
 * @param jerkLimit Factor to limit jerk (set between 0.0-1.0, where lower values produce smoother motion)
 * @return True if successful, false otherwise
 */
bool MotionGenerator::finaliseTrajectory(double jerkLimit)
{
    if (_path.empty()) {
        return false;
    }

    QElapsedTimer timer;
    timer.start();
    LOG_INFO << "Finalising trajectory for execution...";

    // Step 1: Extract path data for optimization
    int numPoints = _path.size();
    int numJoints = _path[0].position.size();

    // Create splines for smooth interpolation
    std::vector<std::vector<double>> jointPositions(numJoints);
    std::vector<double> timePoints;

    for (const auto &point : _path) {
        timePoints.push_back(point.time);
        for (int i = 0; i < numJoints; i++) {
            jointPositions[i].push_back(point.position[i]);
        }
    }

    // Step 2: Create path parameter (s) representation with normalized arc length
    std::vector<double> pathParams(numPoints);
    pathParams[0] = 0.0;

    for (int i = 1; i < numPoints; i++) {
        double segmentLength = 0.0;
        for (int j = 0; j < numJoints; j++) {
            double diff = jointPositions[j][i] - jointPositions[j][i - 1];
            segmentLength += diff * diff;
        }
        pathParams[i] = pathParams[i - 1] + std::sqrt(segmentLength);
    }

    // Normalize path parameter to [0,1]
    double totalLength = pathParams.back();
    for (auto &param : pathParams) {
        param /= totalLength;
    }

    // Step 3: Compute velocity and acceleration limits along the path
    std::vector<double> maxVelocities(numPoints);
    std::vector<double> maxAccelerations(numPoints);

    for (int i = 0; i < numPoints; i++) {
        // Find the most restrictive joint at each point
        double minVelocityLimit = std::numeric_limits<double>::max();
        double minAccelerationLimit = std::numeric_limits<double>::max();

        for (int j = 0; j < numJoints; j++) {
            // Compute path derivatives
            double velocityLimit = _maxJointVelocities[j] / totalLength;
            double accelerationLimit = _maxJointAccelerations[j] / totalLength;

            minVelocityLimit = std::min(minVelocityLimit, velocityLimit);
            minAccelerationLimit = std::min(minAccelerationLimit, accelerationLimit);
        }

        maxVelocities[i] = minVelocityLimit;
        maxAccelerations[i] = minAccelerationLimit * (1.0 - jerkLimit); // Apply jerk limiting
    }

    // Step 4: Apply phase-plane optimization for time-optimal scaling
    std::vector<double> timeScaling(numPoints);
    std::vector<double> velocityProfile(numPoints);

    // Initial and final velocities are zero
    timeScaling[0] = 0.0;
    velocityProfile[0] = 0.0;
    velocityProfile[numPoints - 1] = 0.0;

    // Forward pass (maximum acceleration)
    for (int i = 1; i < numPoints; i++) {
        double ds = pathParams[i] - pathParams[i - 1];
        double vPrev = velocityProfile[i - 1];
        double vMax = maxVelocities[i];
        double aMax = maxAccelerations[i];

        // Apply maximum acceleration constraint
        double vNext = std::sqrt(vPrev * vPrev + 2 * aMax * ds);
        velocityProfile[i] = std::min(vNext, vMax);

        timeScaling[i] = timeScaling[i - 1]
                         + 2 * ds / (velocityProfile[i - 1] + velocityProfile[i]);
    }

    // Backward pass (ensure we can stop at the end)
    for (int i = numPoints - 2; i >= 0; i--) {
        double ds = pathParams[i + 1] - pathParams[i];
        double vNext = velocityProfile[i + 1];
        double aMax = maxAccelerations[i];

        // Maximum velocity allowing deceleration to next point
        double vMaxDecel = std::sqrt(vNext * vNext + 2 * aMax * ds);
        if (velocityProfile[i] > vMaxDecel) {
            velocityProfile[i] = vMaxDecel;
            // Update timing
            timeScaling[i + 1] = timeScaling[i]
                                 + 2 * ds / (velocityProfile[i] + velocityProfile[i + 1]);
        }
    }

    // Step 5: Generate the final time-optimal trajectory
    std::vector<TrajectoryPoint> finalPath;

    // Create splines for smooth interpolation of velocity and acceleration
    std::vector<boost::math::interpolators::cardinal_quintic_b_spline<double>> jointSplines;
    for (int j = 0; j < numJoints; j++) {
        boost::math::interpolators::cardinal_quintic_b_spline<double> spline(jointPositions[j],
                                                                             0.0,
                                                                             1.0,
                                                                             {0.0, 0.0},
                                                                             {0.0, 0.0});
        jointSplines.push_back(spline);
    }

    // Generate dense trajectory with optimized timing
    double finalTime = timeScaling.back();
    double dt = 0.01; // 10ms sampling

    for (double t = 0; t <= finalTime; t += dt) {
        // Find the path parameter at this time using binary search
        double s = 0.0;
        int idx = 0;

        while (idx < numPoints - 1 && timeScaling[idx + 1] <= t) {
            idx++;
        }

        if (idx < numPoints - 1) {
            // Interpolate path parameter
            double alpha = (t - timeScaling[idx]) / (timeScaling[idx + 1] - timeScaling[idx]);
            s = pathParams[idx] + alpha * (pathParams[idx + 1] - pathParams[idx]);
        } else {
            s = pathParams.back();
        }

        // Create trajectory point
        TrajectoryPoint point;
        point.time = t;
        point.position.resize(numJoints);
        point.velocity.resize(numJoints);
        point.acceleration.resize(numJoints);

        // Evaluate splines at the path parameter s
        for (int j = 0; j < numJoints; j++) {
            point.position[j] = jointSplines[j](s);

            // Velocity = (ds/dt) * (dx/ds)
            double ds_dt = velocityProfile[idx];
            double dx_ds = jointSplines[j].prime(s);
            point.velocity[j] = ds_dt * dx_ds;

            // Acceleration calculation with chain rule
            double dds_dt = (idx < numPoints - 1)
                                ? (velocityProfile[idx + 1] - velocityProfile[idx])
                                      / (timeScaling[idx + 1] - timeScaling[idx])
                                : 0.0;
            double ddx_dss = jointSplines[j].double_prime(s);
            point.acceleration[j] = dds_dt * dx_ds + ds_dt * ds_dt * ddx_dss;
        }

        finalPath.push_back(point);
    }

    _path = finalPath;
    LOG_INFO << "Trajectory finalised in " << timer.elapsed() << "ms. Duration: " << finalTime
             << "s, Points: " << _path.size();

    return true;
}

std::vector<MotionGenerator::TrajectoryPoint> MotionGenerator::generateTrajectoryFromCheckpoints(
    const std::vector<Eigen::VectorXd> &checkpoints)
{
    std::vector<TrajectoryPoint> trajectory;
    if (checkpoints.empty())
        return trajectory;

    const size_t n_joints = _maxJointVelocities.size();
    const size_t n_checkpoints = checkpoints.size();

    // Early exit for single checkpoint
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

    // Convert member vectors to Eigen types
    Eigen::VectorXd max_vel = Eigen::Map<const Eigen::VectorXd>(_maxJointVelocities.data(),
                                                                n_joints);
    Eigen::VectorXd max_acc = Eigen::Map<const Eigen::VectorXd>(_maxJointAccelerations.data(),
                                                                n_joints);

    // 1. Compute segment durations based on velocity and acceleration limits
    std::vector<double> segment_times;
    std::vector<double> absolute_times{0.0};

    for (size_t i = 1; i < n_checkpoints; ++i) {
        Eigen::VectorXd delta = (checkpoints[i] - checkpoints[i - 1]).cwiseAbs();

        // Time based on velocity limits
        Eigen::VectorXd t_vel = delta.cwiseQuotient(max_vel);
        double t_vel_max = t_vel.maxCoeff();

        // Time based on acceleration limits (for quintic with rest constraints)
        Eigen::VectorXd t_acc = (delta * 15.0 / 2.0).cwiseSqrt().cwiseQuotient(max_acc.cwiseSqrt());
        double t_acc_max = t_acc.maxCoeff();

        double segment_time = std::max(t_vel_max, t_acc_max);
        segment_times.push_back(segment_time);
        absolute_times.push_back(absolute_times.back() + segment_time);
    }

    // 2. Compute velocities/accelerations for interior checkpoints
    std::vector<Eigen::VectorXd> velocities(n_checkpoints, Eigen::VectorXd::Zero(n_joints));
    std::vector<Eigen::VectorXd> accelerations(n_checkpoints, Eigen::VectorXd::Zero(n_joints));

    // First and last checkpoint: zero velocity/acceleration (start and stop)
    // Leave as initialized with zeros

    // Interior checkpoints: compute for continuity
    for (size_t i = 1; i < n_checkpoints - 1; ++i) {
        // Compute velocity using central difference approximation
        const double dt_prev = segment_times[i - 1];
        const double dt_next = segment_times[i];
        const double dt_total = dt_prev + dt_next;

        for (size_t j = 0; j < n_joints; ++j) {
            // Central difference for velocity
            velocities[i][j] = (checkpoints[i + 1][j] - checkpoints[i - 1][j]) / dt_total;

            // Clamp velocity to limits
            velocities[i][j] = std::clamp(velocities[i][j], -max_vel[j], max_vel[j]);
        }
    }

    // 3. Generate quintic polynomial coefficients
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

            // Position at t=0
            c[0] = q0;

            // Velocity at t=0
            c[1] = v0;

            // Acceleration at t=0
            c[2] = a0 / 2.0;

            // Solve for remaining coefficients to enforce final conditions
            const double T2 = T * T;
            const double T3 = T2 * T;
            const double T4 = T3 * T;
            const double T5 = T4 * T;

            Eigen::Matrix3d A;
            A << T3, T4, T5, 3 * T2, 4 * T3, 5 * T4, 6 * T, 12 * T2, 20 * T3;

            Eigen::Vector3d b;
            b << qf - q0 - v0 * T - 0.5 * a0 * T2, vf - v0 - a0 * T, af - a0;

            Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
            
            // Assign the solved coefficients to complete the quintic polynomial
            c[3] = x[0];
            c[4] = x[1]; 
            c[5] = x[2];
            
            coeffs[j][i] = c;
        }
    }

    // 4. Sample trajectory at control rate
    const double control_rate = 1000.0;
    const double dt = 1.0 / control_rate;
    const double total_time = absolute_times.back();

    for (double t = 0.0; t <= total_time + 1e-6; t += dt) {
        TrajectoryPoint pt;
        pt.position.resize(n_joints);
        pt.velocity.resize(n_joints);
        pt.acceleration.resize(n_joints);
        pt.time = t;

        // Find current segment (clamp t to total_time)
        const double clamped_t = std::min(t, total_time);
        size_t segment_idx = 0;
        while (segment_idx < segment_times.size() - 1
               && clamped_t > absolute_times[segment_idx + 1]) {
            ++segment_idx;
        }

        const double t_segment = clamped_t - absolute_times[segment_idx];

        for (size_t j = 0; j < n_joints; ++j) {
            const Eigen::VectorXd &c = coeffs[j][segment_idx];

            // Position: c0 + c1*t + c2*t² + c3*t³ + c4*t⁴ + c5*t⁵
            pt.position[j] = c[0] + c[1] * t_segment + c[2] * t_segment * t_segment
                             + c[3] * t_segment * t_segment * t_segment
                             + c[4] * t_segment * t_segment * t_segment * t_segment
                             + c[5] * t_segment * t_segment * t_segment * t_segment * t_segment;

            // Velocity: c1 + 2*c2*t + 3*c3*t² + 4*c4*t³ + 5*c5*t⁴
            pt.velocity[j] = c[1] + 2 * c[2] * t_segment + 3 * c[3] * t_segment * t_segment
                             + 4 * c[4] * t_segment * t_segment * t_segment
                             + 5 * c[5] * t_segment * t_segment * t_segment * t_segment;

            // Acceleration: 2*c2 + 6*c3*t + 12*c4*t² + 20*c5*t³
            pt.acceleration[j] = 2 * c[2] + 6 * c[3] * t_segment + 12 * c[4] * t_segment * t_segment
                                 + 20 * c[5] * t_segment * t_segment * t_segment;
        }

        trajectory.push_back(pt);
    }

    return trajectory;
}

std::vector<MotionGenerator::TrajectoryPoint> MotionGenerator::computeTimeOptimalScaling(
    const std::vector<TrajectoryPoint> &path)
{
    if (path.empty())
        return {};

    const size_t numPoints = path.size();
    const size_t numJoints = path[0].position.size();

    if (numPoints < 2) {
        qWarning() << "Path has insufficient points for time-optimal scaling";
        return path;
    }

    LOG_INFO << "Starting improved time-optimal scaling with " << numPoints << " points";

    // 1. Create robust arc length parameterization
    std::vector<double> s(numPoints);
    std::vector<double> ds(numPoints - 1);
    s[0] = 0.0;

    double totalLength = 0.0;
    for (size_t i = 1; i < numPoints; ++i) {
        double segmentLength = 0.0;
        for (size_t j = 0; j < numJoints; ++j) {
            double diff = path[i].position[j] - path[i - 1].position[j];
            segmentLength += diff * diff;
        }
        ds[i - 1] = std::sqrt(segmentLength);
        s[i] = s[i - 1] + ds[i - 1];
        totalLength += ds[i - 1];
    }

    if (totalLength < 1e-10) {
        qWarning() << "Path has zero length, returning original";
        return path;
    }

    LOG_DEBUG << "Path length: " << totalLength << " radians";

    // 2. Compute path derivatives using robust finite differences
    std::vector<std::vector<double>> q_dot(numPoints, std::vector<double>(numJoints, 0.0));
    std::vector<std::vector<double>> q_ddot(numPoints, std::vector<double>(numJoints, 0.0));

    // First derivatives (velocity profile in path parameter space)
    for (size_t i = 0; i < numPoints; ++i) {
        for (size_t j = 0; j < numJoints; ++j) {
            if (i == 0 && numPoints > 1) {
                // Forward difference
                if (ds[0] > 1e-10) {
                    q_dot[i][j] = (path[1].position[j] - path[0].position[j]) / ds[0];
                }
            } else if (i == numPoints - 1) {
                // Backward difference
                if (ds[i - 1] > 1e-10) {
                    q_dot[i][j] = (path[i].position[j] - path[i - 1].position[j]) / ds[i - 1];
                }
            } else {
                // Central difference with proper weighting
                double ds_total = ds[i - 1] + ds[i];
                if (ds_total > 1e-10) {
                    q_dot[i][j] = (path[i + 1].position[j] - path[i - 1].position[j]) / ds_total;
                }
            }
        }
    }

    // Apply Gaussian smoothing to derivatives to reduce noise
    const int smoothingRadius = std::min(3, static_cast<int>(numPoints / 10));
    if (smoothingRadius > 0) {
        for (int iter = 0; iter < 2; ++iter) {
            std::vector<std::vector<double>> smoothed_q_dot = q_dot;
            for (size_t i = smoothingRadius; i < numPoints - smoothingRadius; ++i) {
                for (size_t j = 0; j < numJoints; ++j) {
                    double sum = 0.0;
                    double weight_sum = 0.0;
                    for (int k = -smoothingRadius; k <= smoothingRadius; ++k) {
                        double weight = std::exp(-0.5 * k * k); // Gaussian kernel
                        sum += weight * q_dot[i + k][j];
                        weight_sum += weight;
                    }
                    smoothed_q_dot[i][j] = sum / weight_sum;
                }
            }
            q_dot = smoothed_q_dot;
        }
    }

    // Second derivatives (acceleration profile)
    for (size_t i = 1; i < numPoints - 1; ++i) {
        for (size_t j = 0; j < numJoints; ++j) {
            double ds_avg = 0.5 * (ds[i - 1] + ds[i]);
            if (ds_avg > 1e-10) {
                q_ddot[i][j] = (q_dot[i + 1][j] - q_dot[i - 1][j]) / (2.0 * ds_avg);
            }
        }
    }

    // 3. Compute velocity limits with better constraint handling
    std::vector<double> v_max(numPoints);
    const double SAFETY_FACTOR = 0.75; // 5% safety margin

    for (size_t i = 0; i < numPoints; ++i) {
        v_max[i] = std::numeric_limits<double>::max();

        for (size_t j = 0; j < numJoints; ++j) {
            double abs_q_dot = std::abs(q_dot[i][j]);
            if (abs_q_dot > 1e-8) {
                // Velocity constraint: |q̇_j| = |q'_j| * v ≤ v_max_j
                double v_limit = (_maxJointVelocities[j] * SAFETY_FACTOR) / abs_q_dot;
                v_max[i] = std::min(v_max[i], v_limit);
            }
        }

        // Ensure reasonable bounds
        if (v_max[i] == std::numeric_limits<double>::max() || v_max[i] > 10.0) {
            v_max[i] = 1.0; // Conservative default
        }
        v_max[i] = std::max(v_max[i], 0.001); // Minimum velocity
    }

    // 4. Compute acceleration limits
    std::vector<double> a_max(numPoints);

    for (size_t i = 0; i < numPoints; ++i) {
        a_max[i] = std::numeric_limits<double>::max();

        for (size_t j = 0; j < numJoints; ++j) {
            double abs_q_dot = std::abs(q_dot[i][j]);
            if (abs_q_dot > 1e-8) {
                // Simplified acceleration constraint (ignoring curvature term for stability)
                // |q̈_j| ≈ |q'_j| * a ≤ a_max_j
                double a_limit = (_maxJointAccelerations[j] * SAFETY_FACTOR) / abs_q_dot;
                a_max[i] = std::min(a_max[i], a_limit);
            }
        }

        // Apply reasonable bounds
        if (a_max[i] == std::numeric_limits<double>::max() || a_max[i] > 100.0) {
            a_max[i] = 1.0;
        }
        a_max[i] = std::max(a_max[i], 0.01);
    }

    // 5. Forward pass - acceleration phase
    std::vector<double> v_forward(numPoints, 0.0);
    v_forward[0] = 0.0; // Start from rest

    for (size_t i = 1; i < numPoints; ++i) {
        if (ds[i - 1] > 1e-10) {
            // Kinematic equation: v² = v₀² + 2a*ds
            double v_squared = v_forward[i - 1] * v_forward[i - 1] + 2.0 * a_max[i] * ds[i - 1];
            double v_kinematic = std::sqrt(std::max(0.0, v_squared));

            // Apply velocity limit
            v_forward[i] = std::min(v_kinematic, v_max[i]);
        } else {
            v_forward[i] = v_forward[i - 1];
        }
    }

    // 6. Backward pass - deceleration phase
    std::vector<double> v_backward(numPoints, 0.0);
    v_backward[numPoints - 1] = 0.0; // End at rest

    for (int i = numPoints - 2; i >= 0; --i) {
        if (ds[i] > 1e-10) {
            // Backward kinematic equation
            double v_squared = v_backward[i + 1] * v_backward[i + 1] + 2.0 * a_max[i] * ds[i];
            double v_kinematic = std::sqrt(std::max(0.0, v_squared));

            v_backward[i] = std::min(v_kinematic, v_max[i]);
        } else {
            v_backward[i] = v_backward[i + 1];
        }
    }

    // 7. Take minimum of forward and backward profiles
    std::vector<double> v_optimal(numPoints);
    for (size_t i = 0; i < numPoints; ++i) {
        v_optimal[i] = std::min(v_forward[i], v_backward[i]);
    }

    // 8. Generate time-stamped trajectory
    std::vector<TrajectoryPoint> result = path;
    double current_time = 0.0;
    result[0].time = current_time;

    // Initialize first point
    for (size_t j = 0; j < numJoints; ++j) {
        result[0].velocity[j] = 0.0;
        result[0].acceleration[j] = 0.0;
    }

    for (size_t i = 1; i < numPoints; ++i) {
        // Compute time step using trapezoidal integration
        double avg_velocity = 0.5 * (v_optimal[i - 1] + v_optimal[i]);
        avg_velocity = std::max(avg_velocity, 1e-6); // Prevent division by zero

        double dt = ds[i - 1] / avg_velocity;
        current_time += dt;
        result[i].time = current_time;

        // Compute joint velocities and accelerations
        for (size_t j = 0; j < numJoints; ++j) {
            // Joint velocity: q̇_j = q'_j * v_path
            double joint_velocity = q_dot[i][j] * v_optimal[i];

            // Apply hard velocity limit as final safety check
            double vel_limit = _maxJointVelocities[j] * 0.99; // 1% margin for numerical safety
            if (std::abs(joint_velocity) > vel_limit) {
                joint_velocity = std::copysign(vel_limit, joint_velocity);
            }

            result[i].velocity[j] = joint_velocity;

            // Joint acceleration via finite differences
            if (dt > 1e-8) {
                double joint_acceleration = (result[i].velocity[j] - result[i - 1].velocity[j])
                                            / dt;

                // Apply hard acceleration limit
                double acc_limit = _maxJointAccelerations[j] * 0.99;
                if (std::abs(joint_acceleration) > acc_limit) {
                    joint_acceleration = std::copysign(acc_limit, joint_acceleration);
                }

                result[i].acceleration[j] = joint_acceleration;
            } else {
                result[i].acceleration[j] = 0.0;
            }
        }
    }

    // 9. Final verification and statistics
    double max_vel = 0.0, max_acc = 0.0;
    int vel_violations = 0, acc_violations = 0;

    for (size_t i = 0; i < result.size(); ++i) {
        for (size_t j = 0; j < numJoints; ++j) {
            double vel = std::abs(result[i].velocity[j]);
            double acc = std::abs(result[i].acceleration[j]);

            max_vel = std::max(max_vel, vel);
            max_acc = std::max(max_acc, acc);

            if (vel > _maxJointVelocities[j])
                vel_violations++;
            if (acc > _maxJointAccelerations[j])
                acc_violations++;
        }
    }

    LOG_INFO << "Time-optimal scaling completed:";
    LOG_INFO << "  Duration: " << result.back().time << " seconds";
    LOG_DEBUG << "  Max velocity: " << max_vel << " rad/s (limit: " << _maxJointVelocities[0] << ")";
    LOG_DEBUG << "  Max acceleration: " << max_acc << " rad/s² (limit: " << _maxJointAccelerations[0]
             << ")";
    LOG_DEBUG << "  Velocity violations: " << vel_violations;
    LOG_DEBUG << "  Acceleration violations: " << acc_violations;

    return result;
}

// Implementation of TaskSpacePathTrackingCostCalculator
TaskSpacePathTrackingCostCalculator::TaskSpacePathTrackingCostCalculator(
    RobotArm arm,
    const std::vector<Eigen::VectorXd> &jointCheckpoints,
    double positionWeight,
    double orientationWeight)
    : _arm(arm)
    , _positionWeight(positionWeight)
    , _orientationWeight(orientationWeight)
{
    // Convert joint space checkpoints to task space poses
    for (const auto &jointConfig : jointCheckpoints) {
        RobotArm tempArm = _arm;
        tempArm.setJointAngles(jointConfig);
        Eigen::Affine3d pose = tempArm.getEndeffectorPose();
        _taskSpacePositions.push_back(pose.translation());
        _taskSpaceOrientations.push_back(pose.linear());
    }

    // Calculate arc length for path parameterization
    _cumulativeDistances.push_back(0.0);
    _totalDistance = 0.0;

    for (size_t i = 1; i < _taskSpacePositions.size(); ++i) {
        double segmentDist = (_taskSpacePositions[i] - _taskSpacePositions[i - 1]).norm();
        _totalDistance += segmentDist;
        _cumulativeDistances.push_back(_totalDistance);
    }
}

/**
 * @brief Performs STOMP optimization with predefined checkpoints
 *
 * This function optimizes a trajectory using STOMP while ensuring it passes through
 * specified checkpoints. It can start with an already initialized trajectory or
 * generate one from the checkpoints if none is provided.
 *
 * @param checkpoints Vector of joint configurations the trajectory must pass through
 * @param initialTrajectory Optional initial trajectory (if empty, generated from checkpoints)
 * @param config Configuration parameters for the STOMP algorithm
 * @return True if a collision-free trajectory was found, false otherwise
 */
bool MotionGenerator::performSTOMPWithCheckpoints(
    const std::vector<Eigen::VectorXd> &checkpoints,
    std::vector<TrajectoryPoint> initialTrajectory,
    const StompConfig &config,
    std::shared_ptr<boost::asio::thread_pool> sharedPool)
{
    QElapsedTimer timer;
    timer.start();
    LOG_INFO << "Starting STOMP with checkpoints";

    _path.clear();
    createSDF();
    initializeCostCalculatorCheckpoints(checkpoints);

    std::unique_ptr<boost::asio::thread_pool> localPool;
    boost::asio::thread_pool *pool;

    if (sharedPool) {
        pool = sharedPool.get();
    } else {
        unsigned int numThreads = std::max(2u, std::thread::hardware_concurrency() / 2);
        localPool = std::make_unique<boost::asio::thread_pool>(numThreads);
        pool = localPool.get();
    }

    // Generate trajectory from checkpoints if no initial trajectory provided
    if (initialTrajectory.empty() && !checkpoints.empty()) {
        initialTrajectory = generateTrajectoryFromCheckpoints(checkpoints);
    }

    if (initialTrajectory.empty()) {
        LOG_ERROR << "Error: No valid initial trajectory or checkpoints provided";
        return false;
    }

    _path = initialTrajectory;

    // TODO: Implement actual STOMP optimization with checkpoints
    LOG_INFO << "performSTOMPWithCheckpoints: Using initial trajectory as-is (optimization not implemented)";
    
    return true;
}

Eigen::MatrixXd MotionGenerator::applyQuinticPolynomialResampling(const Eigen::MatrixXd &trajectory,
                                                                  double inputDt,
                                                                  double outputFrequency)
{
    if (outputFrequency <= 0.0 || trajectory.rows() < 2 || inputDt <= 0.0) {
        qWarning() << "Invalid parameters for quintic polynomial resampling. Returning original "
                      "trajectory.";
        return trajectory; // Return original if no resampling needed
    }

    const int numJoints = trajectory.cols();
    const int inputPoints = trajectory.rows();
    const double totalTime = (inputPoints - 1) * inputDt;
    const double outputDt = 1.0 / outputFrequency;
    const int outputPoints = static_cast<int>(std::ceil(totalTime / outputDt)) + 1;

    LOG_DEBUG << "Quintic polynomial resampling:";
    LOG_DEBUG << "Input points: " << inputPoints << " Total time: " << totalTime << "s";
    LOG_DEBUG << "Output frequency: " << outputFrequency << "Hz, Output points: " << outputPoints;

    // Create time vectors
    Eigen::VectorXd inputTimes = Eigen::VectorXd::LinSpaced(inputPoints, 0.0, totalTime);
    Eigen::VectorXd outputTimes = Eigen::VectorXd::LinSpaced(outputPoints, 0.0, totalTime);

    Eigen::MatrixXd resampledTrajectory(outputPoints, numJoints);

    // Fit quintic polynomial for each joint
    for (int joint = 0; joint < numJoints; ++joint) {
        // Extract joint positions
        Eigen::VectorXd jointPositions = trajectory.col(joint);

        // Boundary conditions: zero velocity and acceleration at endpoints
        double q0 = jointPositions(0); // Start position
        double q0_dot = 0.0;           // Start velocity = 0
        double q0_ddot = 0.0;          // Start acceleration = 0

        double qf = jointPositions(inputPoints - 1); // End position
        double qf_dot = 0.0;                         // End velocity = 0
        double qf_ddot = 0.0;                        // End acceleration = 0

        // Set up the quintic polynomial coefficient matrix
        // q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        // With boundary conditions:
        // q(0) = q0, q'(0) = q0_dot, q''(0) = q0_ddot
        // q(T) = qf, q'(T) = qf_dot, q''(T) = qf_ddot

        Eigen::Matrix<double, 6, 6> A;
        A << 1, 0, 0, 0, 0, 0, // q(0) = a0
            0, 1, 0, 0, 0, 0,  // q'(0) = a1
            0, 0, 2, 0, 0, 0,  // q''(0) = 2*a2
            1, totalTime, std::pow(totalTime, 2), std::pow(totalTime, 3), std::pow(totalTime, 4),
            std::pow(totalTime, 5), // q(T)
            0, 1, 2 * totalTime, 3 * std::pow(totalTime, 2), 4 * std::pow(totalTime, 3),
            5 * std::pow(totalTime, 4), // q'(T)
            0, 0, 2, 6 * totalTime, 12 * std::pow(totalTime, 2),
            20 * std::pow(totalTime, 3); // q''(T)

        Eigen::VectorXd b(6);
        b << q0, q0_dot, q0_ddot, qf, qf_dot, qf_ddot;

        // Solve for polynomial coefficients
        Eigen::VectorXd coeffs;
        try {
            coeffs = A.llt().solve(b);
        } catch (const std::exception &e) {
            qWarning() << "Failed to solve quintic polynomial for joint" << joint << ":"
                       << e.what();
            // Fallback to linear interpolation
            for (int i = 0; i < outputPoints; ++i) {
                double t = outputTimes(i);
                double alpha = t / totalTime;
                resampledTrajectory(i, joint) = (1.0 - alpha) * q0 + alpha * qf;
            }
            continue;
        }

        // If we have intermediate points, minimize squared error to the discrete trajectory
        if (inputPoints > 2) {
            // Set up least squares problem to minimize ||A_ls * coeffs - b_ls||^2
            // subject to the boundary constraints

            Eigen::MatrixXd A_ls(inputPoints - 2, 6); // Exclude boundary points
            Eigen::VectorXd b_ls(inputPoints - 2);

            for (int i = 1; i < inputPoints - 1; ++i) {
                double t = inputTimes(i);
                A_ls.row(i - 1) << 1, t, std::pow(t, 2), std::pow(t, 3), std::pow(t, 4),
                    std::pow(t, 5);
                b_ls(i - 1) = jointPositions(i);
            }

            // Weighted least squares: minimize boundary constraint violation + trajectory error
            double lambda = 100.0; // Weight for boundary constraints

            Eigen::MatrixXd A_total(6 + inputPoints - 2, 6);
            Eigen::VectorXd b_total(6 + inputPoints - 2);

            // Boundary constraints (heavily weighted)
            A_total.topRows(6) = lambda * A;
            b_total.head(6) = lambda * b;

            // Trajectory fitting constraints
            A_total.bottomRows(inputPoints - 2) = A_ls;
            b_total.tail(inputPoints - 2) = b_ls;

            // Solve weighted least squares
            try {
                coeffs = A_total.colPivHouseholderQr().solve(b_total);
            } catch (const std::exception &e) {
                qWarning() << "Failed to solve weighted least squares for joint" << joint << ":"
                           << e.what();
                // Keep the boundary-only solution
            }
        }

        // Evaluate polynomial at output time points
        for (int i = 0; i < outputPoints; ++i) {
            double t = outputTimes(i);
            double value = coeffs(0) + coeffs(1) * t + coeffs(2) * std::pow(t, 2)
                           + coeffs(3) * std::pow(t, 3) + coeffs(4) * std::pow(t, 4)
                           + coeffs(5) * std::pow(t, 5);
            resampledTrajectory(i, joint) = value;
        }
    }

    return resampledTrajectory;
}

Eigen::MatrixXd MotionGenerator::applyQuinticPolynomialResamplingWithTimes(
    const Eigen::MatrixXd &trajectory, const Eigen::VectorXd &times, double outputFrequency)
{
    if (outputFrequency <= 0.0 || trajectory.rows() < 2 || times.size() != trajectory.rows()) {
        qWarning() << "Invalid parameters for quintic polynomial resampling with times. Returning "
                      "original trajectory.";
        return trajectory;
    }

    const int numJoints = trajectory.cols();
    const int inputPoints = trajectory.rows();
    const double totalTime = times(inputPoints - 1) - times(0);
    const double outputDt = 1.0 / outputFrequency;
    const int outputPoints = static_cast<int>(std::ceil(totalTime / outputDt)) + 1;

    LOG_DEBUG << "Path-preserving quintic polynomial resampling:";
    LOG_DEBUG << "Input points: " << inputPoints << " Total time: " << totalTime << "s";
    LOG_DEBUG << "Output frequency: " << outputFrequency << "Hz, Output points: " << outputPoints;

    // Create output time vector - this preserves the timing structure
    Eigen::VectorXd outputTimes = Eigen::VectorXd::LinSpaced(outputPoints,
                                                             times(0),
                                                             times(inputPoints - 1));
    Eigen::MatrixXd resampledTrajectory(outputPoints, numJoints);

    // PATH-PRESERVING APPROACH: The geometric path (sequence of waypoints) must be preserved exactly
    // We only change the timing by fitting quintic polynomials to time segments between waypoints

    // Find the cumulative path length parameter for the original trajectory
    std::vector<double> pathParams(inputPoints);
    pathParams[0] = 0.0;

    for (int i = 1; i < inputPoints; ++i) {
        double segmentLength = 0.0;
        for (int joint = 0; joint < numJoints; ++joint) {
            double diff = trajectory(i, joint) - trajectory(i - 1, joint);
            segmentLength += diff * diff;
        }
        pathParams[i] = pathParams[i - 1] + std::sqrt(segmentLength);
    }

    double totalPathLength = pathParams[inputPoints - 1];

    // For each output time, find the corresponding path parameter
    std::vector<double> outputPathParams(outputPoints);
    for (int i = 0; i < outputPoints; ++i) {
        double timeRatio = (outputTimes(i) - times(0)) / totalTime;
        outputPathParams[i] = timeRatio * totalPathLength;
    }

    // For each joint, interpolate along the path preserving the geometric structure
    for (int joint = 0; joint < numJoints; ++joint) {
        Eigen::VectorXd jointPositions = trajectory.col(joint);

        // Create a quintic spline that interpolates all waypoints with respect to path parameter
        // This preserves the geometric path exactly
        for (int i = 0; i < outputPoints; ++i) {
            double targetParam = outputPathParams[i];

            // Find the segment in the original trajectory
            int segmentStart = 0;
            for (int k = 0; k < inputPoints - 1; ++k) {
                if (pathParams[k] <= targetParam && targetParam <= pathParams[k + 1]) {
                    segmentStart = k;
                    break;
                }
            }

            // Handle edge cases
            if (targetParam <= pathParams[0]) {
                resampledTrajectory(i, joint) = jointPositions(0);
                continue;
            }
            if (targetParam >= pathParams[inputPoints - 1]) {
                resampledTrajectory(i, joint) = jointPositions(inputPoints - 1);
                continue;
            }

            // For the segment, use quintic interpolation that preserves the path
            int segmentEnd = segmentStart + 1;
            double localParam = (targetParam - pathParams[segmentStart])
                                / (pathParams[segmentEnd] - pathParams[segmentStart]);

            // Get positions at segment boundaries
            double q0 = jointPositions(segmentStart);
            double q1 = jointPositions(segmentEnd);

            // Estimate velocities at boundaries (finite differences with path parameter)
            double v0 = 0.0, v1 = 0.0;
            if (segmentStart > 0) {
                double dp_prev = pathParams[segmentStart] - pathParams[segmentStart - 1];
                if (dp_prev > 1e-10) {
                    v0 = (jointPositions(segmentStart) - jointPositions(segmentStart - 1))
                         / dp_prev;
                }
            }
            if (segmentEnd < inputPoints - 1) {
                double dp_next = pathParams[segmentEnd + 1] - pathParams[segmentEnd];
                if (dp_next > 1e-10) {
                    v1 = (jointPositions(segmentEnd + 1) - jointPositions(segmentEnd)) / dp_next;
                }
            }

            // Apply boundary conditions for start/end of trajectory (zero velocity)
            if (segmentStart == 0)
                v0 = 0.0;
            if (segmentEnd == inputPoints - 1)
                v1 = 0.0;

            // Quintic Hermite interpolation preserving path geometry
            double t = localParam;
            double t2 = t * t;
            double t3 = t2 * t;
            double t4 = t3 * t;
            double t5 = t4 * t;

            // Hermite basis functions for quintic polynomial with zero acceleration at boundaries
            double h0 = 1 - 10 * t3 + 15 * t4 - 6 * t5;
            double h1 = 10 * t3 - 15 * t4 + 6 * t5;
            double h2 = t - 6 * t3 + 8 * t4 - 3 * t5;
            double h3 = -4 * t3 + 7 * t4 - 3 * t5;

            double segmentLength = pathParams[segmentEnd] - pathParams[segmentStart];
            resampledTrajectory(i, joint) = h0 * q0 + h1 * q1 + h2 * v0 * segmentLength
                                            + h3 * v1 * segmentLength;
        }
    }

    return resampledTrajectory;
}
