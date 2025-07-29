#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Robot/franka_ik_He.h"
#include <cmath> // Required for std::exp
#include <future>
#include <atomic> // Required for atomic collision checking
#include <set> // Required for combined sampling times

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

bool MotionGenerator::trajectoryViolatesConstraints(const Eigen::MatrixXd &trajectory, double dt)
{
    int N = trajectory.rows();
    int D = trajectory.cols();

    // Use a safety margin - allow some constraint violations for STOMP optimization
    const double velocityMargin = 1.5;  // Allow 50% higher velocities during optimization
    const double accelerationMargin = 1.5;  // Allow 50% higher accelerations during optimization

    // Compute velocities and accelerations
    for (int i = 0; i < N - 1; ++i) {
        for (int j = 0; j < D; ++j) {
            double velocity = (trajectory(i + 1, j) - trajectory(i, j)) / dt;
            if (std::abs(velocity) > _maxJointVelocities[j] * velocityMargin) {
                return true; // Severe velocity constraint violated
            }
        }
    }

    for (int i = 0; i < N - 2; ++i) {
        for (int j = 0; j < D; ++j) {
            double velocity_curr = (trajectory(i + 1, j) - trajectory(i, j)) / dt;
            double velocity_next = (trajectory(i + 2, j) - trajectory(i + 1, j)) / dt;
            double acceleration = (velocity_next - velocity_curr) / dt;
            if (std::abs(acceleration) > _maxJointAccelerations[j] * accelerationMargin) {
                return true; // Severe acceleration constraint violated
            }
        }
    }

    return false; // No severe constraint violations
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

// Constructor that accepts pre-computed SDF data to avoid recomputation
MotionGenerator::MotionGenerator(RobotArm arm,
                               const std::vector<std::vector<std::vector<double>>>& sdf,
                               const Eigen::Vector3d& sdfMinPoint,
                               const Eigen::Vector3d& sdfMaxPoint,
                               double sdfResolution,
                               std::shared_ptr<BVHTree> obstacleTree)
{
    _arm = arm;
    _sdf = sdf;
    _sdfMinPoint = sdfMinPoint;
    _sdfMaxPoint = sdfMaxPoint;
    _sdfResolution = sdfResolution;
    _sdfInitialized = true;
    
    if (obstacleTree) {
        setObstacleTree(obstacleTree);
    }
}

void MotionGenerator::setWaypoints(const Eigen::MatrixXd &waypoints)
{
    _waypoints = waypoints;
    _arm.setJointAngles(waypoints.row(0));
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
            v_max = std::min(v_max, _maxJointVelocities[k] * 0.75);
            a_max = std::min(a_max, _maxJointAccelerations[k] * 0.75);
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

    Eigen::Vector3d minPoint(-1., -1., -0.1);
    Eigen::Vector3d maxPoint(1., 1., 1.5);
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
    
    LOG_INFO << "STOMP: " << config.numNoisyTrajectories << " samples, " 
             << config.maxIterations << " max iterations";
    timer.restart();

    _path.clear();
    createSDF();
    initializeCostCalculator(config);

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

    Eigen::MatrixXd theta;
    int N = config.N;  // Use fixed N from config
    
    // Always use simple start-to-goal initialization (straight line)
    LOG_DEBUG << "Using start-to-goal initialization with fixed N=" << N;
    auto segment = computeTimeOptimalSegment(start, end, 0.0);
    double estimatedTime = segment.back().time;
    double dt = estimatedTime / (N - 1);  // Calculate dt based on fixed N and estimated time
    
    LOG_DEBUG << "Estimated time: " << estimatedTime << "s, dt: " << dt << "s";
    
    theta = initializeTrajectory(goalVec, startVec, N, config.numJoints);

    initializeMatrices(N, dt);

    auto limits = _arm.jointLimits();
    double prevTrajectoryCost = std::numeric_limits<double>::max();
    std::vector<double> prevSmoothnessCost(N, std::numeric_limits<double>::max());

    const double costConvergenceThreshold = 1e-6; 
    const int convergencePatience = 3; 
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
            LOG_INFO << "Time limit exceeded: " << config.maxComputeTimeMs 
                     << "ms after " << iteration << " iterations";
            if (!success) {
                throw StompTimeoutException("STOMP exceeded time limit without finding collision-free solution");
            } else {
                LOG_DEBUG << "Using collision-free solution (cost: " 
                         << std::fixed << std::setprecision(4) << bestCollisionFreeCost << ")";
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
        theta += config.learningRate * deltaS;

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

        // Simplified collision and constraint checking - avoid nested parallelization that can cause deadlock
        std::atomic<bool> collides{false};
        std::atomic<bool> severeConstraintViolation{false};

        // Check for severe constraint violations (more lenient than before)
        if (trajectoryViolatesConstraints(theta, dt)) {
            severeConstraintViolation.store(true);
        }

        // Check for collisions
        RobotArm checkArm = _arm;
        for (int i = 1; i < N - 1 && !collides.load(); ++i) {
            checkArm.setJointAngles(theta.row(i));
            if (armHasCollision(checkArm)) {
                collides.store(true);
                break;
            }
        }

        // Accept trajectory if it's collision-free, even with minor constraint violations
        if (!collides.load()) {
            success = true;
            if (trajectoryCost < bestCollisionFreeCost) {
                bestCollisionFreeCost = trajectoryCost;
                bestCollisionFreeTheta = theta;
                LOG_DEBUG << "Found better collision-free solution at iteration " << iteration
                         << " with cost " << std::fixed << std::setprecision(6) << trajectoryCost;
            }
            
            // Early stopping: increment counter when collision-free trajectory is found
            if (enableEarlyStopping) {
                earlyStoppingCounter++;
                LOG_DEBUG << "Valid solution found (collision-free"
                         << (severeConstraintViolation.load() ? " with minor constraint violations" : "") 
                         << ") (" << earlyStoppingCounter << "/" << earlyStoppingPatience << ")";
                
                if (earlyStoppingCounter >= earlyStoppingPatience) {
                    LOG_INFO << "Early stopping: valid solution after " 
                             << iteration + 1 << " iterations (cost: " 
                             << std::fixed << std::setprecision(4) << bestCollisionFreeCost << ")";
                    break;
                }
            }
        } else {
            // Reset early stopping counter if we hit collision
            if (enableEarlyStopping) {
                earlyStoppingCounter = 0;
            }
            
            // Optional: Log reason for trajectory rejection
            if (collides.load()) {
                LOG_FAST_DEBUG << "Trajectory rejected: collision detected";
            }
        }

        if (trajectoryCostDiff < costConvergenceThreshold) {
            noChangeCounter++;
            if (noChangeCounter >= convergencePatience) {
                LOG_INFO << "Cost converged after " << iteration + 1
                         << " iterations (cost: " 
                         << std::fixed << std::setprecision(6) << trajectoryCost << ")";
                break;
            }
        } else {
            noChangeCounter = 0;
        }
    }

    LOG_FAST_DEBUG << "Step 6 - Optimization loop: " << timer.elapsed() << "ms";

    if (success && bestCollisionFreeTheta.rows() > 0) {
        LOG_INFO << "STOMP complete: collision-free solution cost " 
                 << std::fixed << std::setprecision(4) << bestCollisionFreeCost
                 << " (" << timer.elapsed() << "ms)";
        theta = bestCollisionFreeTheta;
    } else {
        LOG_INFO << "STOMP complete: using best trajectory found (cost: " 
                 << std::fixed << std::setprecision(4) << prevTrajectoryCost
                 << ") - no collision-free solution found (" << timer.elapsed() << "ms)";
        // Use the current theta as the best trajectory found
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
                radius = std::max(radius, corner.norm());
            }

            if (i >= 0 && i < _sdf.size() && j >= 0 && j < _sdf[0].size() && k >= 0
                && k < _sdf[0][0].size()) {
                double dist = _sdf[i][j][k];
                cost += std::max(radius - dist, 0.0) * ((center - centerOld) / dt).norm();
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

ConstraintCostCalculator::ConstraintCostCalculator(const std::vector<double> &maxVel,
                                                   const std::vector<double> &maxAcc,
                                                   double velPenalty,
                                                   double accPenalty,
                                                   bool useQuadratic)
    : _maxJointVelocities(maxVel)
    , _maxJointAccelerations(maxAcc)
    , _velocityViolationPenalty(velPenalty)
    , _accelerationViolationPenalty(accPenalty)
    , _useQuadraticPenalty(useQuadratic)
{}

double ConstraintCostCalculator::computeCost(const Eigen::MatrixXd &trajectory, double dt)
{
    double cost = 0;
    int N = trajectory.rows();
    int D = trajectory.cols();

    Eigen::MatrixXd velocity = Eigen::MatrixXd::Zero(N - 1, D);
    Eigen::MatrixXd acceleration = Eigen::MatrixXd::Zero(N - 2, D);

    // Compute velocities
    for (int i = 0; i < N - 1; ++i) {
        velocity.row(i) = (trajectory.row(i + 1) - trajectory.row(i)) / dt;
    }

    // Compute accelerations
    for (int i = 0; i < N - 2; ++i) {
        acceleration.row(i) = (velocity.row(i + 1) - velocity.row(i)) / dt;
    }

    // Velocity constraint violations
    for (int i = 0; i < N - 1; ++i) {
        for (int j = 0; j < D; ++j) {
            double violation = std::max(0.0, std::abs(velocity(i, j)) - _maxJointVelocities[j]);
            if (violation > 0.0) {
                if (_useQuadraticPenalty) {
                    cost += _velocityViolationPenalty * violation * violation;
                } else {
                    cost += _velocityViolationPenalty * violation;
                }
            }
        }
    }

    // Acceleration constraint violations
    for (int i = 0; i < N - 2; ++i) {
        for (int j = 0; j < D; ++j) {
            double violation = std::max(0.0, std::abs(acceleration(i, j)) - _maxJointAccelerations[j]);
            if (violation > 0.0) {
                if (_useQuadraticPenalty) {
                    cost += _accelerationViolationPenalty * violation * violation;
                } else {
                    cost += _accelerationViolationPenalty * violation;
                }
            }
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

void MotionGenerator::initializeCostCalculator(const StompConfig &config)
{
    _costCalculator = std::make_unique<CompositeCostCalculator>();

    // Add obstacle avoidance cost
    _costCalculator->addCostCalculator(std::make_unique<ObstacleCostCalculator>(_arm,
                                                                                _obstacleTree,
                                                                                _sdf,
                                                                                _sdfMinPoint,
                                                                                _sdfMaxPoint,
                                                                                _sdfResolution),
                                       config.obstacleCostWeight);
    
    LOG_DEBUG << "Added obstacle cost calculator with weight: " << config.obstacleCostWeight;
    
    // Add constraint violation cost if weight > 0
    if (config.constraintCostWeight > 0.0) {
        _costCalculator->addCostCalculator(std::make_unique<ConstraintCostCalculator>(_maxJointVelocities,
                                                                                      _maxJointAccelerations),
                                           config.constraintCostWeight);
        
        LOG_DEBUG << "Added constraint cost calculator with weight: " << config.constraintCostWeight;
    }
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

std::vector<MotionGenerator::TrajectoryPoint> MotionGenerator::generateTrajectoryFromCheckpoints(
    const std::vector<Eigen::VectorXd> &checkpoints)
{
    std::vector<TrajectoryPoint> trajectory;
    if (checkpoints.empty())
        return trajectory;

    const size_t n_joints = _maxJointVelocities.size();
    const size_t n_checkpoints = checkpoints.size();

    // Validate input dimensions
    for (const auto& checkpoint : checkpoints) {
        if (static_cast<size_t>(checkpoint.size()) != n_joints) {
            throw std::invalid_argument("Checkpoint dimension mismatch");
        }
    }

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

        // Minimum segment time to ensure feasible trajectory
        double segment_time = std::max({t_vel_max, t_acc_max, 0.001}); // Minimum 1ms per segment
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
            A << T3, T4, T5, 
                 3 * T2, 4 * T3, 5 * T4, 
                 6 * T, 12 * T2, 20 * T3;

            Eigen::Vector3d b;
            b << qf - q0 - v0 * T - 0.5 * a0 * T2, 
                 vf - v0 - a0 * T, 
                 af - a0;

            // Use robust solver with error checking
            Eigen::ColPivHouseholderQR<Eigen::Matrix3d> solver(A);
            if (solver.rank() < 3) {
                throw std::runtime_error("Singular matrix in quintic polynomial generation");
            }
            
            Eigen::Vector3d x = solver.solve(b);
            
            // Assign the solved coefficients to complete the quintic polynomial
            c[3] = x[0];
            c[4] = x[1]; 
            c[5] = x[2];
            
            coeffs[j][i] = c;
        }
    }

    // 4. Generate time points that guarantee final time inclusion
    const double control_rate = 100.0;  // Use 100 Hz control rate
    const double dt = 1.0 / control_rate;
    const double total_time = absolute_times.back();

    std::vector<double> time_points;
    
    // Add regular time points
    for (double t = 0.0; t < total_time; t += dt) {
        time_points.push_back(t);
    }
    
    // Always include the final time exactly
    if (time_points.empty() || std::abs(time_points.back() - total_time) > 1e-9) {
        time_points.push_back(total_time);
    }

    // 5. Sample trajectory at all time points
    trajectory.reserve(time_points.size()); // Pre-allocate for efficiency
    
    for (double t : time_points) {
        TrajectoryPoint pt;
        pt.position.resize(n_joints);
        pt.velocity.resize(n_joints);
        pt.acceleration.resize(n_joints);
        pt.time = t;

        // Improved segment finding with robust boundary handling
        size_t segment_idx = 0;
        double t_segment = 0.0;
        
        // Handle final time point explicitly to avoid numerical precision issues
        if (t >= total_time - 1e-9) {
            segment_idx = segment_times.size() - 1;
            t_segment = segment_times[segment_idx];
        } else {
            // Find appropriate segment for interior points
            for (size_t i = 0; i < absolute_times.size() - 1; ++i) {
                if (t <= absolute_times[i + 1] + 1e-9) {
                    segment_idx = i;
                    t_segment = t - absolute_times[segment_idx];
                    break;
                }
            }
            // Ensure we don't go out of bounds
            segment_idx = std::min(segment_idx, segment_times.size() - 1);
            t_segment = std::max(0.0, std::min(t_segment, segment_times[segment_idx]));
        }

        // Evaluate quintic polynomials
        for (size_t j = 0; j < n_joints; ++j) {
            const Eigen::VectorXd &c = coeffs[j][segment_idx];

            // Position: c0 + c1*t + c2*t² + c3*t³ + c4*t⁴ + c5*t⁵
            const double t2 = t_segment * t_segment;
            const double t3 = t2 * t_segment;
            const double t4 = t3 * t_segment;
            const double t5 = t4 * t_segment;
            
            pt.position[j] = c[0] + c[1] * t_segment + c[2] * t2 + c[3] * t3 + c[4] * t4 + c[5] * t5;

            // Velocity: c1 + 2*c2*t + 3*c3*t² + 4*c4*t³ + 5*c5*t⁴
            pt.velocity[j] = c[1] + 2 * c[2] * t_segment + 3 * c[3] * t2 + 4 * c[4] * t3 + 5 * c[5] * t4;

            // Acceleration: 2*c2 + 6*c3*t + 12*c4*t² + 20*c5*t³
            pt.acceleration[j] = 2 * c[2] + 6 * c[3] * t_segment + 12 * c[4] * t2 + 20 * c[5] * t3;
        }

        trajectory.push_back(pt);
    }

    // 6. Final checkpoint guarantee and validation
    if (!trajectory.empty()) {
        auto& finalPoint = trajectory.back();
        const auto& finalCheckpoint = checkpoints.back();
        
        // Force exact final position to eliminate any remaining numerical errors
        for (size_t j = 0; j < n_joints; ++j) {
            finalPoint.position[j] = finalCheckpoint[j];
        }
        
        // Ensure final velocity and acceleration are zero (rest condition)
        std::fill(finalPoint.velocity.begin(), finalPoint.velocity.end(), 0.0);
        std::fill(finalPoint.acceleration.begin(), finalPoint.acceleration.end(), 0.0);
        
        // Set exact final time
        finalPoint.time = total_time;
        
        // Validation: Verify all intermediate checkpoints are hit exactly
        for (size_t cp_idx = 0; cp_idx < n_checkpoints; ++cp_idx) {
            const double checkpoint_time = absolute_times[cp_idx];
            
            // Find trajectory point at this checkpoint time
            auto it = std::find_if(trajectory.begin(), trajectory.end(),
                [checkpoint_time](const TrajectoryPoint& pt) {
                    return std::abs(pt.time - checkpoint_time) < 1e-9;
                });
            
            if (it != trajectory.end()) {
                // Force exact checkpoint position
                for (size_t j = 0; j < n_joints; ++j) {
                    it->position[j] = checkpoints[cp_idx][j];
                }
                
                // Set boundary conditions for first and last checkpoints
                if (cp_idx == 0 || cp_idx == n_checkpoints - 1) {
                    std::fill(it->velocity.begin(), it->velocity.end(), 0.0);
                    std::fill(it->acceleration.begin(), it->acceleration.end(), 0.0);
                }
            }
        }
    }

    return trajectory;
}