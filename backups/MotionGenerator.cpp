#include "MotionGenerator.h"
#include <future>

Eigen::VectorXd computeAccelerations(const Eigen::VectorXd &positions, double dt)
{
    int n = positions.size();

    Eigen::VectorXd accelerations(n - 2);

    for (int i = 1; i < n - 1; ++i) {
        double temp1 = (positions(i + 1) - 2.0 * positions(i) + positions(i - 1));
        accelerations(i - 1) = (positions(i + 1) - 2.0 * positions(i) + positions(i - 1))
                               / (dt * dt);
    }

    return accelerations;
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
        if (i < 4)
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

    const int numSteps = 20; // Number of interpolation steps
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
    _maxJointVelocities = std::vector<double>(7, 0.1);
    _maxJointAccelerations = std::vector<double>(7, 0.5);
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
            v_max = std::min(v_max, _maxJointVelocities[k]);
            a_max = std::min(a_max, _maxJointAccelerations[k]);
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
void MotionGenerator::performHauser(unsigned int maxIterations, const std::string &out)
{
    _path.clear();
    ParabolicRamp::DynamicPath traj;
    traj.Init({0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}, {1., 1., 1., 1., 1., 1., 1.});

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
    int res = traj.Shortcut(50, feasibilityChecker, out);

    // qDebug() << "Shortcut time: " << traj.GetTotalTime();
    double time = 0.0;
    for (double t = 0.0; t <= traj.GetTotalTime() + 0.001; t += 0.001) {
        std::vector<double> out;
        traj.Evaluate(t, out);
        TrajectoryPoint newPoint;
        newPoint.position = out;
        newPoint.velocity = std::vector<double>(_numJoints, 0.0);
        traj.Derivative(t, newPoint.velocity);
        newPoint.time = time;
        _path.push_back(newPoint);
        time += 0.1;
    }
}

void MotionGenerator::setObstacleTree(const std::shared_ptr<BVHTree> &newObstacleTree)
{
    _obstacleTree = newObstacleTree;
}

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
        if (i < 4)
            continue;

        auto [center, halfDims, axes] = bBox;

        if (_obstacleTree->isBoxIntersecting(center, halfDims, axes)) {
            auto end = std::chrono::high_resolution_clock::now(); // End timing
            return true;
        }
    }

    auto end = std::chrono::high_resolution_clock::now(); // End timing
    std::chrono::duration<double> duration = end - start;

    return false;
}

void MotionGenerator::createSDF()
{
    if (_sdfInitialized) {
        return;
    }

    Eigen::Vector3d minPoint(-1.5, -1.5, -0.1);
    Eigen::Vector3d maxPoint(1.5, 1.5, 1.5);
    double resolution = 0.025;
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
void MotionGenerator::generateDenseTrajectory(Eigen::MatrixXd theta, double dt, int N)
{
    std::vector<boost::math::interpolators::cardinal_quintic_b_spline<double>> jointSplines;
    std::vector<std::vector<double>> jointData(_numJoints);
    std::vector<double> timeVec;
    timeVec.reserve(N);

    double time = 0.0;
    for (int i = 0; i < N; i++) {
        Eigen::VectorXd jointAngles = theta.row(i);
        for (int k = 0; k < _numJoints; ++k) {
            jointData[k].push_back(jointAngles[k]);
        }
        timeVec.push_back(time);
        time += dt;
    }

    for (int k = 0; k < _numJoints; ++k) {
        std::pair<double, double> left_endpoint_derivatives{0., 0.};
        std::pair<double, double> right_endpoint_derivatives{0., 0.};
        boost::math::interpolators::cardinal_quintic_b_spline<double> spline(jointData[k],
                                                                             0.0,
                                                                             dt,
                                                                             {0., 0.},
                                                                             {0., 0.});
        jointSplines.push_back(spline);
    }

    time = 0.;
    while (time <= timeVec.back()) {
        TrajectoryPoint point;
        point.time = time;
        for (int k = 0; k < _numJoints; ++k) {
            double position = jointSplines[k](time);
            double velocity = jointSplines[k].prime(time);
            double acceleration = jointSplines[k].double_prime(time);

            point.position.push_back(position);
            point.velocity.push_back(velocity);
            point.acceleration.push_back(acceleration);
        }
        _path.push_back(point);
        time += 0.001;
    }
}

bool MotionGenerator::performSTOMP(const StompConfig &config)
{
    QElapsedTimer timer;
    timer.start();
    qDebug() << "Starting STOMP";
    timer.restart();

    createSDF();
    initializeCostCalculator();

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

    auto segment = computeTimeOptimalSegment(start, end, 0.0);
    double estimatedTime = segment.back().time;
    double dt = config.dt;
    int N = static_cast<int>(estimatedTime / dt) + 1;
    Eigen::MatrixXd theta = initializeTrajectory(goalVec, startVec, N, config.numJoints);

    initializeMatrices(N, dt);

    auto limits = _arm.jointLimits();
    std::vector<std::future<void>> futures;
    double prevTrajectoryCost = std::numeric_limits<double>::max();
    std::vector<double> prevSmoothnessCost(N, std::numeric_limits<double>::max());

    // Cost convergence parameters
    const double costConvergenceThreshold = 1e-2; // Stop when changes are smaller than this
    const double smoothnessConvergenceThreshold = 1e-3;
    const int convergencePatience = 5; // Number of consecutive iterations with small changes
    int noChangeCounter = 0;

    std::vector<std::pair<Eigen::MatrixXd, double>> bestSamples;

    timer.restart();
    bool success = false;
    Eigen::MatrixXd bestCollisionFreeTheta;
    double bestCollisionFreeCost = std::numeric_limits<double>::max();
    boost::asio::thread_pool pool(std::thread::hardware_concurrency());

    for (int iteration = 0; iteration < config.maxIterations; iteration++) {
        std::vector<Eigen::MatrixXd> noisyTrajectories(config.numNoisyTrajectories
                                                       + config.numBestSamples);
        futures.clear();

        for (int k = 0; k < config.numNoisyTrajectories; ++k) {
            futures.push_back(std::async(std::launch::async, [&, k]() {
                noisyTrajectories[k] = generateNoisyTrajectory(theta, config.jointStdDevs, limits);
            }));
        }

        for (auto &future : futures) {
            future.wait();
        }

        futures.clear();

        for (int b = 0; b < std::min(config.numBestSamples, static_cast<int>(bestSamples.size()));
             b++) {
            noisyTrajectories[config.numNoisyTrajectories + b] = bestSamples[b].first;
        }

        int totalSamples = config.numNoisyTrajectories
                           + std::min(config.numBestSamples, static_cast<int>(bestSamples.size()));
        Eigen::VectorXd S(totalSamples);
        Eigen::VectorXd P(totalSamples);
        std::vector<double> totalCosts(totalSamples);

        for (int k = 0; k < totalSamples; k++) {
            futures.push_back(std::async(std::launch::async, [&, k]() {
                double trajectoryCost = _costCalculator->computeCost(noisyTrajectories[k], dt);
                totalCosts[k] = trajectoryCost;
                S(k) = trajectoryCost;
            }));
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

        double trajectoryCost = _costCalculator->computeCost(theta, dt);
        double trajectoryCostDiff = std::abs(trajectoryCost - prevTrajectoryCost);

        std::vector<double> smoothnessCosts(N, 0.0);
        double maxDiff = 0.0;
        for (int i = 0; i < config.numJoints; i++) {
            smoothnessCosts[i] += 0.5 * (theta.col(i).transpose() * _R * theta.col(i))(0);
            maxDiff = std::max(maxDiff, std::abs(smoothnessCosts[i] - prevSmoothnessCost[i]));
        }

        qDebug() << "Iteration" << iteration << "Trajectory cost:" << trajectoryCost
                 << "Trajectory cost diff:" << trajectoryCostDiff << "Max diff:" << maxDiff;

        prevTrajectoryCost = trajectoryCost;
        prevSmoothnessCost = smoothnessCosts;

        for (int i = 0; i < N - 1; i++) {
            for (int d = 0; d < config.numJoints; d++) {
                theta(i, d) = std::clamp(theta(i, d),
                                         std::get<0>(limits[d]),
                                         std::get<1>(limits[d]));
            }
        }

        // Check if trajectory is collision-free (but don't terminate early)
        bool collides = false;
        RobotArm startArm = _arm;
        for (int i = 1; i < N - 1; ++i) {
            startArm.setJointAngles(theta.row(i));
            if (armHasCollision(startArm)) {
                collides = true;
                break;
            }
        }

        if (!collides) {
            success = true;
            if (trajectoryCost < bestCollisionFreeCost) {
                bestCollisionFreeCost = trajectoryCost;
                bestCollisionFreeTheta = theta;
                qDebug() << "Found better collision-free solution at iteration" << iteration
                         << "with cost" << trajectoryCost;
            }
        }

        // Check for cost convergence
        if (trajectoryCostDiff < costConvergenceThreshold) {
            noChangeCounter++;
            qDebug() << "Cost stable for" << noChangeCounter << "iterations";
            if (noChangeCounter >= convergencePatience) {
                qDebug() << "Cost converged after" << iteration << "iterations";
                if (success) {
                    qDebug() << "Final solution is collision-free";
                } else {
                    qDebug() << "Final solution has collisions";
                }
                break;
            }
        } else {
            noChangeCounter = 0; // Reset if cost changed significantly
        }
    }
    qDebug() << "Step 6 - Optimization loop:" << timer.elapsed() << "ms";

    // Use the best collision-free solution if available and not using the final one
    if (success && bestCollisionFreeTheta.rows() > 0) {
        if (bestCollisionFreeCost < prevTrajectoryCost) {
            qDebug() << "Using best collision-free solution with cost" << bestCollisionFreeCost
                     << "instead of final solution with cost" << prevTrajectoryCost;
            theta = bestCollisionFreeTheta;
        }
    }

    for (int i = 0; i < N; ++i) {
        TrajectoryPoint point;
        point.time = i * dt;
        point.position.resize(config.numJoints);
        point.velocity.resize(config.numJoints);
        point.acceleration.resize(config.numJoints);

        // Set positions
        for (int d = 0; d < config.numJoints; ++d) {
            point.position[d] = theta(i, d);
        }

        // Calculate velocities using finite differences
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

    return success;
}

Eigen::MatrixXd MotionGenerator::smoothTrajectoryUpdate(const Eigen::MatrixXd &dTheta)
{
    // Apply the smoothing matrix to the trajectory data
    Eigen::MatrixXd smoothed = _M * dTheta;

    return smoothed;
}
#include <cmath> // Required for std::exp

double MotionGenerator::computeObstacleCost(const RobotArm &curArm,
                                            const RobotArm &prevArm,
                                            double dt)
{
    double cost = 0;
    auto bboxesNew = curArm.getLinkBoundingBoxes();
    auto bboxesOld = prevArm.getLinkBoundingBoxes();

    for (int iter = 2; iter <= bboxesNew.size() - 1; ++iter) {
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
            double diff = radius + 0.05 - dist;

            // Modified section: Exponential scaling
            if (diff > 0) {
                cost += std::exp(diff * ((center - centerOld) / dt).norm());
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

    for (int i = 1; i < N; ++i) {
        RobotArm curArm = _arm;
        curArm.setJointAngles(trajectory.row(i));

        auto bboxesNew = curArm.getLinkBoundingBoxes();
        auto bboxesOld = prevArm.getLinkBoundingBoxes();

        for (int iter = 2; iter <= bboxesNew.size() - 1; ++iter) {
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
                cost += std::max(radius + 0.05 - dist, 0.0) * ((center - centerOld) / dt).norm();
            }
        }

        prevArm = curArm;
    }

    qDebug() << " Obstacle cost " << cost;

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

EndEffectorMovementCostCalculator::EndEffectorMovementCostCalculator(RobotArm arm, double weight)
    : _arm(arm)
    , _weight(weight)
{}

double EndEffectorMovementCostCalculator::computeCost(const Eigen::MatrixXd &trajectory, double dt)
{
    double cost = 0.0;
    int N = trajectory.rows();

    // Store previous end-effector pose
    RobotArm prevArm = _arm;
    prevArm.setJointAngles(trajectory.row(0));
    Eigen::Affine3d prevPose = prevArm.getEndeffectorPose();
    Eigen::Vector3d prevEndEffPos = prevPose.translation();

    // Calculate cost based on end-effector movement
    for (int i = 1; i < N; ++i) {
        RobotArm curArm = _arm;
        curArm.setJointAngles(trajectory.row(i));
        Eigen::Affine3d curPose = curArm.getEndeffectorPose();
        Eigen::Vector3d curEndEffPos = curPose.translation();

        // Calculate distance moved by end-effector
        double displacement = (curEndEffPos - prevEndEffPos).norm();

        // Add weighted squared displacement to cost
        cost += _weight * displacement * displacement;

        prevEndEffPos = curEndEffPos;
    }

    qDebug() << " Endeffector cost " << cost;

    return cost;
}

MagnetEndPositionCostCalculator::MagnetEndPositionCostCalculator(
    RobotArm arm, const Eigen::VectorXd &targetJointAngles)
    : _arm(arm)
    , _targetJointAngles(targetJointAngles)
{}

double MagnetEndPositionCostCalculator::computeCost(const Eigen::MatrixXd &trajectory, double dt)
{
    // Get the final configuration
    int N = trajectory.rows();
    if (N <= 0)
        return 0.0;

    // Get the final joint configuration from the trajectory
    Eigen::VectorXd finalJointAngles = trajectory.row(N - 1);

    // Calculate the difference between target and final joint angles
    double distanceSquared = (_targetJointAngles - finalJointAngles).squaredNorm();

    // Report the distance (for debugging)
    qDebug() << " Joint target distance: " << sqrt(distanceSquared);

    // Return the raw cost (weighting happens in CompositeCostCalculator)
    return distanceSquared;
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

    // Add obstacle cost calculator
    _costCalculator->addCostCalculator(std::make_unique<ObstacleCostCalculator>(_arm,
                                                                                _obstacleTree,
                                                                                _sdf,
                                                                                _sdfMinPoint,
                                                                                _sdfMaxPoint,
                                                                                _sdfResolution),
                                       10.0); // Weight = 1.0

    // // Add constraint cost calculator
    // _costCalculator
    //     ->addCostCalculator(std::make_unique<ConstraintCostCalculator>(_maxJointVelocities,
    //                                                                    _maxJointAccelerations),
    //                         1.0); // Weight = 1.0

    // Get the target joint configuration from the last row
    Eigen::VectorXd targetJointAngles = _waypoints.row(_waypoints.rows() - 1);

    // _costCalculator
    //     ->addCostCalculator(std::make_unique<MagnetEndPositionCostCalculator>(_arm,
    //                                                                           targetJointAngles),
    //                         10000000.0); // Weight specified here in the CompositeCostCalculator

    _costCalculator->addCostCalculator(std::make_unique<EndEffectorMovementCostCalculator>(_arm),
                                       100.0); // Weight = 1.0
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

MotionGenerator::TrajectoryEvaluation MotionGenerator::evaluateTrajectory(
    const Eigen::MatrixXd &trajectory, double dt)
{
    TrajectoryEvaluation result;

    // Calculate cost
    result.cost = _costCalculator->computeCost(trajectory, dt);

    // Check for collisions
    result.isCollisionFree = true;
    RobotArm checkArm = _arm;
    for (int i = 1; i < trajectory.rows() - 1; ++i) {
        checkArm.setJointAngles(trajectory.row(i));
        if (armHasCollision(checkArm)) {
            result.isCollisionFree = false;
            break;
        }
    }

    // Calculate end-effector metrics if needed
    // (e.g., smoothness, distance from goal, etc.)

    return result;
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
    qDebug() << "Finalising trajectory for execution...";

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
    qDebug() << "Trajectory finalised in" << timer.elapsed() << "ms. Duration:" << finalTime
             << "s, Points:" << _path.size();

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

            c[3] = x[0];
            c[4] = x[1];
            c[5] = x[2];

            coeffs[j][i] = c;
        }
    }

    // 4. Sample trajectory at control rate
    const double control_rate = 100.0;
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
