#ifndef MOTIONGENERATOR_H
#define MOTIONGENERATOR_H

#include "GeometryLib/BVHTree.h"
#include "Hauser10/DynamicPath.h"
#include "TrajectoryLib/Robot/RobotArm.h"
#include "TrajectoryLib/Core/spline.h"
#include <boost/asio.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>
#include <boost/math/interpolators/cardinal_quintic_b_spline.hpp>
#include <boost/math/interpolators/cubic_b_spline.hpp>

#include <QElapsedTimer>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <execution>
#include <random>
#include <unsupported/Eigen/Splines>
#include <vector>

class ArmFeasibilityChecker : public ParabolicRamp::FeasibilityCheckerBase
{
public:
    ArmFeasibilityChecker(RobotArm, const std::shared_ptr<BVHTree> &obstacleTree);
    bool ConfigFeasible(const ParabolicRamp::Vector &x);
    bool SegmentFeasible(const ParabolicRamp::Vector &a, const ParabolicRamp::Vector &b);

private:
    std::shared_ptr<BVHTree> _obstacleTree;
    RobotArm _arm;
};

struct StompConfig
{
    int numNoisyTrajectories = 4;
    int numBestSamples = 4;
    int maxIterations = 500;
    double dt = 0.1;
    double learningRate = .4;
    double temperature = 10.0;
    double numJoints = 7;

    Eigen::VectorXd jointStdDevs
        = 0.2 * (Eigen::VectorXd(7) << 0.05, 0.8, 1.0, 0.8, 0.4, 0.4, 0.4).finished();
};

class CostCalculator
{
public:
    virtual double computeCost(const Eigen::MatrixXd &trajectory, double dt) = 0;
    virtual ~CostCalculator() = default;
};

class ObstacleCostCalculator : public CostCalculator
{
private:
    RobotArm _arm;
    std::shared_ptr<BVHTree> _obstacleTree;
    std::vector<std::vector<std::vector<double>>> _sdf;
    Eigen::Vector3d _sdfMinPoint;
    Eigen::Vector3d _sdfMaxPoint;
    double _sdfResolution;

public:
    ObstacleCostCalculator(RobotArm arm,
                           std::shared_ptr<BVHTree> obstacleTree,
                           const std::vector<std::vector<std::vector<double>>> &sdf,
                           const Eigen::Vector3d &sdfMinPoint,
                           const Eigen::Vector3d &sdfMaxPoint,
                           double sdfResolution);

    double computeCost(const Eigen::MatrixXd &trajectory, double dt) override;
};

class CheckpointTransitionCostCalculator : public CostCalculator
{
public:
    /**
     * Constructor that initializes the indices of checkpoints in trajectory
     * @param checkpointIndices Indices of checkpoint positions in trajectory
     */
    CheckpointTransitionCostCalculator(const std::vector<int> &checkpointIndices);

    /**
     * Compute cost that penalizes velocity discontinuities at checkpoints
     * @param trajectory Joint trajectory in matrix form
     * @param dt Time step
     * @return Transition cost
     */
    double computeCost(const Eigen::MatrixXd &trajectory, double dt) override;

private:
    std::vector<int> _checkpointIndices;
};

class ApproachDepartureCostCalculator : public CostCalculator
{
private:
    RobotArm _arm;
    double _approachDistance; // Distance to approach/depart along z-axis
    double _weight;           // Weight for the cost term
    double _phaseFraction;    // Fraction of trajectory for approach/departure

public:
    ApproachDepartureCostCalculator(RobotArm arm,
                                    double approachDistance = 10.,
                                    double weight = 1.0,
                                    double phaseFraction = 0.5);

    double computeCost(const Eigen::MatrixXd &trajectory, double dt) override;
};

class ConstraintCostCalculator : public CostCalculator
{
private:
    std::vector<double> _maxJointVelocities;
    std::vector<double> _maxJointAccelerations;

public:
    ConstraintCostCalculator(const std::vector<double> &maxVel, const std::vector<double> &maxAcc);

    double computeCost(const Eigen::MatrixXd &trajectory, double dt) override;
};

class MagnetEndPositionCostCalculator : public CostCalculator
{
public:
    MagnetEndPositionCostCalculator(RobotArm arm, const Eigen::VectorXd &targetJointAngles);

    double computeCost(const Eigen::MatrixXd &trajectory, double dt) override;

private:
    RobotArm _arm;
    Eigen::VectorXd _targetJointAngles;
};

class TaskSpacePathTrackingCostCalculator : public CostCalculator
{
public:
    /**
     * Constructor that converts joint space checkpoints to task space path
     * @param arm Robot arm model for forward kinematics
     * @param jointCheckpoints List of joint configurations defining the path
     * @param positionWeight Weight for position deviation cost
     * @param orientationWeight Weight for orientation deviation cost
     */
    TaskSpacePathTrackingCostCalculator(RobotArm arm,
                                        const std::vector<Eigen::VectorXd> &jointCheckpoints,
                                        double positionWeight = 2.0,
                                        double orientationWeight = 1.0);

    /**
     * Compute the cost of a trajectory by measuring deviations from the task space path
     * @param trajectory Joint trajectory in matrix form
     * @param dt Time step
     * @return Total cost
     */
    double computeCost(const Eigen::MatrixXd &trajectory, double dt) override;

private:
    /**
     * Interpolate position and orientation along the path
     * @param s Path parameter (0 to 1)
     * @return Interpolated position and orientation
     */
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> interpolateTaskSpacePath(double s);

    RobotArm _arm;
    std::vector<Eigen::Vector3d> _taskSpacePositions;
    std::vector<Eigen::Matrix3d> _taskSpaceOrientations;
    std::vector<double> _cumulativeDistances;
    double _totalDistance;
    double _positionWeight;
    double _orientationWeight;
};

class CompositeCostCalculator : public CostCalculator
{
private:
    std::vector<std::unique_ptr<CostCalculator>> _costCalculators;
    std::vector<double> _weights;

public:
    void addCostCalculator(std::unique_ptr<CostCalculator> calculator, double weight);
    double computeCost(const Eigen::MatrixXd &trajectory, double dt) override;
};

class EndEffectorMovementCostCalculator : public CostCalculator
{
private:
    RobotArm _arm;
    double _weight;

public:
    EndEffectorMovementCostCalculator(RobotArm arm, double weight = 1.0);
    double computeCost(const Eigen::MatrixXd &trajectory, double dt) override;
};

class MotionGenerator
{
public:
    struct TrajectoryPoint
    {
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> acceleration;
        double time;
    };

    struct TrajectorySegment
    {
        std::vector<TrajectoryPoint> points;
    };

    MotionGenerator(RobotArm arm);

    void setWaypoints(const Eigen::MatrixXd &waypoints);

    void performHauser(unsigned int maxIterations, const std::string &out = "");
    void extracted(double &obstacleCost, Eigen::VectorXd &jointVelocity);
    bool performSTOMP(const StompConfig &config,
                      std::shared_ptr<boost::asio::thread_pool> sharedPool = nullptr);

    std::vector<TrajectoryPoint> getPath() const { return _path; }

    void setObstacleTree(const std::shared_ptr<BVHTree> &newObstacleTree);

    void saveTrajectoryToCSV(const std::string &filename);

    void setExplorationConstant(double newExplorationConstant);

    void createSDF();

    Eigen::MatrixXd initializeTrajectory(Eigen::Matrix<double, 7, 1> goalVec,
                                         Eigen::Matrix<double, 7, 1> startVec,
                                         int N,
                                         const int D);

    void initializeCostCalculator();

    struct TrajectoryEvaluation
    {
        double cost;
        bool isCollisionFree;
        Eigen::VectorXd endEffectorMetrics;
    };

    TrajectoryEvaluation evaluateTrajectory(const Eigen::MatrixXd &trajectory, double dt);

    // Apply the probability-weighted update to a trajectory
    Eigen::MatrixXd applyProbabilisticUpdate(const Eigen::MatrixXd &currentTrajectory,
                                             const std::vector<Eigen::MatrixXd> &sampleTrajectories,
                                             const Eigen::VectorXd &probabilities,
                                             double learningRate);

    bool finaliseTrajectory(double jerkLimit = 0.5);

    std::vector<MotionGenerator::TrajectoryPoint> generateTrajectoryFromCheckpoints(
        const std::vector<Eigen::VectorXd> &checkpoints);

    bool armHasCollision(std::vector<double> jointPositions);
    bool armHasCollision(RobotArm &arm);

    bool performSTOMPWithCheckpoints(const std::vector<Eigen::VectorXd> &checkpoints,
                                     std::vector<TrajectoryPoint> initialTrajectory,
                                     const StompConfig &config,
                                     std::shared_ptr<boost::asio::thread_pool> sharedPool = nullptr);

    bool performSTOMPWithEarlyTermination(const StompConfig &config,
                                          std::shared_ptr<boost::asio::thread_pool> sharedPool,
                                          std::vector<Eigen::MatrixXd> &intermediateThetas);

private:
    std::unique_ptr<CompositeCostCalculator> _costCalculator;

    std::vector<TrajectoryPoint> _path;
    std::vector<double> _maxJointVelocities;
    std::vector<double> _maxJointAccelerations;

    Eigen::MatrixXd _M;
    Eigen::MatrixXd _R;
    Eigen::MatrixXd _L;
    bool _matricesInitialized = false;

    double _explorationConstant = 0.1;

    Eigen::MatrixXd _waypoints;
    const int _numJoints = 7;

    std::shared_ptr<BVHTree> _obstacleTree;
    RobotArm _arm;

    std::vector<std::vector<std::vector<double>>> _sdf;
    Eigen::Vector3d _sdfMinPoint;
    Eigen::Vector3d _sdfMaxPoint;
    double _sdfResolution;
    bool _sdfInitialized = false;

    bool isShortcutCollisionFree(const TrajectorySegment &shortcut);
    void generateInitialTrajectory();
    void convertToTimeOptimal();
    std::vector<MotionGenerator::TrajectoryPoint> computeTimeOptimalSegment(
        const TrajectoryPoint &start, const TrajectoryPoint &end, double startTime);
    Eigen::MatrixXd computeAMatrix(int N, double dt);
    double computeCost(const RobotArm &curArm,
                       const RobotArm &prevArm,
                       double dt,
                       Eigen::VectorXd &jointVelocity);
    double computeCost(const RobotArm &arm);
    std::vector<double> computeEndeffectorSpeedOverPath();
    double computeObstacleCost(const RobotArm &curArm, const RobotArm &prevArm, double dt);
    double computeTrajectoryCost(const Eigen::MatrixXd &theta);
    double computeCost(const RobotArm &curArm, const RobotArm &prevArm, double dt);
    double computeTrajectoryCost(const Eigen::MatrixXd &theta, double dt);
    void initializeMatrices(const int &N, const double &dt);
    std::vector<Eigen::MatrixXd> generateNoisyTrajectories(
        const Eigen::MatrixXd &theta,
        const Eigen::VectorXd &startVec,
        const Eigen::VectorXd &goalVec,
        std::vector<std::pair<double, double>> limits,
        Eigen::VectorXd jointStdDevs,
        int K,
        int N,
        int D);
    Eigen::VectorXd evaluateQuinticPolynomial(const std::vector<Eigen::VectorXd> &coeffs, double t);
    std::vector<Eigen::VectorXd> generateQuinticPolynomialCoeffs(const Eigen::VectorXd &start,
                                                                 const Eigen::VectorXd &end,
                                                                 double T);
    Eigen::VectorXd computeTorques(const RobotArm &arm,
                                   const Eigen::VectorXd &velocities,
                                   const Eigen::VectorXd &accelerations);
    Eigen::MatrixXd smoothTrajectoryUpdate(const Eigen::MatrixXd &theta);

    Eigen::MatrixXd generateNoisyTrajectory(const Eigen::MatrixXd &baseTrajectory,
                                            const Eigen::VectorXd &stdDevs,
                                            const std::vector<std::pair<double, double>> &limits);
    std::vector<TrajectoryPoint> computeTimeOptimalScaling(const std::vector<TrajectoryPoint> &path);
    void initializeCostCalculatorCheckpoints();
    void initializeCostCalculatorCheckpoints(const std::vector<Eigen::VectorXd> &checkpoints);
};

#endif // MOTIONGENERATOR_H
