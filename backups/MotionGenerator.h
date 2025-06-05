#ifndef MOTIONGENERATOR_H
#define MOTIONGENERATOR_H

#include "BVHTree.h"
#include "DynamicPath.h"
#include "RobotArm.h"
#include "spline.h"
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
    int numNoisyTrajectories = 6; // K: Number of noisy trajectories per iteration
    int numBestSamples = 4;       // B: Number of best samples to keep
    int maxIterations = 1000;     // Maximum optimization iterations
    double dt = 0.1;              // Time step for trajectory discretization
    double learningRate = 0.2;    // Rate for trajectory updates
    double temperature = 10.0;    // Temperature parameter for probability weighting
    double numJoints = 7;

    // Joint-specific parameters
    Eigen::VectorXd jointStdDevs
        = 0.02 * (Eigen::VectorXd(7) << 0.05, 0.8, 1.0, 0.8, 0.4, 0.4, 0.4).finished();
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
    bool performSTOMP(const StompConfig &config);

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
    bool armHasCollision(std::vector<double> jointPositions);
    void generateInitialTrajectory();
    bool armHasCollision(RobotArm &arm);
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
    void generateDenseTrajectory(Eigen::MatrixXd theta, double dt, int N);

    Eigen::MatrixXd generateNoisyTrajectory(const Eigen::MatrixXd &baseTrajectory,
                                            const Eigen::VectorXd &stdDevs,
                                            const std::vector<std::pair<double, double>> &limits);
};

#endif // MOTIONGENERATOR_H
