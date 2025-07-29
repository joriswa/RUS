#ifndef MOTIONGENERATOR_H
#define MOTIONGENERATOR_H

#include "GeometryLib/BVHTree.h"
#include "Hauser10/DynamicPath.h"
#include "TrajectoryLib/Robot/RobotArm.h"
#include "TrajectoryLib/Core/spline.h"
#include "TrajectoryLib/Logger.h"
#include <boost/asio.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>
#include <boost/math/interpolators/cubic_b_spline.hpp>

#include <QElapsedTimer>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <execution>
#include <random>
#include <unsupported/Eigen/Splines>
#include <vector>
#include <algorithm>

/**
 * @brief Exception thrown when STOMP optimization times out
 */
class StompTimeoutException : public std::runtime_error {
public:
    explicit StompTimeoutException(const std::string& message) 
        : std::runtime_error(message) {}
};

/**
 * @brief Exception thrown when STOMP optimization fails to find solution
 */
class StompFailedException : public std::runtime_error {
public:
    explicit StompFailedException(const std::string& message) 
        : std::runtime_error(message) {}
};

/**
 * @brief Feasibility checker for robot arm configurations using collision detection
 * 
 * Implements the FeasibilityCheckerBase interface from Hauser10 library to provide
 * collision checking for robot arm configurations and path segments.
 */
class ArmFeasibilityChecker : public ParabolicRamp::FeasibilityCheckerBase
{
public:
    /**
     * @brief Construct feasibility checker with robot arm and obstacle tree
     * @param arm Robot arm for collision checking
     * @param obstacleTree BVH tree containing obstacles
     */
    ArmFeasibilityChecker(RobotArm, const std::shared_ptr<BVHTree> &obstacleTree);
    
    /**
     * @brief Check if a configuration is collision-free
     * @param x Joint configuration vector
     * @return True if configuration is feasible (collision-free)
     */
    bool ConfigFeasible(const ParabolicRamp::Vector &x);
    
    /**
     * @brief Check if a straight-line path segment is collision-free
     * @param a Start configuration
     * @param b End configuration
     * @return True if entire segment is collision-free
     */
    bool SegmentFeasible(const ParabolicRamp::Vector &a, const ParabolicRamp::Vector &b);

private:
    std::shared_ptr<BVHTree> _obstacleTree;
    RobotArm _arm;
};

/**
 * @brief STOMP optimization configuration
 * 
 * Contains tunable parameters for STOMP trajectory optimization.
 * Default values are research-optimized for best performance.
 */
struct StompConfig
{
    int numNoisyTrajectories = 32;           ///< Number of noisy trajectory samples per iteration
    int numBestSamples = 4;                ///< Number of best samples to use for updates
    int maxIterations = 300;               ///< Maximum optimization iterations
    double dt = 0.07393752094959168;       ///< Time step for trajectory discretization
    double learningRate = 0.2;             ///< Learning rate for trajectory updates
    double temperature = 23.756654804919137; ///< Temperature parameter for sample weighting
    int numJoints = 7;                     ///< Number of robot joints
    int N = 75;                           ///< Fixed number of trajectory points
    double outputFrequency = 1000.0;         ///< Output frequency in Hz for quintic polynomial fitting
    double constraintCostWeight = 1000.0;    ///< Weight for constraint violation cost (velocity/acceleration limits)
    double obstacleCostWeight = 1.0;      ///< Weight for obstacle avoidance cost

    Eigen::VectorXd jointStdDevs           ///< Standard deviations for noise per joint
        = 0.2 * (Eigen::VectorXd(7) << 1., 0.8, 1.0, 0.8, 0.4, 0.4, 0.4).finished();

    // Early stopping configuration
    bool enableEarlyStopping = false;      ///< Enable early stopping when collision-free trajectory found
    int earlyStoppingPatience = 1;         ///< Number of consecutive collision-free iterations before stopping

    // Time-based termination configuration
    double maxComputeTimeMs = 0.0;         ///< Maximum computation time in milliseconds (0 = no limit)

    /**
     * @brief Create optimized configuration
     * @return Config with research-optimized parameters
     */
    static StompConfig optimized() {
        StompConfig config;
        return config;
    }

    /**
     * @brief Create configuration with early stopping
     * @param patience Consecutive collision-free iterations before stopping
     * @return Config with early stopping enabled
     */
    static StompConfig withEarlyStopping(int patience = 1) {
        StompConfig config;
        config.enableEarlyStopping = true;
        config.earlyStoppingPatience = patience;
        return config;
    }

    /**
     * @brief Create configuration optimized for speed
     * @return Config optimized for faster execution
     */
    static StompConfig fast() {
        StompConfig config;
        config.numNoisyTrajectories = 6;
        config.numBestSamples = 3;
        config.maxIterations = 150;
        config.learningRate = 0.25;
        config.temperature = 20.0;
        config.enableEarlyStopping = true;
        config.earlyStoppingPatience = 1;
        config.N = 50;  // Fewer points for speed
        config.constraintCostWeight = 0.5;  // Lower constraint cost for speed
        return config;
    }

    /**
     * @brief Create configuration optimized for quality
     * @return Config optimized for trajectory quality
     */
    static StompConfig quality() {
        StompConfig config;
        config.numNoisyTrajectories = 12;
        config.numBestSamples = 6;
        config.maxIterations = 500;
        config.learningRate = 0.15;
        config.temperature = 30.0;
        config.N = 100;  // More points for quality
        config.constraintCostWeight = 2.0;  // Higher constraint cost for better compliance
        return config;
    }

    /**
     * @brief Create configuration for hybrid STOMP-BiRRT approach
     * @param timeLimit Time limit in milliseconds
     * @return Config with time limit for hybrid planning
     */
    static StompConfig hybrid(double timeLimit = 3000.0) {
        StompConfig config = fast();
        config.maxComputeTimeMs = timeLimit;
        config.N = 60;  // Moderate points for hybrid
        config.constraintCostWeight = 1.0;  // Balanced constraint cost
        return config;
    }

    /**
     * @brief Create configuration with custom parameters
     * @param numNoisy Number of noisy trajectory samples
     * @param numBest Number of best samples for updates
     * @param maxIter Maximum iterations
     * @param lr Learning rate
     * @param temp Temperature parameter
     * @param N Number of trajectory points
     * @param timeStep Time step dt
     * @return Config with custom parameters
     */
    static StompConfig custom(int numNoisy, int numBest, int maxIter, 
                             double lr, double temp, int N = 75, 
                             double timeStep = 0.07393752094959168) {
        StompConfig config;
        config.numNoisyTrajectories = numNoisy;
        config.numBestSamples = numBest;
        config.maxIterations = maxIter;
        config.learningRate = lr;
        config.temperature = temp;
        config.N = N;
        config.dt = timeStep;
        return config;
    }
    
    /**
     * @brief Create configuration with strict constraint compliance
     * @return Config with high constraint cost weight for better velocity/acceleration compliance
     */
    static StompConfig strictConstraints() {
        StompConfig config;
        config.constraintCostWeight = 5.0;  // High constraint penalty
        config.numNoisyTrajectories = 10;   // More samples to find compliant trajectories
        config.maxIterations = 400;         // More iterations for convergence
        config.learningRate = 0.15;         // Slower learning for stability
        return config;
    }
};

/**
 * @brief Abstract base class for trajectory cost calculation
 * 
 * Defines the interface for computing costs associated with robot trajectories.
 * Derived classes implement specific cost functions for different objectives.
 */
class CostCalculator
{
public:
    /**
     * @brief Compute cost for a given trajectory
     * @param trajectory Joint space trajectory matrix (time x joints)
     * @param dt Time step between trajectory points
     * @return Computed cost value
     */
    virtual double computeCost(const Eigen::MatrixXd &trajectory, double dt) = 0;
    virtual ~CostCalculator() = default;
};

/**
 * @brief Cost calculator for obstacle avoidance using signed distance field
 * 
 * Computes trajectory costs based on proximity to obstacles using a precomputed
 * signed distance field for efficient collision avoidance.
 */
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
    std::vector<double> _maxJointVelocities = std::vector<double>(7, .4);
    std::vector<double> _maxJointAccelerations = std::vector<double>(7, .5);
    double _velocityViolationPenalty = 10.0;      ///< Penalty factor for velocity violations
    double _accelerationViolationPenalty = 5.0;  ///< Penalty factor for acceleration violations
    bool _useQuadraticPenalty = false;            ///< Use quadratic instead of linear penalty

public:
    ConstraintCostCalculator(const std::vector<double> &maxVel, const std::vector<double> &maxAcc);
    
    /**
     * @brief Enhanced constructor with configurable penalty factors
     * @param maxVel Maximum joint velocities
     * @param maxAcc Maximum joint accelerations  
     * @param velPenalty Penalty factor for velocity violations
     * @param accPenalty Penalty factor for acceleration violations
     * @param useQuadratic Use quadratic penalty (more aggressive near limits)
     */
    ConstraintCostCalculator(const std::vector<double> &maxVel, 
                           const std::vector<double> &maxAcc,
                           double velPenalty, 
                           double accPenalty,
                           bool useQuadratic = false);

    double computeCost(const Eigen::MatrixXd &trajectory, double dt) override;
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
    
    // Constructor that accepts pre-computed SDF data to avoid recomputation
    MotionGenerator(RobotArm arm,
                   const std::vector<std::vector<std::vector<double>>>& sdf,
                   const Eigen::Vector3d& sdfMinPoint,
                   const Eigen::Vector3d& sdfMaxPoint,
                   double sdfResolution,
                   std::shared_ptr<BVHTree> obstacleTree = nullptr);

    void setWaypoints(const Eigen::MatrixXd &waypoints);
    
    void performHauser(unsigned int maxIterations, const std::string &out = "", unsigned int outputFrequency = 100);
    bool performSTOMP(const StompConfig &config,
                      std::shared_ptr<boost::asio::thread_pool> sharedPool = nullptr,
                      int trajectoryIndex = -1);

    std::vector<TrajectoryPoint> getPath() const { return _path; }

    void setObstacleTree(const std::shared_ptr<BVHTree> &newObstacleTree);

    void saveTrajectoryToCSV(const std::string &filename);


    void createSDF();
    
    // SDF data getters for sharing computed SDF across multiple instances
    const std::vector<std::vector<std::vector<double>>>& getSdf() const { return _sdf; }
    const Eigen::Vector3d& getSdfMinPoint() const { return _sdfMinPoint; }
    const Eigen::Vector3d& getSdfMaxPoint() const { return _sdfMaxPoint; }
    double getSdfResolution() const { return _sdfResolution; }
    bool isSdfInitialized() const { return _sdfInitialized; }

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

    // Apply the probability-weighted update to a trajectory
    Eigen::MatrixXd applyProbabilisticUpdate(const Eigen::MatrixXd &currentTrajectory,
                                             const std::vector<Eigen::MatrixXd> &sampleTrajectories,
                                             const Eigen::VectorXd &probabilities,
                                             double learningRate);

    std::vector<MotionGenerator::TrajectoryPoint> generateTrajectoryFromCheckpoints(
        const std::vector<Eigen::VectorXd> &checkpoints);

    bool armHasCollision(std::vector<double> jointPositions);
    bool armHasCollision(RobotArm &arm);
    bool trajectoryViolatesConstraints(const Eigen::MatrixXd &trajectory, double dt);

    bool performSTOMPWithCheckpoints(const std::vector<Eigen::VectorXd> &checkpoints,
                                     std::vector<TrajectoryPoint> initialTrajectory,
                                     const StompConfig &config,
                                     std::shared_ptr<boost::asio::thread_pool> sharedPool = nullptr);

    std::vector<MotionGenerator::TrajectoryPoint> computeTimeOptimalSegment(
        const TrajectoryPoint &start, const TrajectoryPoint &end, double startTime);

private:
    std::unique_ptr<CompositeCostCalculator> _costCalculator;

    std::vector<TrajectoryPoint> _path;
    std::vector<double> _maxJointVelocities = std::vector<double>(7, .4);
    std::vector<double> _maxJointAccelerations = std::vector<double>(7, .5);

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

    /**
     * @brief Apply quintic polynomial fitting with non-uniform time spacing
     * @param trajectory Input trajectory matrix (N x 7)
     * @param times Time vector corresponding to trajectory points
     * @param outputFrequency Desired output frequency in Hz
     * @return Resampled trajectory at uniform output frequency
     */
    Eigen::MatrixXd applyQuinticPolynomialResamplingWithTimes(const Eigen::MatrixXd &trajectory, 
                                                              const Eigen::VectorXd &times, 
                                                              double outputFrequency);

    Eigen::MatrixXd generateNoisyTrajectory(const Eigen::MatrixXd &baseTrajectory,
                                            const Eigen::VectorXd &stdDevs,
                                            const std::vector<std::pair<double, double>> &limits);
    std::vector<TrajectoryPoint> computeTimeOptimalScaling(const std::vector<TrajectoryPoint> &path);
    
    /**
     * @brief Improved pipeline: quintic polynomial fitting followed by time-optimal scaling
     * @param rawPath Raw trajectory from STOMP with initial timing
     * @param config STOMP configuration for output frequency
     * @return Final trajectory with smooth geometry and constraint-satisfying timing
     */
    std::vector<TrajectoryPoint> processTrajectoryPipeline(const std::vector<TrajectoryPoint> &rawPath, 
                                                           const StompConfig &config);
    
    // Helper functions for time-optimal trajectory generation
    Eigen::MatrixXd computePathDerivatives(const std::vector<std::vector<double>>& jointPositions, 
                                          const std::vector<double>& pathParams);
    std::vector<double> computeVelocityLimits(const Eigen::MatrixXd& pathDerivatives);
    std::vector<double> computeAccelerationLimits(const Eigen::MatrixXd& pathDerivatives, double jerkLimit);
    double calculateSegmentTiming(double ds, double vPrev, double vNext);
    
    void initializeCostCalculator(const StompConfig &config = StompConfig::optimized());
    void initializeCostCalculatorCheckpoints(const std::vector<Eigen::VectorXd> &checkpoints,
                                           const StompConfig &config = StompConfig::optimized());

};

#endif // MOTIONGENERATOR_H
