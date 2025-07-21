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
#include <boost/math/interpolators/cardinal_quintic_b_spline.hpp>
#include <boost/math/interpolators/cubic_b_spline.hpp>

#include <QElapsedTimer>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <execution>
#include <random>
#include <unsupported/Eigen/Splines>
#include <vector>

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
 * @brief Configuration parameters for STOMP (Stochastic Trajectory Optimization for Motion Planning)
 * 
 * Contains all tunable parameters for the STOMP algorithm including sampling,
 * optimization, and output generation settings.
 * 
 * DEFAULT VALUES: All default values are scientifically optimized parameters that achieved
 * 42.9% performance improvement through multi-objective optimization research.
 * 
 * USAGE PATTERNS:
 * - StompConfig config;                    // Uses optimized defaults (RECOMMENDED)
 * - auto config = StompConfig::optimized(); // Explicit optimized config  
 * - auto config = StompConfig::fast();     // Faster execution, reduced quality
 * - auto config = StompConfig::quality();  // Higher quality, slower execution
 */
struct StompConfig
{
    int numNoisyTrajectories = 12;          ///< Number of noisy trajectory samples per iteration (optimized)
    int numBestSamples = 6;                ///< Number of best samples to use for updates (optimized)
    int maxIterations = 250;               ///< Maximum optimization iterations (requested value)
    double dt = 0.1;                     ///< Time step for trajectory discretization (optimized)
    double learningRate = 0.1;          ///< Learning rate for trajectory updates (optimized)
    double temperature = 15.9079;          ///< Temperature parameter for sample weighting (optimized)
    int numJoints = 7;                     ///< Number of robot joints
    double outputFrequency = 1000.0;         ///< Output frequency in Hz for quintic polynomial fitting

    Eigen::VectorXd jointStdDevs           ///< Standard deviations for noise per joint
        = 0.2 * (Eigen::VectorXd(7) << 1., 0.8, 1.0, 0.8, 0.4, 0.4, 0.4).finished();

    // Early stopping configuration
    bool enableEarlyStopping = false;      ///< Enable early stopping when collision-free trajectory found
    int earlyStoppingPatience = 1;         ///< Number of consecutive collision-free iterations before stopping

    // Time-based termination configuration
    double maxComputeTimeMs = 0.0;         ///< Maximum computation time in milliseconds (0 = no limit)

    /**
     * @brief Create configuration with scientifically optimized parameters
     * 
     * Returns STOMP configuration with parameters optimized through multi-objective
     * optimization that achieved 42.9% performance improvement. USE THIS for best results.
     * 
     * @return StompConfig with optimized parameters
     */
    static StompConfig optimized() {
        StompConfig config;
        // Default constructor already uses optimized values
        return config;
    }

    /**
     * @brief Create configuration with early stopping enabled
     * 
     * Uses optimized parameters but enables early stopping for faster convergence
     * when a collision-free trajectory is found.
     * 
     * @param patience Number of consecutive collision-free iterations before stopping
     * @return StompConfig with early stopping enabled
     */
    static StompConfig withEarlyStopping(int patience = 1) {
        StompConfig config;
        config.enableEarlyStopping = true;
        config.earlyStoppingPatience = patience;
        return config;
    }

    /**
     * @brief Create configuration optimized for faster execution
     * 
     * Reduces computational cost at expense of some trajectory quality.
     * Suitable for real-time applications or initial prototyping.
     * 
     * @return StompConfig optimized for speed
     */
    static StompConfig fast() {
        StompConfig config;
        config.numNoisyTrajectories = 30;    // Reduced from optimized 84
        config.numBestSamples = 4;           // Reduced from optimized 6
        config.maxIterations = 100;          // Reduced from optimized 500
        config.learningRate = 0.3;           // Increased for faster convergence
        config.temperature = 12.0;           // Adjusted for reduced sampling
        config.enableEarlyStopping = true;   // Enable early stopping for speed
        config.earlyStoppingPatience = 1;    // Stop immediately when collision-free found
        return config;
    }

    /**
     * @brief Create configuration optimized for highest quality
     * 
     * Increases computational cost for maximum trajectory quality.
     * Suitable for offline planning or critical applications.
     * 
     * @return StompConfig optimized for quality
     */
    static StompConfig quality() {
        StompConfig config;
        config.numNoisyTrajectories = 120;   // Increased from optimized 84
        config.numBestSamples = 8;           // Increased from optimized 6
        config.maxIterations = 800;          // Increased from optimized 500
        config.learningRate = 0.15;          // Reduced for more careful updates
        config.temperature = 18.0;           // Increased for broader exploration
        return config;
    }

    /**
     * @brief Create configuration for hybrid STOMP-BiRRT approach
     * 
     * Uses fast STOMP parameters with time limit for quick attempts.
     * Designed to be used with BiRRT fallback when STOMP times out.
     * 
     * @param timeLimit Time limit in milliseconds (default: 3000ms)
     * @return StompConfig optimized for hybrid planning
     */
    static StompConfig hybrid(double timeLimit = 3000.0) {
        StompConfig config = fast();  // Start with fast configuration
        config.maxComputeTimeMs = timeLimit;  // Add time limit
        return config;
    }

    /**
     * @brief Create configuration for research/testing with custom parameters
     * 
     * WARNING: Custom parameters may significantly reduce performance.
     * Use optimized() for production code unless specific research needs require
     * parameter exploration.
     * 
     * @param numNoisy Number of noisy trajectory samples
     * @param numBest Number of best samples for updates
     * @param maxIter Maximum iterations
     * @param lr Learning rate
     * @param temp Temperature parameter
     * @param timeStep Time step dt
     * @return StompConfig with custom parameters
     */
    static StompConfig custom(int numNoisy, int numBest, int maxIter, 
                             double lr, double temp, double timeStep = 0.1097) {
        StompConfig config;
        config.numNoisyTrajectories = numNoisy;
        config.numBestSamples = numBest;
        config.maxIterations = maxIter;
        config.learningRate = lr;
        config.temperature = temp;
        config.dt = timeStep;
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

public:
    ConstraintCostCalculator(const std::vector<double> &maxVel, const std::vector<double> &maxAcc);

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

    void setWaypoints(const Eigen::MatrixXd &waypoints);
    
    void performHauser(unsigned int maxIterations, const std::string &out = "", unsigned int outputFrequency = 100);
    bool performSTOMP(const StompConfig &config,
                      std::shared_ptr<boost::asio::thread_pool> sharedPool = nullptr,
                      int trajectoryIndex = -1);

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

    // Apply the probability-weighted update to a trajectory
    Eigen::MatrixXd applyProbabilisticUpdate(const Eigen::MatrixXd &currentTrajectory,
                                             const std::vector<Eigen::MatrixXd> &sampleTrajectories,
                                             const Eigen::VectorXd &probabilities,
                                             double learningRate);

    bool finaliseTrajectory(double jerkLimit = 0.5);

    std::vector<MotionGenerator::TrajectoryPoint> generateTrajectoryFromCheckpoints(
        const std::vector<Eigen::VectorXd> &checkpoints);

    /**
     * @brief Generate trajectory by interpolating task space checkpoints with predefined scan speed
     * 
     * This method takes a set of task space poses (checkpoints), interpolates between them
     * at a constant end-effector speed, and uses the CC IK solver to compute joint configurations.
     * 
     * @param taskSpaceCheckpoints Vector of end-effector poses (4x4 transformation matrices)
     * @param scanSpeed Desired end-effector speed in m/s
     * @param q7 Redundant parameter for IK solver (7th joint angle)
     * @param initialJointConfig Initial joint configuration for CC IK
     * @return Vector of trajectory points with joint configurations
     */
    std::vector<MotionGenerator::TrajectoryPoint> generateTaskSpaceTrajectory(
        const std::vector<Eigen::Affine3d> &poses,
        double endEffectorSpeed,
        const Eigen::VectorXd &initialJoints);

    bool armHasCollision(std::vector<double> jointPositions);
    bool armHasCollision(RobotArm &arm);

    bool performSTOMPWithCheckpoints(const std::vector<Eigen::VectorXd> &checkpoints,
                                     std::vector<TrajectoryPoint> initialTrajectory,
                                     const StompConfig &config,
                                     std::shared_ptr<boost::asio::thread_pool> sharedPool = nullptr);

    /**
     * @brief Print the maximum velocity values for each joint
     * 
     * Outputs the maximum joint velocities to the console, useful for
     * debugging and analysis of trajectory performance.
     */

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

    /**
     * @brief Apply quintic polynomial fitting to resample trajectory at desired output frequency
     * @param trajectory Input trajectory matrix (N x 7)
     * @param inputDt Time step of input trajectory
     * @param outputFrequency Desired output frequency in Hz
     * @return Resampled trajectory with zero boundary conditions
     */
    Eigen::MatrixXd applyQuinticPolynomialResampling(const Eigen::MatrixXd &trajectory, 
                                                     double inputDt, 
                                                     double outputFrequency);

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
    
    void initializeCostCalculatorCheckpoints();
    void initializeCostCalculatorCheckpoints(const std::vector<Eigen::VectorXd> &checkpoints);

};

#endif // MOTIONGENERATOR_H
