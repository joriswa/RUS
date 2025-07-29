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
    std::shared_ptr<BVHTree> _obstacleTree;        ///< BVH tree for collision detection
    RobotArm _arm;                                 ///< Robot arm model
};

/**
 * @brief Configuration parameters for STOMP trajectory optimization
 */
struct StompConfig
{
    int numNoisyTrajectories = 12;          ///< Number of noisy trajectory samples per iteration
    int numBestSamples = 6;                ///< Number of best samples to use for updates
    int maxIterations = 250;               ///< Maximum optimization iterations
    int N = 75;                           ///< Number of trajectory points
    double dt = 0.1;                     ///< Time step for trajectory discretization
    double learningRate = 0.1;          ///< Learning rate for trajectory updates
    double temperature = 15.9079;          ///< Temperature parameter for sample weighting
    int numJoints = 7;                     ///< Number of robot joints
    double outputFrequency = 1000.0;         ///< Output frequency in Hz for quintic polynomial fitting
    double obstacleCostWeight = 1.0;       ///< Weight for obstacle cost in composite cost function
    double constraintCostWeight = 1.0;     ///< Weight for constraint cost in composite cost function

    Eigen::VectorXd jointStdDevs           ///< Standard deviations for noise per joint
        = 0.2 * (Eigen::VectorXd(7) << 1., 0.8, 1.0, 0.8, 0.4, 0.4, 0.4).finished();

    bool enableEarlyStopping = false;      ///< Enable early stopping when collision-free trajectory found
    int earlyStoppingPatience = 1;         ///< Number of consecutive collision-free iterations before stopping
    double maxComputeTimeMs = 0.0;         ///< Maximum computation time in milliseconds (0 = no limit)
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
    RobotArm _arm;                                                ///< Robot arm model
    std::shared_ptr<BVHTree> _obstacleTree;                      ///< BVH tree for collision detection
    std::vector<std::vector<std::vector<double>>> _sdf;          ///< Signed distance field
    Eigen::Vector3d _sdfMinPoint;                                ///< SDF bounding box minimum
    Eigen::Vector3d _sdfMaxPoint;                                ///< SDF bounding box maximum
    double _sdfResolution;                                       ///< SDF grid resolution

public:
    /**
     * @brief Construct obstacle cost calculator
     * @param arm Robot arm model
     * @param obstacleTree BVH tree for collision detection
     * @param sdf Signed distance field grid
     * @param sdfMinPoint SDF bounding box minimum corner
     * @param sdfMaxPoint SDF bounding box maximum corner  
     * @param sdfResolution SDF grid resolution
     */
    ObstacleCostCalculator(RobotArm arm,
                           std::shared_ptr<BVHTree> obstacleTree,
                           const std::vector<std::vector<std::vector<double>>> &sdf,
                           const Eigen::Vector3d &sdfMinPoint,
                           const Eigen::Vector3d &sdfMaxPoint,
                           double sdfResolution);

    double computeCost(const Eigen::MatrixXd &trajectory, double dt) override;
};

/**
 * @brief Cost calculator for joint velocity and acceleration constraints
 * 
 * Computes penalty costs when trajectory violates joint velocity or acceleration limits.
 */
class ConstraintCostCalculator : public CostCalculator
{
private:
    std::vector<double> _maxJointVelocities = std::vector<double>(7, .4);
    std::vector<double> _maxJointAccelerations = std::vector<double>(7, .5);

public:
    /**
     * @brief Construct constraint cost calculator
     * @param maxVel Maximum joint velocities
     * @param maxAcc Maximum joint accelerations  
     */
    ConstraintCostCalculator(const std::vector<double> &maxVel, const std::vector<double> &maxAcc);

    double computeCost(const Eigen::MatrixXd &trajectory, double dt) override;
};

/**
 * @brief Composite cost calculator combining multiple cost functions
 * 
 * Allows combining different cost calculators with individual weights
 * to create complex objective functions for trajectory optimization.
 */
class CompositeCostCalculator : public CostCalculator
{
private:
    std::vector<std::unique_ptr<CostCalculator>> _costCalculators; ///< Individual cost calculators
    std::vector<double> _weights;                                  ///< Weights for each calculator

public:
    /**
     * @brief Add a cost calculator with specified weight
     * @param calculator Cost calculator to add
     * @param weight Weight for this calculator in total cost
     */
    void addCostCalculator(std::unique_ptr<CostCalculator> calculator, double weight);
    
    double computeCost(const Eigen::MatrixXd &trajectory, double dt) override;
};

/**
 * @brief STOMP and Hauser trajectory optimization for robotic ultrasound scanning
 */
class MotionGenerator
{
public:
    /**
     * @brief Single trajectory point with position, velocity, acceleration and time
     */
    struct TrajectoryPoint
    {
        std::vector<double> position;     ///< Joint positions
        std::vector<double> velocity;     ///< Joint velocities  
        std::vector<double> acceleration; ///< Joint accelerations
        double time;                      ///< Time stamp
    };

    /**
     * @brief Construct motion generator with robot arm
     * @param arm Robot arm model
     */
    MotionGenerator(RobotArm arm);
    
    /**
     * @brief Construct motion generator with pre-computed SDF to avoid recomputation
     * @param arm Robot arm model
     * @param sdf Pre-computed signed distance field
     * @param sdfMinPoint SDF bounding box minimum point
     * @param sdfMaxPoint SDF bounding box maximum point
     * @param sdfResolution SDF grid resolution
     * @param obstacleTree Obstacle tree for collision checking
     */
    MotionGenerator(RobotArm arm,
                   const std::vector<std::vector<std::vector<double>>>& sdf,
                   const Eigen::Vector3d& sdfMinPoint,
                   const Eigen::Vector3d& sdfMaxPoint,
                   double sdfResolution,
                   std::shared_ptr<BVHTree> obstacleTree);

    /**
     * @brief Set waypoints for trajectory planning
     * @param waypoints Matrix where each row is a joint configuration
     */
    void setWaypoints(const Eigen::MatrixXd &waypoints);
    
    /**
     * @brief Perform trajectory optimization using Hauser shortcutting algorithm
     * @param maxIterations Maximum optimization iterations
     * @param out Output parameter (unused)
     * @param outputFrequency Output frequency (unused)
     */
    void performHauser(unsigned int maxIterations, const std::string &out = "", unsigned int outputFrequency = 100);
    
    /**
     * @brief Perform trajectory optimization using STOMP algorithm
     * @param config STOMP configuration parameters
     * @param sharedPool Optional shared thread pool for parallelization
     * @param trajectoryIndex Optional trajectory index for logging
     * @return True if collision-free trajectory found
     */
    bool performSTOMP(const StompConfig &config,
                      std::shared_ptr<boost::asio::thread_pool> sharedPool = nullptr,
                      int trajectoryIndex = -1);

    /**
     * @brief Get the optimized trajectory
     * @return Vector of trajectory points
     */
    std::vector<TrajectoryPoint> getPath() const { return _path; }

    /**
     * @brief Set obstacle tree for collision checking
     * @param newObstacleTree BVH tree containing obstacles
     */
    void setObstacleTree(const std::shared_ptr<BVHTree> &newObstacleTree);

    /**
     * @brief Create signed distance field from obstacle tree
     */
    void createSDF();

    /**
     * @brief Generate smooth trajectory from joint space checkpoints
     * @param checkpoints Vector of joint configurations
     * @return Vector of trajectory points with smooth interpolation
     */
    std::vector<TrajectoryPoint> generateTrajectoryFromCheckpoints(
        const std::vector<Eigen::VectorXd> &checkpoints);

    /**
     * @brief Check if robot arm has collision at given joint positions
     * @param jointPositions Joint angle values
     * @return True if collision detected
     */
    bool armHasCollision(std::vector<double> jointPositions);
    
    /**
     * @brief Check if robot arm has collision
     * @param arm Robot arm to check
     * @return True if collision detected
     */
    bool armHasCollision(RobotArm &arm);

    /**
     * @brief Check if SDF is initialized
     * @return True if SDF has been computed
     */
    bool isSdfInitialized() const;
    
    /**
     * @brief Get computed SDF data
     * @return Reference to 3D SDF grid
     */
    const std::vector<std::vector<std::vector<double>>>& getSdf() const;
    
    /**
     * @brief Get SDF bounding box minimum point
     * @return Minimum corner of SDF bounding box
     */
    const Eigen::Vector3d& getSdfMinPoint() const;
    
    /**
     * @brief Get SDF bounding box maximum point
     * @return Maximum corner of SDF bounding box
     */
    const Eigen::Vector3d& getSdfMaxPoint() const;
    
    /**
     * @brief Get SDF grid resolution
     * @return Grid spacing in meters
     */
    double getSdfResolution() const;

    /**
     * @brief Compute time-optimal trajectory segment
     * @param start Starting trajectory point
     * @param end Ending trajectory point
     * @param startTime Time at segment start
     * @return Vector of trajectory points for segment
     */
    std::vector<TrajectoryPoint> computeTimeOptimalSegment(
        const TrajectoryPoint &start, const TrajectoryPoint &end, double startTime);

    /**
     * @brief Save trajectory to CSV file
     * @param filename Output CSV file path
     */
    void saveTrajectoryToCSV(const std::string &filename);

private:
    /**
     * @brief State tracking for STOMP convergence and early stopping
     */
    struct STOMPConvergenceState {
        bool success = false;
        Eigen::MatrixXd bestCollisionFreeTheta;
        double bestCollisionFreeCost = std::numeric_limits<double>::max();
        double prevTrajectoryCost = std::numeric_limits<double>::max();
        int noChangeCounter = 0;
        int earlyStoppingCounter = 0;
        QElapsedTimer overallTimer;
        
        // Convergence parameters
        static constexpr double costConvergenceThreshold = 1e-6;
        static constexpr int convergencePatience = 3;
        
        STOMPConvergenceState() {
            overallTimer.start();
        }
    };

    /**
     * @brief STOMP initialization data structure
     */
    struct STOMPInitData {
        Eigen::MatrixXd theta;
        boost::asio::thread_pool* pool;
        std::unique_ptr<boost::asio::thread_pool> localPool;
        int N;
        double dt;
        std::vector<std::pair<double, double>> limits;
    };

    std::unique_ptr<CompositeCostCalculator> _costCalculator;     ///< Cost function for optimization
    std::vector<TrajectoryPoint> _path;                          ///< Optimized trajectory
    std::vector<double> _maxJointVelocities = std::vector<double>(7, .4);  ///< Joint velocity limits
    std::vector<double> _maxJointAccelerations = std::vector<double>(7, .5); ///< Joint acceleration limits
    Eigen::MatrixXd _M, _R, _L;                                  ///< STOMP smoothing matrices
    bool _matricesInitialized = false;                          ///< Flag for matrix initialization
    Eigen::MatrixXd _waypoints;                                  ///< Waypoints for trajectory planning
    const int _numJoints = 7;                                   ///< Number of robot joints
    std::shared_ptr<BVHTree> _obstacleTree;                     ///< BVH tree for collision detection
    RobotArm _arm;                                              ///< Robot arm model
    std::vector<std::vector<std::vector<double>>> _sdf;         ///< Signed distance field
    Eigen::Vector3d _sdfMinPoint, _sdfMaxPoint;                 ///< SDF bounding box
    double _sdfResolution;                                      ///< SDF grid resolution
    bool _sdfInitialized = false;                               ///< Flag for SDF initialization

    // Original helper methods
    void generateInitialTrajectory();
    void convertToTimeOptimal();
    void initializeMatrices(const int &N, const double &dt);
    Eigen::MatrixXd initializeTrajectory(Eigen::Matrix<double, 7, 1> goalVec,
                                                      Eigen::Matrix<double, 7, 1> startVec,
                                                      int N,
                                                      const int D);
    Eigen::MatrixXd smoothTrajectoryUpdate(const Eigen::MatrixXd &theta);
    Eigen::MatrixXd generateNoisyTrajectory(const Eigen::MatrixXd &baseTrajectory,
                                            const Eigen::VectorXd &stdDevs,
                                            const std::vector<std::pair<double, double>> &limits);
    void initializeCostCalculator();
    void initializeCostCalculatorCheckpoints(const std::vector<Eigen::VectorXd> &checkpoints);

    // STOMP helper methods
    /**
     * @brief Initialize STOMP optimization data structures
     * @param config STOMP configuration
     * @param sharedPool Optional shared thread pool
     * @return Initialization data structure
     */
    STOMPInitData initializeSTOMPExecution(const StompConfig &config,
                                           std::shared_ptr<boost::asio::thread_pool> sharedPool);
    
    /**
     * @brief Generate noisy trajectory samples for STOMP iteration
     * @param config STOMP configuration
     * @param theta Current trajectory estimate
     * @param limits Joint limits
     * @param bestSamples Best samples from previous iterations
     * @param pool Thread pool for parallelization
     * @return Vector of noisy trajectory samples
     */
    std::vector<Eigen::MatrixXd> generateNoisySamples(const StompConfig &config,
                                                      const Eigen::MatrixXd &theta,
                                                      const std::vector<std::pair<double, double>> &limits,
                                                      const std::vector<std::pair<Eigen::MatrixXd, double>> &bestSamples,
                                                      boost::asio::thread_pool* pool);
    
    /**
     * @brief Evaluate costs and compute weights for trajectory samples
     * @param trajectories Trajectory samples to evaluate
     * @param config STOMP configuration
     * @param dt Time step
     * @param pool Thread pool for parallelization
     * @return Pair of costs and probability weights
     */
    std::pair<std::vector<double>, Eigen::VectorXd> evaluateTrajectories(
        const std::vector<Eigen::MatrixXd> &trajectories,
        const StompConfig &config,
        double dt,
        boost::asio::thread_pool* pool);
    
    /**
     * @brief Update best samples with lowest costs from current iteration
     * @param trajectories Current iteration trajectories
     * @param costs Corresponding trajectory costs
     * @param bestSamples Best samples storage (modified)
     * @param numBestSamples Number of best samples to keep
     */
    void updateBestSamples(const std::vector<Eigen::MatrixXd> &trajectories,
                           const std::vector<double> &costs,
                           std::vector<std::pair<Eigen::MatrixXd, double>> &bestSamples,
                           int numBestSamples);
    
    /**
     * @brief Apply STOMP trajectory update using weighted samples
     * @param theta Current trajectory estimate
     * @param noisyTrajectories Noisy trajectory samples
     * @param weights Probability weights for samples
     * @param config STOMP configuration
     * @param N Number of trajectory points
     * @param limits Joint limits
     * @return Updated trajectory estimate
     */
    Eigen::MatrixXd applyTrajectoryUpdate(const Eigen::MatrixXd &theta,
                                          const std::vector<Eigen::MatrixXd> &noisyTrajectories,
                                          const Eigen::VectorXd &weights,
                                          const StompConfig &config,
                                          int N,
                                          const std::vector<std::pair<double, double>> &limits);
    
    /**
     * @brief Check trajectory for collisions using parallel processing
     * @param theta Trajectory to check
     * @param N Number of trajectory points
     * @return True if collision detected
     */
    bool checkCollisions(const Eigen::MatrixXd &theta, int N);
    
    /**
     * @brief Update convergence state and early stopping counters
     * @param state Convergence state (modified)
     * @param theta Current trajectory
     * @param config STOMP configuration
     * @param iteration Current iteration number
     * @param dt Time step
     * @param collisionFree Whether trajectory is collision-free
     */
    void updateConvergenceState(STOMPConvergenceState &state,
                                const Eigen::MatrixXd &theta,
                                const StompConfig &config,
                                int iteration,
                                double dt,
                                bool collisionFree);
    
    /**
     * @brief Check if time limit exceeded
     * @param config STOMP configuration
     * @param timer Elapsed time timer
     * @param iteration Current iteration
     * @param success Whether solution found
     * @param bestCost Best cost found
     * @return True if time limit exceeded
     */
    bool checkTimeLimit(const StompConfig &config,
                        const QElapsedTimer &timer,
                        int iteration,
                        bool success,
                        double bestCost);
    
    /**
     * @brief Check convergence criteria for early termination
     * @param state Convergence state
     * @param config STOMP configuration
     * @param iteration Current iteration
     * @return True if convergence criteria met
     */
    bool checkConvergence(const STOMPConvergenceState &state,
                          const StompConfig &config,
                          int iteration);
    
    /**
     * @brief Finalize STOMP optimization and generate trajectory points
     * @param theta Final trajectory estimate
     * @param state Convergence state
     * @param config STOMP configuration
     * @param N Number of trajectory points
     * @param dt Time step
     * @param timer Elapsed time timer
     * @return True if successful
     */
    bool finalizeSTOMPResult(const Eigen::MatrixXd &theta,
                             const STOMPConvergenceState &state,
                             const StompConfig &config,
                             int N,
                             double dt,
                             const QElapsedTimer &timer);
};

#endif // MOTIONGENERATOR_H
