#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <QDebug>
#include <QObject>
#include <QThread>
#include <QWaitCondition>
#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>
#include <boost/bind.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <limits>
#include <random>
#include <unsupported/Eigen/Splines>
#include <vector>

#include "GeometryLib/BVHTree.h"
#include "TrajectoryLib/Robot/RobotArm.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

/**
 * @brief Represents a 7-dimensional state in Cartesian space bettween [0, 1] mapped to the joint limits.
 */
typedef boost::geometry::model::point<double, 7, boost::geometry::cs::cartesian> State;

/**
 * @brief Shared pointer to a PathNode.
 */
typedef std::shared_ptr<class PathNode> NodePtr;

/**
 * @brief Constant used in nearest neighbor calculation.
 */
const double kRRG = 2.71828 * (1 + 1.0 / 7.0);

/**
 * @brief Enum representing different path planning algorithms.
 */
enum Algorithm
{
    RRT,
    RRTStar,
    InformedRRTStar,
    RRTConnect
};

/**
 * @brief Structure to hold parameters for path planning.
 */
struct Params
{
    Algorithm algo = RRT;
    double stepSize = 0.05;
    double goalBiasProbability = 0.5;
    bool customCost = false;
    int maxIterations = 5000;
};

/**
 * @brief Represents a node in the path planning tree.
 */
class PathNode : public std::enable_shared_from_this<PathNode>
{
public:
    State _state;
    NodePtr _parent;
    double _cost;
    double _costToGoal;
    RobotArm _arm;

    /**
     * @brief Constructs a PathNode.
     * @param state The state of the node.
     * @param arm The robot arm configuration.
     * @param parent The parent node.
     * @param cost The cost to reach this node.
     * @param costToGoal The estimated cost to the goal.
     */
    PathNode(const State &state,
             RobotArm arm,
             NodePtr parent = nullptr,
             double cost = 0.0,
             double costToGoal = std::numeric_limits<double>::max());

    /**
     * @brief Creates a child node.
     * @param point The state of the child node.
     * @param arm The robot arm configuration for the child.
     * @return A shared pointer to the new child node.
     */
    NodePtr createChild(const State &point, RobotArm arm);

    /**
     * @brief Estimates the total cost for this node.
     * @return The estimated total cost (cost + cost to goal).
     */
    double estimateCost();
};

/**
 * @brief Class responsible for path planning operations.
 */
class PathPlanner
{
private:
    Params _params;
    QThread *_worker;
    std::shared_ptr<BVHTree> _obstacleTree;
    bgi::rtree<std::pair<State, NodePtr>, bgi::rstar<16>> _tree;
    NodePtr _pathRoot;
    std::vector<std::vector<std::vector<double>>> _sdf;
    Eigen::Vector3d _sdfMinPoint;
    Eigen::Vector3d _sdfMaxPoint;
    double _sdfResolution;
    std::vector<std::pair<double, double>> _jointLimits;
    RobotArm _startArm;
    RobotArm _goalArm;
    Eigen::Vector3d _goalTranslation;
    Eigen::Matrix3d _goalRotation;
    State _startPoint;
    std::vector<std::tuple<State, RobotArm>> _path;
    bool _goalConfigSpecified = false;

    /**
     * @brief Checks if the motion between two arm configurations is valid.
     * @param armA The starting arm configuration.
     * @param endArm The ending arm configuration.
     * @param inflate Whether to inflate the collision geometry.
     * @return True if the motion is valid, false otherwise.
     */
    bool motionIsValid(RobotArm &armA, RobotArm &endArm, bool inflate = false);

    /**
     * @brief Performs the RRT (Rapidly-exploring Random Tree) algorithm.
     * @return True if a path to the goal was found, false otherwise.
     */
    bool performRRT();

    /**
     * @brief Performs the RRT-Connect algorithm for bidirectional tree growth.
     * @return True if a path was found, false otherwise.
     */
    bool performRRTConnect();

    /**
     * @brief Performs the RRT* algorithm with asymptotic optimality guarantees.
     * @param costTrackingFile Optional file path to save cost tracking data.
     * @return True if a path was found, false otherwise.
     */
    bool performRRTStar(const std::string &costTrackingFile = "");

    /**
     * @brief Performs the Informed RRT* algorithm with ellipsoidal sampling.
     * @param costTrackingFile Optional file path to save cost tracking data.
     * @return True if a path was found, false otherwise.
     */
    bool performInformedRRTStar(const std::string &costTrackingFile = "");

    /**
     * @brief Finds the nearest node in joint space to a given point.
     * @param point The target point in configuration space.
     * @return Pointer to the nearest node in the tree.
     */
    NodePtr nearestJointNode(const State &point);

    /**
     * @brief Generates a new node by extending towards a random point in joint space.
     * @param randomPoint The random point to extend towards (modified in-place).
     * @param newNode Reference to store the generated new node.
     * @return True if a valid new node was generated, false otherwise.
     */
    bool getNewJointNode(State &randomPoint, NodePtr &newNode);

    /**
     * @brief Finds the k-nearest neighbors of a node.
     * @param node The node to find neighbors for.
     * @param k The number of neighbors to find.
     * @return A vector of the k-nearest neighbors.
     */
    std::vector<NodePtr> findNearestNeighbors(const NodePtr &node, unsigned int k);

    /**
     * @brief Rewires the path through a new node.
     * @param newNode The new node to consider for rewiring.
     * @param neighboringNodes The neighboring nodes to consider for rewiring.
     * @return A vector of nodes that were rewired.
     */
    std::vector<NodePtr> rewirePath(NodePtr newNode, std::vector<NodePtr> &neighboringNodes);

    /**
     * @brief Checks if an arm configuration is sufficiently close to the goal.
     * @param arm The arm configuration to check.
     * @return True if the arm is close enough to the goal, false otherwise.
     */
    bool closeToGoal(RobotArm arm);

    /**
     * @brief Computes the sampling transformation matrix for informed sampling.
     * @param cBest The best cost found so far.
     * @return The transformation matrix for ellipsoidal sampling.
     */
    Eigen::MatrixXd getSamplingMatrix(double cBest);

    /**
     * @brief Samples a point uniformly from a unit ball in high-dimensional space.
     * @return A vector representing a point sampled from the unit ball.
     */
    Eigen::VectorXd sampleUnitBall();

    /**
     * @brief Samples a point within an ellipsoid for informed RRT*.
     * @param cmax The maximum cost defining the ellipsoid size.
     * @param C The transformation matrix for the ellipsoid orientation.
     * @return A state sampled from within the ellipsoid.
     */
    State sampleWithinEllipsoid(double cmax, Eigen::MatrixXd C);

    /**
     * @brief Computes the rotation matrix for world frame transformation.
     * @param start The start state.
     * @param goal The goal state.
     * @return The rotation matrix aligning start and goal directions.
     */
    Eigen::MatrixXd computeRotationWorldFrame(const State &start, const State &goal);

    /**
     * @brief Computes the Euclidean distance metric between two states.
     * @param a The first state.
     * @param b The second state.
     * @return The Euclidean distance between the states in configuration space.
     */
    double metric(const State &a, const State &b);

    /**
     * @brief Constructs the final path by backtracking from goal to start.
     * @param startNode The start node of the path.
     * @param goalNode The goal node of the path.
     */
    void constructPath(NodePtr startNode, NodePtr goalNode);

    /**
     * @brief Finds the nearest neighbor node in a given tree.
     * @param tree The R-tree to search in.
     * @param point The query point.
     * @return Pointer to the nearest neighbor node.
     */
    NodePtr nearestNeighbor(const bgi::rtree<std::pair<State, NodePtr>, bgi::rstar<16>> &tree,
                            const State &point);

    /**
     * @brief Extends a tree towards a target point by creating a new node.
     * @param tree The tree to extend.
     * @param target The target point to extend towards.
     * @param newNode Reference to store the newly created node.
     * @return True if the extension was successful, false otherwise.
     */
    bool extendTree(bgi::rtree<std::pair<State, NodePtr>, bgi::rstar<16>> &tree,
                    const State &target,
                    NodePtr &newNode);

    /**
     * @brief Generates a new node by extending from the nearest node towards a random point.
     * @param randomPoint The random point to extend towards (modified in-place).
     * @param newNode Reference to store the generated new node.
     * @param tree The tree containing existing nodes.
     * @return True if a valid new node was generated, false otherwise.
     */
    bool getNewNode(State &randomPoint,
                    NodePtr &newNode,
                    const bgi::rtree<std::pair<State, NodePtr>, bgi::rstar<16>> &tree);

    /**
     * @brief Connects path waypoints with densely sampled straight-line segments.
     */
    void connectWithStraightLines();

    /**
     * @brief Computes the total displacement along the current path.
     * @return The total displacement in meters.
     */
    double computeTotalDisplacement();

    /**
     * @brief Computes the total distance to environment obstacles along the path.
     * @return The cumulative distance to obstacles.
     */
    double computeTotalDistanceToEnviornment();

    /**
     * @brief Finds the k-nearest neighbors of a node in the tree.
     * @param tree The R-tree to search in.
     * @param node The query node.
     * @param k The number of neighbors to find.
     * @return Vector of k-nearest neighbor nodes.
     */
    std::vector<NodePtr> findNearestNeighbors(
        bgi::rtree<std::pair<State, NodePtr>, bgi::rstar<16>> &tree,
        const NodePtr &node,
        unsigned int k);

public:
    /**
     * @brief Default constructor for PathPlanner.
     */
    PathPlanner();

    /**
     * @brief Shortcut the path by removing unnecessary waypoints when direct connections are collision-free.
     * This method iteratively tries to connect non-adjacent waypoints directly, removing intermediate
     * waypoints if the direct path is collision-free. This reduces path length and execution time.
     * @param maxIterations Maximum number of shortcutting iterations (default: 100)
     * @param maxAttempts Maximum attempts per iteration to find shortcuts (default: 20)
     */
    void shortcutPath(int maxIterations = 100, int maxAttempts = 20);

    /**
     * @brief Checks if the given arm configuration has a collision.
     * @param arm The robot arm to check.
     * @param inflate Whether to inflate the collision geometry.
     * @return True if a collision is detected, false otherwise.
     */
    bool armHasCollision(RobotArm &arm, bool inflate = false);

    /**
     * @brief Structure to hold clearance metrics for a robot arm configuration.
     */
    struct ClearanceMetrics
    {
        double min_clearance = std::numeric_limits<double>::infinity(); ///< Minimum distance to any obstacle
        double avg_clearance = 0.0;                                     ///< Average clearance across all links
        double weighted_clearance = 0.0;                                ///< Volume-weighted clearance
        std::vector<double> link_clearances;                            ///< Individual clearance for each link
        std::vector<Vec3> closest_points;                               ///< Closest points on obstacles for each link
        bool has_collision = false;                                     ///< Whether any link is in collision
        int num_links_checked = 0;                                      ///< Number of links that were checked
    };

    /**
     * @brief Computes clearance metrics for a given arm configuration.
     * @param arm The robot arm configuration to analyze.
     * @param inflate Whether to inflate the collision geometry.
     * @return ClearanceMetrics structure containing detailed clearance information.
     */
    ClearanceMetrics computeArmClearance(RobotArm &arm, bool inflate = false);

    /**
     * @brief Computes minimum clearance distance for a given arm configuration.
     * @param arm The robot arm configuration to analyze.
     * @param inflate Whether to inflate the collision geometry.
     * @return Minimum distance to any obstacle, or -1 if in collision.
     */
    double getMinimumClearance(RobotArm &arm, bool inflate = false);

    /**
     * @brief Sets the start pose for path planning.
     * @param arm The starting arm configuration.
     */
    void setStartPose(RobotArm arm);

    /**
     * @brief Sets the goal pose for path planning.
     * @param t The goal translation.
     * @param r The goal rotation.
     */
    void setGoalPose(Eigen::Vector3d t, Eigen::Matrix3d r);

    /**
     * @brief Sets the algorithm to be used for path planning.
     * @param algorithm The chosen path planning algorithm.
     */
    void setAlgorithm(Algorithm algorithm);

    /**
     * @brief Gets the obstacle tree used for collision detection.
     * @return Shared pointer to the BVHTree representing obstacles.
     */
    std::shared_ptr<BVHTree> obstacleTree() const;

    /**
     * @brief Sets the obstacle tree for collision detection.
     * @param newObstacleTree The new BVHTree containing obstacle geometry.
     */
    void setObstacleTree(const std::shared_ptr<BVHTree> &newObstacleTree);

    /**
     * @brief Gets the computed path as end-effector positions and arm configurations.
     * @return Vector of tuples containing 3D positions and robot arm states.
     */
    std::vector<std::tuple<Vec3, RobotArm>> getPath();

    /**
     * @brief Selects an optimal goal pose using optimization techniques.
     * @param pose The target end-effector pose.
     * @return Pair containing the selected robot arm configuration and success status.
     */
    std::pair<RobotArm, bool> selectGoalPose(const Eigen::Affine3d &pose);

    /**
     * @brief Selects a goal pose using simulated annealing optimization with default parameters.
     * @param pose The target end-effector pose.
     * @return Pair containing the selected robot arm configuration and success status.
     */
    std::pair<RobotArm, bool> selectGoalPoseSimulatedAnnealing(
        const Eigen::Affine3d &pose,
        double T_max = 50.0,          // Higher initial temperature for better exploration
        double T_min = 0.0001,        // Lower final temperature for finer convergence
        double alpha = 0.995,         // Slower cooling rate for more thorough search
        int max_iterations = 100000,  // More iterations for comprehensive optimization
        int max_no_improvement = 5000 // Higher patience for better convergence
    );

    /**
     * @brief Saves the joint angles of the computed path to a CSV file.
     * @param filename The path to the output file.
     */
    void saveJointAnglesToFile(const std::string &filename);

    /**
     * @brief Gets the path as a matrix of joint angles for each waypoint.
     * @return Eigen matrix where each row represents joint angles at a waypoint.
     */
    Eigen::MatrixXd getAnglesPath() const;

    /**
     * @brief Sets the path planning parameters.
     * @param newParams The new parameter configuration.
     */
    void setParams(const Params &newParams);

    /**
     * @brief Executes the configured path planning algorithm.
     * @return True if a valid path was found, false otherwise.
     */
    bool runPathFinding();

    /**
     * @brief Sets a specific goal configuration instead of pose-based planning.
     * @param arm The target robot arm configuration.
     */
    void setGoalConfiguration(RobotArm arm);

    /**
     * @brief Plans checkpoints and trajectory segments for ultrasound scanning
     *
     * This function computes robot arm configurations for scan positions and automatically
     * segments them into valid trajectory parts based on boundary detection and angle deviations.
     * It performs the complete high-level path planning including finding valid configurations,
     * detecting boundaries, and organizing segments for trajectory generation.
     *
     * @param scanPoses A vector of Eigen::Affine3d poses representing the scan checkpoints
     * @param currentJoints Current joint configuration to start from
     * @return A structure containing checkpoints, valid segments, and jump pairs
     */
    struct CheckpointPlanResult
    {
        std::vector<std::pair<RobotArm, bool>> checkpoints;
        std::vector<std::pair<size_t, size_t>> validSegments;
        std::vector<std::pair<size_t, size_t>> jumpPairs;
        size_t firstValidIndex;
    };

    CheckpointPlanResult planCheckpoints(
        const std::vector<Eigen::Affine3d> &scanPoses,
        const Eigen::VectorXd &currentJoints);

private:
    /**
     * @brief Densifies sparse task space poses through linear interpolation.
     * @param originalPoses The original sparse set of poses.
     * @return Vector of densified poses with intermediate waypoints.
     */
    std::vector<Eigen::Affine3d> densifyTaskSpacePath(
        const std::vector<Eigen::Affine3d> &originalPoses);

    /**
     * @brief Solves inverse kinematics for densified poses with continuity optimization.
     * @param densifiedPoses The densified poses to solve IK for.
     * @param currentJoints Current joint configuration as starting point.
     * @param result Reference to store the planning results.
     * @param selectGoalPoseFallbacks Vector to track fallback usage.
     */
    void solveInverseKinematics(
        const std::vector<Eigen::Affine3d> &densifiedPoses,
        const Eigen::VectorXd &currentJoints,
        CheckpointPlanResult &result,
        std::vector<size_t> &selectGoalPoseFallbacks);

    /**
     * @brief Attempts continuous IK solving using small joint variations.
     * @param targetMatrix The target transformation matrix.
     * @param currentJointAngles Current joint configuration.
     * @param gen Random number generator.
     * @param dis Random distribution for sampling.
     * @return Pair containing the IK solution and success status.
     */
    std::pair<std::array<double, 7>, bool> attemptContinuousIK(
        const Eigen::Matrix<double, 4, 4> &targetMatrix,
        const std::array<double, 7> &currentJointAngles,
        std::mt19937 &gen,
        std::uniform_real_distribution<> &dis);

    /**
     * @brief Handles IK fallback when continuous solving fails.
     * @param pose The target pose requiring fallback.
     * @param poseIdx Index of the pose in the sequence.
     * @param currentJointAngles Current joint configuration (modified).
     * @param result Reference to store the planning results.
     * @param selectGoalPoseFallbacks Vector to track fallback usage.
     */
    void handleIKFallback(
        const Eigen::Affine3d &pose,
        size_t poseIdx,
        std::array<double, 7> &currentJointAngles,
        CheckpointPlanResult &result,
        std::vector<size_t> &selectGoalPoseFallbacks);

    /**
     * @brief Segments the path into continuous trajectory pieces and jump pairs.
     * @param result Reference to store the segmentation results.
     * @param selectGoalPoseFallbacks Vector indicating which poses used fallbacks.
     */
    void segmentPath(
        CheckpointPlanResult &result,
        const std::vector<size_t> &selectGoalPoseFallbacks);

public:
    /**
     * @brief Generates a smooth trajectory from timed checkpoints using spline interpolation.
     *
     * This function uses cubic spline interpolation to create a smooth trajectory that
     * passes through the given checkpoints at their specified times. It creates intermediate
     * points at regular time intervals for continuous motion control.
     *
     * @param checkpointsWithTimings Vector of pairs containing RobotArm configurations and timestamps.
     * @param timeStep The desired time step between interpolated points in seconds.
     * @return Vector of pairs containing interpolated RobotArm configurations and timestamps.
     */
    std::vector<std::pair<RobotArm, double>> generateSmoothTrajectory(
        const std::vector<std::pair<RobotArm, double>> &checkpointsWithTimings, double timeStep);

    /**
     * @brief Gets the current robot arm configuration.
     * @return The current RobotArm instance.
     */
    RobotArm getArm();

    /**
     * @brief Evaluates the cost function used in selectGoalPose for a specific free angle and pose.
     * @param pose The target pose.
     * @param q7 The free angle (7th joint) to use for IK computation.
     * @return A pair containing the computed cost value and success status.
     *         Returns {cost, true} if IK solution is found and valid (no collision),
     *         Returns {std::numeric_limits<double>::infinity(), false} if no valid solution.
     */
    std::pair<double, bool> evaluateSelectGoalPoseCost(const Eigen::Affine3d &pose, double q7);
};

#endif // PATHPLANNER_H
