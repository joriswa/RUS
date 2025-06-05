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
#include "TrajectoryLib/RobotArm.h"

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
enum Algorithm { RRT, RRTStar, InformedRRTStar, RRTConnect };

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
     * @return The estimated total cost.
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
     * @brief Performs the RRT algorithm.
     */
    bool performRRT();

    /**
     * @brief Performs the RRT-Connect algorithm.
     * @return True if a path was found, false otherwise.
     */
    bool performRRTConnect();

    /**
     * @brief Performs the RRT* algorithm.
     */
    bool performRRTStar(const std::string &costTrackingFile = "");

    /**
     * @brief Performs the Informed RRT* algorithm.
     */
    bool performInformedRRTStar(const std::string &costTrackingFile = "");

    /**
     * @brief Finds the nearest node in joint space.
     * @param point The point to find the nearest node to.
     * @return The nearest node.
     */
    NodePtr nearestJointNode(const State &point);

    /**
     * @brief Generates a new node in joint space.
     * @param randomPoint The random point to extend towards.
     * @param newNode The new node generated.
     * @return True if a new node was successfully generated, false otherwise.
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
     * @brief Checks if an arm configuration is close to the goal.
     * @param arm The arm configuration to check.
     * @return True if the arm is close to the goal, false otherwise.
     */
    bool closeToGoal(RobotArm arm);

    /**
     * @brief Gets the sampling matrix for informed sampling.
     * @param cBest The best cost found so far.
     * @return The sampling matrix.
     */
    Eigen::MatrixXd getSamplingMatrix(double cBest);

    /**
     * @brief Samples a point from a unit ball.
     * @return A vector representing a point in the unit ball.
     */
    Eigen::VectorXd sampleUnitBall();

    /**
     * @brief Samples a point within an ellipsoid.
     * @param cmax The maximum cost.
     * @param C The transformation matrix for the ellipsoid.
     * @return A state sampled from within the ellipsoid.
     */
    State sampleWithinEllipsoid(double cmax, Eigen::MatrixXd C);

    /**
     * @brief Computes the rotation matrix in the world frame.
     * @param start The start state.
     * @param goal The goal state.
     * @return The rotation matrix.
     */
    Eigen::MatrixXd computeRotationWorldFrame(const State &start, const State &goal);

    /**
     * @brief Computes the metric distance between two states.
     * @param a The first state.
     * @param b The second state.
     * @return The metric distance between the states.
     */
    double metric(const State &a, const State &b);

    /**
     * @brief Constructs the final path from the start node to the goal node.
     * @param startNode The start node.
     * @param goalNode The goal node.
     */
    void constructPath(NodePtr startNode, NodePtr goalNode);

    /**
     * @brief Finds the nearest neighbor in the tree.
     * @param tree The tree to search.
     * @param point The point to find the nearest neighbor to.
     * @return The nearest neighbor node.
     */
    NodePtr nearestNeighbor(const bgi::rtree<std::pair<State, NodePtr>, bgi::rstar<16>> &tree,
                            const State &point);

    /**
     * @brief Extends the tree towards a target point.
     * @param tree The tree to extend.
     * @param target The target point to extend towards.
     * @param newNode The new node created by the extension.
     * @return True if the extension was successful, false otherwise.
     */
    bool extendTree(bgi::rtree<std::pair<State, NodePtr>, bgi::rstar<16>> &tree,
                    const State &target,
                    NodePtr &newNode);

    /**
     * @brief Generates a new node in the tree.
     * @param randomPoint The random point to extend towards.
     * @param newNode The new node generated.
     * @param tree The tree to extend.
     * @return True if a new node was successfully generated, false otherwise.
     */
    bool getNewNode(State &randomPoint,
                    NodePtr &newNode,
                    const bgi::rtree<std::pair<State, NodePtr>, bgi::rstar<16>> &tree);

    /**
     * @brief Connects the path with straight lines.
     */
    void connectWithStraightLines();

    /**
     * @brief Computes the total joint distance of the current path.
     * @return The total joint distance.
     */
    double computeTotalJointDistance() const;

    /**
     * @brief Computes the total jerk of the current path.
     * @return The total jerk.
     */
    double computeTotalJerk() const;

    /**
     * @brief Computes the total displacement of the current path.
     * @return The total displacement.
     */
    double computeTotalDisplacement();

    /**
     * @brief Computes the total distance to the environment of the current path.
     * @return The total distance to the environment.
     */
    double computeTotalDistanceToEnviornment();

    /**
     * @brief Performs the RRT*-Connect algorithm.
     * @return True if a path was found, false otherwise.
     */
    bool performRRTStarConnect();

    /**
     * @brief Finds the k-nearest neighbors in the tree.
     * @param tree The tree to search.
     * @param node The node to find neighbors for.
     * @param k The number of neighbors to find.
     * @return A vector of the k-nearest neighbors.
     */
    std::vector<NodePtr> findNearestNeighbors(
        bgi::rtree<std::pair<State, NodePtr>, bgi::rstar<16>> &tree,
        const NodePtr &node,
        unsigned int k);

public:
    /**
     * @brief Constructs a PathPlanner object.
     * @param worker The QThread to be used for path planning operations.
     */
    PathPlanner();

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
    struct ClearanceMetrics {
        double min_clearance = std::numeric_limits<double>::infinity();     ///< Minimum distance to any obstacle
        double avg_clearance = 0.0;                                         ///< Average clearance across all links
        double weighted_clearance = 0.0;                                    ///< Volume-weighted clearance
        std::vector<double> link_clearances;                                ///< Individual clearance for each link
        std::vector<Vec3> closest_points;                                   ///< Closest points on obstacles for each link
        bool has_collision = false;                                         ///< Whether any link is in collision
        int num_links_checked = 0;                                          ///< Number of links that were checked
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
     * @param algorithm The chosen algorithm.
     */
    void setAlgorithm(Algorithm algorithm);

    /**
     * @brief Gets the obstacle tree.
     * @return A shared pointer to the BVHTree representing obstacles.
     */
    std::shared_ptr<BVHTree> obstacleTree() const;

    /**
     * @brief Sets the obstacle tree.
     * @param newObstacleTree The new obstacle tree.
     */
    void setObstacleTree(const std::shared_ptr<BVHTree> &newObstacleTree);

    /**
     * @brief Gets the computed path.
     * @return A vector of tuples containing states and arm configurations.
     */
    std::vector<std::tuple<Vec3, RobotArm>> getPath();

    /**
     * @brief Selects a goal pose for the robot arm.
     * @param pose The target pose.
     * @return The selected robot arm configuration.
     */
    std::pair<RobotArm, bool> selectGoalPose(const Eigen::Affine3d &pose);

    /**
     * @brief Saves the joint angles of the path to a file.
     * @param filename The name of the file to save to.
     */
    void saveJointAnglesToFile(const std::string &filename);

    /**
     * @brief Gets the path as a matrix of joint angles.
     * @return An Eigen::MatrixXd containing the joint angles for each step in the path.
     */
    Eigen::MatrixXd getAnglesPath() const;

    /**
     * @brief Sets the parameters for path planning.
     * @param newParams The new parameters to use.
     */
    void setParams(const Params &newParams);

    bool runPathFinding();
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
    struct CheckpointPlanResult {
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
     * @brief Densifies sparse task space poses through interpolation
     */
    std::vector<Eigen::Affine3d> densifyTaskSpacePath(
        const std::vector<Eigen::Affine3d> &originalPoses);
    
    /**
     * @brief Solves inverse kinematics for densified poses with continuity optimization
     */
    void solveInverseKinematics(
        const std::vector<Eigen::Affine3d> &densifiedPoses,
        const Eigen::VectorXd &currentJoints,
        CheckpointPlanResult &result,
        std::vector<size_t> &selectGoalPoseFallbacks);
    
    /**
     * @brief Attempts continuous IK solving using small variations
     */
    std::pair<std::array<double, 7>, bool> attemptContinuousIK(
        const Eigen::Matrix<double, 4, 4> &targetMatrix,
        const std::array<double, 7> &currentJointAngles,
        std::mt19937 &gen,
        std::uniform_real_distribution<> &dis);
    
    /**
     * @brief Handles IK fallback when continuous solving fails
     */
    void handleIKFallback(
        const Eigen::Affine3d &pose,
        size_t poseIdx,
        std::array<double, 7> &currentJointAngles,
        CheckpointPlanResult &result,
        std::vector<size_t> &selectGoalPoseFallbacks);
    
    /**
     * @brief Segments the path into continuous trajectory pieces and jump pairs
     */
    void segmentPath(
        CheckpointPlanResult &result,
        const std::vector<size_t> &selectGoalPoseFallbacks);

public:
    /**
     * @brief Generates a smooth trajectory from a set of timed checkpoints
     * 
     * This function uses cubic spline interpolation to create a smooth trajectory that
     * passes through the given checkpoints at their specified times. It creates intermediate
     * points at regular time intervals for continuous motion.
     * 
     * @param checkpointsWithTimings A vector of pairs containing RobotArm configurations and timestamps
     * @param timeStep The desired time step between interpolated points in seconds
     * @return A vector of pairs containing interpolated RobotArm configurations and timestamps
     */
    std::vector<std::pair<RobotArm, double>> generateSmoothTrajectory(
        const std::vector<std::pair<RobotArm, double>> &checkpointsWithTimings, double timeStep);
    RobotArm getArm();
};

#endif // PATHPLANNER_H
