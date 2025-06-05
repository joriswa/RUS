#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include <QObject>


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xyz.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/math/interpolators/cubic_b_spline.hpp>

#include <random>
#include <limits>
#include <cmath>
#include <thread>
#include <mutex>
#include <vector>
#include <QMutex>
#include <QWaitCondition>
#include <QDebug>
#include <QThread>
#include <chrono>
#include <QObject>
#include <algorithm>
#include <Qt3DRender/QObjectPicker>

#include "GeometryLib/BVHTree.h"
#include "TrajectoryLib/RobotArm.h"

using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Vector<double, 6>;

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef boost::geometry::model::point<double, 6, boost::geometry::cs::cartesian> Point;
typedef std::shared_ptr<class PathNode> NodePtr;

const double kRRG = 2.71828 * (1 + 1.0 / 6.0);

struct Snapshot {
    double time;
    double nnLookup;
    double rewiring;
    double currentcost;
};

enum Algorithm
{
    RRT,
    RRTStar,
    InformedRRTStar,
    RRTConnect
};

class PathNode : public std::enable_shared_from_this<PathNode> {
public:
    Point _state;
    NodePtr _parent;
    double _cost;
    double _costToGoal;
    RobotArm _arm; // Acts a robot arm state to track the movement

    PathNode(const Point& state, RobotArm arm, NodePtr parent = nullptr, double cost = 0.0, double _costToGoal = std::numeric_limits<double>::max())
        : _state(state), _arm(arm), _parent(parent), _cost(cost) {}

    NodePtr createChild(const Point& point, RobotArm arm) {
        return std::make_shared<PathNode>(point, arm, shared_from_this());
    }

    double estimateCost() {
        return this->_cost + this->_costToGoal;
    }
};

class TrajectoryPlanner : public QObject
{
    Q_OBJECT
private:
    double stepSize = 0.05;
    Algorithm _algorithm = RRT;
    double _obstacleEpsilon = .25;

    double _max_x, _max_y, _max_z;
    double _min_x, _min_y, _min_z;
    double _min_Theta, _max_Theta;
    Eigen::Matrix<double, 6, 1> _limits;

    QThread* _worker;
    std::shared_ptr<BVHTree> _obstacleTree;
    bgi::rtree<std::pair<Point, NodePtr>, bgi::rstar<8, 4>> _pathTree;
    NodePtr _pathRoot;
    std::vector<std::vector<Vec3>> _paths;

    Point _startPoint;
    Point _goalPoint;
    RobotArm _startArm;
    RobotArm _goalArm;
    double _goalRadius = 0.;
    Eigen::Vector3d _goalTranslation;
    Eigen::Matrix3d _goalRotation;
    std::vector<std::tuple<Point, RobotArm>> _path;

    bool motionIsValid(RobotArm& armA, RobotArm& endArm);

    // Helper functions for RRT
    void performRRT();
    NodePtr nearestPathNode(const Point& point);
    bool getNewNode(Point& randomPoint, NodePtr& newNode);

    // Helper functions for RRT*
    void performRRTStar();
    std::vector<NodePtr> findGeometricNeighbors(const NodePtr& node, double radius);
    void rewirePath(NodePtr newNode, std::vector<NodePtr> &neighboringNodes);

    // Helper functions and member variables for  RRT*
    Eigen::VectorXd sampleUnitBall();
    Eigen::MatrixXd computeRotationWorldFrame(const Point& start, const Point& goal);
    Point sampleWithinEllipsoid(double cmax, Eigen::MatrixXd C);
    void performInformedRRTStar();

    Point normalizeState(const Point&);
    Point denormalizeState(const Point&);
    double metric(const Point&, const Point&);

    void smoothPath();
    void prunePath();
    void smoothBSpline(unsigned int maxSteps, double minChange);
    bool shortcutPath(unsigned int maxSteps, unsigned int maxEmptySteps, double rangeRatio, double snapToVertex);

    bool closeToGoal(RobotArm arm);
    void computeStraightLinePath();
    NodePtr nearestNeighbor(const bgi::rtree<std::pair<Point, NodePtr>, bgi::rstar<8, 4> > &tree, const Point &point);
    void constructPath(NodePtr startNode, NodePtr goalNode);
    bool extendTree(bgi::rtree<std::pair<Point, NodePtr>, bgi::rstar<8, 4> > &tree, const Point &target, NodePtr &newNode);
    bool getNewNode(Point &randomPoint, NodePtr &newNode, const bgi::rtree<std::pair<Point, NodePtr>, bgi::rstar<8, 4> > &tree);
    bool performBiRRT();
    bool armHasCollision(RobotArm &arm, const BVHTree &tree);
    std::vector<NodePtr> findNearestNeighbors(bgi::rtree<std::pair<Point, NodePtr>, bgi::rstar<8, 4> > &tree, const NodePtr &node, unsigned int k);
    bool performBiRRTStar();

    Eigen::VectorXd computeObstacleGradient(const RobotArm &arm) const;
    double obstacleDistanceCost(double distance) const;
    Eigen::MatrixXd computeAMatrix(int trajLength) const;
    void performCHOMP();

    void initializeSTOMPMatrices();
    Eigen::MatrixXd generateNoisyTrajectories(const Eigen::MatrixXd& theta);
    Eigen::MatrixXd computeCosts(const Eigen::MatrixXd& trajectories);
    Eigen::MatrixXd computeProbabilities(const Eigen::MatrixXd& costs);
    Eigen::VectorXd computeUpdate(const Eigen::MatrixXd& noisyTrajectories, const Eigen::MatrixXd& probabilities);
    double computeTrajectoryCost(const Eigen::VectorXd& theta);

    Eigen::MatrixXd A, R, Rinv, M, L;
    int N;
    int K;
    double lambda;

    double computeCost(const RobotArm &arm);
public:
    TrajectoryPlanner(const RobotArm& arm, QThread* worker);
    // ~TrajectoryPlanner();
    bool armHasCollision(RobotArm& arm);

    void addSphereObstacle(Vec3 center, double radius);
    std::vector<std::tuple<Vec3, double>> getSphereObstacleGeometries();

    void setStartPose(RobotArm arm);
    void setGoalPose(Eigen::Vector3d t, Eigen::Matrix3d r);

    std::vector<std::tuple<Vec3, RobotArm>> getPath();
    std::vector<std::vector<Vec3>> getPaths();

    void setAlgorithm(Algorithm algorithm);

    std::shared_ptr<BVHTree> obstacleTree() const;
    void setObstacleTree(const std::shared_ptr<BVHTree>& newObstacleTree);

    std::vector<NodePtr> findNearestNeighbors(const NodePtr &node, unsigned int k);
public slots:
    void runPathFinding();
    void performMultiple();


signals:
    void finished();

};

#endif // TRAJECTORYPLANNER_H
