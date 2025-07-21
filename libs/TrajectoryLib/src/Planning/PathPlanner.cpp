#include "TrajectoryLib/Planning/PathPlanner.h"
#include "TrajectoryLib/Robot/franka_ik_He.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <future>

PathPlanner::PathPlanner() {}

PathNode::PathNode(const State &state, RobotArm arm, NodePtr parent, double cost, double costToGoal)
    : _state(state)
    , _arm(arm)
    , _parent(parent)
    , _cost(cost)
    , _costToGoal(costToGoal)
{}

NodePtr PathNode::createChild(const State &point, RobotArm arm)
{
    return std::make_shared<PathNode>(point, arm, shared_from_this());
}

double PathNode::estimateCost()
{
    return this->_cost + this->_costToGoal;
}

NodePtr PathPlanner::nearestJointNode(const State &point)
{
    std::vector<std::pair<State, NodePtr>> results;
    _tree.query(bgi::nearest(point, 1), std::back_inserter(results));
    return results.front().second;
}

bool PathPlanner::getNewJointNode(State &randomPoint, NodePtr &newNode)
{
    NodePtr nearestNode = nearestJointNode(randomPoint);
    State nearestState = nearestNode->_state;

    bg::subtract_point(randomPoint, nearestState);
    double length = std::sqrt(bg::dot_product(randomPoint, randomPoint));

    if (length > 0) {
        bg::divide_value(randomPoint, length);
        bg::multiply_value(randomPoint, _params.stepSize);
        bg::add_point(randomPoint, nearestState);
    }

    bg::set<0>(randomPoint, std::clamp(bg::get<0>(randomPoint), 0.0, 1.0));
    bg::set<1>(randomPoint, std::clamp(bg::get<1>(randomPoint), 0.0, 1.0));
    bg::set<2>(randomPoint, std::clamp(bg::get<2>(randomPoint), 0.0, 1.0));
    bg::set<3>(randomPoint, std::clamp(bg::get<3>(randomPoint), 0.0, 1.0));
    bg::set<4>(randomPoint, std::clamp(bg::get<4>(randomPoint), 0.0, 1.0));
    bg::set<5>(randomPoint, std::clamp(bg::get<5>(randomPoint), 0.0, 1.0));
    bg::set<6>(randomPoint, std::clamp(bg::get<6>(randomPoint), 0.0, 1.0));

    RobotArm closestArmState = nearestNode->_arm;

    if (!closestArmState.moveToJointState(randomPoint)) {
        return false;
    }

    RobotArm temp = closestArmState;
    if (!motionIsValid(nearestNode->_arm, temp, true)) {
        return false;
    }

    newNode = nearestNode->createChild(randomPoint, closestArmState);
    newNode->_cost = nearestNode->_cost + metric(randomPoint, nearestState);

    return true;
}

Eigen::MatrixXd PathPlanner::getAnglesPath() const
{
    Eigen::MatrixXd angles(_path.size(), 7);
    int i = 0;
    for (auto step : _path) {
        auto [state, arm] = step;
        angles.row(i) = arm.getJointAngles();
        ++i;
    }
    return angles;
}

std::vector<std::tuple<Vec3, RobotArm>> PathPlanner::getPath()
{
    std::vector<std::tuple<Vec3, RobotArm>> path;
    for (const auto &[p, arm] : _path) {
        path.push_back(std::make_tuple(arm.getEndeffectorPose().translation(), arm));
    }
    return path;
}

bool PathPlanner::armHasCollision(RobotArm &arm, bool inflate)
{
    if (!_obstacleTree) {
        return false;
    }

    auto bBoxes = arm.getCollisionBoxes();
    auto links = arm.getLinks();
    int i = 0;
    for (const auto& bBox : bBoxes) {
        ++i;
        if (i < 4)
            continue;

        auto [center, halfDims, axes] = bBox;

        if (inflate) {
            halfDims *= 1.1;
        }

        if (_obstacleTree->isBoxIntersecting(center, halfDims, axes)) {
            return true;
        }
    }

    return false;
}

void PathPlanner::setParams(const Params &newParams)
{
    _params = newParams;
}

bool PathPlanner::motionIsValid(RobotArm &startArm, RobotArm &endArm, bool inflate)
{
    if (armHasCollision(startArm) || armHasCollision(endArm)) {
        return false;
    }
    RobotArm tmpArm(startArm);
    State startState = startArm.getJointState();
    State endState = endArm.getJointState();
    double dist = bg::distance(startState, endState);
    auto oldAngles = tmpArm.getJointAngles();
    if (dist > 0.01) {
        int numSteps = static_cast<int>(std::ceil(dist / 0.01));
        bg::subtract_point(endState, startState);
        bg::divide_value(endState, numSteps);

        for (int i = 1; i < numSteps; ++i) {
            bg::add_point(startState, endState);
            if (!tmpArm.moveToJointState(startState)) {
                return false;
            }
            if (armHasCollision(tmpArm, inflate)) {
                return false;
            }
        }
    }
    auto newAngles = tmpArm.getJointAngles();
    return ((oldAngles - newAngles).array().abs() < 2.0 * M_PI).all();
}

bool PathPlanner::performRRT()
{
    _path.clear();
    _tree.clear();
    _tree.insert(std::make_pair(_pathRoot->_state, _pathRoot));

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    bool success = false;

    for (size_t i = 0; i < _params.maxIterations; ++i) {
        State randomPoint;
        std::string lala;
        if (dis(gen) < _params.goalBiasProbability) {
            randomPoint = _goalArm.getJointState();
        } else {
            bg::set<0>(randomPoint, dis(gen));
            bg::set<1>(randomPoint, dis(gen));
            bg::set<2>(randomPoint, dis(gen));
            bg::set<3>(randomPoint, dis(gen));
            bg::set<4>(randomPoint, dis(gen));
            bg::set<5>(randomPoint, dis(gen));
            bg::set<6>(randomPoint, dis(gen));
        }

        NodePtr newNode;
        if (getNewJointNode(randomPoint, newNode)) {
            _tree.insert(std::make_pair(newNode->_state, newNode));
            if (closeToGoal(newNode->_arm)) {
                NodePtr node = newNode;
                while (node) {
                    _path.push_back(std::make_tuple(node->_state, node->_arm));
                    node = node->_parent;
                    success = true;
                }
                std::reverse(_path.begin(), _path.end());
                _path.push_back(std::make_pair(_goalArm.getJointState(), _goalArm));
                break;
            }
        }
    }

    return success;
}

void PathPlanner::setStartPose(RobotArm arm)
{
    _startPoint = arm.getJointState();
    _pathRoot = std::make_shared<PathNode>(_startPoint, arm);
    _startArm = arm;
    _tree.insert(std::make_pair(_pathRoot->_state, _pathRoot));
}

void PathPlanner::setGoalPose(Eigen::Vector3d t, Eigen::Matrix3d r)
{
    _goalTranslation = t;
    _goalRotation = r;
    _goalConfigSpecified = false;
}

void PathPlanner::setGoalConfiguration(RobotArm arm)
{
    _goalConfigSpecified = true;
    _goalArm = arm;
}

void PathPlanner::setObstacleTree(const std::shared_ptr<BVHTree> &newObstacleTree)
{
    _obstacleTree = newObstacleTree;
}

bool PathPlanner::performRRTStar(const std::string &costTrackingFile)
{
    _tree.clear();
    _tree.insert(std::make_pair(_pathRoot->_state, _pathRoot));

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    double bestCostToGoal = std::numeric_limits<double>::max();
    NodePtr bestGoalNode = nullptr;

    // Time tracking variables
    auto startTime = std::chrono::high_resolution_clock::now();
    std::ofstream costTrackingStream;
    if (!costTrackingFile.empty()) {
        costTrackingStream.open(costTrackingFile);
        if (costTrackingStream.is_open()) {
            costTrackingStream << "Iteration,Time(ms),BestCost\n";
        }
    }

    bool success = false;

    for (size_t i = 0; i < _params.maxIterations; ++i) {
        State randomPoint;
        if (dis(gen) < _params.goalBiasProbability && bestGoalNode == nullptr) {
            randomPoint = _goalArm.getJointState();
        } else {
            bg::set<0>(randomPoint, dis(gen));
            bg::set<1>(randomPoint, dis(gen));
            bg::set<2>(randomPoint, dis(gen));
            bg::set<3>(randomPoint, dis(gen));
            bg::set<4>(randomPoint, dis(gen));
            bg::set<5>(randomPoint, dis(gen));
            bg::set<6>(randomPoint, dis(gen));
        }

        NodePtr newNode;
        if (getNewJointNode(randomPoint, newNode)) {
            size_t n = _tree.size();
            size_t k = std::max(static_cast<size_t>(std::ceil(kRRG * std::log(n))), 5ul);
            std::vector<NodePtr> neighborhood = findNearestNeighbors(newNode, k);
            NodePtr bestParent = newNode->_parent;
            double bestCost = newNode->_cost;

            for (auto &neighbor : neighborhood) {
                double cost = neighbor->_cost + metric(neighbor->_state, newNode->_state);
                if (cost < bestCost && motionIsValid(neighbor->_arm, newNode->_arm, true)) {
                    bestParent = neighbor;
                    bestCost = cost;
                }
            }

            if (bestParent != newNode->_parent) {
                newNode->_parent = bestParent;
                newNode->_cost = bestCost;
            }

            rewirePath(newNode, neighborhood);
            _tree.insert(std::make_pair(newNode->_state, newNode));

            double totalCost = newNode->_cost
                               + metric(newNode->_arm.getJointState(), _goalArm.getJointState());
            if (closeToGoal(newNode->_arm) && totalCost < bestCostToGoal) {
                bestCostToGoal = totalCost;
                bestGoalNode = newNode;
                success = true;
            }
        }

        if (costTrackingStream.is_open()) {
            auto currentTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime
                                                                                  - startTime);
            costTrackingStream << i + 1 << "," << duration.count() << "," << bestCostToGoal << "\n";
        }
    }

    if (costTrackingStream.is_open()) {
        costTrackingStream.close();
    }

    if (success) {
        NodePtr node = bestGoalNode;
        while (node) {
            _path.push_back(std::make_tuple(node->_state, node->_arm));
            node = node->_parent;
        }
        std::reverse(_path.begin(), _path.end());
        _path.push_back(std::make_pair(_goalArm.getJointState(), _goalArm));
    }

    return success;
}

std::vector<NodePtr> PathPlanner::findNearestNeighbors(const NodePtr &node, unsigned int k)
{
    std::vector<NodePtr> nodes;
    std::vector<std::pair<State, NodePtr>> results;
    _tree.query(bgi::nearest(node->_state, k), std::back_inserter(results));

    for (auto const &pair : results) {
        nodes.push_back(pair.second);
    }

    return nodes;
}

std::vector<NodePtr> PathPlanner::rewirePath(NodePtr newNode, std::vector<NodePtr> &neighboringNodes)
{
    std::vector<NodePtr> changedNodes;
    for (NodePtr &neighbor : neighboringNodes) {
        double newCost = newNode->_cost + metric(newNode->_state, neighbor->_state);
        if (newCost < neighbor->_cost && motionIsValid(newNode->_arm, neighbor->_arm, true)) {
            neighbor->_parent = newNode;
            neighbor->_cost = newCost;
            changedNodes.push_back(neighbor);
        }
    }
    return changedNodes;
}

bool PathPlanner::closeToGoal(RobotArm arm)
{
    Eigen::Vector<double, 7> currentJoints = arm.getJointAngles();
    Eigen::Vector<double, 7> goalJoints = _goalArm.getJointAngles();

    return motionIsValid(arm, _goalArm);
}

bool PathPlanner::runPathFinding()
{
    if (!_goalConfigSpecified) {
        Eigen::Affine3d goalTrafo;
        goalTrafo.linear() = _goalRotation;
        goalTrafo.translation() = _goalTranslation;
        _goalArm = selectGoalPose(goalTrafo).first;
    }

    _path.clear();
    bool res = false;
    switch (_params.algo) {
    case RRT:
        res = performRRT();
        break;
    case RRTStar:
        res = performRRTStar();
        break;
    case InformedRRTStar:
        res = performInformedRRTStar();
        break;
    case RRTConnect:
        res = performRRTConnect();
        break;
    }

    return res;
}

double PathPlanner::metric(const State &a, const State &b)
{
    double sum = 0.0;
    sum += (bg::get<0>(a) - bg::get<0>(b)) * (bg::get<0>(a) - bg::get<0>(b));
    sum += (bg::get<1>(a) - bg::get<1>(b)) * (bg::get<1>(a) - bg::get<1>(b));
    sum += (bg::get<2>(a) - bg::get<2>(b)) * (bg::get<2>(a) - bg::get<2>(b));
    sum += (bg::get<3>(a) - bg::get<3>(b)) * (bg::get<3>(a) - bg::get<3>(b));
    sum += (bg::get<4>(a) - bg::get<4>(b)) * (bg::get<4>(a) - bg::get<4>(b));
    sum += (bg::get<5>(a) - bg::get<5>(b)) * (bg::get<5>(a) - bg::get<5>(b));
    sum += (bg::get<6>(a) - bg::get<6>(b)) * (bg::get<6>(a) - bg::get<6>(b));
    return std::sqrt(sum);
}

std::pair<RobotArm, bool> PathPlanner::selectGoalPose(const Eigen::Affine3d &pose)
{
    _path.clear();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-2.8973, 2.8973);

    Eigen::Matrix<double, 7, 1> jointAngles = _startArm.getJointAngles();
    std::array<double, 7> jointAnglesArray;
    Eigen::Map<Eigen::Matrix<double, 7, 1>>(jointAnglesArray.data()) = jointAngles;

    Eigen::Matrix3d Rx = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d axes = Rx * Ry * Rz;

    Eigen::Affine3d conv;
    conv.linear() = axes;

    Eigen::Affine3d total = pose * _startArm.getEndeffectorTransform().inverse();

    Eigen::Matrix<double, 7, 1> bestSolution;
    double bestCost = std::numeric_limits<double>::infinity();

    int noImprovementCounter = 0;
    const int maxNoImprovement = 200;

    for (int iter = 0; iter < 20000; ++iter) {
        double randomValue;
        if (iter > 1000 && bestCost < std::numeric_limits<double>::infinity()) {
            double bestQ7 = bestSolution[6];
            double samplingRadius = 2.8973 * std::max(0.1, 1.0 - iter / 10000.0);
            std::uniform_real_distribution<> adaptiveDis(std::max(-2.8973, bestQ7 - samplingRadius),
                                                         std::min(2.8973, bestQ7 + samplingRadius));
            randomValue = adaptiveDis(gen);
        } else {
            randomValue = dis(gen);
        }

        std::array<std::array<double, 7>, 4> ikSolutions = franka_IK_EE(total,
                                                                        randomValue,
                                                                        jointAnglesArray);

        bool improved = false;

        for (const auto &sol : ikSolutions) {
            if (std::any_of(sol.begin(), sol.end(), [](double val) { return std::isnan(val); })) {
                continue;
            }

            Eigen::Map<const Eigen::Matrix<double, 7, 1>> angles(sol.data());
            RobotArm temp = _startArm;
            temp.setJointAngles(angles);

            if (armHasCollision(temp)) {
                continue;
            }

            double penalty = 0.0;
            const double threshold = 0.3; // 10 cm
            int numLinks = static_cast<int>(temp.getLinkBoundingBoxes().size());
            double maxPenaltyPerLink = 1.0 / (numLinks - 2);

            for (int linkIdx = 2; linkIdx < numLinks; ++linkIdx) {
                auto bboxCenter = std::get<0>(temp.getLinkBoundingBoxes()[linkIdx]);
                double dist = 0.0;
                if (_obstacleTree) {
                    dist = std::get<0>(_obstacleTree->getDistanceAndClosestPoint(bboxCenter));
                }

                if (dist < threshold) {
                    double linkPenalty = 1.0 / (1.0 + std::exp(100 * (dist - threshold)));
                    penalty += linkPenalty * maxPenaltyPerLink;
                }
            }

            penalty = std::min(penalty, 1.0);
            penalty += 0.25 * temp.computeManipulability();

            if (penalty < bestCost) {
                bestCost = penalty;
                bestSolution = angles;
                improved = true;
            }
        }

        if (improved) {
            noImprovementCounter = 0;
        } else {
            noImprovementCounter++;
        }

        if (noImprovementCounter >= maxNoImprovement) {
            break;
        }
    }

    RobotArm finalArm = _startArm;
    bool success = true;

    if (bestCost == std::numeric_limits<double>::infinity()) {
        success = false;
    } else {
        finalArm.setJointAngles(bestSolution);
        
        // Sanity check: Verify pose accuracy before declaring success
        Eigen::Affine3d achieved_pose = finalArm.getEndeffectorPose();
        Eigen::Vector3d position_error = pose.translation() - achieved_pose.translation();
        double pos_error = position_error.norm();
        
        Eigen::Matrix3d rotation_error = pose.linear() * achieved_pose.linear().transpose();
        Eigen::AngleAxisd angle_axis(rotation_error);
        double orientation_error = std::abs(angle_axis.angle());
        
        // Define acceptable tolerances (aligned with Franka Panda ±0.1mm repeatability)
        const double max_position_error = 0.001;     // 1mm tolerance
        const double max_orientation_error = 0.017;  // ~1 degree tolerance (0.017 radians)
        
        // Fail if pose accuracy is insufficient
        if (pos_error > max_position_error || orientation_error > max_orientation_error) {
            success = false;
        }
    }

    return std::make_pair(finalArm, success);
}

void PathPlanner::constructPath(NodePtr startNode, NodePtr goalNode)
{
    _path.clear();

    for (NodePtr node = startNode; node != nullptr; node = node->_parent) {
        _path.insert(_path.begin(), std::make_tuple(node->_state, node->_arm));
    }

    for (NodePtr node = goalNode; node != nullptr; node = node->_parent) {
        _path.push_back(std::make_tuple(node->_state, node->_arm));
    }
}

NodePtr PathPlanner::nearestNeighbor(
    const bgi::rtree<std::pair<State, NodePtr>, bgi::rstar<16>> &tree, const State &point)
{
    std::vector<std::pair<State, NodePtr>> result;
    tree.query(bgi::nearest(point, 1), std::back_inserter(result));
    return result[0].second;
}

bool PathPlanner::getNewNode(State &randomPoint,
                             NodePtr &newNode,
                             const bgi::rtree<std::pair<State, NodePtr>, bgi::rstar<16>> &tree)
{
    NodePtr nearestNode = nearestNeighbor(tree, randomPoint);
    State nearestState = nearestNode->_state;
    bg::subtract_point(randomPoint, nearestState);
    double length = std::sqrt(bg::dot_product(randomPoint, randomPoint));

    if (length > 0) {
        bg::divide_value(randomPoint, length);
        bg::multiply_value(randomPoint, _params.stepSize);
        bg::add_point(randomPoint, nearestNode->_state);
    }

    bg::set<0>(randomPoint, std::clamp(bg::get<0>(randomPoint), 0.0, 1.0));
    bg::set<1>(randomPoint, std::clamp(bg::get<1>(randomPoint), 0.0, 1.0));
    bg::set<2>(randomPoint, std::clamp(bg::get<2>(randomPoint), 0.0, 1.0));
    bg::set<3>(randomPoint, std::clamp(bg::get<3>(randomPoint), 0.0, 1.0));
    bg::set<4>(randomPoint, std::clamp(bg::get<4>(randomPoint), 0.0, 1.0));
    bg::set<5>(randomPoint, std::clamp(bg::get<5>(randomPoint), 0.0, 1.0));
    bg::set<6>(randomPoint, std::clamp(bg::get<6>(randomPoint), 0.0, 1.0));

    RobotArm closestArmState = nearestNode->_arm;

    if (!closestArmState.moveToJointState(randomPoint)) {
        return false;
    }

    if (!motionIsValid(nearestNode->_arm, closestArmState)) {
        return false;
    }

    newNode = nearestNode->createChild(randomPoint, closestArmState);
    newNode->_cost = nearestNode->_cost + metric(randomPoint, nearestNode->_state);

    return true;
}

bool PathPlanner::extendTree(bgi::rtree<std::pair<State, NodePtr>, bgi::rstar<16>> &tree,
                             const State &target,
                             NodePtr &newNode)
{
    State newState = target;
    if (!getNewNode(newState, newNode, tree)) {
        return false;
    }
    tree.insert(std::make_pair(newState, newNode));
    return true;
}

bool PathPlanner::performRRTConnect()
{
    bgi::rtree<std::pair<State, NodePtr>, bgi::rstar<16>> startTree;
    bgi::rtree<std::pair<State, NodePtr>, bgi::rstar<16>> goalTree;
    _path.clear();

    NodePtr start = std::make_shared<PathNode>(_startArm.getJointState(), _startArm);
    NodePtr goal = std::make_shared<PathNode>(_goalArm.getJointState(), _goalArm);

    // Check for immediate connection before iterating
    if (motionIsValid(_startArm, _goalArm, true)) {
        constructPath(start, goal);
        return true;
    }

    startTree.insert(std::make_pair(_startArm.getJointState(), start));
    goalTree.insert(std::make_pair(_goalArm.getJointState(), goal));

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);

    bool isStartTree = true;
    for (unsigned int i = 0; i < _params.maxIterations; ++i) {
        State randomPoint;
        if (dis(gen) < _params.goalBiasProbability) {
            randomPoint = isStartTree ? _goalArm.getJointState() : _startArm.getJointState();
        } else {
            bg::set<0>(randomPoint, dis(gen));
            bg::set<1>(randomPoint, dis(gen));
            bg::set<2>(randomPoint, dis(gen));
            bg::set<3>(randomPoint, dis(gen));
            bg::set<4>(randomPoint, dis(gen));
            bg::set<5>(randomPoint, dis(gen));
            bg::set<6>(randomPoint, dis(gen));
        }

        NodePtr newNode;
        if (extendTree(isStartTree ? startTree : goalTree, randomPoint, newNode)) {
            NodePtr nearestOther = nearestNeighbor(isStartTree ? goalTree : startTree,
                                                   newNode->_state);
            if (motionIsValid(newNode->_arm, nearestOther->_arm, true)) {
                constructPath(isStartTree ? newNode : nearestOther,
                              isStartTree ? nearestOther : newNode);
                return true;
            }
        }

        isStartTree = !isStartTree;
    }
    return false;
}

void PathPlanner::connectWithStraightLines()
{
    if (_path.size() < 2)
        return; // No need to connect if path is too short

    double targetDistance = 0.05;

    std::vector<std::tuple<State, RobotArm>> newPath;
    newPath.reserve(_path.size() * 2); // Initial estimate, may grow

    auto interpolate = [](const State &p1, const State &p2, double t) {
        State result = p1;
        State tmp = p2;
        bg::multiply_value(result, (1. - t));
        bg::multiply_value(tmp, t);
        bg::add_point(result, tmp);
        return result;
    };

    for (size_t i = 0; i < _path.size() - 1; ++i) {
        auto [startState, startArm] = _path[i];
        auto [endState, endArm] = _path[i + 1];

        newPath.push_back(_path[i]); // Add the start point of this segment

        double segmentLength = bg::distance(startState, endState);
        int numIntermediatePoints
            = std::max(1, static_cast<int>(std::ceil(segmentLength / targetDistance)) - 1);

        for (int j = 1; j <= numIntermediatePoints; ++j) {
           double t = static_cast<double>(j) / (numIntermediatePoints + 1);
           State intermediateState = interpolate(startState, endState, t);
           RobotArm intermediateArm = startArm;
           if (intermediateArm.moveToJointState(intermediateState)) {
                newPath.push_back(std::make_tuple(intermediateState, intermediateArm));
           } else {
                qDebug() << "Warning: Could not move to interpolated state at t =" << t;
           }
        }
    }

    // Add the final point
    newPath.push_back(_path.back());
    _path = std::move(newPath);
}

void PathPlanner::saveJointAnglesToFile(const std::string &filename)
{
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        qDebug() << "Failed to open file for writing:" << QString::fromStdString(filename);
        return;
    }

    outFile << "Waypoint,Joint1,Joint2,Joint3,Joint4,Joint5,Joint6,Joint7\n";
    auto angles = getAnglesPath();
    for (size_t i = 0; i < _path.size(); ++i) {
        const auto &[jointState, arm] = _path[i];
        outFile << i;
        for (int k = 0; k < 7; ++k) {
            outFile << "," << angles.row(i)[k];
        }
        outFile << "\n";
    }

    outFile.close();
    // qDebug() << "Joint angles saved to:" << QString::fromStdString(filename);
}

Eigen::MatrixXd PathPlanner::getSamplingMatrix(double cBest)
{
    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(7, 7);
    Eigen::Matrix3d rotation = _goalRotation.transpose();
    Eigen::Vector3d translation = _goalTranslation
                                  - _pathRoot->_arm.getEndeffectorPose().translation();

    double cMin = translation.norm();
    double r = std::sqrt(cBest * cBest - cMin * cMin) / 2.0;
    Eigen::Vector3d d = (translation / cMin) * r;

    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(7, 7);
    C.block<3, 3>(0, 0) = rotation;
    C.block<3, 1>(0, 6) = d;

    L.diagonal() << r, r, r, r, r, r, r;

    return L * C;
}

Eigen::VectorXd PathPlanner::sampleUnitBall()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    std::uniform_real_distribution<> gauss(0.0, 1.0);

    Eigen::VectorXd p(8);
    for (size_t i = 0; i < 7; ++i) {
        p[i] = gauss(gen);
    }

    p.normalize();

    double radius = std::pow(dis(gen), 1.0 / 8.0);
    p *= radius;

    p[7] = 0.0;

    return p;
}

State PathPlanner::sampleWithinEllipsoid(double cmax, Eigen::MatrixXd C)
{
    State xcenter = _goalArm.getJointState();
    bg::add_point(xcenter, _startPoint);
    bg::divide_value(xcenter, 2.0);

    double cmin = bg::distance(_startArm.getJointState(), _goalArm.getJointState());

    double r1 = cmax / 2.0;
    double r2 = std::sqrt((cmax * cmax) - (cmin * cmin) / 2.0);

    Eigen::DiagonalMatrix<double, 8> L;
    L.diagonal() << r1, r2, r2, r2, r2, r2, r2, r2;

    auto x = sampleUnitBall();

    Eigen::VectorXd xEllipsoid = C * L * x;

    State xball;

    bg::set<0>(xball, xEllipsoid[0]);
    bg::set<1>(xball, xEllipsoid[1]);
    bg::set<2>(xball, xEllipsoid[2]);
    bg::set<3>(xball, xEllipsoid[3]);
    bg::set<4>(xball, xEllipsoid[4]);
    bg::set<5>(xball, xEllipsoid[5]);
    bg::set<6>(xball, xEllipsoid[6]);

    bg::add_point(xball, xcenter);

    return xball;
}

Eigen::MatrixXd PathPlanner::computeRotationWorldFrame(const State &start, const State &goal)
{
    State tmp = goal;
    double length = bg::distance(start, goal);
    bg::subtract_point(tmp, start);
    bg::divide_value(tmp, length);

    std::vector<double> a1(8);
    a1[0] = bg::get<0>(tmp);
    a1[1] = bg::get<1>(tmp);
    a1[2] = bg::get<2>(tmp);
    a1[3] = bg::get<3>(tmp);
    a1[4] = bg::get<4>(tmp);
    a1[5] = bg::get<5>(tmp);
    a1[6] = bg::get<6>(tmp);
    a1[7] = 0.0;

    auto M = Eigen::Map<Eigen::VectorXd>(&*a1.begin(), a1.size())
             * Eigen::MatrixXd::Identity(1, a1.size());
    auto svd = Eigen::JacobiSVD<Eigen::MatrixXd>(M, Eigen::ComputeFullU | Eigen::ComputeFullV);

    auto diag_v = std::vector<double>(a1.size(), 1.0);
    diag_v[diag_v.size() - 1] = svd.matrixV().determinant();
    diag_v[diag_v.size() - 2] = svd.matrixU().determinant();

    return svd.matrixU() * Eigen::Map<Eigen::VectorXd>(&*diag_v.begin(), diag_v.size()).asDiagonal()
           * svd.matrixV().transpose();
}

bool PathPlanner::performInformedRRTStar(const std::string &costTrackingFile)
{
    _tree.clear();
    _tree.insert(std::make_pair(_pathRoot->_state, _pathRoot));

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0., 1.0);

    double cBest = std::numeric_limits<double>::infinity();
    Eigen::MatrixXd C = computeRotationWorldFrame(_startArm.getJointState(),
                                                  _goalArm.getJointState());
    NodePtr bestGoalNode = nullptr;
    int iter = 0;
    std::vector<NodePtr> goalNodes;

    // Time tracking variables
    auto startTime = std::chrono::high_resolution_clock::now();
    std::ofstream costTrackingStream;
    if (!costTrackingFile.empty()) {
        costTrackingStream.open(costTrackingFile);
        if (costTrackingStream.is_open()) {
            costTrackingStream << "Iteration,Time(ms),BestCost\n";
        }
    }

    bool success = false;

    for (size_t i = 0; i < _params.maxIterations; ++i) {
        State randomPoint;
        if (cBest < std::numeric_limits<double>::infinity()) {
            randomPoint = sampleWithinEllipsoid(cBest, C);
        } else {
            bg::set<0>(randomPoint, dis(gen));
            bg::set<1>(randomPoint, dis(gen));
            bg::set<2>(randomPoint, dis(gen));
            bg::set<3>(randomPoint, dis(gen));
            bg::set<4>(randomPoint, dis(gen));
            bg::set<5>(randomPoint, dis(gen));
            bg::set<6>(randomPoint, dis(gen));
        }

        if (dis(gen) < _params.goalBiasProbability && bestGoalNode == nullptr) {
            randomPoint = _goalArm.getJointState();
        }

        NodePtr newNode;
        if (getNewJointNode(randomPoint, newNode)) {
            size_t n = _tree.size();
            size_t k = std::max(static_cast<size_t>(std::ceil(kRRG * std::log(n))), 5ul);
            std::vector<NodePtr> neighborhood = findNearestNeighbors(newNode, k);
            NodePtr bestParent = newNode->_parent;
            double bestCost = newNode->_cost;

            for (auto &neighbor : neighborhood) {
                double cost = neighbor->_cost + metric(neighbor->_state, newNode->_state);
                if (cost < bestCost && motionIsValid(neighbor->_arm, newNode->_arm, true)) {
                    bestParent = neighbor;
                    bestCost = cost;
                }
            }

            if (bestParent != newNode->_parent) {
                newNode->_parent = bestParent;
                newNode->_cost = bestCost;
            }

            newNode->_costToGoal = bg::distance(_goalArm.getJointState(),
                                                newNode->_arm.getJointState());

            rewirePath(newNode, neighborhood);

            _tree.insert(std::make_pair(newNode->_state, newNode));

            newNode->_costToGoal = bg::distance(_goalArm.getJointState(),
                                                newNode->_arm.getJointState());

            if (closeToGoal(newNode->_arm)) {
                goalNodes.push_back(newNode);
                success = true;
            }

            for (const auto &node : goalNodes) {
                if (bestGoalNode == nullptr) {
                    bestGoalNode = newNode;
                    cBest = newNode->estimateCost();
                }

                if (node->estimateCost() < bestGoalNode->estimateCost()) {
                    RobotArm tmp = node->_arm;
                    bestGoalNode = node;
                    cBest = bestGoalNode->estimateCost();
                }
            }
        }

        if (costTrackingStream.is_open()) {
            auto currentTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime
                                                                                  - startTime);
            costTrackingStream << iter << "," << duration.count() << "," << cBest << "\n";
        }
    }

    if (costTrackingStream.is_open()) {
        costTrackingStream.close();
    }

    if (success) {
        NodePtr node = bestGoalNode;
        while (node) {
            _path.push_back(std::make_tuple(node->_state, node->_arm));
            node = node->_parent;
        }
        std::reverse(_path.begin(), _path.end());
        _path.push_back(std::make_pair(_goalArm.getJointState(), _goalArm));
    }

    return success;
}

PathPlanner::CheckpointPlanResult PathPlanner::planCheckpoints(
    const std::vector<Eigen::Affine3d> &originalScanPoses,
    const Eigen::VectorXd &currentJoints)
{
    CheckpointPlanResult result;
    if (originalScanPoses.empty())
        return result;

    std::vector<size_t> repositionIndices;
    
    // First pose - always use selectGoalPose
    auto [firstArm, firstValid] = selectGoalPose(originalScanPoses[0]);
    result.checkpoints.push_back({firstArm, firstValid});
    if (firstValid) repositionIndices.push_back(0);

    // Setup for remaining poses
    std::array<double, 7> currentJointAngles;
    for (int i = 0; i < 7; i++) {
        currentJointAngles[i] = (i < currentJoints.size()) ? currentJoints[i] : firstArm.getJointAngles()[i];
    }

    const double MAX_JOINT_DISTANCE = M_PI_2;
    const double MAX_SINGLE_JOINT = M_PI_4;
    std::default_random_engine gen(static_cast<unsigned>(time(0)));
    std::uniform_real_distribution<double> dis(-1.0, 1.0);

    // Process remaining poses
    for (size_t poseIdx = 1; poseIdx < originalScanPoses.size(); ++poseIdx) {
        const auto &pose = originalScanPoses[poseIdx];
        Eigen::Matrix<double, 4, 4> targetMatrix = pose.matrix() * _startArm.getEndeffectorTransform().inverse().matrix();

        std::array<double, 7> bestSolution;
        bool foundSolution = false;
        double bestCost = std::numeric_limits<double>::infinity();

        // Try IK to find a solution within joint limits
        for (int attempt = 0; attempt < 100; ++attempt) {
            double q7Value = currentJointAngles[6] + dis(gen) * 0.2;
            
            try {
                std::array<double, 7> sol = franka_IK_EE_CC(targetMatrix, q7Value, currentJointAngles);
                
                if (std::any_of(sol.begin(), sol.end(), [](double v) { return std::isnan(v); })) continue;

                // Check joint configuration changes
                double totalDist = 0, maxDiff = 0;
                bool validConfig = true;
                for (int i = 0; i < 7; i++) {
                    double diff = std::abs(sol[i] - currentJointAngles[i]);
                    totalDist += diff * diff;
                    maxDiff = std::max(maxDiff, diff);
                    if (diff > MAX_SINGLE_JOINT) validConfig = false;
                }
                totalDist = std::sqrt(totalDist);
                
                if (!validConfig || totalDist > MAX_JOINT_DISTANCE) continue;

                // Check collision
                Eigen::Matrix<double, 7, 1> solEigen;
                for (int i = 0; i < 7; i++) solEigen[i] = sol[i];
                
                RobotArm tempArm = _startArm;
                tempArm.setJointAngles(solEigen);
                if (armHasCollision(tempArm)) continue;

                if (totalDist < bestCost) {
                    bestCost = totalDist;
                    bestSolution = sol;
                    foundSolution = true;
                    if (totalDist < 0.5) break; // Good enough
                }
            } catch (...) {
                continue;
            }
        }

        // Use repositioning if IK failed or no valid solution found
        if (!foundSolution) {
            auto [repoArm, repoValid] = selectGoalPose(pose);
            if (repoValid && !armHasCollision(repoArm)) {
                result.checkpoints.push_back({repoArm, true});
                repositionIndices.push_back(poseIdx);
                auto repoJoints = repoArm.getJointAngles();
                for (int i = 0; i < 7; i++) currentJointAngles[i] = repoJoints[i];
            } else {
                // Invalid checkpoint
                RobotArm invalidArm = _startArm;
                Eigen::Matrix<double, 7, 1> currEigen;
                for (int i = 0; i < 7; i++) currEigen[i] = currentJointAngles[i];
                invalidArm.setJointAngles(currEigen);
                result.checkpoints.push_back({invalidArm, false});
            }
        } else {
            // Valid IK solution
            RobotArm nextArm = _startArm;
            Eigen::Matrix<double, 7, 1> bestEigen;
            for (int i = 0; i < 7; i++) bestEigen[i] = bestSolution[i];
            nextArm.setJointAngles(bestEigen);
            result.checkpoints.push_back({nextArm, true});
            for (int i = 0; i < 7; i++) currentJointAngles[i] = bestSolution[i];
        }
    }

    // Build segments - find invalid checkpoints and repositioning points as boundaries
    std::vector<size_t> boundaries;
    for (size_t i = 0; i < result.checkpoints.size(); ++i) {
        if (!result.checkpoints[i].second) boundaries.push_back(i); // Invalid checkpoints
    }
    for (size_t idx : repositionIndices) {
        if (idx > 0) boundaries.push_back(idx); // Repositioning points (except first)
    }
    
    std::sort(boundaries.begin(), boundaries.end());
    boundaries.erase(std::unique(boundaries.begin(), boundaries.end()), boundaries.end());

    // Create segments between boundaries
    if (boundaries.empty()) {
        // Single segment
        size_t lastValid = result.checkpoints.size() - 1;
        while (lastValid > 0 && !result.checkpoints[lastValid].second) lastValid--;
        if (result.checkpoints[0].second && result.checkpoints[lastValid].second) {
            result.validSegments.push_back({0, lastValid});
        }
    } else {
        size_t start = 0;
        for (size_t boundary : boundaries) {
            if (boundary > start) {
                size_t end = boundary - 1;
                while (end >= start && !result.checkpoints[end].second) {
                    if (end == 0) break;
                    end--;
                }
                if (end >= start && result.checkpoints[start].second && result.checkpoints[end].second) {
                    result.validSegments.push_back({start, end});
                }
            }
            start = result.checkpoints[boundary].second ? boundary : boundary + 1;
        }
        
        // Final segment
        if (start < result.checkpoints.size()) {
            size_t end = result.checkpoints.size() - 1;
            while (end >= start && !result.checkpoints[end].second) {
                if (end == 0) break;
                end--;
            }
            if (end >= start && result.checkpoints[start].second && result.checkpoints[end].second) {
                result.validSegments.push_back({start, end});
            }
        }
    }

    for (size_t i = 0; i < result.checkpoints.size(); ++i) {
        if (result.checkpoints[i].second) {
            result.firstValidIndex = i;
            break;
        }
    }

    return result;
}

RobotArm PathPlanner::getArm()
{
    return _startArm;
}

PathPlanner::ClearanceMetrics PathPlanner::computeArmClearance(RobotArm &arm, bool inflate)
{
    ClearanceMetrics metrics;
    
    // Initialize with infinite clearance
    metrics.min_clearance = std::numeric_limits<double>::infinity();
    metrics.avg_clearance = std::numeric_limits<double>::infinity();
    metrics.weighted_clearance = std::numeric_limits<double>::infinity();
    metrics.num_links_checked = 0;
    
    if (!_obstacleTree) {
        // No obstacles, so return infinite clearance
        return metrics;
    }

    // Get bounding boxes for all links (same method as selectGoalPose)
    auto bBoxes = arm.getLinkBoundingBoxes();
    
    // Initialize metrics
    double total_clearance = 0.0;
    int valid_links = 0;
    metrics.min_clearance = std::numeric_limits<double>::infinity();

    // Check clearance for each link (starting from index 2 like selectGoalPose)
    for (int linkIdx = 2; linkIdx < static_cast<int>(bBoxes.size()); ++linkIdx) {
        auto bboxCenter = std::get<0>(bBoxes[linkIdx]);
        
        // Compute distance to nearest obstacle using the same method as selectGoalPose
        double dist = std::get<0>(_obstacleTree->getDistanceAndClosestPoint(bboxCenter));
        
        // Apply inflation if requested (using similar inflation factor as armHasCollision)
        if (inflate) {
            // Estimate inflation based on bounding box size
            auto halfDims = std::get<1>(bBoxes[linkIdx]);
            double avgDim = (halfDims[0] + halfDims[1] + halfDims[2]) / 3.0;
            dist -= avgDim * 0.1; // 10% inflation similar to armHasCollision
        }

        // Store clearance for this link
        metrics.link_clearances.push_back(dist);
        
        // Update metrics
        metrics.min_clearance = std::min(metrics.min_clearance, dist);
        total_clearance += dist;
        valid_links++;
    }

    // Compute averages
    if (valid_links > 0) {
        metrics.avg_clearance = total_clearance / valid_links;
        metrics.weighted_clearance = metrics.avg_clearance; // Simplified weighting
    } else {
        metrics.avg_clearance = std::numeric_limits<double>::infinity();
        metrics.weighted_clearance = std::numeric_limits<double>::infinity();
        metrics.min_clearance = std::numeric_limits<double>::infinity();
    }
    
    metrics.num_links_checked = valid_links;
    
    return metrics;
}

std::pair<RobotArm, bool> PathPlanner::selectGoalPoseSimulatedAnnealing(const Eigen::Affine3d &pose,
                                                                         double T_max, 
                                                                         double T_min, 
                                                                         double alpha, 
                                                                         int max_iterations, 
                                                                         int max_no_improvement)
{
    // Note: Function name kept for API compatibility, but now implements parallelized exhaustive grid search
    
    // Get starting configuration
    Eigen::Matrix<double, 7, 1> jointAngles = _startArm.getJointAngles();
    std::array<double, 7> jointAnglesArray;
    Eigen::Map<Eigen::Matrix<double, 7, 1>>(jointAnglesArray.data()) = jointAngles;
    
    // Transform target pose (same as original)
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d axes = Rx * Ry * Rz;
    
    Eigen::Affine3d conv;
    conv.linear() = axes;
    
    Eigen::Affine3d total = pose * _startArm.getEndeffectorTransform().inverse();
    
    // Grid search parameters based on Franka Emika Panda 14-bit encoder precision
    const double q7_min = -2.8973;  // Joint 7 limits for Franka Panda
    const double q7_max = 2.8973;
    const int encoder_steps = 16384;  // 14-bit encoder precision (2^14)
    
    // Generate q7 grid values based on encoder precision
    std::vector<double> q7_values;
    q7_values.reserve(encoder_steps);
    double q7_range = q7_max - q7_min;
    double step_size = q7_range / (encoder_steps - 1);
    
    for (int i = 0; i < encoder_steps; ++i) {
        double q7 = q7_min + i * step_size;
        q7_values.push_back(q7);
    }
    
    // Parallel processing setup - always use parallel processing
    size_t effective_threads = std::thread::hardware_concurrency();
    
    // Best solution tracking
    Eigen::Matrix<double, 7, 1> best_solution;
    double best_cost = std::numeric_limits<double>::infinity();
    bool has_valid_solution = false;
    std::mutex result_mutex;
    
    // PARALLEL PROCESSING with Boost.Asio (always enabled)
    boost::asio::thread_pool pool(effective_threads);
    
    // Calculate optimal chunk size for 14-bit encoder precision
    size_t target_chunks = effective_threads * 4;  // More chunks for better load balancing with high precision
    size_t chunk_size = std::max(static_cast<size_t>(256), q7_values.size() / target_chunks);  // Minimum 256 per chunk
    
    // Vector to hold futures
    std::vector<std::future<void>> futures;
    futures.reserve((q7_values.size() + chunk_size - 1) / chunk_size);
        
        // Launch parallel tasks
        for (size_t start = 0; start < q7_values.size(); start += chunk_size) {
            size_t end = std::min(start + chunk_size, q7_values.size());
            
            // Create packaged task for this chunk
            auto task = std::make_shared<std::packaged_task<void()>>(
                [this, start, end, &q7_values, total, jointAnglesArray, &best_solution, &best_cost, &has_valid_solution, &result_mutex, pose]() {
                    
                    // Local best for this chunk
                    Eigen::Matrix<double, 7, 1> local_best_solution;
                    double local_best_cost = std::numeric_limits<double>::infinity();
                    bool local_has_solution = false;
                    
                    // Process chunk of q7 values
                    for (size_t i = start; i < end; ++i) {
                        double q7 = q7_values[i];
                        
                        // Solve IK for this q7 value
                        std::array<std::array<double, 7>, 4> ikSolutions = franka_IK_EE(total, q7, jointAnglesArray);
                        
                        // Evaluate all IK solutions
                        for (const auto& sol : ikSolutions) {
                            if (std::any_of(sol.begin(), sol.end(), [](double val) { return std::isnan(val); })) {
                                continue;
                            }
                            
                            Eigen::Map<const Eigen::Matrix<double, 7, 1>> angles(sol.data());
                            RobotArm temp = _startArm;
                            temp.setJointAngles(angles);
                            
                            // Check for collisions
                            if (armHasCollision(temp)) {
                                continue;
                            }
                            
                            // Compute comprehensive cost (same as ComparisonIK implementation)
                            double cost = 0.0;
                            
                            // 1. POSE ERROR (HIGHEST PRIORITY)
                            Eigen::Affine3d current_pose = temp.getEndeffectorPose();
                            Eigen::Vector3d position_error = pose.translation() - current_pose.translation();
                            double pos_error = position_error.norm();
                            
                            Eigen::Matrix3d rotation_error = pose.linear() * current_pose.linear().transpose();
                            Eigen::AngleAxisd angle_axis(rotation_error);
                            double orientation_error = std::abs(angle_axis.angle());
                            
                            double pose_error = pos_error + 0.1 * orientation_error;
                            cost += 100.0 * pose_error; // Heavy weight to prioritize pose accuracy
                            
                            // 2. Clearance-based cost (if obstacle tree is available)
                            if (_obstacleTree) {
                                auto clearance_metrics = computeArmClearance(temp);
                                const double threshold = 0.3;
                                
                                // Penalty based on minimum clearance
                                if (clearance_metrics.min_clearance < threshold) {
                                    double dist = clearance_metrics.min_clearance;
                                    double clearance_penalty = 1.0 / (1.0 + std::exp(100 * (dist - threshold)));
                                    cost += clearance_penalty;
                                }
                            }
                            
                            // 3. Add manipulability (lowest priority)
                            double manipulability = temp.computeManipulability();
                            cost += 0.25 * manipulability;

                            // Update local best
                            if (!local_has_solution || cost < local_best_cost) {
                                local_best_solution = angles;
                                local_best_cost = cost;
                                local_has_solution = true;
                            }
                        }
                    }
                    
                    // Update global best (thread-safe)
                    if (local_has_solution) {
                        std::lock_guard<std::mutex> lock(result_mutex);
                        if (!has_valid_solution || local_best_cost < best_cost) {
                            best_solution = local_best_solution;
                            best_cost = local_best_cost;
                            has_valid_solution = true;
                        }
                    }
                }
            );
            
            futures.push_back(task->get_future());
            boost::asio::post(pool, [task]() { (*task)(); });
        }
        
        // Wait for all tasks to complete
        pool.join();
        
        // Wait for all futures
        for (auto& fut : futures) {
            fut.get();
        }
    
    // Return result with pose accuracy sanity check
    RobotArm finalArm = _startArm;
    bool success = has_valid_solution && best_cost != std::numeric_limits<double>::infinity();
    
    if (success) {
        finalArm.setJointAngles(best_solution);
        
        // Sanity check: Verify pose accuracy before declaring success
        Eigen::Affine3d achieved_pose = finalArm.getEndeffectorPose();
        Eigen::Vector3d position_error = pose.translation() - achieved_pose.translation();
        double pos_error = position_error.norm();
        
        Eigen::Matrix3d rotation_error = pose.linear() * achieved_pose.linear().transpose();
        Eigen::AngleAxisd angle_axis(rotation_error);
        double orientation_error = std::abs(angle_axis.angle());
        
        // Define acceptable tolerances (aligned with Franka Panda ±0.1mm repeatability)
        const double max_position_error = 0.001;     // 1mm tolerance
        const double max_orientation_error = 0.017;  // ~1 degree tolerance (0.017 radians)
        
        // Fail if pose accuracy is insufficient
        if (pos_error > max_position_error || orientation_error > max_orientation_error) {
            success = false;
        }
    }

    return std::make_pair(finalArm, success);
}

std::pair<double, bool> PathPlanner::evaluateSelectGoalPoseCost(const Eigen::Affine3d &pose, double q7)
{
    // Get starting configuration
    Eigen::Matrix<double, 7, 1> jointAngles = _startArm.getJointAngles();
    std::array<double, 7> jointAnglesArray;
    Eigen::Map<Eigen::Matrix<double, 7, 1>>(jointAnglesArray.data()) = jointAngles;
    
    // Transform target pose (same as selectGoalPose methods)
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d axes = Rx * Ry * Rz;
    
    Eigen::Affine3d conv;
    conv.linear() = axes;
    
    Eigen::Affine3d total = pose * _startArm.getEndeffectorTransform().inverse();
    
    // Solve IK for the specified q7 value
    std::array<std::array<double, 7>, 4> ikSolutions = franka_IK_EE(total, q7, jointAnglesArray);
    
    double bestCost = std::numeric_limits<double>::infinity();
    bool foundValidSolution = false;
    
    // Evaluate all IK solutions for this q7
    for (const auto& sol : ikSolutions) {
        if (std::any_of(sol.begin(), sol.end(), [](double val) { return std::isnan(val); })) {
            continue;
        }
        
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> angles(sol.data());
        RobotArm temp = _startArm;
        temp.setJointAngles(angles);
        
        // Check for collisions
        if (armHasCollision(temp)) {
            continue;
        }
        
        // Compute cost using EXACT same function as selectGoalPoseSimulatedAnnealing
        double cost = 0.0;
        const double collision_threshold = 0.1;   // Critical collision threshold (10cm) 
        const double safety_threshold = 0.20;     // Safety comfort zone (20cm)
        const double optimal_threshold = 0.30;    // Optimal clearance target (30cm)
        
        int numLinks = static_cast<int>(temp.getLinkBoundingBoxes().size());
        double min_clearance = std::numeric_limits<double>::infinity();
        double total_clearance = 0.0;
        int valid_links = 0;
        
        // Compute clearance metrics for all links (excluding base and end-effector)
        for (int linkIdx = 2; linkIdx < numLinks - 2; ++linkIdx) {
            auto bboxCenter = std::get<0>(temp.getLinkBoundingBoxes()[linkIdx]);
            double dist = 0.0;
            if (_obstacleTree) {
                dist = std::get<0>(_obstacleTree->getDistanceAndClosestPoint(bboxCenter));
            }
            
            min_clearance = std::min(min_clearance, dist);
            total_clearance += dist;
            valid_links++;
            
            // Multi-tier clearance penalty system
            double link_penalty = 0.0;
            
            if (dist < collision_threshold) {
                // Critical penalty for very close to collision
                link_penalty = 2.0 * (1.0 - dist / collision_threshold);
            } else if (dist < safety_threshold) {
                // High penalty for unsafe proximity
                link_penalty = 1.0 * (1.0 - (dist - collision_threshold) / (safety_threshold - collision_threshold));
            } else if (dist < optimal_threshold) {
                // Moderate penalty for suboptimal clearance
                link_penalty = 0.3 * (1.0 - (dist - safety_threshold) / (optimal_threshold - safety_threshold));
            }
            // No penalty for distances >= optimal_threshold
            
            cost += link_penalty / valid_links;
        }
        
        // Additional clearance-based cost components
        double avg_clearance = (valid_links > 0) ? total_clearance / valid_links : 0.0;
        
        // Penalty for low minimum clearance (prioritize worst-case safety)
        if (min_clearance < optimal_threshold) {
            double min_clearance_penalty = 0.4 * std::exp(-min_clearance / 0.1);
            cost += min_clearance_penalty;
        }
        
        // Reward configurations with good average clearance
        if (avg_clearance > safety_threshold) {
            double clearance_bonus = -0.2 * std::min(1.0, (avg_clearance - safety_threshold) / (optimal_threshold - safety_threshold));
            cost += clearance_bonus;
        }
        
        // Cap the clearance-based cost component
        cost = std::max(0.0, std::min(cost, 2.0));
        
        // Add manipulability bonus (reduced weight to prioritize safety)
        cost += 0.15 * temp.computeManipulability();
        
        // Track the best cost for this q7 value
        if (cost < bestCost) {
            bestCost = cost;
            foundValidSolution = true;
        }
    }
    
    if (foundValidSolution) {
        return std::make_pair(bestCost, true);
    } else {
        return std::make_pair(std::numeric_limits<double>::infinity(), false);
    }
}

double PathPlanner::getMinimumClearance(RobotArm &arm, bool inflate)
{
    auto metrics = computeArmClearance(arm, inflate);
    return metrics.min_clearance;
}
