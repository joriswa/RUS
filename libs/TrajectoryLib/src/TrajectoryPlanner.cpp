#include "TrajectoryLib/TrajectoryPlanner.h"
#include <QDebug>
#include <iostream>


TrajectoryPlanner::TrajectoryPlanner(const RobotArm &arm, QThread *worker) {

    this->moveToThread(worker);

    _min_x = _min_y = _min_z = -2.5;
    _max_x = _max_y = _max_z = 2.5;
    _limits << _min_x, _max_x, _min_y, _max_y, _min_z, _max_z;
}

NodePtr TrajectoryPlanner::nearestPathNode(const Point &point) {
    std::vector<std::pair<Point, NodePtr>> results;
    _pathTree.query(bgi::nearest(point, 1), std::back_inserter(results));

    return results.front().second;
}

bool TrajectoryPlanner::getNewNode(Point &randomPoint, NodePtr &newNode) {

    NodePtr nearestNode = nearestPathNode(randomPoint);
    TaskState nearestState = nearestNode->_state;
    bg::subtract_point(randomPoint, nearestState);
    double length = std::sqrt(bg::dot_product(randomPoint, randomPoint));

    if (length > 0) {
        bg::divide_value(randomPoint, length);
        bg::multiply_value(randomPoint, stepSize);
        bg::add_point(randomPoint, nearestNode->_state);
    }

    bg::set<0>(randomPoint, std::clamp(bg::get<0>(randomPoint), 0.0, 1.0));
    bg::set<1>(randomPoint, std::clamp(bg::get<1>(randomPoint), 0.0, 1.0));
    bg::set<2>(randomPoint, std::clamp(bg::get<2>(randomPoint), 0.0, 1.0));
    bg::set<3>(randomPoint, std::clamp(bg::get<3>(randomPoint), 0.0, 1.0));
    bg::set<4>(randomPoint, std::clamp(bg::get<4>(randomPoint), 0.0, 1.0));
    bg::set<5>(randomPoint, std::clamp(bg::get<5>(randomPoint), 0.0, 1.0));

    RobotArm closestArmState = nearestNode->_arm;

    if (!closestArmState.moveToTaskState(randomPoint, _limits)) {
        return false;
    }

    if (!motionIsValid(nearestNode->_arm, closestArmState)) {
        return false;
    }

    newNode = nearestNode->createChild(randomPoint, closestArmState);
    newNode->_cost = nearestNode->_cost + metric(randomPoint, nearestNode->_state);

    return true;
}

bool TrajectoryPlanner::getNewNode(Point &randomPoint, NodePtr &newNode, const bgi::rtree<std::pair<Point, NodePtr>, bgi::rstar<8, 4>>& tree) {

    NodePtr nearestNode = nearestNeighbor(tree, randomPoint);
    TaskState nearestState = nearestNode->_state;
    bg::subtract_point(randomPoint, nearestState);
    double length = std::sqrt(bg::dot_product(randomPoint, randomPoint));

    if (length > 0) {
        bg::divide_value(randomPoint, length);
        bg::multiply_value(randomPoint, stepSize);
        bg::add_point(randomPoint, nearestNode->_state);
    }

    bg::set<0>(randomPoint, std::clamp(bg::get<0>(randomPoint), 0.0, 1.0));
    bg::set<1>(randomPoint, std::clamp(bg::get<1>(randomPoint), 0.0, 1.0));
    bg::set<2>(randomPoint, std::clamp(bg::get<2>(randomPoint), 0.0, 1.0));
    bg::set<3>(randomPoint, std::clamp(bg::get<3>(randomPoint), 0.0, 1.0));
    bg::set<4>(randomPoint, std::clamp(bg::get<4>(randomPoint), 0.0, 1.0));
    bg::set<5>(randomPoint, std::clamp(bg::get<5>(randomPoint), 0.0, 1.0));

    RobotArm closestArmState = nearestNode->_arm;

    if (!closestArmState.moveToTaskState(randomPoint, _limits)) {
        return false;
    }

    if (!motionIsValid(nearestNode->_arm, closestArmState)) {
        return false;
    }

    newNode = nearestNode->createChild(randomPoint, closestArmState);
    newNode->_cost = nearestNode->_cost + metric(randomPoint, nearestNode->_state);

    return true;
}


bool TrajectoryPlanner::armHasCollision(RobotArm &arm) {

    auto bBoxes = arm.getLinkBoundingBoxes();

    for (const auto& bBox : bBoxes) {
        auto [center, halfDims, axes] = bBox;

        if (_obstacleTree->isBoxIntersecting(center, halfDims, axes))
            return true;
    }

    if(arm.getEndeffectorPose().translation().head<2>().norm() < 0.35) {
        return true;
    }

    return false;
}

bool TrajectoryPlanner::motionIsValid(RobotArm &startArm, RobotArm &endArm) {

    if (armHasCollision(startArm) || armHasCollision(endArm)) {
        return false;
    }

    RobotArm tmpArm(startArm);
    Point startState = startArm.getTaskState(_limits);
    Point endState = endArm.getTaskState(_limits);
    double dist = bg::distance(startState, endState);

    if (dist > stepSize * 0.25) {
        int numSteps = static_cast<int>(std::ceil(dist / stepSize * 8.));
        bg::subtract_point(endState, startState);
        bg::divide_value(endState, numSteps);

        for (int i = 1; i < numSteps; ++i) {
            bg::add_point(startState, endState);
            if (!tmpArm.moveToTaskState(startState, _limits)) {
                return false;
            }
            if (armHasCollision(tmpArm)) {
                return false;
            }

        }
    }

    return true;
}

void TrajectoryPlanner::performRRT() {

    _path.clear();
    _pathTree.clear();
    _pathTree.insert(std::make_pair(_pathRoot->_state, _pathRoot));

    size_t maxIterations = 1000000;
    double goalBiasProbability = 0.25;

    std::vector<Vec3> path;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    int iter = 0;
    double smallest = 9999.;
    qDebug() << "performing rrt";
    for (size_t i = 0; i < maxIterations; ++i) {

        if (i >= 1000) {
            i = 0;
            _pathTree.clear();
            _pathTree.insert(std::make_pair(_pathRoot->_state, _pathRoot));
        }

        Point randomPoint;
        boost::geometry::set<0>(randomPoint, dis(gen));
        boost::geometry::set<1>(randomPoint, dis(gen));
        boost::geometry::set<2>(randomPoint, dis(gen));
        boost::geometry::set<3>(randomPoint, dis(gen));
        boost::geometry::set<4>(randomPoint, dis(gen));
        boost::geometry::set<5>(randomPoint, dis(gen));

        if (abs(dis(gen)) < goalBiasProbability) {
            randomPoint = _goalPoint;
        }

        NodePtr newNode;
        if (getNewNode(randomPoint, newNode)) {
            _pathTree.insert(std::make_pair(newNode->_state, newNode));
            double dist = metric(randomPoint, _goalPoint);
            if (motionIsValid(newNode->_arm, _goalArm)) {
                NodePtr goalNode = std::make_shared<PathNode>(_goalPoint, _goalArm);
                goalNode->_parent = newNode;
                NodePtr node = goalNode;
                while (node) {
                    _path.push_back(std::make_pair(node->_state, node->_arm));
                    node = node->_parent;
                }

                break;
            }
        }

        ++iter;
    }
    qDebug() << " path size: " << _path.size();
    std::reverse(_path.rbegin(), _path.rend());
}

void TrajectoryPlanner::performRRTStar() {

    _path.clear();
    _pathTree.clear();
    _pathTree.insert(std::make_pair(_pathRoot->_state, _pathRoot));

    size_t maxIterations = 7500;
    double goalBiasProbability = 0.25;

    std::vector<Vec3> path;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    double smallest = 9999.;
    NodePtr bestGoalNode = nullptr;
    double bestCostToGoal = std::numeric_limits<double>::max();

    for (size_t i = 0; i < maxIterations; ++i) {

        if (i >= 1000 && bestGoalNode == nullptr) {
            i = 0;
            _pathTree.clear();
            _pathTree.insert(std::make_pair(_pathRoot->_state, _pathRoot));
        }

        Point randomPoint;
        boost::geometry::set<0>(randomPoint, dis(gen));
        boost::geometry::set<1>(randomPoint, dis(gen));
        boost::geometry::set<2>(randomPoint, dis(gen));
        boost::geometry::set<3>(randomPoint, dis(gen));
        boost::geometry::set<4>(randomPoint, dis(gen));
        boost::geometry::set<5>(randomPoint, dis(gen));

        if (dis(gen) < goalBiasProbability) {
            randomPoint = _goalPoint;
        }

        NodePtr newNode;
        if (getNewNode(randomPoint, newNode)) {

            size_t n = _pathTree.size();
            size_t k = std::max(static_cast<size_t>(std::ceil(kRRG * std::log(n))), 5ul);
            std::vector<NodePtr> neighborhood = findNearestNeighbors(newNode, 3.107 * _pathTree.size());

            NodePtr bestParent = newNode->_parent;
            double bestCost = newNode->_cost;

            for (auto &neighbor : neighborhood) {
                double cost =
                    newNode->_cost + metric(newNode->_state, neighbor->_state);
                if (cost < bestCost && motionIsValid(newNode->_arm, neighbor->_arm)) {
                    bestParent = neighbor;
                    bestCost = cost;
                }
            }

            if (bestParent != newNode->_parent) {
                newNode->_parent = bestParent;
                newNode->_cost = bestCost;
            }

            rewirePath(newNode, neighborhood);

            _pathTree.insert(std::make_pair(newNode->_state, newNode));

            double dist = metric(randomPoint, _goalPoint);
            newNode->_costToGoal = dist;

            double totalCost = newNode->_cost + dist;
            if (motionIsValid(newNode->_arm, _goalArm) && totalCost < bestCostToGoal) {
                bestCostToGoal = totalCost;
                bestGoalNode = newNode;
                qDebug() << bestCostToGoal;
                goalBiasProbability = 0.1;
            }
        }
    }

    _path.clear();
    _path.push_back(std::make_pair(_goalPoint, _goalArm));
    NodePtr node = bestGoalNode;
    while (node) {
        _path.push_back(std::make_pair(node->_state, node->_arm));
        node = node->_parent;
    }
    std::reverse(_path.begin(), _path.end());
}

void TrajectoryPlanner::setStartPose(RobotArm arm) {
    Point start = arm.getTaskState(_limits);
    _startPoint = start;
    _startArm = arm;
    _pathRoot = std::make_shared<PathNode>(start, arm);
    _pathTree.insert(std::make_pair(_pathRoot->_state, _pathRoot));
}

void TrajectoryPlanner::setGoalPose(Eigen::Vector3d t, Eigen::Matrix3d r) {
    _goalTranslation = t;
    _goalRotation = r;

    RobotArm tmp = _pathRoot->_arm;
    if(tmp.moveToCartesian(t, r)) {
        qDebug() << "Succesfully set goal point";
    } else {
        qDebug() << "Couldnt set goal point";
    }

    _goalArm = tmp;
    _goalPoint = tmp.getTaskState(_limits);
}


std::vector<std::tuple<Vec3, RobotArm>> TrajectoryPlanner::getPath() {
    std::vector<std::tuple<Vec3, RobotArm>> path;
    for (const auto &[p, arm] : _path) {
        Point denormed = denormalizeState(p);
        path.push_back(std::make_pair(
            Vec3(denormed.get<0>(), denormed.get<1>(), denormed.get<2>()), arm));
    }
    return path;
}

std::vector<std::vector<Vec3> > TrajectoryPlanner::getPaths()
{
    return _paths;
}

void TrajectoryPlanner::prunePath() {

    for (int i = 0; i < _path.size() && _path.size() > 4; ++i) {
        auto [state, arm] = _path[i];
        for (int j = i + 1; j < _path.size(); j++) {
            auto [tmpState, tmpArm] = _path[j];

            if (!motionIsValid(arm, tmpArm)) {
                break;
            }

            if (j - i > 1) {
                _path.erase(_path.begin() + i + 1, _path.begin() + j);
                i--;
                break;
            }
        }
    }
}

std::vector<NodePtr>
TrajectoryPlanner::findGeometricNeighbors(const NodePtr &node, double radius) {

    std::vector<NodePtr> nodes;
    std::vector<std::pair<Point, NodePtr>> results;
    _pathTree.query(bgi::satisfies([&](const std::pair<Point, NodePtr> &p) {
                        return metric(p.first, node->_state) <= radius;
                    }),
                    std::back_inserter(results));

    for (auto const &pair : results) {
        nodes.push_back(pair.second);
    }

    return nodes;
}

void
TrajectoryPlanner::rewirePath(NodePtr newNode,
                              std::vector<NodePtr> &neighboringNodes) {
    for (NodePtr &neighbor : neighboringNodes) {
        double newCost = newNode->_cost + metric(newNode->_state, neighbor->_state);
        if (newCost < neighbor->_cost &&
            motionIsValid(newNode->_arm, neighbor->_arm)) {
            neighbor->_parent = newNode;
            neighbor->_cost = newCost;
        }
    }
}

void TrajectoryPlanner::setAlgorithm(Algorithm algorithm) {
    _algorithm = algorithm;
}

void TrajectoryPlanner::setObstacleTree(const std::shared_ptr<BVHTree> &newObstacleTree)
{
    _obstacleTree = newObstacleTree;
}

void TrajectoryPlanner::runPathFinding() {
    using namespace std::chrono;

    auto start = high_resolution_clock::now();

    switch (_algorithm) {
        case RRT:
            performRRT();
            break;
        case RRTStar:
            performRRTStar();
            break;
        case InformedRRTStar:
            performInformedRRTStar();
            break;
        case RRTConnect:
            performCHOMP();
            break;
        }

    auto end = high_resolution_clock::now();

    duration<double> duration = end - start;

//    prunePath();
//    smoothBSpline(std::max(10., _path.size() * 0.05), 0.005);
    qDebug() << _path.size();
    emit finished();
}

void TrajectoryPlanner::smoothBSpline(unsigned int maxSteps, double minChange) {
    if (_path.size() < 3)
        return;

    auto interpolate = [](const Point &p1, const Point &p2, double t) {
        Point result = p1;
        Point tmp = p2;
        bg::multiply_value(result, (1. - t));
        bg::multiply_value(tmp, t);
        bg::add_point(result, tmp);
        return result;
    };

    for (unsigned int s = 0; s < maxSteps; ++s) {
        qDebug() << "step: " << s;
        // Subdivide the path
        std::vector<std::tuple<Point, RobotArm>> waypoints(1, _path[0]);
        for (unsigned int i = 1; i < _path.size(); ++i) {

            auto [startState, startArm] = waypoints.back();
            auto [endState, endArm] = _path[i];

            bg::add_point(startState, endState);
            bg::divide_value(startState, 2.0);

            Point denormed = denormalizeState(startState);
            startArm.moveToCartesian(
                Vec3(denormed.get<0>(), denormed.get<1>(), denormed.get<2>()),
                Vec3(denormed.get<3>(), denormed.get<4>(), denormed.get<5>()));

            waypoints.push_back(std::make_pair(startState, startArm));
            waypoints.push_back(_path[i]);
        }

        unsigned int i = 2, u = 0, n1 = waypoints.size() - 1;
        while (i < n1) {

            if (!armHasCollision(std::get<1>(waypoints[i - 1]))) {

                Point temp1 = interpolate(std::get<0>(waypoints[i - 1]), std::get<0>(waypoints[i]), 0.5);
                Point temp2 = interpolate(std::get<0>(waypoints[i]), std::get<0>(waypoints[i + 2]), 0.5);

                temp1 = interpolate(temp1, temp2, 0.5);

                RobotArm temp1Arm = std::get<1>(waypoints[i - 1]);
                Point denormed = denormalizeState(temp1);
                if (!temp1Arm.moveToCartesian(
                        Vec3(denormed.get<0>(), denormed.get<1>(), denormed.get<2>()),
                        Vec3(denormed.get<3>(), denormed.get<4>(), denormed.get<5>())))
                    return;

                if (motionIsValid(std::get<1>(waypoints[i - 1]), temp1Arm) && motionIsValid(temp1Arm, std::get<1>(waypoints[i]))) {
                    if (bg::distance(std::get<0>(waypoints[i]), temp1) > minChange) {
                        waypoints[i] = std::make_pair(temp1, temp1Arm);
                        ++u;
                    }
                }
            }
            i += 2;

        }
        if (u == 0) break;
        _path = waypoints;
    }
}

bool TrajectoryPlanner::shortcutPath(unsigned int maxSteps,
                                     unsigned int maxEmptySteps,
                                     double rangeRatio, double snapToVertex) {
    if (_path.size() < 3)
        return false;

    if (maxSteps == 0)
        maxSteps = _path.size();

    if (maxEmptySteps == 0)
        maxEmptySteps = _path.size();

    std::vector<std::tuple<Point, RobotArm>> waypoints = _path;
    std::vector<double> dists(waypoints.size(), 0.0);
    for (unsigned int i = 1; i < waypoints.size(); ++i) {
        Point curState = std::get<0>(waypoints[i]);
        Point prevState = std::get<0>(waypoints[i - 1]);
        dists[i] = dists[i - 1] + bg::distance(prevState, curState);
    }

    double threshold = dists.back() * snapToVertex;
    double rd = rangeRatio * dists.back();

    bool result = false;
    unsigned int nochange = 0;

    auto interpolate = [](const Point &p1, const Point &p2, double t) {
        Point result = p1;
        Point tmp = p2;
        bg::multiply_value(result, (1. - t));
        bg::multiply_value(tmp, t);
        bg::add_point(result, tmp);
        return result;
    };

    std::random_device rand;
    std::mt19937 gen(rand());
    std::uniform_real_distribution<> dis(0, 1);
    for (unsigned int i = 0; i < maxSteps && nochange < maxEmptySteps;
         ++i, ++nochange) {

        double t0 = 0.0;
        double distTo0 = dis(gen) * dists.back();
        auto pit = std::lower_bound(dists.begin(), dists.end(), distTo0);
        int pos0 = pit == dists.end() ? dists.size() - 1 : pit - dists.begin();

        int index0 = -1;
        if (pos0 == 0 || dists[pos0] - distTo0 < threshold)
            index0 = pos0;
        else {
            while (pos0 > 0 && distTo0 < dists[pos0])
                --pos0;
            if (distTo0 - dists[pos0] < threshold)
                index0 = pos0;
        }

        double minDist = std::max(0.0, distTo0 - rd);
        double maxDist = std::min(distTo0 + rd, dists.back());
        double distTo1 = dis(gen) * (maxDist - minDist) + minDist;
        pit = std::lower_bound(dists.begin(), dists.end(), distTo1);
        int pos1 = pit == dists.end() ? dists.size() - 1 : pit - dists.begin();

        int index1 = -1;
        if (pos1 == 0 || dists[pos1] - distTo1 < threshold)
            index1 = pos1;
        else {
            while (pos1 > 0 && distTo1 < dists[pos1])
                --pos1;
            if (distTo1 - dists[pos1] < threshold)
                index1 = pos1;
        }

        if (pos0 == pos1 || index0 == pos1 || index1 == pos0 ||
            pos0 + 1 == index1 || pos1 + 1 == index0 || index1 < index0 ||
            index0 < 0 || index1 < 0 ||
            (index0 >= 0 && index1 >= 0 && abs(index0 - index1) < 2))
            continue;

        Point s0, s1;
        if (index0 >= 0) {
            s0 = std::get<0>(waypoints[index0]);
        } else {
            double t0 = (distTo0 - dists[pos0]) / (dists[pos0 + 1] - dists[pos0]);
            s0 = interpolate(std::get<0>(waypoints[pos0]),
                             std::get<0>(waypoints[pos0 + 1]), t0);
        }

        if (index1 >= 0) {
            s1 = std::get<0>(waypoints[index1]);
        } else {
            double t1 = (distTo1 - dists[pos1]) / (dists[pos1 + 1] - dists[pos1]);
            s1 = interpolate(std::get<0>(waypoints[pos1]),
                             std::get<0>(waypoints[pos1 + 1]), t1);
        }

        RobotArm arm0(std::get<1>(waypoints[index0]));
        RobotArm arm1(std::get<1>(waypoints[index1]));
        Point denormeds0 = denormalizeState(s0);
        Point denormeds1 = denormalizeState(s1);
        arm0.moveToCartesian(
            Vec3(denormeds0.get<0>(), denormeds0.get<1>(), denormeds0.get<2>()),
            Vec3(denormeds0.get<3>(), denormeds0.get<4>(), denormeds0.get<5>()));
        arm1.moveToCartesian(
            Vec3(denormeds1.get<0>(), denormeds1.get<1>(), denormeds1.get<2>()),
            Vec3(denormeds1.get<3>(), denormeds1.get<4>(), denormeds1.get<5>()));

        if (motionIsValid(arm0, arm1)) {
            if (index0 < 0 && index1 < 0) {
                if (pos0 + 1 == pos1) {
                    std::get<0>(waypoints[pos1]) = s0;
                    waypoints.insert(waypoints.begin() + pos1 + 1,
                                     std::make_pair(s1, arm1));
                } else {
                    waypoints[pos0 + 1] = std::make_pair(s0, arm0);
                    waypoints[pos1] = std::make_pair(s1, arm1);
                    waypoints.erase(waypoints.begin() + pos0 + 2,
                                    waypoints.begin() + pos1);
                }
            } else if (index0 >= 0 && index1 >= 0) {
                waypoints.erase(waypoints.begin() + index0 + 1,
                                waypoints.begin() + index1);
            } else if (index0 < 0 && index1 >= 0) {
                waypoints[pos0 + 1] = std::make_pair(s0, arm0);
                waypoints.erase(waypoints.begin() + pos0 + 2,
                                waypoints.begin() + index1);
            } else if (index0 >= 0 && index1 < 0) {
                waypoints[pos1] = std::make_pair(s1, arm1);
                waypoints.erase(waypoints.begin() + index0 + 1,
                                waypoints.begin() + pos1);
            }

            dists.resize(waypoints.size(), 0.0);
            for (unsigned int j = 1; j < waypoints.size(); ++j) {
                dists[j] = dists[j - 1] + bg::distance(std::get<0>(waypoints[j - 1]),
                                                       std::get<0>(waypoints[j]));
            }

            threshold = dists.back() * snapToVertex;
            rd = rangeRatio * dists.back();
            result = true;
            nochange = 0;
        }
    }

    _path = waypoints;
    return result;
}

Point TrajectoryPlanner::denormalizeState(const Point &point) {
    Point tmp = point;

    bg::set<0>(tmp, bg::get<0>(point) * (_max_x - _min_x) + _min_x);
    bg::set<1>(tmp, bg::get<1>(point) * (_max_y - _min_y) + _min_y);
    bg::set<2>(tmp, bg::get<2>(point) * (_max_z - _min_z) + _min_z);

    bg::set<3>(tmp, bg::get<3>(point) * 2.0 * M_PI - M_PI);
    bg::set<4>(tmp, bg::get<4>(point) * 2.0 * M_PI - M_PI);
    bg::set<5>(tmp, bg::get<5>(point) * 2.0 * M_PI - M_PI);

    return tmp;
}

Point TrajectoryPlanner::normalizeState(const Point &point) {
    Point tmp = point;

    bg::set<0>(tmp, (bg::get<0>(point) - _min_x) / (_max_x - _min_x));
    bg::set<1>(tmp, (bg::get<1>(point) - _min_y) / (_max_y - _min_y));
    bg::set<2>(tmp, (bg::get<2>(point) - _min_z) / (_max_z - _min_z));

    bg::set<3>(tmp, (bg::get<3>(point) + M_PI) / (2.0 * M_PI));
    bg::set<4>(tmp, (bg::get<4>(point) + M_PI) / (2.0 * M_PI));
    bg::set<5>(tmp, (bg::get<5>(point) + M_PI) / (2.0 * M_PI));

    return tmp;
}

Eigen::VectorXd TrajectoryPlanner::sampleUnitBall() {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    std::normal_distribution<> gauss(0.0, 1.0);

    Eigen::VectorXd p(7);
    for (size_t i = 0; i < 6; ++i) {
        p[i] = gauss(gen);
    }

    p.normalize();

    double radius = std::pow(dis(gen), 1.0 / 6);
    p *= radius;

    p[6] = 0.0;

    return p;
}

Point TrajectoryPlanner::sampleWithinEllipsoid(double cmax, Eigen::MatrixXd C) {

    Point xcenter = _goalPoint;
    bg::add_point(xcenter, _startPoint);
    bg::divide_value(xcenter, 2.0);

    double cmin = metric(_startPoint, _goalPoint);

    double r1 = cmax / 2.0;
    double r2 = std::sqrt((cmax * cmax) - (cmin * cmin) / 2.0);

    Eigen::DiagonalMatrix<double, 7> L;
    L.diagonal() << r1, r2, r2, r2, r2, r2, r2;

    auto x = sampleUnitBall();

    Eigen::VectorXd xEllipisoid = C * L * x;

    Point xball;
    bg::set<0>(xball, xEllipisoid[0]);
    bg::set<1>(xball, xEllipisoid[1]);
    bg::set<2>(xball, xEllipisoid[2]);
    bg::set<3>(xball, xEllipisoid[3]);
    bg::set<4>(xball, xEllipisoid[4]);
    bg::set<5>(xball, xEllipisoid[5]);

    bg::add_point(xball, xcenter);

    return xball;
}

Eigen::MatrixXd
TrajectoryPlanner::computeRotationWorldFrame(const Point &start,
                                             const Point &goal) {

    Point tmp = goal;
    bg::subtract_point(tmp, start);
    bg::divide_value(tmp, metric(_goalPoint, _startPoint));

    std::vector<double> a1(7);
    a1[0] = bg::get<0>(tmp);
    a1[1] = bg::get<1>(tmp);
    a1[2] = bg::get<2>(tmp);
    a1[3] = bg::get<3>(tmp);
    a1[4] = bg::get<4>(tmp);
    a1[5] = bg::get<5>(tmp);
    a1[6] = 0.0;

    auto M = Eigen::Map<Eigen::VectorXd>(&*a1.begin(), a1.size()) *
             Eigen::MatrixXd::Identity(1, a1.size());
    auto svd = Eigen::JacobiSVD<Eigen::MatrixXd>(M, Eigen::ComputeFullU |
                                                        Eigen::ComputeFullV);

    auto diag_v = std::vector<double>(a1.size(), 1.0);
    diag_v[diag_v.size() - 1] = svd.matrixV().determinant();
    diag_v[diag_v.size() - 2] = svd.matrixU().determinant();

    return svd.matrixU() *
           Eigen::Map<Eigen::VectorXd>(&*diag_v.begin(), diag_v.size())
               .asDiagonal() *
           svd.matrixV().transpose();
}

void TrajectoryPlanner::performInformedRRTStar() {
    _path.clear();
    _pathTree.clear();
    _pathTree.insert(std::make_pair(_pathRoot->_state, _pathRoot));

    size_t maxIterations = 10000;
    double goalBiasProbability = 0.25;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0., 1.0);

    double cBest = std::numeric_limits<double>::infinity();
    Eigen::MatrixXd C = computeRotationWorldFrame(_startPoint, _goalPoint);
    NodePtr bestGoalNode = nullptr;

    int iter = 0;
    int misses = 0;
    std::vector<NodePtr> goalNodes;
    for (size_t i = 0; i < maxIterations; ++i) {

        if (i >= 1000 && bestGoalNode == nullptr) {
            i = 0;
            _pathTree.clear();
            _pathTree.insert(std::make_pair(_pathRoot->_state, _pathRoot));
        }

        Point randomPoint;
        if (cBest < std::numeric_limits<double>::infinity()) {
            randomPoint = sampleWithinEllipsoid(cBest, C);
        } else {
            bg::set<0>(randomPoint, dis(gen));
            bg::set<1>(randomPoint, dis(gen));
            bg::set<2>(randomPoint, dis(gen));
            bg::set<3>(randomPoint, dis(gen));
            bg::set<4>(randomPoint, dis(gen));
            bg::set<5>(randomPoint, dis(gen));
        }

        if (dis(gen) < goalBiasProbability) {
            randomPoint = _goalPoint;
        }

        NodePtr newNode;
        if (getNewNode(randomPoint, newNode)) {

            size_t n = _pathTree.size();
            size_t k = std::max(static_cast<size_t>(std::ceil(kRRG * std::log(n))), 5ul);
            std::vector<NodePtr> neighborhood = findNearestNeighbors(newNode, k * n);

            NodePtr bestParent = newNode->_parent;
            double bestCost = newNode->_cost;
            for (auto &neighbor : neighborhood) {
                double cost =
                    newNode->_cost + metric(newNode->_state, neighbor->_state);
                if (cost < bestCost && motionIsValid(newNode->_arm, neighbor->_arm)) {
                    bestParent = neighbor;
                    bestCost = cost;
                }
            }

            if (bestParent != newNode->_parent) {
                newNode->_parent = bestParent;
                newNode->_cost = bestCost;
            }
            newNode->_costToGoal = metric(_goalPoint, newNode->_state);
            _pathTree.insert(std::make_pair(newNode->_state, newNode));

            rewirePath(newNode, neighborhood);

            if (closeToGoal(newNode->_arm)) {
                goalNodes.push_back(newNode);
            }

            for (const auto &node : goalNodes) {
                if (bestGoalNode == nullptr || node->estimateCost() < bestGoalNode->estimateCost()) {
                    bestGoalNode = node;
                    cBest = node->estimateCost();
                    goalBiasProbability = 0.0;
                    C = computeRotationWorldFrame(_startPoint, node->_state);
                }
            }
        } else {
            ++misses;
        }
        ++iter;
        if(bestGoalNode != nullptr)
            qDebug() << bestGoalNode->estimateCost();
    }
    qDebug() << "done";
    qDebug() << "iters: " << iter;
    qDebug() << "misses: " << misses;

    _path.clear();
    NodePtr node = bestGoalNode;
    qDebug() << "final: " << node->_cost;
    while (node) {
        _path.push_back(std::make_pair(node->_state, node->_arm));
        node = node->_parent;
    }

    qDebug() << iter;
    qDebug() << "done";
    std::reverse(_path.begin(), _path.end());
}

double TrajectoryPlanner::metric(const Point &a, const Point &b) {

      double dx = std::abs(bg::get<0>(a) - bg::get<0>(b));
      double dy = std::abs(bg::get<1>(a) - bg::get<1>(b));
      double dz = std::abs(bg::get<2>(a) - bg::get<2>(b));

      double dtheta_x = std::abs(bg::get<3>(a) - bg::get<3>(b));
      double dtheta_y = std::abs(bg::get<4>(a) - bg::get<4>(b));
      double dtheta_z = std::abs(bg::get<5>(a) - bg::get<5>(b));

      double euclideanDist = std::sqrt(dx*dx + dy*dy + dz*dz);
      double eulerDist = std::sqrt(dtheta_x*dtheta_x + dtheta_y*dtheta_y + dtheta_z*dtheta_z);

      return euclideanDist + 0.05 * eulerDist;
}

bool TrajectoryPlanner::closeToGoal(RobotArm arm) {
      Eigen::Affine3d pose = arm.getEndeffectorPose();

      bool translation = (pose.translation() - _goalTranslation).norm() < 0.1 * stepSize * _limits.norm();

      Eigen::Quaterniond qCurrent(pose.rotation());
      Eigen::Quaterniond qGoal(_goalRotation);

      double angleDifference = qCurrent.angularDistance(qGoal);

      bool rotation = angleDifference <= M_PI / 4.0;

      return translation && rotation;
}

std::vector<NodePtr> TrajectoryPlanner::findNearestNeighbors(const NodePtr& node, unsigned int k) {

      std::vector<NodePtr> nodes;
      std::vector<std::pair<Point, NodePtr>> results;
      _pathTree.query(bgi::nearest(node->_state, k), std::back_inserter(results));
      for (auto const &pair : results) {
        nodes.push_back(pair.second);
      }

      return nodes;
}

std::vector<NodePtr> TrajectoryPlanner::findNearestNeighbors(bgi::rtree<std::pair<Point, NodePtr>, bgi::rstar<8, 4>>& tree, const NodePtr& node, unsigned int k) {

      std::vector<NodePtr> nodes;
      std::vector<std::pair<Point, NodePtr>> results;
      _pathTree.query(bgi::nearest(node->_state, k), std::back_inserter(results));
      for (auto const &pair : results) {
        nodes.push_back(pair.second);
      }

      return nodes;
}

void TrajectoryPlanner::computeStraightLinePath() {

      _path.clear();

      RobotArm startArm(_pathRoot->_arm);
      RobotArm endArm(startArm);

      endArm.moveToTaskState(_goalPoint, _limits);

      Point startState = startArm.getTaskState(_limits);
      Point endState = endArm.getTaskState(_limits);

      double dist = bg::distance(startState, endState);

      if (dist > stepSize * 2.) {
        int numSteps = static_cast<int>(std::ceil(dist / stepSize * 2.));
        bg::subtract_point(endState, startState);
        bg::divide_value(endState, numSteps);
        for (int i = 1; i < numSteps; ++i) {
            bg::add_point(startState, endState);
            if (!startArm.moveToTaskState(startState, _limits)) {
                continue;
            }
            _path.push_back(std::make_tuple(startState, startArm));
        }
    }
}

NodePtr TrajectoryPlanner::nearestNeighbor(const bgi::rtree<std::pair<Point, NodePtr>, bgi::rstar<8, 4>>& tree, const Point& point) {
    std::vector<std::pair<Point, NodePtr>> result;
    tree.query(bgi::nearest(point, 1), std::back_inserter(result));
    return result[0].second;
}

bool TrajectoryPlanner::extendTree(bgi::rtree<std::pair<Point, NodePtr>, bgi::rstar<8, 4>>& tree, const Point& target, NodePtr& newNode) {
    Point newState = target;
    if (!getNewNode(newState, newNode, tree)) {
        return false;
    }
    tree.insert(std::make_pair(newState, newNode));
    return true;
}

void TrajectoryPlanner::constructPath(NodePtr startNode, NodePtr goalNode) {
    _path.clear();

    for (NodePtr node = startNode; node != nullptr; node = node->_parent) {
        _path.insert(_path.begin(), std::make_tuple(node->_state, node->_arm));
    }

    for (NodePtr node = goalNode; node != nullptr; node = node->_parent) {
        _path.push_back(std::make_tuple(node->_state, node->_arm));
    }
}

void TrajectoryPlanner::performMultiple() {

    _paths.clear();
    for (int i = 0; i < 5; i++)
    {
        std::vector<Vec3> path;
        performBiRRTStar();
        smoothBSpline(std::max(10., _path.size() * 0.05), 0.0005);
        qDebug() << "done with one iteration";
        for (auto [p, arm] : _path) {
            Point state = arm.getTaskState(_limits);
            path.push_back(arm.getEndeffectorPose().translation());
        }
        _paths.push_back(path);
        break;
    }

    emit finished();
}

bool TrajectoryPlanner::performBiRRT() {
    bgi::rtree<std::pair<Point, NodePtr>, bgi::rstar<8, 4>> startTree;
    bgi::rtree<std::pair<Point, NodePtr>, bgi::rstar<8, 4>> goalTree;
    _path.clear();

    NodePtr startNode = _pathRoot;
    RobotArm startArm = startNode->_arm;
    startArm.moveToTaskState(_goalPoint, _limits);
    NodePtr goalNode = std::make_shared<PathNode>(_goalPoint, startArm);
    startTree.insert(std::make_pair(startNode->_state, startNode));
    goalTree.insert(std::make_pair(goalNode->_state, goalNode));

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);
    double goalBiasProbability = 0.2;
    unsigned int maxIterations = 75000;
    bool isStartTree = true;
    for (unsigned int i = 0; i < maxIterations; ++i) {
        Point randomPoint;

        if (dis(gen) < goalBiasProbability) {
            randomPoint = isStartTree ? _goalPoint : startNode->_state;
        } else {
            bg::set<0>(randomPoint, dis(gen));
            bg::set<1>(randomPoint, dis(gen));
            bg::set<2>(randomPoint, dis(gen));
            bg::set<3>(randomPoint, dis(gen));
            bg::set<4>(randomPoint, dis(gen));
            bg::set<5>(randomPoint, dis(gen));
        }

        NodePtr newNode;
        if (extendTree(isStartTree ? startTree : goalTree, randomPoint, newNode)) {
            NodePtr nearestOther = nearestNeighbor(isStartTree ? goalTree : startTree, newNode->_state);
            if (motionIsValid(newNode->_arm, nearestOther->_arm)) {
                constructPath(isStartTree ? newNode : nearestOther,
                              isStartTree ? nearestOther : newNode);
                return true;
            }
        }

        isStartTree = !isStartTree;
    }
    return false;
}

bool TrajectoryPlanner::performBiRRTStar() {
    bgi::rtree<std::pair<Point, NodePtr>, bgi::rstar<8, 4>> startTree;
    bgi::rtree<std::pair<Point, NodePtr>, bgi::rstar<8, 4>> goalTree;
    _path.clear();

    NodePtr startNode = _pathRoot;
    RobotArm startArm = startNode->_arm;
    startArm.moveToTaskState(_goalPoint, _limits);
    NodePtr goalNode = std::make_shared<PathNode>(_goalPoint, startArm);
    startTree.insert(std::make_pair(startNode->_state, startNode));
    goalTree.insert(std::make_pair(goalNode->_state, goalNode));

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    double goalBiasProbability = 0.2;
    unsigned int maxIterations = 7500;
    bool isStartTree = true;

    NodePtr bestStartNode = nullptr;
    NodePtr bestConnectNode = nullptr;
    NodePtr bestGoalNode = nullptr;
    double bestPathCost = std::numeric_limits<double>::infinity();

    for (unsigned int i = 0; i < maxIterations; ++i) {
        Point randomPoint;
        if (dis(gen) < goalBiasProbability) {
            randomPoint = isStartTree ? _goalPoint : startNode->_state;
        } else {
            bg::set<0>(randomPoint, dis(gen));
            bg::set<1>(randomPoint, dis(gen));
            bg::set<2>(randomPoint, dis(gen));
            bg::set<3>(randomPoint, dis(gen));
            bg::set<4>(randomPoint, dis(gen));
            bg::set<5>(randomPoint, dis(gen));
        }

        auto& currentTree = isStartTree ? startTree : goalTree;
        auto& otherTree = isStartTree ? goalTree : startTree;

        NodePtr newNode;
        if (extendTree(currentTree, randomPoint, newNode)) {
            size_t n = startTree.size() + goalTree.size();
            size_t k = std::max(static_cast<size_t>(std::ceil(kRRG * std::log(n))), 5ul);

            std::vector<NodePtr> neighborhood = findNearestNeighbors(currentTree, newNode, k);

            NodePtr bestParent = newNode->_parent;
            double bestCost = newNode->_cost;
            for (auto& neighbor : neighborhood) {
                double cost = neighbor->_cost + metric(neighbor->_state, newNode->_state);
                if (cost < bestCost && motionIsValid(neighbor->_arm, newNode->_arm)) {
                    bestParent = neighbor;
                    bestCost = cost;
                }
            }
            newNode->_parent = bestParent;
            newNode->_cost = bestCost;

            for (auto& neighbor : neighborhood) {
                double newCost = newNode->_cost + metric(newNode->_state, neighbor->_state);
                if (newCost < neighbor->_cost && motionIsValid(newNode->_arm, neighbor->_arm)) {
                    neighbor->_parent = newNode;
                    neighbor->_cost = newCost;
                }
            }

            NodePtr nearestOther = nearestNeighbor(otherTree, newNode->_state);
            if (motionIsValid(newNode->_arm, nearestOther->_arm)) {
                double connectionCost = newNode->_cost + nearestOther->_cost + metric(newNode->_state, nearestOther->_state);

                if (!isStartTree) {
                    connectionCost += metric(nearestOther->_state, goalNode->_state);
                }

                if (connectionCost < bestPathCost) {
                    bestStartNode = isStartTree ? newNode : nearestOther;
                    bestConnectNode = isStartTree ? nearestOther : newNode;
                    bestPathCost = connectionCost;
                }
            }

            if (isStartTree && motionIsValid(newNode->_arm, goalNode->_arm)) {
                double directGoalCost = newNode->_cost + metric(newNode->_state, goalNode->_state);
                if (directGoalCost < bestPathCost) {
                    bestStartNode = newNode;
                    bestConnectNode = goalNode;
                    bestPathCost = directGoalCost;
                }
            }
        }

        isStartTree = !isStartTree;
    }

    if (bestStartNode && bestConnectNode) {
        std::vector<std::tuple<Point, RobotArm>> path;

        NodePtr current = bestStartNode;
        while (current) {
            path.push_back(std::make_pair(current->_state, current->_arm));
            current = current->_parent;
        }
        std::reverse(path.begin(), path.end());

        path.push_back(std::make_pair(bestConnectNode->_state, bestConnectNode->_arm));

        if (bestConnectNode != goalNode) {
            current = bestConnectNode->_parent;
            while (current) {
                path.push_back(std::make_pair(current->_state, current->_arm));
                current = current->_parent;
            }

            path.push_back(std::make_pair(goalNode->_state, goalNode->_arm));
        }

        _path = path;
        return true;
    }

    return false;
}

Eigen::VectorXd TrajectoryPlanner::computeObstacleGradient(const RobotArm& arm) const {

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(7);

    return gradient;
}



double TrajectoryPlanner::obstacleDistanceCost(double distance) const {
    if (distance < 0) {
        return -distance + 0.5 * _obstacleEpsilon;
    } else if (distance <= _obstacleEpsilon) {
        double diff = _obstacleEpsilon - distance;
        return 0.5 * diff * diff / _obstacleEpsilon;
    }
    return 0.0;
}

Eigen::MatrixXd TrajectoryPlanner::computeAMatrix(int trajLength) const {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(trajLength, trajLength);
    A.diagonal(-1).setConstant(-1);
    A.diagonal(0).setConstant(2);
    A.diagonal(1).setConstant(-1);
    A(0, 0) = 1;
    A(trajLength - 1, trajLength - 1) = 1;
    return A;
}

void TrajectoryPlanner::performCHOMP() {
    const int trajLength = 10;
    const int numJoints = _startArm.getJointAngles().size();
    const double learningRate = .1;
    const int maxIterations = 1000;
    const double convergenceThreshold = 1e-12;

    Eigen::MatrixXd trajectory = Eigen::MatrixXd::Zero(trajLength, numJoints);
    Eigen::VectorXd startJoints = _startArm.getJointAngles();
    Eigen::VectorXd goalJoints = _goalArm.getJointAngles();

    for (int i = 0; i < trajLength; ++i) {
        double t = static_cast<double>(i) / (trajLength - 1);
        trajectory.row(i) = (1 - t) * startJoints + t * goalJoints;
    }

    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        Eigen::VectorXd obstacleGradient = Eigen::VectorXd::Zero(trajLength * numJoints);

        for (int i = 0; i < trajLength; ++i) {
            RobotArm arm = _startArm;
            arm.setJointAngles(trajectory.row(i));
            Eigen::VectorXd stepObstacleGradient = computeObstacleGradient(arm);
            obstacleGradient.segment(i * numJoints, numJoints) = stepObstacleGradient;
        }

        Eigen::VectorXd update = -learningRate * obstacleGradient;
        trajectory += update.reshaped(numJoints, trajLength).transpose();

        for (int i = 0; i < trajLength; ++i) {
            for (int j = 0; j < numJoints; ++j) {
                trajectory(i, j) = std::clamp(trajectory(i, j),
                                              _startArm._joints[j]->limit_min,
                                              _startArm._joints[j]->limit_max);
            }
        }


        if (update.norm() < convergenceThreshold) {
            qDebug() << "CHOMP converged after" << iteration + 1 << "iterations";
            break;
        }

        if (iteration % 10 == 0) {
            qDebug() << "Iteration" << iteration << "Total cost:" << update.norm();
        }
    }

    _path.clear();
    for (int i = 0; i < trajLength; ++i) {
        RobotArm arm = _startArm;
        arm.setJointAngles(trajectory.row(i));
        Point state = arm.getTaskState(_limits);
        _path.push_back(std::make_pair(state, arm));
    }
}

void TrajectoryPlanner::initializeSTOMPMatrices() {
    A = Eigen::MatrixXd::Zero(N, N);
    A.diagonal(-1).setConstant(1);
    A.diagonal(0).setConstant(-2);
    A.diagonal(1).setConstant(1);
    A(0, 0) = -1; A(0, 1) = 1;
    A(N-1, N-2) = -1; A(N-1, N-1) = 1;

    R = A.transpose() * A;
    Rinv = R.inverse();

    Eigen::LLT<Eigen::MatrixXd> cholSolver(Rinv);
    L = cholSolver.matrixL();

    M = Rinv;
    for (int i = 0; i < N; i++) {
        double maxElement = M.col(i).cwiseAbs().maxCoeff();
        M.col(i) *= 1.0 / (N * maxElement);
    }
}


double TrajectoryPlanner::computeCost(const RobotArm& arm) {
    double cost = 0.0;

    auto bboxes = arm.getLinkBoundingBoxes();
    double minDist = std::numeric_limits<double>::max();

    for (auto bbox : bboxes) {
        auto center = std::get<0>(bbox);
        double dist = std::get<0>(_obstacleTree->getDistanceAndClosestPoint(center));

        cost += 1. / dist;

    }

    return cost;
}
