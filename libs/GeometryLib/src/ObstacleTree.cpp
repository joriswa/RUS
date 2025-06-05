#include "GeometryLib/ObstacleTree.h"

ObstacleTree::ObstacleTree(const Vec3& lowerBound, const Vec3& upperBound)
{

    BoundingBox box;
    box.min = lowerBound;
    box.max = upperBound;

    initializeTree(box);

}


void ObstacleTree::insertObstacle(Obstacle* obstacle) {


}


void ObstacleTree::subdivideNode(QuadTreeNode* node) {

}

void ObstacleTree::findIntersectingNodes(QuadTreeNode* node, const Vec3& start, const Vec3& end, std::vector<Obstacle*>& obstacles) {


}

void ObstacleTree::findIntersectingNodes(QuadTreeNode* node, const Vec3& obbCenter, const Vec3& obbHalfDims, const Eigen::Matrix3d obbAxes, std::vector<Obstacle*>& obstacles) {


}

bool ObstacleTree::hasCollision(const Vec3& start, const Vec3& end) {

    return false;
}

bool ObstacleTree::hasCollision(const Vec3& centerBBox, const Vec3& halfDims, const Eigen::Matrix3d& axes) {


    return false;
}

std::vector<std::tuple<Vec3, double>> ObstacleTree::getSphereObstacleGeometries() {


}

void ObstacleTree::initializeTree(const BoundingBox& workspace) {

    _root = std::make_unique<QuadTreeNode>();
    _root->bounds = workspace;

}

ObstacleTree::ObstacleTree(const ObstacleTree& other) {

    for (const auto& obs : other._obstacles) {
        _obstacles.push_back(std::unique_ptr<Obstacle>(obs->clone()));
    }

    if (other._root) {
        _root = std::make_unique<QuadTreeNode>(*other._root);
    }
}

ObstacleTree& ObstacleTree::operator=(const ObstacleTree& other) {
    if (this == &other) return *this;

    _obstacles.clear();
    for (const auto& obs : other._obstacles) {
        _obstacles.push_back(std::unique_ptr<Obstacle>(obs->clone()));
    }

    if (other._root) {
        _root = std::make_unique<QuadTreeNode>(*other._root);
    } else {
        _root.reset();
    }

    return *this;
}
