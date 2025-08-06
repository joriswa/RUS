#ifndef OBSTACLETREE_H
#define OBSTACLETREE_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "GeometryLib/Obstacle.h"

#include <memory>

using Vec3 = Eigen::Vector3d;

struct BoundingBox {
    Vec3 min;
    Vec3 max;

    bool doesSegmentIntersect(Vec3 start, Vec3 end) const {

        for (int i = 0; i < 3; i++) {
            if(start[i] < min[i] && end[i] < min[i]) return false;
            if(start[i] > max[i] && end[i] > max[i]) return false;
        }

        double tMax = std::numeric_limits<double>::max();
        double tMin = std::numeric_limits<double>::lowest();

        for (int i = 0; i < 3; i++) {
            double t1 = (min[i] - start[i]) / (end[i] - start[i]);
            double t2 = (max[i] - start[i]) / (end[i] - start[i]);

            tMin = std::max(tMin, std::min(t1, t2));
            tMax = std::min(tMax, std::max(t1, t2));
        }

        return tMax >= tMin && tMax >= 0.0;
    }

    bool doesOBBIntersect(const Eigen::Vector3d& obbCenter, const Eigen::Vector3d& obbHalfDims, const Eigen::Matrix3d& obbAxes) {

        Eigen::Vector3d aabbCenter = (min + max) / 2.0;
        Eigen::Vector3d aabbHalfDims = (max - min) / 2.0;

        Eigen::Vector3d T = obbCenter - aabbCenter;
        T = T.transpose() * obbAxes;

        Eigen::Matrix3d R;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R(i, j) = obbAxes.col(i).dot(Eigen::Vector3d::Unit(j));
            }
        }

        Eigen::Matrix3d AbsR = R.cwiseAbs();
        Eigen::Vector3d aabbHalfDimsInOBB = aabbHalfDims.transpose() * AbsR;

        for (int i = 0; i < 3; ++i) {
            if (fabs(T(i)) > (obbHalfDims(i) + aabbHalfDims(i))) {
                return false;
            }
        }

        for (int i = 0; i < 3; ++i) {
            if (fabs(T.dot(R.col(i))) > (obbHalfDims(i) + aabbHalfDimsInOBB(i))) {
                return false;
            }
        }

        for (int i = 0; i < 3; ++i) {
            if (fabs(T.dot(R.row(i))) > (obbHalfDims.dot(AbsR.row(i)) + aabbHalfDims(i))) {
                return false;
            }
        }

        return true;
    }
};

struct QuadTreeNode {

    BoundingBox bounds;
    std::vector<Obstacle*> obstacles;
    const int maxObstaclesPerNode = 4;

    std::unique_ptr<QuadTreeNode> topLeft;
    std::unique_ptr<QuadTreeNode> topRight;
    std::unique_ptr<QuadTreeNode> bottomLeft;
    std::unique_ptr<QuadTreeNode> bottomRight;

    bool isLeaf() const {
        return topLeft == nullptr;
    }

    QuadTreeNode() = default;

    QuadTreeNode(const QuadTreeNode& other) {

        bounds = other.bounds;

        for (const auto& obs : other.obstacles) {
//            obstacles.push_back(obs->clone());
        }

        if (other.topLeft) topLeft = std::make_unique<QuadTreeNode>(*other.topLeft);
        if (other.topRight) topRight = std::make_unique<QuadTreeNode>(*other.topRight);
        if (other.bottomLeft) bottomLeft = std::make_unique<QuadTreeNode>(*other.bottomLeft);
        if (other.bottomRight) bottomRight = std::make_unique<QuadTreeNode>(*other.bottomRight);

    }
};


class ObstacleTree
{
private:
    std::vector<std::unique_ptr<Obstacle>> _obstacles;
    std::unique_ptr<QuadTreeNode> _root;

    void subdivideNode(QuadTreeNode* node);
    void initializeTree(const BoundingBox& workspace);
    void findIntersectingNodes(QuadTreeNode* node, const Vec3& start, const Vec3& end, std::vector<Obstacle*>& obstacles);
    void findIntersectingNodes(QuadTreeNode* node, const Vec3& obbCenter, const Vec3& obbHalfDims, const Eigen::Matrix3d obbAxes, std::vector<Obstacle*>& obstacles);


public:
    ObstacleTree(const Vec3& lowerBound, const Vec3& upperBound);
    ObstacleTree(const ObstacleTree& other);
    ObstacleTree() = default;
    ObstacleTree& operator=(const ObstacleTree& other);
    void insertObstacle(Obstacle* obstacle);
    bool hasCollision(const Vec3& start, const Vec3& end);
    bool hasCollision(const Vec3& centerBBox, const Vec3& halfDims, const Eigen::Matrix3d& axes);
    bool intersectsBox(const Vec3& max, const Vec3& min);
    std::vector<std::tuple<Vec3, double>> getSphereObstacleGeometries();
};

#endif // OBSTACLETREE_H
