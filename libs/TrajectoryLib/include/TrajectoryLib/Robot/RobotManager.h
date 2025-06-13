#ifndef ROBOTMANAGER_H
#define ROBOTMANAGER_H

#include <QCuboidMesh>
#include <QEffect>
#include <QGraphicsApiFilter>
#include <QRenderPass>
#include <QSharedPointer>
#include <QSphereMesh>
#include <QTechnique>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/QPhongAlphaMaterial>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DRender/QBlendEquation>
#include <Qt3DRender/QDepthTest>
#include <Qt3DRender/QParameter>
#include "GeometryLib/Obstacle.h"
#include "TrajectoryLib/Robot/Robot.h"
#include <boost/property_tree/ptree.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class RobotManager
{
public:
    void addRobot(std::shared_ptr<Robot> actor);
    void removeRobot(const std::shared_ptr<Robot> &actor);
    std::vector<std::shared_ptr<Robot>> getRobots() const;
    void transformRobots(const std::shared_ptr<Robot> &actor, const Eigen::Affine3d &transform);
    void parseXML(const std::string &filename);
    void createEntities(Qt3DCore::QEntity *rootEntity, bool showBBoxes);
    std::unordered_map<std::shared_ptr<Robot>, QSharedPointer<Qt3DCore::QEntity>> getNodeEntities()
        const;
    void updateEntities(bool showBoundingVolumes);
    std::vector<std::shared_ptr<Obstacle>> getTransformedObstacles() const;

    void clear();

    void parseURDF(const std::string &filename);

private:
    std::vector<std::shared_ptr<Robot>> _robots;
    Qt3DCore::QEntity* _rootEntity;
    std::unordered_map<std::shared_ptr<Robot>, QSharedPointer<Qt3DCore::QEntity>> _nodeEntities;
    std::unordered_map<std::shared_ptr<Obstacle>, QSharedPointer<Qt3DCore::QEntity>> _obstacleEntities;

    Eigen::Affine3d parseTransform(const boost::property_tree::ptree& node);
    Eigen::Vector3d parseScale(const boost::property_tree::ptree& node);
    std::shared_ptr<Obstacle> parseObstacle(const std::string& type, const boost::property_tree::ptree& node);
    std::shared_ptr<Robot> parseActorNode(const boost::property_tree::ptree &node);
    std::shared_ptr<Robot> parseNode(
        const boost::property_tree::ptree &node,
        std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<Joint>>> rotations,
        std::shared_ptr<std::vector<std::shared_ptr<Obstacle>>> obstacles);
    void updateEntityTransform(Qt3DCore::QTransform *transform,
                               std::shared_ptr<Robot> actor,
                               const Eigen::Affine3d &parentTransform,
                               bool showBBoxes);
    void traverseActorForTransformedObstacles(
        std::shared_ptr<Robot> actor,
        const Eigen::Affine3d &parentTransform,
        std::vector<std::shared_ptr<Obstacle>> &transformedObstacles) const;

    void createNodeEntities(std::shared_ptr<Robot> actor,
                            Qt3DCore::QEntity *parentEntity,
                            const Eigen::Affine3d &parentTransform,
                            bool showBBoxes);
    void createObstacleEntity(std::shared_ptr<Obstacle> obstacle, const Eigen::Affine3d& globalTransform, Qt3DCore::QEntity *parentEntity);
    void updateObstacleEntity(std::shared_ptr<Obstacle> obstacle, const Eigen::Affine3d &globalTransform);
    void createCollisionActor(const boost::property_tree::ptree &node);
    std::shared_ptr<Robot> parseRobotNode(const boost::property_tree::ptree &node);
    void parseLinkNode(const boost::property_tree::ptree &node, std::shared_ptr<Robot> actor);
    void parseVisualNode(const boost::property_tree::ptree &visual, std::shared_ptr<Robot> actor);
    void parseCollisionNode(const boost::property_tree::ptree &collision,
                            std::shared_ptr<Robot> actor);
    void parseJointNode(const boost::property_tree::ptree &node,
                        std::unordered_map<std::string, std::shared_ptr<Robot>> &link_map,
                        std::unordered_map<std::string, Eigen::Affine3d> &joint_transforms);
    void parseVisualNode(const boost::property_tree::ptree &visual,
                         std::shared_ptr<Robot> actor,
                         const Eigen::Affine3d &linkTransform);
    void parseCollisionNode(const boost::property_tree::ptree &collision,
                            std::shared_ptr<Robot> actor,
                            const Eigen::Affine3d &linkTransform);
};

#endif // ROBOTMANAGER_H
