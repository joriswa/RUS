#include "TrajectoryLib/Robot/RobotManager.h"

#include <QMatrix4x4>
#include <Qt3DCore/QEntity>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DRender/QGeometryRenderer>
#include "TrajectoryLib/Core/Util.h"
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <sstream>

void RobotManager::removeRobot(const std::shared_ptr<Robot> &actor)
{
    _robots.erase(std::remove(_robots.begin(), _robots.end(), actor), _robots.end());
}

std::vector<std::shared_ptr<Robot>> RobotManager::getRobots() const
{
    if (_robots.size() <= 1) {
        return {};
    }
    return std::vector<std::shared_ptr<Robot>>(_robots.begin() + 1, _robots.end());
}

void RobotManager::transformRobots(const std::shared_ptr<Robot> &actor,
                                   const Eigen::Affine3d &transform)
{
    actor->setFixedTransform(transform);
}

Eigen::Vector3d RobotManager::parseScale(const boost::property_tree::ptree &node)
{
    Eigen::Vector3d scale(1, 1, 1);
    if (auto scale_node = node.get_child_optional("scale")) {
        auto scale_xyz = scale_node->get<std::string>("<xmlattr>.xyz");
        std::istringstream scale_stream(scale_xyz);
        scale_stream >> scale.x() >> scale.y() >> scale.z();
    }
    return scale;
}

std::shared_ptr<Obstacle> RobotManager::parseObstacle(const std::string &type,
                                                      const boost::property_tree::ptree &node)
{
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();

    // Parse transformation if specified
    if (auto transform_node = node.get_child_optional("transform")) {
        transform = parseTransform(node);
    }

    // Parse scale if specified
    if (auto scale_node = node.get_child_optional("scale")) {
        scale = parseScale(node);
    }

    // Create obstacle based on type
    if (type == "box") {
        return std::make_shared<BoxObstacle>(scale, transform);
    }
    return nullptr;
}

std::shared_ptr<Robot> RobotManager::parseActorNode(const boost::property_tree::ptree &node)
{
    std::string actor_name = node.get<std::string>("<xmlattr>.name", "Unnamed");

    auto actor = std::make_shared<Robot>(actor_name, true);

    if (auto transform_node = node.get_child_optional("transform")) {
        Eigen::Affine3d transform = parseTransform(node);
        actor->setFixedTransform(transform);
    }

    if (auto mesh_node = node.get_child_optional("visual")) {
        actor->setMesh(mesh_node->get<std::string>("<xmlattr>.source"));
    }

    if (const auto& collision_node = node.get_child_optional("collision")) {
        for (const auto& primitive_node : *collision_node) {
            auto obstacle = parseObstacle(primitive_node.first, primitive_node.second);
            actor->addObstacle(obstacle);
        }
    }

    for (const auto& child_node : node) {
        if (child_node.first == "node") {
            auto childActor = parseNode(child_node.second,
                                        actor->getJoints(),
                                        actor->getObstacles());
            actor->addChild(childActor);
        }
    }

    return actor;
}

std::shared_ptr<Robot> RobotManager::parseNode(
    const boost::property_tree::ptree &node,
    std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<Joint>>> rotations,
    std::shared_ptr<std::vector<std::shared_ptr<Obstacle>>> obstacles)
{
    std::string actor_name = node.get<std::string>("<xmlattr>.name", "Unnamed");

    auto actor = std::make_shared<Robot>(actor_name, false, rotations);

    if (auto transform_node = node.get_child_optional("transform")) {
        Eigen::Affine3d transform = parseTransform(node);
        actor->setFixedTransform(transform);
    }

    if (auto mesh_node = node.get_child_optional("visual")) {
        actor->setMesh(mesh_node->get<std::string>("<xmlattr>.source"));
    }

    if (const auto& collision_node = node.get_child_optional("collision")) {
        for (const auto& primitive_node : *collision_node) {
            auto obstacle = parseObstacle(primitive_node.first, primitive_node.second);
            actor->addObstacle(obstacle);
        }
    }

    if (auto articulate_node = node.get_child_optional("articulate")) {
        for (const auto& articulation_node : *articulate_node) {
            if (articulation_node.first == "rotate") {
                std::string name = articulation_node.second.get<std::string>("<xmlattr>.name", "Unnamed");
                std::string origin_str = articulation_node.second.get<std::string>("<xmlattr>.origin", "0 0 0");
                std::string axis_str = articulation_node.second.get<std::string>("<xmlattr>.axis", "1 0 0");
                std::string range_str = articulation_node.second.get<std::string>("<xmlattr>.range", "-3.14 3.14");

                std::istringstream originStream(origin_str);
                std::istringstream axisStream(axis_str);
                std::istringstream rangeStream(range_str);

                Eigen::Vector3d origin, axis;
                double min_range, max_range;
                originStream >> origin[0] >> origin[1] >> origin[2];
                axisStream >> axis[0] >> axis[1] >> axis[2];
                rangeStream >> min_range >> max_range;

                auto articulation = std::make_shared<Joint>(name, origin, axis, min_range, max_range);
                actor->addJoint(name, articulation);
            }
        }
    }

    for (const auto& child_node : node) {
        if (child_node.first == "node") {
            auto childActor = parseNode(child_node.second, rotations, obstacles);
            actor->addChild(childActor);
        }
    }

    return actor;
}

void RobotManager::parseXML(const std::string &filename)
{
    using namespace boost::property_tree;
    ptree tree;
    read_xml(filename, tree);

    auto space_node = tree.get_child("space");

    if (auto collision_node = space_node.get_child_optional("collision")) {
        createCollisionActor(*collision_node);
    }

    for (const auto& actor_node : space_node) {
        if (actor_node.first == "actor") {
            std::string actor_name = actor_node.second.get<std::string>("<xmlattr>.name", "Unnamed");
            auto root_actor = parseActorNode(actor_node.second);
            addRobot(root_actor);
        }
    }
}

void RobotManager::createEntities(Qt3DCore::QEntity *rootEntity, bool showBBoxes)
{
    for (const auto &actor : _robots) {
        createNodeEntities(actor, rootEntity, Eigen::Affine3d::Identity(), showBBoxes);
    }
}

void RobotManager::createNodeEntities(std::shared_ptr<Robot> actor,
                                      Qt3DCore::QEntity *parentEntity,
                                      const Eigen::Affine3d &parentTransform,
                                      bool showBBoxes)
{
    Eigen::Affine3d actorTransform = actor->getTransform();
    Eigen::Affine3d globalTransform = parentTransform * actorTransform;
    _rootEntity = parentEntity;
    if (!actor->getMesh().empty()) {
        Qt3DCore::QEntity *actorEntity = new Qt3DCore::QEntity(parentEntity);
        Qt3DCore::QGeometry *geometry = Util::loadSTL(QString::fromStdString(actor->getMesh()));
        auto *mesh = new Qt3DRender::QGeometryRenderer();
        mesh->setGeometry(geometry);

        Qt3DCore::QTransform *transform = new Qt3DCore::QTransform();
        transform->setMatrix(Util::convertEigenAffine3dToQMatrix4x4(globalTransform));

        Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
        material->setDiffuse(QColor(QRgb(0xBEBEBE)));

        actorEntity->addComponent(mesh);
        actorEntity->addComponent(transform);
        actorEntity->addComponent(material);

        _nodeEntities[actor] = QSharedPointer<Qt3DCore::QEntity>(actorEntity);
    }

    if (showBBoxes) {
        for (const auto& obstacle : *actor->getObstacles()) {
            createObstacleEntity(obstacle, globalTransform, parentEntity);
        }
    }

    for (const auto& child : actor->getChildren()) {
        createNodeEntities(child, parentEntity, globalTransform, showBBoxes);
    }
}

std::unordered_map<std::shared_ptr<Robot>, QSharedPointer<Qt3DCore::QEntity>>
RobotManager::getNodeEntities() const
{
    return _nodeEntities;
}

void RobotManager::updateEntities(bool showBoundinVolumes)
{
    for (const auto& pair : _nodeEntities) {
        std::shared_ptr<Robot> actor = pair.first;
        QSharedPointer<Qt3DCore::QEntity> entity = pair.second;

        Qt3DCore::QTransform *entityTransform = entity->componentsOfType<Qt3DCore::QTransform>().constFirst();
        if (entityTransform) {
            Eigen::Affine3d parentTransform = Eigen::Affine3d::Identity();
            updateEntityTransform(entityTransform, actor, parentTransform, showBoundinVolumes);
        }
    }
    if (showBoundinVolumes && _robots.size()) {
        for (const auto &obstacle : *_robots.begin()->get()->getObstacles()) {
            if (_obstacleEntities.count(obstacle) > 0) {
                updateObstacleEntity(obstacle, Eigen::Affine3d::Identity());
            } else {
                createObstacleEntity(obstacle, Eigen::Affine3d::Identity(), _rootEntity);
            }
        }
    }
    if (!showBoundinVolumes) {
        _obstacleEntities.clear();
    }
}

void RobotManager::updateEntityTransform(Qt3DCore::QTransform *transform,
                                         std::shared_ptr<Robot> actor,
                                         const Eigen::Affine3d &parentTransform,
                                         bool showBoundinVolumes)
{
    Eigen::Affine3d localTransform = parentTransform * actor->getTransform();

    if (showBoundinVolumes) {
        for (const auto& obstacle : *actor->getObstacles()) {
            if (_obstacleEntities.count(obstacle) > 0) {
                updateObstacleEntity(obstacle, localTransform);
            } else {
                createObstacleEntity(obstacle, localTransform, _rootEntity);
            }
        }
    }

    transform->setMatrix(Util::convertEigenAffine3dToQMatrix4x4(localTransform));

    for (const auto& child : actor->getChildren()) {
        auto it = _nodeEntities.find(child);

        if (it != _nodeEntities.end()) {
            Qt3DCore::QTransform *childTransform = it->second->componentsOfType<Qt3DCore::QTransform>().constFirst();
            if (childTransform) {
                updateEntityTransform(childTransform, child, localTransform, showBoundinVolumes);
            }
        }
    }
}

std::vector<std::shared_ptr<Obstacle>> RobotManager::getTransformedObstacles() const
{
    std::vector<std::shared_ptr<Obstacle>> transformedObstacles;

    for (const auto &actor : _robots) {
        traverseActorForTransformedObstacles(actor, Eigen::Affine3d::Identity(), transformedObstacles);
    }

    return transformedObstacles;
}

void RobotManager::traverseActorForTransformedObstacles(
    std::shared_ptr<Robot> actor,
    const Eigen::Affine3d &parentTransform,
    std::vector<std::shared_ptr<Obstacle>> &transformedObstacles) const
{
    Eigen::Affine3d actorTransform = actor->getTransform();
    Eigen::Affine3d globalTransform = parentTransform * actorTransform;

    for (const auto& obstacle : *actor->getObstacles()) {
        std::shared_ptr<Obstacle> transformedObstacle = obstacle->transform(globalTransform);
        transformedObstacles.push_back(std::move(transformedObstacle));
    }

    for (const auto& child : actor->getChildren()) {
        traverseActorForTransformedObstacles(child, globalTransform, transformedObstacles);
    }
}

void RobotManager::createObstacleEntity(std::shared_ptr<Obstacle> obstacle,
                                        const Eigen::Affine3d &globalTransform,
                                        Qt3DCore::QEntity *parentEntity)
{
    std::shared_ptr<Qt3DCore::QEntity> obstacleEntity = nullptr;

    if (auto box = std::dynamic_pointer_cast<BoxObstacle>(obstacle)) {
        Qt3DCore::QEntity *entity = new Qt3DCore::QEntity(parentEntity);

        Eigen::Affine3d scaledTransform = globalTransform * obstacle->getTransform();
        Qt3DExtras::QCuboidMesh *boxMesh = new Qt3DExtras::QCuboidMesh();

        boxMesh->setXExtent(box->getScale().x());
        boxMesh->setYExtent(box->getScale().y());
        boxMesh->setZExtent(box->getScale().z());

        Qt3DCore::QTransform *transform = new Qt3DCore::QTransform();
        transform->setMatrix(Util::convertEigenAffine3dToQMatrix4x4(scaledTransform));

//        Qt3DExtras::QPhongAlphaMaterial *material = new Qt3DExtras::QPhongAlphaMaterial();
        Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
        material->setDiffuse(QColor(0x0000FF));
//        material->setAlpha(0.5);

        entity->addComponent(boxMesh);
        entity->addComponent(transform);
        entity->addComponent(material);

        _obstacleEntities[obstacle] = QSharedPointer<Qt3DCore::QEntity>(entity);
    }
}

void RobotManager::updateObstacleEntity(std::shared_ptr<Obstacle> obstacle,
                                        const Eigen::Affine3d &globalTransform)
{
    if (auto box = std::dynamic_pointer_cast<BoxObstacle>(obstacle)) {
        auto entity = _obstacleEntities[obstacle];

        Eigen::Affine3d scaledTransform = globalTransform * obstacle->getTransform();

        auto transform = entity->componentsOfType<Qt3DCore::QTransform>().constFirst();
        if (transform) {
            transform->setMatrix(Util::convertEigenAffine3dToQMatrix4x4(scaledTransform));
        }
    }
}

void RobotManager::createCollisionActor(const boost::property_tree::ptree &node)
{
    std::string actor_name = "collision_actor";
    auto actor = std::make_shared<Robot>(actor_name, true);

    actor->setFixedTransform(Eigen::Affine3d::Identity());
    for (const auto& primitive_node : node) {
        if (primitive_node.first == "box") {
            auto obstacle = parseObstacle(primitive_node.first, primitive_node.second);
            actor->addObstacle(obstacle);
        }
    }

    addRobot(actor);
}

void RobotManager::clear()
{
    _robots.clear();
    _rootEntity = nullptr;
    _nodeEntities.clear();
    _obstacleEntities.clear();
}

void RobotManager::parseURDF(const std::string &filename)
{
    using namespace boost::property_tree;
    ptree tree;
    read_xml(filename, tree);

    _robots.clear();
    std::shared_ptr<Robot> collisionRobot = nullptr;

    for (const auto &robot_node : tree) {
        if (robot_node.first == "robot") {
            std::string robot_name = robot_node.second.get<std::string>("<xmlattr>.name", "Unnamed");
            auto robot_actor = parseRobotNode(robot_node.second);

            if (robot_name == "collision_robot") {
                collisionRobot = robot_actor;
            } else {
                addRobot(robot_actor);
            }
        }
    }

    if (collisionRobot) {
        _robots.insert(_robots.begin(), collisionRobot);
    }
}

std::shared_ptr<Robot> RobotManager::parseRobotNode(const boost::property_tree::ptree &node)
{
    std::string robot_name = node.get<std::string>("<xmlattr>.name", "Unnamed");
    auto actor = std::make_shared<Robot>(robot_name, robot_name == "collision_robot");

    for (const auto &link_node : node) {
        if (link_node.first == "link") {
            parseLinkNode(link_node.second, actor);
        }
    }

    return actor;
}

void RobotManager::parseLinkNode(const boost::property_tree::ptree &node,
                                 std::shared_ptr<Robot> actor)
{
    std::string link_name = node.get<std::string>("<xmlattr>.name", "Unnamed");

    // Parse link origin if present
    Eigen::Affine3d linkTransform = Eigen::Affine3d::Identity();
    if (auto origin = node.get_child_optional("origin")) {
        linkTransform = parseTransform(*origin);
    }

    // Parse visual element
    if (auto visual = node.get_child_optional("visual")) {
        parseVisualNode(*visual, actor, linkTransform);
    }

    for (const auto &collision : node) {
        if (collision.first == "collision") {
            parseCollisionNode(collision.second, actor, linkTransform);
        }
    }
}

void RobotManager::parseVisualNode(const boost::property_tree::ptree &visual,
                                   std::shared_ptr<Robot> actor,
                                   const Eigen::Affine3d &linkTransform)
{
    Eigen::Affine3d visualTransform = linkTransform;

    if (auto origin = visual.get_child_optional("origin")) {
        visualTransform = linkTransform * parseTransform(*origin);
    }

    if (auto geometry = visual.get_child_optional("geometry")) {
        if (auto mesh = geometry->get_child_optional("mesh")) {
            std::string filename = mesh->get<std::string>("<xmlattr>.filename");
            actor->setMesh(filename);
        }
    }

    actor->setFixedTransform(visualTransform);
}

void RobotManager::parseCollisionNode(const boost::property_tree::ptree &collision,
                                      std::shared_ptr<Robot> actor,
                                      const Eigen::Affine3d &linkTransform)
{
    Eigen::Affine3d collisionTransform = linkTransform;

    if (auto origin = collision.get_child_optional("origin")) {
        collisionTransform = linkTransform * parseTransform(*origin);
    }

    if (auto geometry = collision.get_child_optional("geometry")) {
        for (const auto &shape : *geometry) {
            if (shape.first == "box") {
                Eigen::Vector3d size;
                std::istringstream(shape.second.get<std::string>("<xmlattr>.size")) >> size.x()
                    >> size.y() >> size.z();
                auto obstacle = std::make_shared<BoxObstacle>(size, collisionTransform);
                actor->addObstacle(obstacle);
            }
            // Add support for other shapes (sphere, cylinder, etc.) here if needed
        }
    }
}

Eigen::Affine3d RobotManager::parseTransform(const boost::property_tree::ptree &node)
{
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();

    if (auto xyz = node.get_optional<std::string>("<xmlattr>.xyz")) {
        std::istringstream xyz_stream(*xyz);
        Eigen::Vector3d translation;
        xyz_stream >> translation.x() >> translation.y() >> translation.z();
        transform.translate(translation);
    }

    if (auto rpy = node.get_optional<std::string>("<xmlattr>.rpy")) {
        std::istringstream rpy_stream(*rpy);
        Eigen::Vector3d rotation;
        rpy_stream >> rotation.x() >> rotation.y() >> rotation.z();
        transform.rotate(Eigen::AngleAxisd(rotation.x(), Eigen::Vector3d::UnitX())
                         * Eigen::AngleAxisd(rotation.y(), Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(rotation.z(), Eigen::Vector3d::UnitZ()));
    }

    return transform;
}

void RobotManager::addRobot(std::shared_ptr<Robot> actor)
{
    if (actor->getName() == "collision_robot") {
        _robots.insert(_robots.begin(), actor);
    } else {
        _robots.push_back(actor);
    }
}
