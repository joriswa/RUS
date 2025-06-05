#include "TrajectoryLib/Robot.h"
#include <QDebug>

Robot::Robot(const std::string& name, bool isRoot, std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<Joint>>> rotations, std::shared_ptr<std::vector<std::shared_ptr<Obstacle>>> obstacles)
    : _name(name), _isRoot(isRoot) {
    if (rotations) {
        _rotations = rotations;
    } else {
        _rotations = std::make_shared<std::unordered_map<std::string, std::shared_ptr<Joint>>>();
    }
    if (obstacles) {
        _obstacles = obstacles;
    } else {
        _obstacles = std::make_shared<std::vector<std::shared_ptr<Obstacle>>>();
    }
}

void Robot::addJoint(const std::string& name, std::shared_ptr<Joint> articulation) {
    (*_rotations)[name] = articulation;
    _localRotations.push_back(articulation);
}

void Robot::addObstacle(std::shared_ptr<Obstacle> obstacle) {
    _obstacles->push_back(obstacle);
    _localObstacles.push_back(obstacle);
}

void Robot::addChild(std::shared_ptr<Robot> child) {
    _children.push_back(child);
}

void Robot::rotate(const Eigen::Vector3d& point, const Eigen::AngleAxisd& rotation) {
    Eigen::Vector3d translatedPosition = position - point;
    translatedPosition = rotation * translatedPosition;
    position = translatedPosition + point;

    orientation = rotation;

    _articulatedTransform = Eigen::Translation3d(position) * orientation;
}

void Robot::translate(const Eigen::Vector3d& translation) {
    position += translation;

    _articulatedTransform = Eigen::Translation3d(position) * orientation;
}

void Robot::update() {
    for (const auto& articulation : _localRotations) {
        Eigen::AngleAxisd rotation(articulation->current, articulation->axis);
        rotate(articulation->origin, rotation);
    }

    for (const auto& child : _children) {
        child->update();
    }
}

Eigen::Affine3d Robot::getTransform() const {
    return _fixedTransform * _articulatedTransform;
}

void Robot::setFixedTransform(const Eigen::Affine3d& transform) {
    _fixedTransform = transform;
}
