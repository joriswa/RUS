#ifndef ROBOT_H
#define ROBOT_H

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <Eigen/Dense>
#include "GeometryLib/Obstacle.h"

struct Joint
{
    std::string name;
    Eigen::Vector3d origin;
    Eigen::Vector3d axis;
    double min_range;
    double max_range;
    double current;

    Joint(const std::string &name,
          const Eigen::Vector3d &origin,
          const Eigen::Vector3d &axis,
          double min_range,
          double max_range)
        : name(name)
        , origin(origin)
        , axis(axis)
        , min_range(min_range)
        , max_range(max_range)
        , current(0.0)
    {}
};

class Robot
{
public:
    Robot(
        const std::string &name,
        bool isRoot = false,
        std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<Joint>>> rotations = nullptr,
        std::shared_ptr<std::vector<std::shared_ptr<Obstacle>>> obstacles = nullptr);

    std::string getName() const { return _name; }
    bool getIsRoot() const { return _isRoot; }
    const std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<Joint>>> &getJoints() const
    {
        return _rotations;
    }
    const std::shared_ptr<std::vector<std::shared_ptr<Obstacle>>>& getObstacles() const { return _obstacles; }
    const std::vector<std::shared_ptr<Obstacle>>& getLocalObstacles() const { return _localObstacles; }
    const std::vector<std::shared_ptr<Joint>> &getLocalRotationArticulations() const
    {
        return _localRotations;
    }
    const std::vector<std::shared_ptr<Robot>> &getChildren() const { return _children; }
    const std::string& getMesh() const { return _mesh; }
    void setMesh(const std::string& mesh) { _mesh = mesh; }

    void addJoint(const std::string &name, std::shared_ptr<Joint> articulation);
    void addObstacle(std::shared_ptr<Obstacle> obstacle);
    void addChild(std::shared_ptr<Robot> child);

    void rotate(const Eigen::Vector3d& point, const Eigen::AngleAxisd& rotation);
    void translate(const Eigen::Vector3d& translation);
    void update();

    Eigen::Affine3d getTransform() const;

    void setFixedTransform(const Eigen::Affine3d& transform);

private:
    std::string _name;
    bool _isRoot;
    std::string _mesh;
    std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<Joint>>> _rotations;
    std::vector<std::shared_ptr<Joint>> _localRotations;
    std::shared_ptr<std::vector<std::shared_ptr<Obstacle>>> _obstacles;
    std::vector<std::shared_ptr<Obstacle>> _localObstacles;
    std::vector<std::shared_ptr<Robot>> _children;

    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();

    Eigen::Affine3d _fixedTransform = Eigen::Affine3d::Identity();
    Eigen::Affine3d _articulatedTransform = Eigen::Affine3d::Identity();
};

#endif // ROBOT_H
