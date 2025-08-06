#ifndef ROBOT_H
#define ROBOT_H

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <Eigen/Dense>
#include "GeometryLib/Obstacle.h"

/**
 * @brief Represents a robotic joint with kinematic properties
 * 
 * A joint defines a single degree of freedom in the robot's kinematic chain,
 * including its position, axis of rotation, and range limits.
 */
struct Joint
{
    std::string name;           ///< Name identifier of the joint
    Eigen::Vector3d origin;     ///< Origin point of the joint in 3D space
    Eigen::Vector3d axis;       ///< Axis of rotation for the joint
    double min_range;           ///< Minimum rotation angle (radians)
    double max_range;           ///< Maximum rotation angle (radians)
    double current;             ///< Current rotation angle (radians)

    /**
     * @brief Construct a new Joint object
     * @param name Joint identifier name
     * @param origin Origin point of the joint
     * @param axis Rotation axis vector
     * @param min_range Minimum rotation limit
     * @param max_range Maximum rotation limit
     */
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

/**
 * @brief Represents a robotic system with kinematics and collision geometry
 * 
 * The Robot class encapsulates the kinematic structure, joints, collision geometry,
 * and hierarchical relationships of a robotic system. It supports articulated
 * transformations and maintains both local and shared state.
 */
class Robot
{
public:
    /**
     * @brief Construct a new Robot object
     * @param name Unique identifier for the robot
     * @param isRoot Whether this robot is the root of a kinematic chain
     * @param rotations Shared map of joint rotations (optional)
     * @param obstacles Shared vector of collision obstacles (optional)
     */
    Robot(
        const std::string &name,
        bool isRoot = false,
        std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<Joint>>> rotations = nullptr,
        std::shared_ptr<std::vector<std::shared_ptr<Obstacle>>> obstacles = nullptr);

    // Getters
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

    // Modification methods
    /**
     * @brief Add a joint to this robot
     * @param name Unique name for the joint
     * @param articulation Joint object to add
     */
    void addJoint(const std::string &name, std::shared_ptr<Joint> articulation);
    
    /**
     * @brief Add a collision obstacle to this robot
     * @param obstacle Obstacle object to add
     */
    void addObstacle(std::shared_ptr<Obstacle> obstacle);
    
    /**
     * @brief Add a child robot to this robot
     * @param child Child robot to add
     */
    void addChild(std::shared_ptr<Robot> child);

    // Transformation methods
    /**
     * @brief Rotate robot around a point
     * @param point Point to rotate around
     * @param rotation Rotation to apply
     */
    void rotate(const Eigen::Vector3d& point, const Eigen::AngleAxisd& rotation);
    
    /**
     * @brief Translate robot by a vector
     * @param translation Translation vector to apply
     */
    void translate(const Eigen::Vector3d& translation);
    
    /**
     * @brief Update robot and all children based on current joint states
     */
    void update();

    /**
     * @brief Get the combined transformation matrix
     * @return Combined fixed and articulated transformation
     */
    Eigen::Affine3d getTransform() const;

    /**
     * @brief Set the fixed base transformation
     * @param transform Fixed transformation to set
     */
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
