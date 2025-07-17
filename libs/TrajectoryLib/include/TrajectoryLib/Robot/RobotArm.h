#ifndef ROBOTARM_H
#define ROBOTARM_H

#include <QDebug>
#include <QFile>
#include <QGenericMatrix>
#include <QIODevice>
#include <QMatrix4x4>
#include <Qt3DCore/QAttribute>
#include <Qt3DCore/QBuffer>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QGeometry>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongAlphaMaterial>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DRender/QBlendEquation>
#include <Qt3DRender/QBlendEquationArguments>
#include <Qt3DRender/QDepthTest>
#include <Qt3DRender/QEffect>
#include <Qt3DRender/QFilterKey>
#include <Qt3DRender/QFrameGraphNode>
#include <Qt3DRender/QGraphicsApiFilter>
#include <Qt3DRender/QLayer>
#include <Qt3DRender/QLayerFilter>
#include <Qt3DRender/QMesh>
#include <Qt3DRender/QParameter>
#include <Qt3DRender/QRenderPass>
#include <Qt3DRender/QRenderPassFilter>
#include <Qt3DRender/QRenderSettings>
#include <Qt3DRender/QRenderStateSet>
#include <Qt3DRender/QShaderProgram>
#include <Qt3DRender/QSortPolicy>
#include <Qt3DRender/QTechnique>
#include <Qt3DRender/QTechniqueFilter>
#include <Qt3DRender/QTexture>
#include <Qt3DRender/QViewport>
#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "GeometryLib/Obstacle.h"
#include "TrajectoryLib/Core/Util.h"

using namespace boost::property_tree;
namespace bg = boost::geometry;

typedef boost::geometry::model::point<double, 6, boost::geometry::cs::cartesian> TaskState;
typedef boost::geometry::model::point<double, 7, boost::geometry::cs::cartesian> JointState;

/**
 * @brief Represents a robotic arm with URDF parsing and kinematics capabilities
 * 
 * The RobotArm class provides comprehensive functionality for robot arm manipulation,
 * including URDF parsing, forward/inverse kinematics, collision detection,
 * visualization, and joint state management.
 */
class RobotArm {

public:
    /**
     * @brief Represents a link in the robot arm kinematic chain
     * 
     * Contains geometric, visual, and physical properties of a robot link
     * including mesh information, bounding boxes, and inertial properties.
     */
    struct Link
    {
        std::string name;                          ///< Name of the link
        std::string mesh_file;                     ///< Path to the mesh file for visualization
        Eigen::Vector3d translation;               ///< Translation offset from parent
        Eigen::Vector3d rotation;                  ///< Rotation offset (RPY) from parent
        Eigen::Vector3d _minBBox;                  ///< Minimum bounding box corner
        Eigen::Vector3d _maxBBox;                  ///< Maximum bounding box corner
        bool _bBox;                                ///< Whether bounding box is computed

        std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>> obbs; ///< Oriented bounding boxes

        double mass;                               ///< Link mass (kg)
        Eigen::Matrix3d inertia;                   ///< Inertia tensor
        Eigen::Vector3d center_of_mass;            ///< Center of mass position

        Link()
            : _bBox(false)
            , mass(0.0)
            , inertia(Eigen::Matrix3d::Zero())
            , center_of_mass(Eigen::Vector3d::Zero())
        {}
        Link(const Link &other) = default;
        Link &operator=(const Link &other) = default;
        ~Link() = default;
    };

    /**
     * @brief Represents a joint connecting two links in the robot arm
     * 
     * Contains kinematic and dynamic properties of a robot joint including
     * transformation parameters, joint limits, and current state.
     */
    struct Joint
    {
        std::string name;                          ///< Name of the joint
        std::string type;                          ///< Joint type (revolute, prismatic, etc.)
        std::shared_ptr<Link> parent;              ///< Parent link in kinematic chain
        std::shared_ptr<Link> child;               ///< Child link in kinematic chain
        Eigen::Vector3d origin_translation;        ///< Translation from parent to joint origin
        Eigen::Vector3d origin_rotation;           ///< Rotation (RPY) from parent to joint origin
        Eigen::Vector3d axis;                      ///< Joint axis of rotation/translation
        double angle;                              ///< Current joint angle/position
        double limit_min;                          ///< Minimum joint limit
        double limit_max;                          ///< Maximum joint limit

        double effort_limit;                       ///< Maximum effort/torque limit
        double velocity_limit;                     ///< Maximum velocity limit

        Joint()
            : angle(0.0)
            , limit_min(-std::numeric_limits<double>::infinity())
            , limit_max(std::numeric_limits<double>::infinity())
            , effort_limit(std::numeric_limits<double>::infinity())
            , velocity_limit(std::numeric_limits<double>::infinity())
        {}
        Joint(const Joint &other) = default;
        Joint &operator=(const Joint &other) = default;
        ~Joint() = default;
    };

    /**
     * @brief Construct robot arm from URDF file
     * @param urdf_file Path to the URDF file
     */
    RobotArm(const std::string& urdf_file);
    
    /**
     * @brief Copy constructor
     * @param other RobotArm to copy from
     */
    RobotArm(const RobotArm& other);
    
    /**
     * @brief Default constructor
     */
    RobotArm() = default;


    /**
     * @brief Parses a URDF (Unified Robot Description Format) file and initializes the robot arm.
     *
     * This function reads the specified URDF file, extracts the robot's structural information,
     * and populates the internal data structures of the RobotArm class. It sets up the links,
     * joints, and their properties as described in the URDF file.
     *
     * @param urdf_file A string containing the path to the URDF file to be parsed.
     *
     * @throws std::runtime_error If there's an error reading or parsing the URDF file.
     *
     * @note This function does not return a value. The parsed information is stored
     *       internally within the RobotArm object.
     */
    void parseURDF(const std::string& urdf_file);
    
    /**
     * @brief Compute transformation matrices for all links
     * @return Map of link names to their transformation matrices
     */
    std::unordered_map<std::string, Eigen::Affine3d> computeLinkTransformations() const;
    
    /**
     * @brief Get all links in the robot arm
     * @return Vector of shared pointers to Link objects
     */
    std::vector<std::shared_ptr<Link>> getLinks() const;
    
    /**
     * @brief Get line segments representing link bounding boxes
     * @return Vector of pairs representing line segments
     */
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> getLinkBoundingBoxLineSegments() const;
    
    /**
     * @brief Get oriented bounding boxes for all links
     * @return Vector of tuples containing center, half-extents, and orientation
     */
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>> getLinkBoundingBoxes() const;

    /**
     * @brief Move end-effector to specified Cartesian position and orientation
     * @param point Target position
     * @param axes Target orientation matrix
     * @return True if movement successful
     */
    bool moveToCartesian(const Eigen::Vector3d& point, const Eigen::Matrix3d& axes);
    
    /**
     * @brief Move end-effector to specified Cartesian position and orientation
     * @param point Target position
     * @param angles Target orientation as Euler angles
     * @return True if movement successful
     */
    bool moveToCartesian(const Eigen::Vector3d& point, const Eigen::Vector3d& angles);

    /**
     * @brief Get current joint angles
     * @return 7x1 matrix of joint angles in radians
     */
    Eigen::Matrix<double, 7, 1> getJointAngles() const;
    
    /**
     * @brief Get position of specified joint
     * @param joint_idx Index of the joint
     * @return 3D position of the joint
     */
    Eigen::Vector3d getJointPosition(int joint_idx) const;

    /**
     * @brief Set all joint angles simultaneously
     * @param angles 7x1 matrix of joint angles in radians
     */
    void setJointAngles(const Eigen::Matrix<double, 7, 1>& angles);
    
    /**
     * @brief Set angle for a specific joint by name
     * @param joint_name Name of the joint
     * @param angle Angle in radians
     */
    void setJointAngle(const std::string& joint_name, double angle);

    // Visualization methods
    /**
     * @brief Create Qt3D entities for robot visualization
     * @param rootEntity Root entity to attach robot visualization to
     */
    void createEntities(Qt3DCore::QEntity *rootEntity);
    
    /**
     * @brief Update visualization entities with current robot state
     */
    void updateEntities();
    
    /**
     * @brief Delete visualization entities
     */
    void deleteEntities();

    /**
     * @brief Assignment operator
     * @param other RobotArm to assign from
     * @return Reference to this robot arm
     */
    RobotArm& operator=(const RobotArm& other);

    /**
     * @brief Check if any joint limits are violated
     * @return True if joint limits are violated
     */
    bool isViolating() const;

    /**
     * @brief Get current task space state
     * @param limits Task space limits for normalization
     * @return TaskState representing current end-effector pose
     */
    TaskState getTaskState(const Eigen::Matrix<double, 6, 1>& limits) const;
    
    /**
     * @brief Get current joint space state
     * @return JointState representing current joint configuration
     */
    JointState getJointState() const;

    /**
     * @brief Move robot to specified task space state
     * @param state Target task space configuration
     * @param limits Task space limits
     * @return True if movement successful
     */
    bool moveToTaskState(const TaskState& state, const Eigen::Matrix<double, 6, 1> &limits);
    
    /**
     * @brief Move robot to specified joint space state
     * @param state Target joint configuration
     * @return True if movement successful
     */
    bool moveToJointState(const JointState& state);
    
    /**
     * @brief Get current end-effector pose
     * @return Affine transformation representing end-effector pose
     */
    Eigen::Affine3d getEndeffectorPose() const;

    /**
     * @brief Get rotation axis for specified joint
     * @param joint_idx Index of the joint
     * @return 3D vector representing joint axis
     */
    Eigen::Vector3d getJointAxis(int joint_idx) const;

    std::vector<std::shared_ptr<Joint>> _joints;
    std::vector<std::pair<double, double>> _jointLimits;
    void computeJacobian(Eigen::MatrixXd &J) const;

    Eigen::Affine3d getEndeffectorTransform();

    std::vector<std::pair<double, double>> jointLimits() const;
    void setJointLimits(const std::vector<std::pair<double, double>> &newJointLimits);
    void drawLinkedBoundingBoxes(Qt3DCore::QEntity *rootEntity);

    void createEntitiesOpaque(Qt3DCore::QEntity *rootEntity);

    double computeManipulability() const;

    Eigen::Matrix<double, 7, 1> getEffortLimits() const;

    Eigen::Matrix<double, 7, 1> getVelocityLimits() const;

    std::vector<Eigen::Matrix3d> getLinkInertias() const;

    std::vector<Eigen::Vector3d> getLinkCentersOfMass() const;

    std::vector<double> getLinkMasses() const;

    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>> getCollisionBoxes()
        const;

    Eigen::Matrix3d getJointRotation(int joint_idx) const;

    void createGhostEntities(Qt3DCore::QEntity *rootEntity);

    void deleteGhostEntities();

    void deleteAllGhostEntities();

private:
    std::vector<std::shared_ptr<Link>> _links;
    std::unordered_map<std::string, QSharedPointer<Qt3DCore::QEntity>> _linkEntities;

    std::vector<std::unordered_map<std::string, QSharedPointer<Qt3DCore::QEntity>>>
        _ghostEntitiesList;

    void calculateLinkTransform(const std::shared_ptr<Joint> &joint,
                                const Eigen::Affine3d &parent_transform);
    void computeBBoxLimits();
    void populateEmptyLinks();
};

#endif // ROBOTARM_H
