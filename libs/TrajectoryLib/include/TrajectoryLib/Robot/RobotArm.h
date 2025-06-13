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


class RobotArm {

public:
    struct Link
    {
        std::string name;
        std::string mesh_file;
        Eigen::Vector3d translation;
        Eigen::Vector3d rotation;
        Eigen::Vector3d _minBBox;
        Eigen::Vector3d _maxBBox;
        bool _bBox;

        std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>> obbs;

        double mass;
        Eigen::Matrix3d inertia;
        Eigen::Vector3d center_of_mass;

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

    struct Joint
    {
        std::string name;
        std::string type;
        std::shared_ptr<Link> parent;
        std::shared_ptr<Link> child;
        Eigen::Vector3d origin_translation;
        Eigen::Vector3d origin_rotation;
        Eigen::Vector3d axis;
        double angle;
        double limit_min;
        double limit_max;

        double effort_limit;
        double velocity_limit;

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

    RobotArm(const std::string& urdf_file);
    RobotArm(const RobotArm& other);
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
    std::unordered_map<std::string, Eigen::Affine3d> computeLinkTransformations() const;
    std::vector<std::shared_ptr<Link>> getLinks() const;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> getLinkBoundingBoxLineSegments() const;
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>> getLinkBoundingBoxes() const;

    bool moveToCartesian(const Eigen::Vector3d& point, const Eigen::Matrix3d& axes);
    bool moveToCartesian(const Eigen::Vector3d& point, const Eigen::Vector3d& angles);

    Eigen::Matrix<double, 7, 1> getJointAngles() const;
    Eigen::Vector3d getJointPosition(int joint_idx) const;

    void setJointAngles(const Eigen::Matrix<double, 7, 1>& angles);
    void setJointAngle(const std::string& joint_name, double angle);

    void createEntities(Qt3DCore::QEntity *rootEntity);
    void updateEntities();
    void deleteEntities();

    RobotArm& operator=(const RobotArm& other);

    bool isViolating() const;

    TaskState getTaskState(const Eigen::Matrix<double, 6, 1>& limits) const;
    JointState getJointState() const;

    bool moveToTaskState(const TaskState& state, const Eigen::Matrix<double, 6, 1> &limits);
    bool moveToJointState(const JointState& state);
    Eigen::Affine3d getEndeffectorPose() const;

    Eigen::Vector3d getJointAxis(int joint_idx) const;

    std::vector<std::shared_ptr<Joint>> _joints;
    std::vector<std::pair<double, double>> _jointLimits;
    void computeJacobian(Eigen::MatrixXd &J) const;

    Eigen::Affine3d getEndeffectorTransform();

    std::vector<std::pair<double, double>> jointLimits() const;
    void setJointLimits(const std::vector<std::pair<double, double>> &newJointLimits);
    void drawLinkedBoundingBoxes(Qt3DCore::QEntity *rootEntity);

    void createEntitiesOpaque(Qt3DCore::QEntity *rootEntity);

    double computeManipulability();

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
