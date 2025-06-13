#include "TrajectoryLib/Robot/RobotArm.h"

RobotArm::RobotArm(const std::string& urdf_file) {
    parseURDF(urdf_file);
    this->computeBBoxLimits();
    this->populateEmptyLinks();
}

void RobotArm::parseURDF(const std::string &urdf_file)
{
    ptree pt;
    read_xml(urdf_file, pt);
    std::unordered_map<std::string, std::shared_ptr<Link>> link_map;

    for (const auto &link_node : pt.get_child("robot")) {
        if (link_node.first == "link") {
            if (link_node.second.get<std::string>("<xmlattr>.name") == "world")
                continue;

            auto robot_link = std::make_shared<Link>();
            robot_link->name = link_node.second.get<std::string>("<xmlattr>.name");

            auto visual = link_node.second.get_child_optional("visual");
            if (visual) {
                auto geometry = visual->get_child_optional("geometry");
                if (geometry) {
                    auto mesh = geometry->get_child_optional("mesh");
                    if (mesh) {
                        robot_link->mesh_file = mesh->get<std::string>("<xmlattr>.filename");
                    }
                }
                auto origin = visual->get_child_optional("origin");
                if (origin) {
                    std::istringstream(origin->get<std::string>("<xmlattr>.xyz", "0 0 0"))
                        >> robot_link->translation.x() >> robot_link->translation.y()
                        >> robot_link->translation.z();
                    std::istringstream(origin->get<std::string>("<xmlattr>.rpy", "0 0 0"))
                        >> robot_link->rotation.x() >> robot_link->rotation.y()
                        >> robot_link->rotation.z();
                }
            }

            for (const auto &collision_node : link_node.second) {
                if (collision_node.first == "collision") {
                    Eigen::Vector3d halfDims(1.0, 1.0, 1.0); // Default half dimensions
                    Eigen::Vector3d center(0, 0, 0);         // Default center
                    Eigen::Matrix3d axes
                        = Eigen::Matrix3d::Identity(); // Default orientation (identity matrix)

                    auto origin = collision_node.second.get_child_optional("origin");
                    if (origin) {
                        std::istringstream(origin->get<std::string>("<xmlattr>.xyz", "0 0 0"))
                            >> center.x() >> center.y() >> center.z();

                        Eigen::Vector3d rpy(0, 0, 0);
                        std::istringstream(origin->get<std::string>("<xmlattr>.rpy", "0 0 0"))
                            >> rpy.x() >> rpy.y() >> rpy.z();

                        axes = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())
                               * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
                               * Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());
                    }

                    auto geometry = collision_node.second.get_child_optional("geometry");
                    if (geometry) {
                        auto box = geometry->get_child_optional("box");
                        if (box) {
                            Eigen::Vector3d size;
                            std::istringstream(box->get<std::string>("<xmlattr>.size", "1 1 1"))
                                >> size.x() >> size.y() >> size.z();
                            halfDims = size / 2.0;
                            robot_link->obbs.emplace_back(center, halfDims, axes);
                        }
                    }
                }
            }

            auto inertial = link_node.second.get_child_optional("inertial");
            if (inertial) {
                robot_link->mass = inertial->get<double>("mass.<xmlattr>.value", 0.0);

                auto inertia = inertial->get_child_optional("inertia");
                if (inertia) {
                    robot_link->inertia << inertia->get<double>("<xmlattr>.ixx", 0.0),
                        inertia->get<double>("<xmlattr>.ixy", 0.0),
                        inertia->get<double>("<xmlattr>.ixz", 0.0),
                        inertia->get<double>("<xmlattr>.ixy", 0.0),
                        inertia->get<double>("<xmlattr>.iyy", 0.0),
                        inertia->get<double>("<xmlattr>.iyz", 0.0),
                        inertia->get<double>("<xmlattr>.ixz", 0.0),
                        inertia->get<double>("<xmlattr>.iyz", 0.0),
                        inertia->get<double>("<xmlattr>.izz", 0.0);
                }

                auto origin = inertial->get_child_optional("origin");
                if (origin) {
                    std::istringstream(origin->get<std::string>("<xmlattr>.xyz", "0 0 0"))
                        >> robot_link->center_of_mass.x() >> robot_link->center_of_mass.y()
                        >> robot_link->center_of_mass.z();
                }
            }

            _links.push_back(robot_link);
            link_map[robot_link->name] = robot_link;
        }
    }

    for (const auto &joint_node : pt.get_child("robot")) {
        if (joint_node.first == "joint") {
            auto robot_joint = std::make_shared<Joint>();
            if (joint_node.second.get<std::string>("<xmlattr>.name") == "panda_joint_world")
                continue;
            robot_joint->name = joint_node.second.get<std::string>("<xmlattr>.name");
            robot_joint->type = joint_node.second.get<std::string>("<xmlattr>.type");
            std::string parent_name = joint_node.second.get<std::string>("parent.<xmlattr>.link");
            std::string child_name = joint_node.second.get<std::string>("child.<xmlattr>.link");
            robot_joint->parent = link_map[parent_name];
            robot_joint->child = link_map[child_name];

            auto origin = joint_node.second.get_child_optional("origin");
            if (origin) {
                std::istringstream(origin->get<std::string>("<xmlattr>.xyz", "0 0 0"))
                    >> robot_joint->origin_translation.x() >> robot_joint->origin_translation.y()
                    >> robot_joint->origin_translation.z();
                std::istringstream(origin->get<std::string>("<xmlattr>.rpy", "0 0 0"))
                    >> robot_joint->origin_rotation.x() >> robot_joint->origin_rotation.y()
                    >> robot_joint->origin_rotation.z();
            }

            auto axis = joint_node.second.get_child_optional("axis");
            if (axis) {
                std::istringstream(axis->get<std::string>("<xmlattr>.xyz", "0 0 0"))
                    >> robot_joint->axis.x() >> robot_joint->axis.y() >> robot_joint->axis.z();
            } else {
                robot_joint->axis = Eigen::Vector3d(0, 0, 0);
            }

            auto limit = joint_node.second.get_child_optional("limit");
            if (limit) {
                robot_joint->limit_min = limit->get<double>("<xmlattr>.lower",
                                                            -std::numeric_limits<double>::infinity());
                robot_joint->limit_max = limit->get<double>("<xmlattr>.upper",
                                                            std::numeric_limits<double>::infinity());
                robot_joint->effort_limit
                    = limit->get<double>("<xmlattr>.effort",
                                         std::numeric_limits<double>::infinity());
                robot_joint->velocity_limit
                    = limit->get<double>("<xmlattr>.velocity",
                                         std::numeric_limits<double>::infinity());

                if (robot_joint->type == "revolute") {
                    _jointLimits.push_back(
                        std::make_pair(robot_joint->limit_min, robot_joint->limit_max));
                }
            }

            robot_joint->angle = 0.0;
            _joints.push_back(robot_joint);
        }
    }
}

// checks if some links do not provide collision geometries and if so they use a default box geometry inferred from the visual stl
void RobotArm::populateEmptyLinks()
{
    for (auto link : _links) {
        if (link->mesh_file.empty()) {
            continue;
        }

        if (link->obbs.empty()) {
            QVector<QVector3D> vertices;

            if (!Util::loadSTLFile(QString::fromStdString(link->mesh_file), vertices)) {
                qWarning() << "Failed to load STL file.";
                return;
            }

            Eigen::Vector3d minVertex, maxVertex;
            Util::findMinMaxVertices(vertices, minVertex, maxVertex);

            Eigen::Vector3d halfDims = (maxVertex - minVertex) / 2.0;
            Eigen::Vector3d center = (minVertex + maxVertex) / 2.0;

            link->obbs.emplace_back(center, halfDims, Eigen::Matrix3d::Identity());
        }
    }
}

std::unordered_map<std::string, Eigen::Affine3d> RobotArm::computeLinkTransformations() const {
    std::unordered_map<std::string, Eigen::Affine3d> transforms;

    for (const auto& link : _links) {
        transforms[link->name] = Eigen::Affine3d::Identity();
    }

    for (const auto& joint : _joints) {
        auto& parent_transform = transforms[joint->parent->name];
        Eigen::Affine3d joint_transform = Eigen::Affine3d::Identity();

        joint_transform.translate(joint->origin_translation);
        joint_transform.rotate(Eigen::AngleAxisd(joint->origin_rotation[0], Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(joint->origin_rotation[1], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(joint->origin_rotation[2], Eigen::Vector3d::UnitZ()));
        joint_transform.rotate(Eigen::AngleAxisd(joint->angle, joint->axis));

        transforms[joint->child->name] = (parent_transform * joint_transform);
    }

    return transforms;
}

std::vector<std::shared_ptr<RobotArm::Link>> RobotArm::getLinks() const {
    return _links;
}

void RobotArm::setJointAngle(const std::string& joint_name, double angle) {
    for (auto& joint : _joints) {
        if (joint->name == joint_name) {
            joint->angle = angle;
            break;
        }
    }
}

void RobotArm::computeBBoxLimits() {
    for (auto link : this->getLinks()) {
        if(link->_bBox ||  link->mesh_file == "") continue;

        QVector<QVector3D> vertices;

        if (!Util::loadSTLFile(QString::fromStdString(link->mesh_file), vertices)) {
            qWarning() << "Failed to load STL file.";
            return;
        }

        Eigen::Vector3d minVertex, maxVertex;
        Util::findMinMaxVertices(vertices, minVertex, maxVertex);

        link->_maxBBox = maxVertex;
        link->_minBBox = minVertex;
        link->_bBox = true;
    }
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> RobotArm::getLinkBoundingBoxLineSegments()
    const
{
    auto transformations = this->computeLinkTransformations();
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> segments;

    for (auto link : this->getLinks()) {
        auto transform = transformations[link->name];
        Eigen::Vector3d min = link->_minBBox;
        Eigen::Vector3d max = link->_maxBBox;

        std::vector<Eigen::Vector3d> corners = {Eigen::Vector3d(min.x(), min.y(), min.z()),
                                                Eigen::Vector3d(max.x(), min.y(), min.z()),
                                                Eigen::Vector3d(min.x(), max.y(), min.z()),
                                                Eigen::Vector3d(max.x(), max.y(), min.z()),
                                                Eigen::Vector3d(min.x(), min.y(), max.z()),
                                                Eigen::Vector3d(max.x(), min.y(), max.z()),
                                                Eigen::Vector3d(min.x(), max.y(), max.z()),
                                                Eigen::Vector3d(max.x(), max.y(), max.z())};

        for (auto &corner : corners) {
            corner = (transform * Eigen::Vector4d(corner.x(), corner.y(), corner.z(), 1.0)).head<3>();
        }

        segments.push_back(std::make_pair(corners[0], corners[1]));
        segments.push_back(std::make_pair(corners[0], corners[2]));
        segments.push_back(std::make_pair(corners[0], corners[4]));
        segments.push_back(std::make_pair(corners[1], corners[3]));
        segments.push_back(std::make_pair(corners[1], corners[5]));
        segments.push_back(std::make_pair(corners[2], corners[3]));
        segments.push_back(std::make_pair(corners[2], corners[6]));
        segments.push_back(std::make_pair(corners[3], corners[7]));
        segments.push_back(std::make_pair(corners[4], corners[5]));
        segments.push_back(std::make_pair(corners[4], corners[6]));
        segments.push_back(std::make_pair(corners[5], corners[7]));
        segments.push_back(std::make_pair(corners[6], corners[7]));
    }

    return segments;
}

std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>> RobotArm::getLinkBoundingBoxes() const {
    auto transformations = this->computeLinkTransformations();
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>> boundingBoxes;

    for (auto link : this->getLinks()) {
        auto transform = transformations[link->name];
        Eigen::Vector3d min = link->_minBBox;
        Eigen::Vector3d max = link->_maxBBox;

        Eigen::Vector3d center = (min + max) / 2.0;
        Eigen::Vector3d halfDims = (max - min) / 2.0;

        Eigen::Matrix3d rotation = transform.matrix().block<3, 3>(0, 0);

        Eigen::Vector3d transformedCenter = (transform * Eigen::Vector4d(center.x(), center.y(), center.z(), 1.0)).head<3>();

        boundingBoxes.push_back(std::make_tuple(transformedCenter, halfDims, rotation));
    }

    return boundingBoxes;
}

RobotArm& RobotArm::operator=(const RobotArm& other) {
    if (this != &other) {
        // Create temporary RobotArm to handle deep copy
        RobotArm temp;

        for (const auto& link : other._links) {
            auto new_link = std::make_shared<Link>(*link);
            temp._links.push_back(new_link);
        }

        std::unordered_map<std::shared_ptr<Link>, std::shared_ptr<Link>> link_map;
        for (size_t i = 0; i < other._links.size(); ++i) {
            link_map[other._links[i]] = temp._links[i];
        }

        for (const auto& joint : other._joints) {
            auto new_joint = std::make_shared<Joint>(*joint);
            new_joint->parent = link_map[joint->parent];
            new_joint->child = link_map[joint->child];
            temp._joints.push_back(new_joint);
        }

        std::swap(_links, temp._links);
        std::swap(_joints, temp._joints);

        this->_jointLimits = other.jointLimits();
    }
    return *this;
}

RobotArm::RobotArm(const RobotArm& other) {
    *this = other;
}


Eigen::Matrix<double, 7, 1> RobotArm::getJointAngles() const {
    if (_joints.size() < 7) {
        throw std::runtime_error("Not enough joints in the robot arm.");
    }

    Eigen::Matrix<double, 7, 1> angles;
    int i = 0;
    for (auto joint : _joints) {
        if (joint->type == "revolute") {
            angles(i) = joint->angle;
            ++i;
        }
    }

    return angles;
}

Eigen::Matrix<double, 7, 1> RobotArm::getEffortLimits() const
{
    if (_joints.size() < 7) {
        throw std::runtime_error("Not enough joints in the robot arm.");
    }
    Eigen::Matrix<double, 7, 1> efforts;
    int i = 0;
    for (auto joint : _joints) {
        if (joint->type == "revolute") {
            efforts(i) = joint->effort_limit;
            ++i;
        }
    }
    return efforts;
}

Eigen::Matrix<double, 7, 1> RobotArm::getVelocityLimits() const
{
    if (_joints.size() < 7) {
        throw std::runtime_error("Not enough joints in the robot arm.");
    }
    Eigen::Matrix<double, 7, 1> velocities;
    int i = 0;
    for (auto joint : _joints) {
        if (joint->type == "revolute") {
            velocities(i) = joint->velocity_limit;
            ++i;
        }
    }
    return velocities;
}

void RobotArm::createEntities(Qt3DCore::QEntity *rootEntity)
{
    auto transformations = computeLinkTransformations();
    auto bboxes = getLinkBoundingBoxes();
    std::vector<Qt3DCore::QEntity *> entities;
    for (auto link : _links) {
        if (link->mesh_file.empty())
            continue;

        auto transformation = transformations[link->name];

        Qt3DCore::QEntity *linkEntity = new Qt3DCore::QEntity(rootEntity);

        Qt3DCore::QGeometry *geometry = Util::loadSTL(QString::fromStdString(link->mesh_file));
        auto *mesh = new Qt3DRender::QGeometryRenderer();
        mesh->setPrimitiveType(Qt3DRender::QGeometryRenderer::Triangles);
        mesh->setGeometry(geometry);

        Qt3DCore::QTransform *transform = new Qt3DCore::QTransform();

        Eigen::Isometry3d tr;
        tr.setIdentity();
        tr.translate(transformation.translation());
        tr.rotate(transformation.rotation());

        QMatrix4x4 matrix;
        matrix.setToIdentity();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                matrix(i, j) = tr(i, j);
            }
        }
        transform->setMatrix(matrix);

        //        Qt3DExtras::QPhongAlphaMaterial *material = new Qt3DExtras::QPhongAlphaMaterial();
        Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
        material->setDiffuse(QColor(255, 255, 255));
        //        material->setAlpha(0.5);

        linkEntity->addComponent(mesh);
        linkEntity->addComponent(transform);
        linkEntity->addComponent(material);

        _linkEntities[link->name] = QSharedPointer<Qt3DCore::QEntity>(linkEntity);
    }
}

void RobotArm::createEntitiesOpaque(Qt3DCore::QEntity *rootEntity)
{
    auto transformations = computeLinkTransformations();
    auto bboxes = getLinkBoundingBoxes();
    std::vector<Qt3DCore::QEntity *> entities;
    for (auto link : _links) {
        if (link->mesh_file.empty())
            continue;

        auto transformation = transformations[link->name];

        Qt3DCore::QEntity *linkEntity = new Qt3DCore::QEntity(rootEntity);

        Qt3DCore::QGeometry *geometry = Util::loadSTL(QString::fromStdString(link->mesh_file));
        auto *mesh = new Qt3DRender::QGeometryRenderer();
        mesh->setPrimitiveType(Qt3DRender::QGeometryRenderer::Triangles);
        mesh->setGeometry(geometry);

        Qt3DCore::QTransform *transform = new Qt3DCore::QTransform();

        Eigen::Isometry3d tr;
        tr.setIdentity();
        tr.translate(transformation.translation());
        tr.rotate(transformation.rotation());

        QMatrix4x4 matrix;
        matrix.setToIdentity();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                matrix(i, j) = tr(i, j);
            }
        }
        transform->setMatrix(matrix);

        //        Qt3DExtras::QPhongAlphaMaterial *material = new Qt3DExtras::QPhongAlphaMaterial();
        Qt3DExtras::QPhongAlphaMaterial *material = new Qt3DExtras::QPhongAlphaMaterial();
        material->setDiffuse(QColor(255, 255, 255));
        //        material->setAlpha(0.5);
        material->setAlpha(0.5);

        linkEntity->addComponent(mesh);
        linkEntity->addComponent(transform);
        linkEntity->addComponent(material);

        _linkEntities[link->name] = QSharedPointer<Qt3DCore::QEntity>(linkEntity);
    }
}

void RobotArm::updateEntities() {
    auto transformations = computeLinkTransformations();
    for (const auto &link : _links) {
        if (link->mesh_file.empty())
            continue;
        if (link->name == "end_effector_link")
            continue;
        if (link->name == "panda_link8")
            continue;
        auto transformation = transformations[link->name];
        Qt3DCore::QTransform *transform
            = _linkEntities[link->name]->componentsOfType<Qt3DCore::QTransform>().constFirst();
        if (transform) {
            Eigen::Isometry3d tr;
            tr.setIdentity();
            tr.translate(transformation.translation());
            tr.rotate(transformation.rotation());

            QMatrix4x4 matrix;
            matrix.setToIdentity();
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    matrix(i, j) = tr(i, j);
                }
            }

            transform->setMatrix(matrix);
        }
    }
}

void RobotArm::setJointAngles(const Eigen::Matrix<double, 7, 1>& angles) {
    if (angles.size() != 7) {
        throw std::invalid_argument("The matrix must have exactly 7 entries.");
    }

    int i = 0;
    for (auto joint : _joints) {
        if (joint->type == "revolute") {
            joint->angle = angles(i);
            ++i;
        }
    }
}

void RobotArm::deleteEntities() {
    _linkEntities.clear();
}

bool RobotArm::isViolating() const
{
    for (const auto &joint : _joints) {
        if (joint->angle < joint->limit_min || joint->angle > joint->limit_max) {
            return true;
        }
    }
    return false;
}

Eigen::Matrix3d RobotArm::getJointRotation(int joint_idx) const
{
    if (joint_idx < 0 || joint_idx >= _joints.size()) {
        throw std::out_of_range("Joint index out of range");
    }

    std::unordered_map<std::string, Eigen::Affine3d> link_transforms;

    for (const auto &link : _links) {
        link_transforms[link->name] = Eigen::Affine3d::Identity();
    }

    int i = 0;
    for (const auto &joint : _joints) {
        auto &parent_transform = link_transforms[joint->parent->name];
        Eigen::Affine3d joint_transform = Eigen::Affine3d::Identity();

        joint_transform.translate(joint->origin_translation);
        joint_transform.rotate(
            Eigen::AngleAxisd(joint->origin_rotation[0], Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(joint->origin_rotation[1], Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(joint->origin_rotation[2], Eigen::Vector3d::UnitZ()));

        joint_transform.rotate(Eigen::AngleAxisd(joint->angle, joint->axis));

        link_transforms[joint->child->name] = parent_transform * joint_transform;

        if (i == joint_idx) {
            return (parent_transform * joint_transform).rotation(); // Extract 3x3 rotation matrix
        }
        if (joint->type == "revolute") {
            i++;
        }
    }

    return Eigen::Matrix3d::Identity(); // Return identity matrix if the index is out of range
}

Eigen::Affine3d RobotArm::getEndeffectorPose() const {
    auto transforms = computeLinkTransformations();

    auto endEffectorLink = _links.back();

    Eigen::Affine3d endeffectorTransform = Eigen::Affine3d::Identity();

    endeffectorTransform.translate(endEffectorLink->translation);
    endeffectorTransform.rotate(
        Eigen::AngleAxisd(endEffectorLink->rotation[0], Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(endEffectorLink->rotation[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(endEffectorLink->rotation[2], Eigen::Vector3d::UnitZ()));

    Eigen::Affine3d transform(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

    return transforms[endEffectorLink->name] * endeffectorTransform * transform;
}

TaskState RobotArm::getTaskState(const Eigen::Matrix<double, 6, 1>& limits) const {
    TaskState s;

    Eigen::Affine3d pose = getEndeffectorPose();
    auto translation = pose.translation();
    auto rotation = pose.linear().eulerAngles(0, 1, 2);

    double min_x = limits[0];
    double max_x = limits[1];
    double min_y = limits[2];
    double max_y = limits[3];
    double min_z = limits[4];
    double max_z = limits[5];

    bg::set<0>(s, (translation[0] - min_x) / (max_x - min_x));
    bg::set<1>(s, (translation[1] - min_y) / (max_y - min_y));
    bg::set<2>(s, (translation[2] - min_z) / (max_z - min_z));

    bg::set<3>(s, (rotation[0] + M_PI) / (2.0 * M_PI));
    bg::set<4>(s, (rotation[1] + M_PI) / (2.0 * M_PI));
    bg::set<5>(s, (rotation[2] + M_PI) / (2.0 * M_PI));

    return s;
}

JointState RobotArm::getJointState() const {
    auto q = getJointAngles();

    for (unsigned int i = 0; i < 7; ++i) {
        q[i] = (q[i] - std::get<0>(_jointLimits[i])) / (std::get<1>(_jointLimits[i]) - std::get<0>(_jointLimits[i]));
    }

    JointState s;

    bg::set<0>(s, q[0]);
    bg::set<1>(s, q[1]);
    bg::set<2>(s, q[2]);
    bg::set<3>(s, q[3]);
    bg::set<4>(s, q[4]);
    bg::set<5>(s, q[5]);
    bg::set<6>(s, q[6]);

    return s;
}

bool RobotArm::moveToJointState(const JointState &state)
{

    Eigen::Matrix<double, 7, 1> q;
    q << bg::get<0>(state), bg::get<1>(state), bg::get<2>(state), bg::get<3>(state), bg::get<4>(state), bg::get<5>(state), bg::get<6>(state);

    for (unsigned int i = 0; i < 7; ++i) {
        q[i] = q[i] * (std::get<1>(_jointLimits[i]) - std::get<0>(_jointLimits[i])) + std::get<0>(_jointLimits[i]);
    }

    setJointAngles(q);

    return !isViolating();
}

Eigen::Vector3d RobotArm::getJointPosition(int joint_idx) const {
    if (joint_idx < 0 || joint_idx >= _joints.size()) {
        throw std::out_of_range("Joint index out of range");
    }

    std::unordered_map<std::string, Eigen::Affine3d> link_transforms;

    for (const auto& link : _links) {
        link_transforms[link->name] = Eigen::Affine3d::Identity();
    }

    int i = 0;
    for (const auto &joint : _joints) {
        auto& parent_transform = link_transforms[joint->parent->name];
        Eigen::Affine3d joint_transform = Eigen::Affine3d::Identity();

        joint_transform.translate(joint->origin_translation);
        joint_transform.rotate(Eigen::AngleAxisd(joint->origin_rotation[0], Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(joint->origin_rotation[1], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(joint->origin_rotation[2], Eigen::Vector3d::UnitZ()));
        joint_transform.rotate(Eigen::AngleAxisd(joint->angle, joint->axis));
        link_transforms[joint->child->name] = (parent_transform * joint_transform);

        if (i == joint_idx) return link_transforms[joint->child->name].translation();
        if (joint->type == "revolute") {
            i++;
        }
    }

    return Eigen::Vector3d::UnitZ();
}

Eigen::Vector3d RobotArm::getJointAxis(int joint_idx) const {
    if (joint_idx < 0 || joint_idx >= _joints.size()) {
        throw std::out_of_range("Joint index out of range");
    }

    std::unordered_map<std::string, Eigen::Affine3d> link_transforms;

    for (const auto& link : _links) {
        link_transforms[link->name] = Eigen::Affine3d::Identity();
    }

    int i = 0;
    for (const auto &joint : _joints) {
        auto &parent_transform = link_transforms[joint->parent->name];
        Eigen::Affine3d joint_transform = Eigen::Affine3d::Identity();

        joint_transform.translate(joint->origin_translation);
        joint_transform.rotate(Eigen::AngleAxisd(joint->origin_rotation[0], Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(joint->origin_rotation[1], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(joint->origin_rotation[2], Eigen::Vector3d::UnitZ()));
        joint_transform.rotate(Eigen::AngleAxisd(joint->angle, joint->axis));
        link_transforms[joint->child->name] = (parent_transform * joint_transform);

        if (i == joint_idx) return (parent_transform * joint_transform).linear() * joint->axis;
        if (joint->type == "revolute") {
            i++;
        }
    }

    return Eigen::Vector3d::UnitZ();
}

std::vector<std::pair<double, double>> RobotArm::jointLimits() const
{
    return _jointLimits;
}

void RobotArm::setJointLimits(const std::vector<std::pair<double, double>> &newJointLimits)
{
    _jointLimits = newJointLimits;
}

void RobotArm::computeJacobian(Eigen::MatrixXd &J) const
{
    // Count only revolute joints
    int numRevoluteJoints = 0;
    for (const auto& joint : _joints) {
        if (joint->type == "revolute") {
            numRevoluteJoints++;
        }
    }
    
    J.resize(6, numRevoluteJoints);
    Eigen::Affine3d endEffectorPose = getEndeffectorPose();
    Eigen::Vector3d endEffectorPosition = endEffectorPose.translation();
    Eigen::Matrix3d endEffectorOrientation = endEffectorPose.rotation();

    int revoluteIndex = 0;
    for (int i = 0; i < _joints.size(); ++i) {
        // Only process revolute joints
        if (_joints[i]->type != "revolute") {
            continue;
        }
        
        Eigen::Vector3d jointPosition = getJointPosition(i);
        Eigen::Vector3d jointAxis = getJointAxis(i);

        Eigen::Vector3d r = endEffectorPosition - jointPosition;
        Eigen::Vector3d linearVelocity = jointAxis.cross(r);
        Eigen::Vector3d angularVelocity = jointAxis;

        J.block<3, 1>(0, revoluteIndex) = linearVelocity;
        J.block<3, 1>(3, revoluteIndex) = angularVelocity;
        
        revoluteIndex++;
    }
}

Eigen::Affine3d RobotArm::getEndeffectorTransform()
{
    auto endEffectorLink = _links.back();
    auto virtualHandJoint = _joints.back();

    Eigen::Affine3d endeffectorTransform = Eigen::Affine3d::Identity();
    Eigen::Affine3d pandaHandTransform = Eigen::Affine3d::Identity();

    endeffectorTransform.translate(endEffectorLink->translation);
    endeffectorTransform.rotate(
        Eigen::AngleAxisd(endEffectorLink->rotation[0], Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(endEffectorLink->rotation[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(endEffectorLink->rotation[2], Eigen::Vector3d::UnitZ()));

    pandaHandTransform.translate(virtualHandJoint->origin_translation);
    pandaHandTransform.rotate(
        Eigen::AngleAxisd(virtualHandJoint->origin_rotation[0], Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(virtualHandJoint->origin_rotation[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(virtualHandJoint->origin_rotation[2], Eigen::Vector3d::UnitZ()));

    Eigen::Affine3d transform(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

    return pandaHandTransform * endeffectorTransform * transform;
}

void RobotArm::drawLinkedBoundingBoxes(Qt3DCore::QEntity* rootEntity) {
    auto bboxes = getLinkBoundingBoxes();

    int i = 0;
    for (const auto& bbox : bboxes) {
        ++i;
        auto [center, halfDims, axes] = bbox;

        Qt3DCore::QEntity* boxEntity = new Qt3DCore::QEntity(rootEntity);

        Qt3DExtras::QCuboidMesh* mesh = new Qt3DExtras::QCuboidMesh();
        mesh->setXExtent(2 * halfDims.x());
        mesh->setYExtent(2 * halfDims.y());
        mesh->setZExtent(2 * halfDims.z());

        Qt3DCore::QTransform* transform = new Qt3DCore::QTransform();
        QMatrix4x4 transformMatrix;
        transformMatrix.translate(center.x(), center.y(), center.z());

        // Create QMatrix3x3 from Eigen::Matrix3d
        QMatrix3x3 rotationMatrix;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotationMatrix(i, j) = axes(i, j);
            }
        }

        transformMatrix.rotate(QQuaternion::fromRotationMatrix(rotationMatrix));
        transform->setMatrix(transformMatrix);

        // Create and set up material
        Qt3DExtras::QPhongMaterial* material = new Qt3DExtras::QPhongMaterial();
        material->setAmbient(QColor(255, 0, 0, 128));  // Semi-transparent red

        // Add components to the entity
        boxEntity->addComponent(mesh);
        boxEntity->addComponent(transform);
        boxEntity->addComponent(material);

    }
}

double RobotArm::computeManipulability()
{
    double product = 1.0;
    int num_joints = _joints.size();
    double k = 1e5;

    for (int j = 0; j < num_joints; ++j) {
        if (_joints[j]->type == "revolute") {
            double theta_j = _joints[j]->angle;
            double l_j_minus = _jointLimits[j].first;
            double l_j_plus = _jointLimits[j].second;

            double normalized_term = (theta_j - l_j_minus) * (l_j_plus - theta_j)
                                     / std::pow((l_j_plus - l_j_minus), 2);

            product *= normalized_term;
        }
    }

    double manipulability = std::exp(-k * product);

    return manipulability;
}

std::vector<Eigen::Matrix3d> RobotArm::getLinkInertias() const
{
    auto transformations = this->computeLinkTransformations();
    std::vector<Eigen::Matrix3d> inertias;
    for (auto link : this->getLinks()) {
        auto transform = transformations[link->name];
        Eigen::Matrix3d rotation = transform.matrix().block<3, 3>(0, 0);
        Eigen::Matrix3d transformedInertia = rotation * link->inertia * rotation.transpose();
        inertias.push_back(transformedInertia);
    }
    return inertias;
}

std::vector<Eigen::Vector3d> RobotArm::getLinkCentersOfMass() const
{
    auto transformations = this->computeLinkTransformations();
    std::vector<Eigen::Vector3d> centersOfMass;
    for (auto link : this->getLinks()) {
        if (link->name.substr(0, 10) == "panda_link" && link->name[10] >= '1'
            && link->name[10] <= '7') {
            auto transform = transformations[link->name];
            Eigen::Vector3d transformedCoM = transform * link->center_of_mass;
            centersOfMass.push_back(transformedCoM);
        }
    }
    return centersOfMass;
}

std::vector<double> RobotArm::getLinkMasses() const
{
    std::vector<double> masses;
    for (auto link : this->getLinks()) {
        if (link->name.substr(0, 10) == "panda_link" && link->name[10] >= '1'
            && link->name[10] <= '7') {
            masses.push_back(link->mass);
        }
    }
    return masses;
}

std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>>
RobotArm::getCollisionBoxes() const
{
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>> transformedObbs;

    auto transformations = computeLinkTransformations();

    for (const auto &link : _links) {
        Eigen::Affine3d link_transform = transformations.at(link->name);

        for (const auto &obb : link->obbs) {
            Eigen::Vector3d center, halfDims;
            Eigen::Matrix3d axes;

            std::tie(center, halfDims, axes) = obb;
            Eigen::Vector3d transformedCenter = link_transform * center;
            Eigen::Matrix3d transformedAxes = link_transform.rotation() * axes;
            transformedObbs.emplace_back(transformedCenter, halfDims, transformedAxes);
        }
    }

    return transformedObbs;
}

void RobotArm::createGhostEntities(Qt3DCore::QEntity *rootEntity)
{
    std::unordered_map<std::string, QSharedPointer<Qt3DCore::QEntity>> ghostEntities;
    auto transformations = computeLinkTransformations();

    for (auto link : _links) {
        if (link->mesh_file.empty())
            continue;

        auto transformation = transformations[link->name];

        Qt3DCore::QEntity *ghostEntity = new Qt3DCore::QEntity(rootEntity);

        Qt3DCore::QGeometry *geometry = Util::loadSTL(QString::fromStdString(link->mesh_file));
        auto *mesh = new Qt3DRender::QGeometryRenderer();
        mesh->setPrimitiveType(Qt3DRender::QGeometryRenderer::Triangles);
        mesh->setGeometry(geometry);

        Qt3DCore::QTransform *transform = new Qt3DCore::QTransform();

        Eigen::Isometry3d tr;
        tr.setIdentity();
        tr.translate(transformation.translation());
        tr.rotate(transformation.rotation());

        QMatrix4x4 matrix;
        matrix.setToIdentity();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                matrix(i, j) = tr(i, j);
            }
        }
        transform->setMatrix(matrix);

        Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
        material->setDiffuse(QColor(255, 255, 255));

        ghostEntity->addComponent(mesh);
        ghostEntity->addComponent(transform);
        ghostEntity->addComponent(material);

        ghostEntities[link->name] = QSharedPointer<Qt3DCore::QEntity>(ghostEntity);
    }

    _ghostEntitiesList.push_back(ghostEntities);
}

void RobotArm::deleteAllGhostEntities()
{
    _ghostEntitiesList.clear();
}
