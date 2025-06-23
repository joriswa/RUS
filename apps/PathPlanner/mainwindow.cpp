#include "mainwindow.h"
#include <QObject>
#include <QThread>
#include <Qt3DExtras/QCylinderMesh>
#include "./ui_mainwindow.h"

#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Eigen/Geometry>

namespace POS1 {
Eigen::Vector3d pos = Eigen::Vector3d(0.3, 0.3, 0.57);
Eigen::Matrix3d Rx = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()).toRotationMatrix();
Eigen::Matrix3d Ry = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix();
Eigen::Matrix3d Rz = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
Eigen::Matrix3d orientation = Rx * Ry * Rz;

// Create and initialize the affine transformation all at once
Eigen::Affine3d pose = [] {
    Eigen::Affine3d p = Eigen::Affine3d::Identity();
    p.linear() = orientation;
    p.translation() = pos;
    return p;
}();
} // namespace POS1

namespace POS2 {
Eigen::Vector3d pos = Eigen::Vector3d(0.475, 0.325, 0.38);
Eigen::Matrix3d Rx = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()).toRotationMatrix();
Eigen::Matrix3d Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()).toRotationMatrix();
Eigen::Matrix3d Rz = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
Eigen::Matrix3d orientation = Rx * Ry * Rz;
} // namespace POS2

// Joint Angles:
//                0.683018
//                1.71984
//                -2.2246
//                -2.16331
//                0.431975
//                2.05656
//                2.69333
// Joint Angles:
//               1.46667
//               0.785627
//               -2.17631
//               -2.18057
//               -0.853356
//               1.76433
//               -2.52522

void saveTrajectoryToCSV(const std::string &filename,
                         const std::vector<MotionGenerator::TrajectoryPoint> &traj)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }
    // Write header
    file << "time,";
    for (int i = 0; i < 7; ++i) {
        file << "position_" << i << ",";
    }
    for (int i = 0; i < 7; ++i) {
        file << "velocity_" << i << ",";
    }
    for (int i = 0; i < 7; ++i) {
        file << "acceleration_" << i << (i == 7 - 1 ? "\n" : ",");
    }

    // Write data
    for (const auto &point : traj) {
        file << std::fixed << std::setprecision(6) << point.time << ",";
        for (const auto &pos : point.position) {
            file << pos << ",";
        }
        for (const auto &vel : point.velocity) {
            file << vel << ",";
        }
        for (size_t i = 0; i < point.acceleration.size(); ++i) {
            file << point.acceleration[i] << (i == point.acceleration.size() - 1 ? "\n" : ",");
        }
    }

    file.close();
    // std::cout << "Trajectory saved to " << filename << std::endl;
}

Qt3DCore::QEntity *createCoordinateFrame(const Eigen::Affine3d &affine,
                                         Qt3DCore::QEntity *rootEntity)
{
    // Convert Eigen::Affine3d to QMatrix4x4
    QMatrix4x4 qMatrix;
    Eigen::Matrix4d matrix = affine.matrix();
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            qMatrix(i, j) = static_cast<float>(matrix(i, j));

    // Create a root entity for the coordinate frame
    Qt3DCore::QEntity *frameEntity = new Qt3DCore::QEntity(rootEntity);

    // Create cylinder mesh for axes
    Qt3DExtras::QCylinderMesh *cylinderMesh = new Qt3DExtras::QCylinderMesh();
    cylinderMesh->setRadius(0.01f);
    cylinderMesh->setLength(1.0f);
    cylinderMesh->setRings(100);
    cylinderMesh->setSlices(20);

    // Create materials for each axis
    Qt3DExtras::QPhongMaterial *xMaterial = new Qt3DExtras::QPhongMaterial();
    xMaterial->setDiffuse(QColor(Qt::red));
    Qt3DExtras::QPhongMaterial *yMaterial = new Qt3DExtras::QPhongMaterial();
    yMaterial->setDiffuse(QColor(Qt::green));
    Qt3DExtras::QPhongMaterial *zMaterial = new Qt3DExtras::QPhongMaterial();
    zMaterial->setDiffuse(QColor(Qt::blue));

    // Create entities for each axis
    Qt3DCore::QEntity *xAxisEntity = new Qt3DCore::QEntity(frameEntity);
    Qt3DCore::QEntity *yAxisEntity = new Qt3DCore::QEntity(frameEntity);
    Qt3DCore::QEntity *zAxisEntity = new Qt3DCore::QEntity(frameEntity);

    // Assign meshes and materials
    xAxisEntity->addComponent(cylinderMesh);
    xAxisEntity->addComponent(xMaterial);
    yAxisEntity->addComponent(cylinderMesh);
    yAxisEntity->addComponent(yMaterial);
    zAxisEntity->addComponent(cylinderMesh);
    zAxisEntity->addComponent(zMaterial);

    // Create and set transformations for each axis
    Qt3DCore::QTransform *xTransform = new Qt3DCore::QTransform();
    Qt3DCore::QTransform *yTransform = new Qt3DCore::QTransform();
    Qt3DCore::QTransform *zTransform = new Qt3DCore::QTransform();

    // X axis: rotate 90 degrees around Z, then translate half length
    xTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), 90));
    xTransform->setTranslation(QVector3D(0.5f, 0, 0));

    // Y axis: no rotation needed, translate half length
    yTransform->setTranslation(QVector3D(0, 0.5f, 0));

    // Z axis: rotate -90 degrees around X, then translate half length
    zTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), -90));
    zTransform->setTranslation(QVector3D(0, 0, 0.5f));

    // Apply the affine transformation to all axes
    xTransform->setMatrix(qMatrix * xTransform->matrix());
    yTransform->setMatrix(qMatrix * yTransform->matrix());
    zTransform->setMatrix(qMatrix * zTransform->matrix());

    // Add the transformations to the entities
    xAxisEntity->addComponent(xTransform);
    yAxisEntity->addComponent(yTransform);
    zAxisEntity->addComponent(zTransform);

    return frameEntity;
}

Qt3DCore::QEntity *createScaledCoordinateFrame(const Eigen::Affine3d &affine,
                                               Qt3DCore::QEntity *rootEntity,
                                               float scale)
{
    QMatrix4x4 qMatrix;
    Eigen::Matrix4d matrix = affine.matrix();
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            qMatrix(i, j) = static_cast<float>(matrix(i, j));

    Qt3DCore::QEntity *frameEntity = new Qt3DCore::QEntity(rootEntity);

    Qt3DExtras::QCylinderMesh *cylinderMesh = new Qt3DExtras::QCylinderMesh();
    cylinderMesh->setRadius(0.003f);
    cylinderMesh->setLength(1.0f * scale);
    cylinderMesh->setRings(100);
    cylinderMesh->setSlices(20);

    Qt3DExtras::QPhongMaterial *xMaterial = new Qt3DExtras::QPhongMaterial();
    xMaterial->setDiffuse(QColor(Qt::red));
    Qt3DExtras::QPhongMaterial *yMaterial = new Qt3DExtras::QPhongMaterial();
    yMaterial->setDiffuse(QColor(Qt::green));
    Qt3DExtras::QPhongMaterial *zMaterial = new Qt3DExtras::QPhongMaterial();
    zMaterial->setDiffuse(QColor(Qt::blue));

    Qt3DCore::QEntity *xAxisEntity = new Qt3DCore::QEntity(frameEntity);
    Qt3DCore::QEntity *yAxisEntity = new Qt3DCore::QEntity(frameEntity);
    Qt3DCore::QEntity *zAxisEntity = new Qt3DCore::QEntity(frameEntity);

    xAxisEntity->addComponent(cylinderMesh);
    xAxisEntity->addComponent(xMaterial);
    yAxisEntity->addComponent(cylinderMesh);
    yAxisEntity->addComponent(yMaterial);
    zAxisEntity->addComponent(cylinderMesh);
    zAxisEntity->addComponent(zMaterial);

    Qt3DCore::QTransform *xTransform = new Qt3DCore::QTransform();
    Qt3DCore::QTransform *yTransform = new Qt3DCore::QTransform();
    Qt3DCore::QTransform *zTransform = new Qt3DCore::QTransform();

    xTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), 90));
    xTransform->setTranslation(QVector3D(0.5f * scale, 0, 0));
    xTransform->setMatrix(qMatrix * xTransform->matrix());

    yTransform->setTranslation(QVector3D(0, 0.5f * scale, 0));
    yTransform->setMatrix(qMatrix * yTransform->matrix());

    zTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), -90));
    zTransform->setTranslation(QVector3D(0, 0, 0.5f * scale));
    zTransform->setMatrix(qMatrix * zTransform->matrix());

    xAxisEntity->addComponent(xTransform);
    yAxisEntity->addComponent(yTransform);
    zAxisEntity->addComponent(zTransform);

    return frameEntity;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    qDebug() << "Starting PathPlanner application initialization";
    
    ui->setupUi(this);
    _robotArm = new RobotArm("/Users/joris/Uni/MA/robot_definition/panda_US.urdf");
    qDebug() << "Robot arm initialized";
    qDebug() << "starting";

    qDebug() << "Setting up 3D scene";
    setup3DScene();
    createFloor();
    qDebug() << "3D scene setup completed";

    _robotArm->createEntities(_rootEntity);
    qDebug() << "Robot entities created in 3D scene";

    Eigen::Vector3d pos = Eigen::Vector3d(0.475, 0.4, 0.5);
    Eigen::Matrix3d Rx, Ry, Rz, axes;
    Rx = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Rz = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    axes = Rx * Ry * Rz;

    _usPlanner = new UltrasoundScanTrajectoryPlanner(
        "/Users/joris/Uni/MA/robot_definition/panda_US.urdf");
    qDebug() << "Ultrasound trajectory planner initialized";

    Eigen::VectorXd angles(7);
    angles << 0.374894, -0.043533, 0.087470, -1.533429, 0.02237, 1.050135, 0.075773;

    Eigen::Matrix<double, 7, 1> jointAngles = _robotArm->getJointAngles();
    std::array<double, 7> jointAnglesArray;
    Eigen::Map<Eigen::Matrix<double, 7, 1>>(jointAnglesArray.data()) = jointAngles;
    auto showpose = POS1::pose * _robotArm->getEndeffectorTransform().inverse();

    _robotArm->setJointAngles(angles);
    _robotArm->updateEntities();
    qDebug() << "Robot arm positioned to initial configuration";

    ui->pushButton_clearEnviornment->setEnabled(false);

    Eigen::Affine3d temp;
    temp.linear() = axes;

    _robotArm->updateEntities();

    qDebug() << "Connecting UI signals and slots";
    connect(ui->pushButton_runPathfinding, &QPushButton::clicked, this, &MainWindow::findPath);

    connect(ui->checkBox_collisionBoxes,
            QOverload<bool>::of(&QCheckBox::clicked),
            this,
            &MainWindow::showBBoxes);
    connect(ui->pushButton_loadEnviornment, &QPushButton::clicked, this, &MainWindow::loadActors);
    connect(ui->pushButton_clearEnviornment, &QPushButton::clicked, this, &MainWindow::clearActors);
    connect(ui->pushButton_clearEnviornment, &QPushButton::clicked, this, &MainWindow::clearActors);
    connect(ui->pushButton_loadScanPoses, &QPushButton::clicked, this, &MainWindow::loadScanPoses);
    connect(ui->pushButton_clearScanPoses, &QPushButton::clicked, this, &MainWindow::clearScanPoses);
    connect(ui->pushButton_SimpleStomp,
            &QPushButton::clicked,
            this,
            &MainWindow::planSTOMPBetweenFirstTwoPoses);
    connect(ui->pushButton_PlottingData,
            &QPushButton::clicked,
            this,
            &MainWindow::createPlottingData);
    connect(ui->listWidget_trajectories,
            &QListWidget::itemSelectionChanged,
            this,
            &MainWindow::onTrajectorySelected);
    connect(ui->trajectorySlider,
            &QSlider::valueChanged,
            this,
            &MainWindow::onTrajectorySliderValueChanged);
    connect(ui->listWidget,
            &QListWidget::itemSelectionChanged,
            this,
            &MainWindow::onScanPoseSelected);
    connect(ui->spinBox,
            QOverload<int>::of(&QSpinBox::valueChanged),
            this,
            &MainWindow::onIterationSpinBoxValueChanged);

    auto pose = _robotArm->getEndeffectorPose();

    // auto all = _usPlanner->getPathPlanner()
    //                ->computeIK(pose * _robotArm->getEndeffectorTransform().inverse(),
    //                            _robotArm->getJointAngles()[6]);
    // Eigen::Map<const Eigen::Matrix<double, 7, 1>> ang(all[0].data());
    // _robotArm->setJointAngles(ang);
    // _robotArm->updateEntities();

    setPredefinedCamera();
    
    qDebug() << "PathPlanner application initialization completed successfully";
}

MainWindow::~MainWindow()
{
    _plannerThread->exit();
    delete ui;
}

Qt3DCore::QEntity* MainWindow::createSphereEntity(const Eigen::Vector3d &position, float radius) {
    Qt3DCore::QEntity *sphereEntity = new Qt3DCore::QEntity(_rootEntity);

    Qt3DExtras::QSphereMesh *sphereMesh = new Qt3DExtras::QSphereMesh();
    sphereMesh->setRadius(radius);

    Qt3DExtras::QPhongMaterial *sphereMaterial = new Qt3DExtras::QPhongMaterial();
    sphereMaterial->setDiffuse(QColor(QRgb(0xFF0000)));

    Qt3DCore::QTransform *sphereTransform = new Qt3DCore::QTransform();
    sphereTransform->setTranslation(QVector3D(position.x(), position.y(), position.z()));

    sphereEntity->addComponent(sphereMesh);
    sphereEntity->addComponent(sphereMaterial);
    sphereEntity->addComponent(sphereTransform);

    return sphereEntity;
}

void MainWindow::createFloor()
{
    _floorEntity = new Qt3DCore::QEntity(_rootEntity);

    // Create plane mesh
    auto planeMesh = new Qt3DExtras::QPlaneMesh(_floorEntity);
    planeMesh->setWidth(10.0f);
    planeMesh->setHeight(10.0f);

    // Create texture material
    auto textureMaterial = new Qt3DExtras::QTextureMaterial(_floorEntity);
    auto texture = new Qt3DRender::QTexture2D(textureMaterial);
    auto textureImage = new Qt3DRender::QTextureImage(texture);

    // Set the path to your PNG texture file
    textureImage->setSource(
        QUrl::fromLocalFile("/Users/joris/Downloads/prototype_512x512_grey1.png"));
    texture->addTextureImage(textureImage);
    textureMaterial->setTexture(texture);
    // Set texture properties
    texture->setWrapMode(Qt3DRender::QTextureWrapMode(Qt3DRender::QTextureWrapMode::Repeat));
    texture->setMinificationFilter(Qt3DRender::QTexture2D::LinearMipMapNearest);
    texture->setMagnificationFilter(Qt3DRender::QTexture2D::LinearMipMapNearest);

    QMatrix3x3 textureMatrix;
    textureMatrix(0, 0) = 10.0f; // Scale X
    textureMatrix(1, 1) = 10.0f; // Scale Y
    textureMatrix(2, 2) = 1.0f;  // No change in Z
    textureMaterial->setTextureTransform(textureMatrix);

    // Set up transform
    auto meshTransform = new Qt3DCore::QTransform(_floorEntity);
    meshTransform->setRotationX(90.0f);
    meshTransform->setTranslation(QVector3D(0.0f, 0.0f, 0.01f));

    // Add components to the entity
    (_floorEntity)->addComponent(planeMesh);
    (_floorEntity)->addComponent(textureMaterial);
    (_floorEntity)->addComponent(meshTransform);
}

void MainWindow::onFloorClicked(Qt3DRender::QPickEvent *event)
{
    if (event->button() == Qt3DRender::QPickEvent::MiddleButton) {
        auto rayOrigin = event->worldIntersection();
        auto rayDirection = event->worldIntersection() - _cameraEntity->position();
        rayDirection.normalize();

        QVector3D normal = QVector3D(0, 0, 1);
        QVector3D point = QVector3D(0, 0, 0);
        double t = QVector3D::dotProduct(point - rayOrigin, normal) / QVector3D::dotProduct(rayDirection, normal);
        if (t > 0) {
            QVector3D intersectionPoint = rayOrigin + t * rayDirection;
            intersectionPoint.setZ(0.5);

            if (_cylinderEntity == nullptr) {
                _cylinderEntity = new Qt3DCore::QEntity(_rootEntity);
                auto cylinderMesh = new Qt3DExtras::QCylinderMesh(_cylinderEntity);
                cylinderMesh->setRadius(0.05f);
                cylinderMesh->setLength(0.5f);
                auto cylinderMaterial = new Qt3DExtras::QPhongMaterial(_cylinderEntity);
                cylinderMaterial->setDiffuse(QColor(QRgb(0xFF0000)));
                _cylinderEntity->addComponent(cylinderMesh);
                _cylinderEntity->addComponent(cylinderMaterial);
                auto layer = new Qt3DRender::QLayer(_cylinderEntity);
                _cylinderEntity->addComponent(layer);
            }

            auto cylinderTransform = new Qt3DCore::QTransform();
            cylinderTransform->setTranslation(intersectionPoint);
            cylinderTransform->setRotationX(-90.f);

            if (_cylinderEntity != nullptr) {
                auto transforms = _cylinderEntity->componentsOfType<Qt3DCore::QTransform>();
                for (auto* transform : transforms) {
                    _cylinderEntity->removeComponent(transform);
                }
            }
            _cylinderEntity->addComponent(cylinderTransform);

            qDebug() << "Cylinder position:" << intersectionPoint;
        }
    }
}

void MainWindow::setup3DScene() {
    _view = new Qt3DExtras::Qt3DWindow();
    _container = QWidget::createWindowContainer(_view, ui->scenePlaceholder);
    _container->setMinimumSize(ui->scenePlaceholder->frameSize());
    _container->setFocusPolicy(Qt::TabFocus);

    _rootEntity = new Qt3DCore::QEntity();

    _cameraEntity = _view->camera();
    _cameraEntity->lens()->setPerspectiveProjection(45.0f, _view->width() / float(_view->height()), 0.01f, 1000.0f);
    _cameraEntity->setPosition(QVector3D(0, 1.0f, 0.0f));
    _cameraEntity->setUpVector(QVector3D(0, 0, 1));
    _cameraEntity->setViewCenter(QVector3D(0, 0, 0.5));

    _camController = new CustomCameraController(_rootEntity);
    _camController->setCamera(_cameraEntity);

    QVector3D lightPositions[] = {
        QVector3D(15.0f, 15.0f, 15.0f),
        QVector3D(-15.0f, 15.0f, 15.0f),
        QVector3D(15.0f, -15.0f, 15.0f),
        QVector3D(-15.0f, -15.0f, 15.0f)
    };

    for (const auto& position : lightPositions) {
        Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(_rootEntity);
        Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
        light->setColor("white");
        light->setIntensity(0.5);
        Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform();
        lightTransform->setTranslation(position);
        lightEntity->addComponent(light);
        lightEntity->addComponent(lightTransform);
    }

    Qt3DRender::QSortPolicy *sortPolicy = new Qt3DRender::QSortPolicy(_rootEntity);
    sortPolicy->setSortTypes(
        QList<Qt3DRender::QSortPolicy::SortType>{Qt3DRender::QSortPolicy::BackToFront});

    // Add to render settings or frame graph
    // Note: You'll need to integrate this into your frame graph structure

    _view->setRootEntity(_rootEntity);
}

void MainWindow::pathAvailable() {
    Vec3 startpose(0, 0, 1);

    // createPathEntity(_rootEntity, points);
}

//void MainWindow::pathAvailable() {
//    Vec3 startpose(0, 0, 1);
//    _pathEntities.clear();
//    auto paths = _trajectoryPlanner->getPaths();

//    for (auto path : paths) {
//        std::vector<QVector3D> points;
//        for (const auto& point : path) {
//            points.push_back(QVector3D(point.x(), point.y(), point.z()));
//        }
//        createPathEntity(_rootEntity, points);
//    }
//}

void MainWindow::setAlgorithm() {

}

void MainWindow::changeState(int i) {

}

void MainWindow::loadActors() {
    QFileInfo fileInfo(ui->lineEdit_enviornmentSource->text());

    if (!fileInfo.exists() || !fileInfo.isFile()) {
        ui->statusbar->showMessage("File does not exist or is not a file.");
        return;
    }

    if (fileInfo.suffix().toLower() != "xml") {
        ui->statusbar->showMessage("File is not an XML file.");
        return;
    }

    _actorManager.parseURDF(ui->lineEdit_enviornmentSource->text().toStdString());

    _actorManager.createEntities(_rootEntity, ui->checkBox_collisionBoxes->isChecked());

    auto obstacleTree = std::make_shared<BVHTree>(_actorManager.getTransformedObstacles());

    _usPlanner->setEnvironment(ui->lineEdit_enviornmentSource->text().toStdString());

    ui->pushButton_clearEnviornment->setEnabled(true);
    ui->pushButton_loadEnviornment->setEnabled(false);
    ui->statusbar->showMessage("Successfully loaded space description.");
}

void MainWindow::clearActors()
{
    ui->pushButton_clearEnviornment->setEnabled(false);
    ui->pushButton_loadEnviornment->setEnabled(true);
    _actorManager.clear();
    ui->statusbar->showMessage("Successfully cleared space.");
}

void MainWindow::addArticulationControls(QTreeWidgetItem *parentItem, std::shared_ptr<Robot> actor) {
    const auto& articulations = actor->getJoints();
    for (const auto& pair : *articulations) {
        auto articulation = pair.second;
        QTreeWidgetItem *articulationItem = new QTreeWidgetItem(parentItem);
        articulationItem->setText(0, QString::fromStdString(articulation->name));

        QSlider *slider = createSlider(static_cast<int>(articulation->min_range * 10),
                                       static_cast<int>(articulation->max_range * 10),
                                       static_cast<int>(articulation->current * 10));

        connect(slider, &QSlider::valueChanged, this, &MainWindow::onSliderValueChanged);
        slider->setProperty("articulation", QVariant::fromValue(articulation.get()));
    }
}

void MainWindow::findPath()
{
    qDebug() << "Starting path planning";
    
    qDebug() << "Starting path planning with current robot configuration";
    _usPlanner->setCurrentJoints(_robotArm->getJointAngles());
    
    qDebug() << "Running trajectory planning";
    _usPlanner->planTrajectories();
    qDebug() << "Trajectory planning completed";

    ui->listWidget_trajectories->clear();

    const auto &trajectories = _usPlanner->getTrajectories();
    qDebug() << "Generated" << trajectories.size() << "trajectories";
    
    for (size_t i = 0; i < trajectories.size(); ++i) {
        const auto &trajectory = trajectories[i].first;
        bool isContactForce = trajectories[i].second;

        qDebug() << "Trajectory" << (i + 1) << ":" << trajectory.size() 
                  << "points, contact force:" << (isContactForce ? "yes" : "no");

        // Create the item text with a visual indicator for contact force trajectories
        QString itemText = QString("Trajectory %1 (%2 points)%3")
                               .arg(i + 1)
                               .arg(trajectory.size())
                               .arg(isContactForce ? " [C]" : "");

        QListWidgetItem *item = new QListWidgetItem(itemText);
        item->setData(Qt::UserRole, QVariant(static_cast<int>(i)));

        if (isContactForce) {
            item->setForeground(QBrush(QColor("blue"))); // Blue for contact force trajectories
        }

        ui->listWidget_trajectories->addItem(item);
    }
    
    qDebug() << "Path planning completed successfully";
}

QSlider* MainWindow::createSlider(int min, int max, int value) {
    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(min, max);
    slider->setValue(value);
    slider->setSingleStep(1);
    slider->setFixedSize(100, 15);
    return slider;
}

void MainWindow::onSliderValueChanged(int value) {
    QSlider *slider = qobject_cast<QSlider*>(sender());
    if (!slider) return;

    Joint *articulation = slider->property("articulation").value<Joint*>();
    if (!articulation) return;

    articulation->current = value / 10.0;
    for (const auto &actor : _actorManager.getRobots()) {
        actor->update();
    }

    _actorManager.updateEntities(ui->checkBox_collisionBoxes->isChecked());
}

void MainWindow::showBBoxes(bool on) {

    _actorManager.updateEntities(on);
}

Qt3DCore::QGeometry* MainWindow::createTubeGeometry(const std::vector<QVector3D>& points, float radius, int segments) {
    if (points.size() < 2 || segments < 3) {
        qWarning() << "Insufficient points or segments for tube geometry.";
        return nullptr;
    }

    auto geometry = new Qt3DCore::QGeometry();

    QByteArray vertexData;
    QByteArray normalData;
    QByteArray indexData;

    int numVertices = (points.size() - 1) * (segments + 1) * 2;
    int numIndices = (points.size() - 1) * segments * 6;

    vertexData.resize(numVertices * 3 * sizeof(float));
    normalData.resize(numVertices * 3 * sizeof(float));
    indexData.resize(numIndices * sizeof(unsigned int));

    float* vertexPtr = reinterpret_cast<float*>(vertexData.data());
    float* normalPtr = reinterpret_cast<float*>(normalData.data());
    unsigned int* indexPtr = reinterpret_cast<unsigned int*>(indexData.data());

    int vertexCount = 0;
    int indexCount = 0;

    for (size_t i = 0; i < points.size(); ++i) {

        const QVector3D& p = points[i];
        QVector3D direction;
        if (i == points.size() - 1) {
            direction = (points[i] - points[i - 1]).normalized();
        } else {
            direction = (points[i + 1] - points[i]).normalized();
        }

        QVector3D up(0.0f, 1.0f, 0.0f);
        if (std::abs(QVector3D::dotProduct(up, direction)) > 0.99f) {
            up = QVector3D(1.0f, 0.0f, 0.0f);
        }
        QVector3D right = QVector3D::crossProduct(direction, up).normalized();
        up = QVector3D::crossProduct(right, direction).normalized();

        for (int j = 0; j <= segments; ++j) {
            float theta = (float(j) / segments) * 2.0f * M_PI;
            float cosTheta = std::cos(theta);
            float sinTheta = std::sin(theta);

            QVector3D offset = radius * (cosTheta * right + sinTheta * up);
            QVector3D vertex = p + offset;
            QVector3D normal = offset.normalized();

            *vertexPtr++ = vertex.x();
            *vertexPtr++ = vertex.y();
            *vertexPtr++ = vertex.z();
            *normalPtr++ = normal.x();
            *normalPtr++ = normal.y();
            *normalPtr++ = normal.z();
        }

        if (i < points.size() - 1) {
            for (int j = 0; j < segments; ++j) {
                unsigned int baseIndex = i * (segments + 1) + j;
                unsigned int indices[] = {
                    baseIndex, baseIndex + segments + 1, baseIndex + 1,
                    baseIndex + 1, baseIndex + segments + 1, baseIndex + segments + 2
                };
                for (int k = 0; k < 6; ++k) {
                    *indexPtr++ = indices[k];
                }
            }
        }
    }

    auto vertexDataBuffer = new Qt3DCore::QBuffer(geometry);
    vertexDataBuffer->setData(vertexData);

    auto normalDataBuffer = new Qt3DCore::QBuffer(geometry);
    normalDataBuffer->setData(normalData);

    auto indexDataBuffer = new Qt3DCore::QBuffer(geometry);
    indexDataBuffer->setData(indexData);

    auto positionAttribute = new Qt3DCore::QAttribute();
    positionAttribute->setName(Qt3DCore::QAttribute::defaultPositionAttributeName());
    positionAttribute->setVertexBaseType(Qt3DCore::QAttribute::Float);
    positionAttribute->setVertexSize(3);
    positionAttribute->setAttributeType(Qt3DCore::QAttribute::VertexAttribute);
    positionAttribute->setBuffer(vertexDataBuffer);
    positionAttribute->setByteStride(3 * sizeof(float));
    positionAttribute->setCount(numVertices);

    auto normalAttribute = new Qt3DCore::QAttribute();
    normalAttribute->setName(Qt3DCore::QAttribute::defaultNormalAttributeName());
    normalAttribute->setVertexBaseType(Qt3DCore::QAttribute::Float);
    normalAttribute->setVertexSize(3);
    normalAttribute->setAttributeType(Qt3DCore::QAttribute::VertexAttribute);
    normalAttribute->setBuffer(normalDataBuffer);
    normalAttribute->setByteStride(3 * sizeof(float));
    normalAttribute->setCount(numVertices);

    auto indexAttribute = new Qt3DCore::QAttribute();
    indexAttribute->setVertexBaseType(Qt3DCore::QAttribute::UnsignedInt);
    indexAttribute->setAttributeType(Qt3DCore::QAttribute::IndexAttribute);
    indexAttribute->setBuffer(indexDataBuffer);
    indexAttribute->setCount(numIndices);

    geometry->addAttribute(positionAttribute);
    geometry->addAttribute(normalAttribute);
    geometry->addAttribute(indexAttribute);

    return geometry;
}

void MainWindow::createPathEntity(Qt3DCore::QEntity *rootEntity, const std::vector<QVector3D> &points) {
    float radius = 0.005f;
    int segments = 8;

    Qt3DCore::QGeometry *geometry = createTubeGeometry(points, radius, segments);
    auto *geometryRenderer = new Qt3DRender::QGeometryRenderer();
    geometryRenderer->setGeometry(geometry);
    geometryRenderer->setPrimitiveType(Qt3DRender::QGeometryRenderer::Triangles);
    auto *material = new Qt3DExtras::QPhongMaterial(rootEntity);
    material->setDiffuse(QColor(0x89cff0));

    if (_pathEntity != nullptr) {
        _pathEntity->deleteLater();
    }

    _pathEntity = new Qt3DCore::QEntity(rootEntity);
    _pathEntity->addComponent(geometryRenderer);
    _pathEntity->addComponent(material);
}

std::vector<Eigen::Matrix4d> MainWindow::readPosesFromCSV(const std::string &filename)
{
    std::vector<Eigen::Matrix4d> poses;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;

        std::vector<double> values;
        while (std::getline(ss, token, ',')) {
            values.push_back(std::stod(token));
        }

        if (values.size() < 7)
            continue;

        double x = values[0];
        double y = values[1];
        double z = values[2];
        Eigen::Quaterniond q(values[3], values[4], values[5], values[6]);
        q.normalize();

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

        const Eigen::Vector3d local_move(0.0, 0.0, -0.02);
        T.block<3, 1>(0, 3) += T.block<3, 3>(0, 0) * local_move;

        poses.push_back(T);
    }
    return poses;
}

void MainWindow::loadScanPoses()
{
    QFileInfo fileInfo(ui->lineEdit_scanPosesSource->text());

    if (!fileInfo.exists() || !fileInfo.isFile()) {
        ui->statusbar->showMessage("File does not exist or is not a file.");
        return;
    }

    if (fileInfo.suffix().toLower() != "csv") {
        ui->statusbar->showMessage("File is not a CSV file.");
        return;
    }

    // Clear previous data
    ui->listWidget->clear();

    // Clear previous visualizations
    for (auto *entity : _poseFrameEntities) {
        entity->setEnabled(false);
        entity->deleteLater();
    }
    _poseFrameEntities.clear();

    // Read poses from CSV
    _scanPoses = readPosesFromCSV(ui->lineEdit_scanPosesSource->text().toStdString());

    // Add each pose to the 3D scene and list widget
    for (size_t i = 0; i < _scanPoses.size(); ++i) {
        Eigen::Affine3d affine;
        affine.matrix() = _scanPoses[i];

        // Create visual representation in 3D scene
        Qt3DCore::QEntity *frameEntity = createScaledCoordinateFrame(affine, _rootEntity, 0.05);
        _poseFrameEntities.push_back(frameEntity);

        // Extract position for display
        Eigen::Vector3d position = affine.translation();

        // Create a formatted string for the list item
        QString itemText = QString("Pose %1: (%2, %3, %4)")
                               .arg(i + 1)
                               .arg(position.x(),
                                    0,
                                    'f',
                                    3) // The 0, 'f', 3 params set field width, format and precision
                               .arg(position.y(), 0, 'f', 3)
                               .arg(position.z(), 0, 'f', 3);

        // Add to list widget
        QListWidgetItem *item = new QListWidgetItem(itemText);
        item->setData(Qt::UserRole, QVariant(static_cast<int>(i))); // Store the index
        ui->listWidget->addItem(item);
    }

    ui->pushButton_clearScanPoses->setEnabled(true);
    ui->pushButton_loadScanPoses->setEnabled(false);

    std::vector<Eigen::Affine3d> affinePoses;
    for (auto pose : _scanPoses) {
        Eigen::Affine3d affine(pose);
        affinePoses.push_back(affine);
    }

    _usPlanner->setPoses(affinePoses);

    ui->statusbar->showMessage(QString("Successfully loaded %1 scan poses.").arg(_scanPoses.size()));
}

void MainWindow::onTrajectorySelected()
{
    QList<QListWidgetItem *> selectedItems = ui->listWidget_trajectories->selectedItems();
    if (selectedItems.isEmpty()) {
        return;
    }

    // Get the trajectory index from the item's user data
    int trajectoryIndex = selectedItems.first()->data(Qt::UserRole).toInt();

    // Get the selected trajectory
    const auto &trajectories = _usPlanner->getTrajectories();
    if (trajectoryIndex >= 0 && trajectoryIndex < trajectories.size()) {
        // Set as current trajectory
        _currentTrajectoryIndex = trajectoryIndex;
        _currentTrajectory = trajectories[trajectoryIndex].first;
        bool isContactForce = trajectories[trajectoryIndex].second;

        // Display the trajectory path
        std::vector<QVector3D> points;
        for (const auto &point : _currentTrajectory) {
            // Extract end-effector position for each point
            RobotArm tempArm = *_robotArm;
            tempArm.setJointAngles(
                Eigen::Map<const Eigen::VectorXd>(point.position.data(), point.position.size()));
            Eigen::Affine3d pose = tempArm.getEndeffectorPose();
            Eigen::Vector3d position = pose.translation();

            points.push_back(QVector3D(position.x(), position.y(), position.z()));
        }

        saveTrajectoryToCSV("/Users/joris/Uni/MA/Code/Python/angles_lol.csv", _currentTrajectory);

        createPathEntity(_rootEntity, points);
        ui->statusbar->showMessage(QString("Selected trajectory %1 (%2 points)%3")
                                       .arg(trajectoryIndex + 1)
                                       .arg(_currentTrajectory.size())
                                       .arg(isContactForce ? " [Contact Force]" : ""));
    }

    ui->trajectorySlider->setMinimum(0);
    ui->trajectorySlider->setMaximum(_currentTrajectory.size() - 1);
    ui->trajectorySlider->setValue(0);

    // Improve slider behavior
    ui->trajectorySlider->setSingleStep(1);  // Set step size to 1
    ui->trajectorySlider->setPageStep(10);   // For clicking in trough area
    ui->trajectorySlider->setTracking(true); // Update continuously

    // Show initial position
    onTrajectorySliderValueChanged(0);
}

void MainWindow::onTrajectorySliderValueChanged(int value)
{
    if (_currentTrajectory.empty())
        return;

    int index = std::clamp(value, 0, static_cast<int>(_currentTrajectory.size() - 1));
    const auto &point = _currentTrajectory[index];

    // Update robot joints
    Eigen::VectorXd jointAngles = Eigen::Map<const Eigen::VectorXd>(point.position.data(),
                                                                    point.position.size());
    _robotArm->setJointAngles(jointAngles);
    _robotArm->updateEntities();

    // Check collision
    bool collides = _usPlanner->getMotionGenerator()->armHasCollision(*_robotArm);
    QString collisionStatus = collides ? "Collision detected!" : "No collision.";

    // Update status
    ui->statusbar->showMessage(QString("Time: %1 s | Point %2/%3 | %4")
                                   .arg(point.time, 0, 'f', 2)
                                   .arg(index + 1)
                                   .arg(_currentTrajectory.size())
                                   .arg(collisionStatus));
}

void MainWindow::clearScanPoses()
{
    ui->listWidget->clear();

    for (auto *entity : _poseFrameEntities) {
        entity->setEnabled(false);
        entity->deleteLater();
    }
    _poseFrameEntities.clear();
    _scanPoses.clear();

    ui->pushButton_clearScanPoses->setEnabled(false);
    ui->pushButton_loadScanPoses->setEnabled(true);
    ui->statusbar->showMessage("Scan poses cleared.");
}

void MainWindow::onScanPoseSelected()
{
    QList<QListWidgetItem *> selectedItems = ui->listWidget->selectedItems();
    if (selectedItems.isEmpty()) {
        return;
    }

    int poseIndex = selectedItems.first()->data(Qt::UserRole).toInt();

    if (poseIndex >= 0 && poseIndex < static_cast<int>(_scanPoses.size())) {
        _currentSelectedPoseIndex = poseIndex;

        Eigen::Matrix4d poseMatrix = _scanPoses[poseIndex];
        Eigen::Affine3d targetPose(poseMatrix);

        auto temp = _usPlanner->getPathPlanner()->selectGoalPose(targetPose).first;
        _robotArm->setJointAngles(temp.getJointAngles());
        _robotArm->updateEntities();
    }
}

void MainWindow::setPredefinedCamera()
{
    _cameraEntity->setPosition(QVector3D(1.22777, -1.06059, 1.19127));
    _cameraEntity->setViewCenter(QVector3D(0, -3.57628e-07, 0.5));
    _cameraEntity->setUpVector(QVector3D(-0.26978, 0.286813, 0.919213));
}

void MainWindow::createPlottingData()
{
    // Use the pathplanner to generate a path using rrtConnect
    auto pathPlanner = _usPlanner->getPathPlanner();

    // get the fourth scan pose
    Eigen::Affine3d pose = Eigen::Affine3d(_scanPoses[3]);

    // convert to translation and rotation
    Eigen::Vector3d translation = pose.translation();
    Eigen::Matrix3d rotation = pose.linear();

    pathPlanner->setGoalPose(translation, rotation);

    pathPlanner->setStartPose(*_robotArm);

    bool res = pathPlanner->runPathFinding();

    pathPlanner->saveJointAnglesToFile("/Users/joris/Uni/MA/Code/Python/angles.csv");

    auto motionGenerator = _usPlanner->getMotionGenerator();

    auto path = pathPlanner->getPath();
    Eigen::MatrixXd waypoints;
    waypoints.resize(path.size(), 7);
    for (size_t i = 0; i < path.size(); ++i) {
        for (int j = 0; j < 7; ++j) {
            waypoints(i, j) = std::get<1>(path[i]).getJointAngles()[j];
        }
    }

    motionGenerator->setWaypoints(waypoints);

    motionGenerator->performHauser(100);

    motionGenerator->saveTrajectoryToCSV("/Users/joris/Uni/MA/Code/Python/angles_timed.csv");
}

void MainWindow::planPathBetweenFirstTwoPoses()
{
    // Check if we have at least 2 scan poses loaded
    if (_scanPoses.size() < 2) {
        ui->statusbar->showMessage("Error: Need at least 2 scan poses loaded for planning.");
        return;
    }

    auto pathPlanner = _usPlanner->getPathPlanner();

    try {
        // Use first scan pose as start
        Eigen::Affine3d startPose = Eigen::Affine3d(_scanPoses[0]);
        auto startConfig = pathPlanner->selectGoalPose(startPose).first;
        _robotArm->setJointAngles(startConfig.getJointAngles());
        _robotArm->updateEntities();
        pathPlanner->setStartPose(*_robotArm);

        // Use second scan pose as goal
        Eigen::Affine3d goalPose = Eigen::Affine3d(_scanPoses[1]);
        Eigen::Vector3d goalTranslation = goalPose.translation();
        Eigen::Matrix3d goalRotation = goalPose.linear();
        pathPlanner->setGoalPose(goalTranslation, goalRotation);

        ui->statusbar->showMessage("Running path planning between first two scan poses...");

        // Run path finding
        bool success = pathPlanner->runPathFinding();

        if (success) {
            ui->statusbar->showMessage("Path planning successful between first two scan poses!");

            // Get the path and convert to trajectory for motion generation
            auto path = pathPlanner->getPath();
            Eigen::MatrixXd waypoints;
            waypoints.resize(path.size(), 7);
            for (size_t i = 0; i < path.size(); ++i) {
                for (int j = 0; j < 7; ++j) {
                    waypoints(i, j) = std::get<1>(path[i]).getJointAngles()[j];
                }
            }

            // Set waypoints for motion generation
            auto motionGenerator = _usPlanner->getMotionGenerator();
            motionGenerator->setWaypoints(waypoints);

            qDebug() << "Path contains" << path.size() << "waypoints";

        } else {
            ui->statusbar->showMessage("Path planning failed between first two scan poses.");
        }

    } catch (const std::exception &e) {
        ui->statusbar->showMessage(QString("Error during path planning: %1").arg(e.what()));
    }
}

void MainWindow::planSTOMPBetweenFirstTwoPoses()
{
    // Check if we have at least 2 scan poses loaded
    if (_scanPoses.size() < 2) {
        ui->statusbar->showMessage("Error: Need at least 2 scan poses loaded for STOMP planning.");
        return;
    }

    _robotArm->deleteEntities();

    auto motionGenerator = _usPlanner->getMotionGenerator();
    auto pathPlanner = _usPlanner->getPathPlanner();

    try {
        // Use first scan pose as start
        Eigen::Affine3d startPose = Eigen::Affine3d(_scanPoses[0]);
        auto startConfig = pathPlanner->selectGoalPose(startPose).first;
        _robotArm->setJointAngles(startConfig.getJointAngles());

        // Use second scan pose as goal
        Eigen::Affine3d goalPose = Eigen::Affine3d(_scanPoses[1]);

        // Create waypoints matrix with start and goal
        Eigen::MatrixXd waypoints(2, 7);
        waypoints.row(0) = startConfig.getJointAngles().transpose();

        // Get goal joint configuration
        auto goalConfig = pathPlanner->selectGoalPose(goalPose).first;
        waypoints.row(1) = goalConfig.getJointAngles().transpose();

        motionGenerator->setWaypoints(waypoints);

        ui->statusbar->showMessage(
            "Running STOMP with early termination between first two scan poses...");

        // Configure STOMP parameters
        StompConfig config;
        config.numJoints = 7;
        config.numNoisyTrajectories = 5;
        config.numBestSamples = 5;
        config.maxIterations = 1000;
        config.learningRate = 0.1;
        config.temperature = 10.0;
        config.dt = 0.1;
        config.jointStdDevs = Eigen::VectorXd::Constant(7, 0.1);

        // Vector to store intermediate theta matrices
        std::vector<Eigen::MatrixXd> intermediateThetas;

        // Run STOMP with early termination
        bool success = motionGenerator
                           ->performSTOMPWithEarlyTermination(config,
                                                              nullptr, // Use default thread pool
                                                              intermediateThetas);

        if (success) {
            _intermediateThetas = intermediateThetas;

            ui->statusbar->showMessage(
                QString("STOMP successful! Found collision-free solution after %1 iterations")
                    .arg(intermediateThetas.size()));

            // Save the final trajectory
            motionGenerator->saveTrajectoryToCSV(
                "/Users/joris/Uni/MA/Code/Python/stomp_early_termination.csv");

            // Save intermediate thetas for analysis
            for (size_t i = 0; i < intermediateThetas.size(); ++i) {
                std::string filename = "/Users/joris/Uni/MA/Code/Python/intermediate_theta_"
                                       + std::to_string(i) + ".csv";

                std::ofstream file(filename);
                if (file.is_open()) {
                    file << "time,";
                    for (int j = 0; j < 7; ++j) {
                        file << "joint_" << j << (j == 6 ? "\n" : ",");
                    }

                    for (int row = 0; row < intermediateThetas[i].rows(); ++row) {
                        file << row * config.dt << ",";
                        for (int col = 0; col < intermediateThetas[i].cols(); ++col) {
                            file << intermediateThetas[i](row, col) << (col == 6 ? "\n" : ",");
                        }
                    }
                    file.close();
                }
            }

            qDebug() << "Saved" << intermediateThetas.size() << "intermediate theta matrices";

            // Get the final path for visualization
            auto path = motionGenerator->getPath();
            std::vector<QVector3D> points;
            for (const auto &point : path) {
                RobotArm tempArm = *_robotArm;
                tempArm.setJointAngles(Eigen::Map<const Eigen::VectorXd>(point.position.data(),
                                                                         point.position.size()));
                Eigen::Affine3d pose = tempArm.getEndeffectorPose();
                Eigen::Vector3d position = pose.translation();
                points.push_back(QVector3D(position.x(), position.y(), position.z()));
            }

            // Create path visualization
            createPathEntity(_rootEntity, points);

        } else {
            _intermediateThetas.clear();
            ui->statusbar->showMessage(
                "STOMP failed to find collision-free solution within iteration limit.");
        }

    } catch (const std::exception &e) {
        ui->statusbar->showMessage(QString("Error during STOMP planning: %1").arg(e.what()));
    }
}

void MainWindow::onIterationSpinBoxValueChanged(int iteration)
{
    if (_intermediateThetas.empty()) {
        return;
    }

    if (_floorEntity) {
        _floorEntity->setEnabled(false);
    }

    _robotArm->deleteAllGhostEntities();

    int index = std::clamp(iteration, 0, static_cast<int>(_intermediateThetas.size() - 1));
    _currentIntermediateThetaIndex = index;

    const Eigen::MatrixXd &currentTheta = _intermediateThetas[index];

    // Generate path points for this iteration's theta matrix
    std::vector<QVector3D> pathPoints;

    for (int i = 0; i < currentTheta.rows(); ++i) {
        // Create temporary arm for each trajectory point
        RobotArm tempArm = *_robotArm;
        Eigen::VectorXd jointAngles = currentTheta.row(i);
        tempArm.setJointAngles(jointAngles);

        // Get end effector position
        Eigen::Affine3d pose = tempArm.getEndeffectorPose();
        Eigen::Vector3d position = pose.translation();
        pathPoints.push_back(QVector3D(position.x(), position.y(), position.z()));
    }

    // Remove previous iteration path entity
    if (_pathEntity) {
        _pathEntity->setParent(static_cast<Qt3DCore::QNode *>(nullptr));
        _pathEntity = nullptr;
    }

    // Create new path entity for current iteration
    if (!pathPoints.empty()) {
        createPathEntity(_rootEntity, pathPoints);
    }

    int numGhostArms = 5; // Number of ghost arms to display
    int trajectoryLength = currentTheta.rows();

    if (trajectoryLength > 1) {
        for (int i = 0; i < numGhostArms; ++i) {
            int trajectoryIndex;

            // Custom positioning for ghost arms 2 and 4 (indices 1 and 3) closer to middle
            if (i == 1) {
                // Arm 2: Move closer to middle (about 35% along trajectory instead of 25%)
                trajectoryIndex = (35 * (trajectoryLength - 1)) / 100;
            } else if (i == 3) {
                // Arm 4: Move closer to middle (about 65% along trajectory instead of 75%)
                trajectoryIndex = (65 * (trajectoryLength - 1)) / 100;
            } else {
                // Keep original spacing for arms 1, 3, and 5 (indices 0, 2, 4)
                trajectoryIndex = (i * (trajectoryLength - 1)) / (numGhostArms - 1);
            }

            // Create ghost arm at this trajectory point
            Eigen::VectorXd jointAngles = currentTheta.row(trajectoryIndex);
            _robotArm->setJointAngles(jointAngles);

            _robotArm->createGhostEntities(_rootEntity);
        }
    }

    // Check collision status for the middle point of trajectory
    int midPoint = currentTheta.rows() / 2;
    bool collides = false;

    if (midPoint < currentTheta.rows()) {
        RobotArm checkArm = *_robotArm;
        Eigen::VectorXd jointAngles = currentTheta.row(midPoint);
        checkArm.setJointAngles(jointAngles);
        collides = _usPlanner->getMotionGenerator()->armHasCollision(checkArm);
    }

    QString collisionStatus = collides ? "Collision detected!" : "No collision.";

    if (_floorEntity) {
        _floorEntity->setEnabled(true);
    }

    // Update status bar with detailed info
    ui->statusbar->showMessage(
        QString("STOMP Iteration %1/%2 | Trajectory points: %3 | Path length: %4")
            .arg(index + 1)
            .arg(_intermediateThetas.size())
            .arg(currentTheta.rows())
            .arg(pathPoints.size()));
}

