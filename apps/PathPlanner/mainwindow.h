#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QDebug>
#include <QFileInfo>
#include <QListWidgetItem>
#include <QMainWindow>
#include <QMutex>
#include <QObject>
#include <QPlaneMesh>
#include <QThread>
#include <QWaitCondition>
#include <Qt3DCore/QEntity>
#include <Qt3DExtras/QFirstPersonCameraController>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DExtras/QPerVertexColorMaterial>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QLayer>
#include <Qt3DRender/QPickEvent>
#include <Qt3DRender/QPointLight>
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include <QDial>
#include <QSlider>
#include <Qt3DExtras/QTextureMaterial>
#include <Qt3DRender/QTextureImageData>

#include <QTreeWidgetItem>

#include "TrajectoryLib/RobotArm.h"
#include "TrajectoryLib/RobotManager.h"
#include "USLib/USTrajectoryPlanner.h"
#include "customcameracontroller.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void loadScanPoses();
    void clearScanPoses();

private:
    Ui::MainWindow *ui;

    RobotArm *_robotArm = nullptr;
    RobotManager _actorManager;
    UltrasoundScanTrajectoryPlanner *_usPlanner;

    std::vector<RobotArm *> visArms;

    QThread *_plannerThread;

    void setup3DScene();
    Qt3DExtras::Qt3DWindow *_view;
    QWidget *_container;
    Qt3DCore::QEntity *_rootEntity;
    Qt3DRender::QCamera *_cameraEntity;
    CustomCameraController *_camController;

    Eigen::Matrix<double, 6, 1> _startPose;
    Eigen::Matrix<double, 6, 1> _goalPose;

    QVector<QSharedPointer<Qt3DCore::QEntity>> _pathEntities;
    Qt3DCore::QEntity *_pathEntity = nullptr;

    Qt3DCore::QEntity *createSphereEntity(const Eigen::Vector3d &position, float radius);

    QSlider *createSlider(int min, int max, int value);
    void addArticulationControls(QTreeWidgetItem *parentItem, std::shared_ptr<Robot> actor);

    bool _showBBoxes;

    void findPath();

    Qt3DCore::QEntity *_cylinderEntity = nullptr;
    Qt3DCore::QEntity *_floorEntity = nullptr;
    // QObjectPicker removed - not available in current Qt version
    // Qt3DRender::QObjectPicker *_floorPicker = nullptr;

    void createFloor();
    void onFloorClicked(Qt3DRender::QPickEvent *event);

    void createPathEntity(Qt3DCore::QEntity *rootEntity, const std::vector<QVector3D> &points);
    Qt3DCore::QGeometry *createTubeGeometry(const std::vector<QVector3D> &points,
                                            float radius,
                                            int segments);
    std::vector<Eigen::Matrix4d> readPosesFromCSV(const std::string &filename);

    std::vector<Eigen::Matrix4d> _scanPoses;
    std::map<QListWidgetItem *, int> _itemToPoseIndex;
    std::vector<Qt3DCore::QEntity *> _poseFrameEntities;
    int _currentSelectedPoseIndex = -1;
    std::vector<RobotArm> _scanCheckpoints;

    size_t _currentTrajectoryIndex;
    std::vector<MotionGenerator::TrajectoryPoint> _currentTrajectory;

    std::vector<Eigen::MatrixXd> _intermediateThetas;
    int _currentIntermediateThetaIndex = 0;

    void onTrajectorySelected();
    void onTrajectorySliderValueChanged(int value);
    void setPredefinedCamera();
    void createPlottingData();
    void planPathBetweenFirstTwoPoses();
    void planSTOMPBetweenFirstTwoPoses();
    void onIterationSpinBoxValueChanged(int iteration);
private slots:
    void pathAvailable();
    void setAlgorithm();
    void loadActors();
    void changeState(int);
    void showBBoxes(bool);
    void onSliderValueChanged(int);
    void clearActors();
    void onScanPoseSelected(); // New slot for handling scan pose selection
};

#endif // MAINWINDOW_H
