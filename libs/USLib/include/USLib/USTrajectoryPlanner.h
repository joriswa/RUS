#ifndef ULTRASOUND_SCAN_TRAJECTORY_PLANNER_H
#define ULTRASOUND_SCAN_TRAJECTORY_PLANNER_H

#include <vector>
#include <string>

#include "TrajectoryLib/Robot/RobotArm.h"
#include "GeometryLib/BVHTree.h"
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Robot/RobotManager.h"
#include "TrajectoryLib/Planning/PathPlanner.h"

struct TrajectoryTask
{
    size_t index;
    std::future<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>> future;
};

class UltrasoundScanTrajectoryPlanner {
public:
    UltrasoundScanTrajectoryPlanner(const std::string& robot_urdf);
    ~UltrasoundScanTrajectoryPlanner();

    void setCurrentJoints(const Eigen::VectorXd& joints);

    void setEnvironment(const std::string& environment);

    void setPoses(const std::vector<Eigen::Affine3d>& poses);

    bool planTrajectories();
    std::vector<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>> getTrajectories();

    MotionGenerator *getMotionGenerator() const { return _motionGenerator; }
    PathPlanner *getPathPlanner() const { return _pathPlanner; }

    std::vector<Eigen::Affine3d> getScanPoses() const;

private:
    Eigen::VectorXd _currentJoints;
    std::string _environment;
    std::vector<Eigen::Affine3d> _poses;

    RobotArm* _arm;
    std::shared_ptr<BVHTree> _obstacleTree;
    MotionGenerator* _motionGenerator;
    PathPlanner* _pathPlanner;
    RobotManager _robotManager;

    // a list of trajectories where each trajectory also contains a flag wether or not its free motion or contact force control
    std::vector<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>> _trajectories;
    
    std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool> planSingleStompTrajectory(
        const Eigen::VectorXd &startJoints,
        const Eigen::VectorXd &targetJoints,
        const StompConfig &config);
};

#endif // ULTRASOUND_SCAN_TRAJECTORY_PLANNER_H
