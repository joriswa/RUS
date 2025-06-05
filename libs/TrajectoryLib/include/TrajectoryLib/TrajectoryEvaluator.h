#ifndef TRAJECTORYEVALUATOR_H
#define TRAJECTORYEVALUATOR_H

#include "GeometryLib/BVHTree.h"
#include "TrajectoryLib/RobotArm.h"
#include <Eigen/Dense>
#include <memory> // For std::shared_ptr
#include <string>
#include <utility>
#include <vector>

class TrajectoryEvaluator
{
public:
    // Constructor that initializes the RobotArm, trajectory, and obstacle tree (as a shared pointer)
    TrajectoryEvaluator(const RobotArm &startArm,
                        const std::vector<std::pair<double, std::vector<double>>> &trajectory,
                        std::shared_ptr<BVHTree> obstacleTree);

    std::vector<double> computeJointDistancesOverTime() const;
    std::vector<double> computeDisplacementOverTime();
    std::vector<double> computeDistanceToEnvironmentOverTime();
    void saveMetricsToCSV(const std::string &filename);

private:
    RobotArm _startArm; // Initial state of the robot arm
    std::vector<std::pair<double, std::vector<double>>>
        _trajectory; // Trajectory with time and joint angles
    std::shared_ptr<BVHTree>
        _obstacleTree; // Obstacle tree as a shared pointer for environment distance computation
};

#endif // TRAJECTORYEVALUATOR_H
