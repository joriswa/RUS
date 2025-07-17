#ifndef TRAJECTORYEVALUATOR_H
#define TRAJECTORYEVALUATOR_H

#include "GeometryLib/BVHTree.h"
#include "TrajectoryLib/Robot/RobotArm.h"
#include <Eigen/Dense>
#include <memory> // For std::shared_ptr
#include <string>
#include <utility>
#include <vector>

/**
 * @brief Evaluates robot trajectory metrics including distances and clearances
 * 
 * The TrajectoryEvaluator class provides functionality to analyze robot trajectories
 * by computing various metrics such as joint distances over time, displacement,
 * and distance to environment obstacles.
 */
class TrajectoryEvaluator
{
public:
    /**
     * @brief Constructor that initializes the robot arm, trajectory, and obstacle tree
     * @param startArm Initial state of the robot arm
     * @param trajectory Trajectory with time and joint angles pairs
     * @param obstacleTree BVH tree of obstacles for distance computation
     */
    TrajectoryEvaluator(const RobotArm &startArm,
                        const std::vector<std::pair<double, std::vector<double>>> &trajectory,
                        std::shared_ptr<BVHTree> obstacleTree);

    /**
     * @brief Compute joint space distances between consecutive trajectory points
     * @return Vector of joint distance values over time
     */
    std::vector<double> computeJointDistancesOverTime() const;
    
    /**
     * @brief Compute end-effector displacement over time
     * @return Vector of displacement values over time
     */
    std::vector<double> computeDisplacementOverTime();
    
    /**
     * @brief Compute minimum distance to environment obstacles over time
     * @return Vector of distance values to obstacles over time
     */
    std::vector<double> computeDistanceToEnvironmentOverTime();
    
    /**
     * @brief Save computed metrics to CSV file
     * @param filename Path to output CSV file
     */
    void saveMetricsToCSV(const std::string &filename);

private:
    RobotArm _startArm;                                            ///< Initial state of the robot arm
    std::vector<std::pair<double, std::vector<double>>> _trajectory; ///< Trajectory with time and joint angles
    std::shared_ptr<BVHTree> _obstacleTree;                        ///< Obstacle tree for environment distance computation
};

#endif // TRAJECTORYEVALUATOR_H
