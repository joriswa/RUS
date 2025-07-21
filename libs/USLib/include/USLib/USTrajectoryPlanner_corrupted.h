#pragma once

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <future>

#include "TrajectoryLib/Robot/RobotArm.h"
#include "GeometryLib/BVHTree.h"
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Robot/RobotManager.h"
#include "TrajectoryLib/Planning/PathPlanner.h"

/**
 * @brief Hardware-aware configuration for optimal performance
 */
struct HardwareConfig {
    size_t numPhysicalCores;
    size_t numLogicalCores;
    size_t optimalBatchSize;
    size_t memoryLimitMB;
    
    static HardwareConfig detect() {
        HardwareConfig config;
        config.numLogicalCores = std::thread::hardware_concurrency();
        config.numPhysicalCores = config.numLogicalCores / 2; // Assume hyperthreading
        
        // Optimal batch size: 1.5-2x physical cores for CPU-bound tasks
        config.optimalBatchSize = std::max(1UL, config.numPhysicalCores * 3 / 2);
        
        // Conservative memory estimate: 1GB for trajectory planning
        config.memoryLimitMB = 1024;
        
        return config;
    }
};RASOUND_SCAN_TRAJECTORY_PLANNER_H
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
    std::shared_ptr<BVHTree> getObstacleTree() const { return _obstacleTree; }
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

    // Hardware-aware configuration
    HardwareConfig _hwConfig;
    std::shared_ptr<boost::asio::thread_pool> _sharedThreadPool;

    // Trajectory planning results
    std::vector<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>> _trajectories;
    
    /**
     * @brief Plan a single STOMP trajectory with optimized parameters and shared thread pool
     */
    std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool> planSingleStompTrajectory(
        const Eigen::VectorXd &startJoints,
        const Eigen::VectorXd &targetJoints,
        const StompConfig &config);
    
    /**
     * @brief Plan multiple trajectories in hardware-optimized batches
     */
    std::vector<std::future<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>>> 
    planTrajectoryBatch(const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& trajectoryPairs,
                        const StompConfig& config);
    
    /**
     * @brief Initialize shared thread pool with optimal sizing
     */
    void initializeThreadPool();
};

#endif // ULTRASOUND_SCAN_TRAJECTORY_PLANNER_H
