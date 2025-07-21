#pragma once

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <future>
#include <boost/asio/thread_pool.hpp>

#include "TrajectoryLib/Robot/RobotArm.h"
#include "GeometryLib/BVHTree.h"
#include "GeometryLib/ObstacleTree.h"
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Robot/RobotManager.h"
#include "TrajectoryLib/Planning/PathPlanner.h"

/**
 * @brief Hardware configuration for optimal performance
 */
struct HardwareConfig {
    unsigned int logicalCores;
    unsigned int physicalCores;
    size_t batchSize;
    size_t memoryLimitMB;
    
    /**
     * @brief Detect optimal hardware configuration
     */
    static HardwareConfig detect() {
        HardwareConfig config;
        config.logicalCores = std::thread::hardware_concurrency();
        
        // Conservative estimate for physical cores (typically logical/2 on hyperthreaded systems)
        config.physicalCores = std::max(1u, config.logicalCores / 2);
        
        // Optimal batch size: balance between parallel efficiency and memory usage
        config.batchSize = std::max(1UL, static_cast<size_t>(config.physicalCores) * 3 / 2);
        
        // Conservative memory estimate: 1GB for trajectory planning
        config.memoryLimitMB = 1024;
        
        return config;
    }
};

struct TrajectoryTask
{
    std::vector<Eigen::Affine3d> poses;
    std::vector<double> currentJoints;
    std::string environmentString;
};

using Trajectory = std::vector<MotionGenerator::TrajectoryPoint>;

class UltrasoundScanTrajectoryPlanner
{
public:
    UltrasoundScanTrajectoryPlanner(const std::string &environmentString, 
                                  const std::vector<double> &currentJoints);
    ~UltrasoundScanTrajectoryPlanner();

    void setPoses(const std::vector<Eigen::Affine3d> &poses);
    void setCurrentJoints(const Eigen::VectorXd &currentJoints);
    void setEnvironment(const std::string &environment);
    
    /**
     * @brief Plan single STOMP trajectory with optimized parameters
     */
    std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool> planSingleStompTrajectory(
        const Eigen::VectorXd &startJoints,
        const Eigen::VectorXd &targetJoints,
        const StompConfig &config);

    bool planTrajectories();
    bool repositionFromDetectedSurface(const BoundingBox &boundingBox);

    std::vector<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>> getTrajectories();
    std::vector<Eigen::Affine3d> getScanPoses() const;
    
    /**
     * @brief Get access to the internal PathPlanner (for compatibility with existing apps)
     */
    PathPlanner* getPathPlanner() { return _pathPlanner; }
    
    /**
     * @brief Get access to the internal MotionGenerator (for compatibility with existing apps)
     */
    MotionGenerator* getMotionGenerator() { return _motionGenerator; }
    
    /**
     * @brief Plan multiple trajectories in hardware-optimized batches
     * @param requests Vector of trajectory planning requests (start poses, target poses)
     * @return Vector of planned trajectories
     */
    std::vector<Trajectory> planTrajectoryBatch(
        const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& requests);

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
    HardwareConfig _hardwareConfig;
    std::shared_ptr<boost::asio::thread_pool> _sharedThreadPool;

    // Trajectory planning results
    std::vector<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>> _trajectories;
    
    /**
     * @brief Initialize shared thread pool with optimal sizing
     */
    void initializeThreadPool();
};
