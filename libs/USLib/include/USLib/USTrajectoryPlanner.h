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
 * @brief Hardware configuration for trajectory planning optimization
 */
struct HardwareConfig {
    unsigned int logicalCores;
    unsigned int physicalCores;
    size_t batchSize;
    size_t optimalThreadsForFlatParallelization;
    
    /**
     * @brief Detect hardware configuration optimized for flat parallelization
     */
    static HardwareConfig detect() {
        HardwareConfig config;
        config.logicalCores = std::thread::hardware_concurrency();
        config.physicalCores = std::max(1u, config.logicalCores);
        
        // Optimized for flat parallelization: use more threads for trajectory-level parallelism
        config.optimalThreadsForFlatParallelization = config.physicalCores;
        config.batchSize = std::max(2UL, static_cast<size_t>(config.physicalCores) / 4); 
        
        return config;
    }
};

using Trajectory = std::vector<MotionGenerator::TrajectoryPoint>;

/**
 * @brief Ultrasound scan trajectory planner
 * 
 * Plans trajectories for ultrasound scanning operations including:
 * - Repositioning movements between scan poses
 * - Contact force trajectories during scanning
 * - Batch planning with hardware optimization
 */
class UltrasoundScanTrajectoryPlanner
{
public:
    /**
     * @brief Initialize planner with environment
     * @param environmentString URDF environment description
     */
    UltrasoundScanTrajectoryPlanner(const std::string &environmentString);
    ~UltrasoundScanTrajectoryPlanner();

    /**
     * @brief Set scan poses for trajectory planning
     * @param poses Target poses for scanning
     */
    void setPoses(const std::vector<Eigen::Affine3d> &poses);
    
    /**
     * @brief Set current robot joint configuration
     * @param joints Current joint angles
     */
    void setCurrentJoints(const Eigen::VectorXd &joints);
    
    /**
     * @brief Update environment for obstacle avoidance
     * @param environment URDF environment description
     */
    void setEnvironment(const std::string &environment);

    /**
     * @brief Plan complete scan trajectory sequence
     * @param useHauserForRepositioning Use RRT+Hauser instead of STOMP for repositioning
     * @param enableShortcutting Apply path shortcutting optimization
     * @return True if planning succeeded
     */
    bool planTrajectories(bool useHauserForRepositioning = false, bool enableShortcutting = true);
    
    /**
     * @brief Plan complete scan trajectory sequence with robust fallback handling
     * @param useHauserForRepositioning Use RRT+Hauser instead of STOMP for repositioning
     * @param enableShortcutting Apply path shortcutting optimization
     * @param minSegmentDuration Minimum segment duration in seconds (segments shorter than this are discarded)
     * @return True if planning succeeded
     */
    bool planTrajectoriesRobust(bool useHauserForRepositioning = false, bool enableShortcutting = true, double minSegmentDuration = 4.0);
    
    /**
     * @brief Get planned trajectories with contact force flags
     * @return Trajectory segments with contact force indicators
     */
    std::vector<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>> getTrajectories();
    
    /**
     * @brief Get scan poses
     * @return Target scan poses
     */
    std::vector<Eigen::Affine3d> getScanPoses() const;
    
    /**
     * @brief Get internal path planner (compatibility)
     */
    PathPlanner* getPathPlanner() { return _pathPlanner; }
    
    /**
     * @brief Get internal motion generator (compatibility)
     */
    MotionGenerator* getMotionGenerator() { return _motionGenerator; }

    /**
     * @brief Get obstacle tree
     */
    std::shared_ptr<BVHTree> getObstacleTree() const { return _obstacleTree; }
    
    /**
     * @brief Print segment timing information
     */
    void printSegmentTimes() const;
    
    /**
     * @brief Plan multiple trajectories using STOMP with fallback and workload-aware parallelization
     * @param requests Start and target joint configurations
     * @param descriptions Trajectory descriptions for logging
     * @param enableShortcutting Apply path shortcutting
     * @return Planned trajectories
     */
    std::vector<Trajectory> planTrajectoryBatch(
        const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& requests,
        const std::vector<std::string>& descriptions,
        bool enableShortcutting = true);
        
    /**
     * @brief Plan multiple trajectories using RRT+Hauser optimization with workload-aware parallelization
     * @param requests Start and target joint configurations  
     * @param descriptions Trajectory descriptions for logging
     * @param enableShortcutting Apply path shortcutting
     * @return Planned trajectories
     */
    std::vector<Trajectory> planTrajectoryBatchHauser(
        const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& requests,
        const std::vector<std::string>& descriptions,
        bool enableShortcutting = true);

    /**
     * @brief Optimize waypoints by detecting and connecting small movement sequences with time-optimal segments
     * @param waypoints Original waypoint vector where each element is a joint configuration
     * @param movementThreshold Maximum joint movement (radians) to consider as "small movement"
     * @return Optimized waypoint vector with small movement sequences replaced by direct connections
     */
    std::vector<Eigen::VectorXd> optimizeSmallMovements(const std::vector<Eigen::VectorXd> &waypoints, double movementThreshold = 0.05);

private:
    Eigen::VectorXd _currentJoints;
    std::string _environment;
    std::vector<Eigen::Affine3d> _poses;

    RobotArm* _arm;
    std::shared_ptr<BVHTree> _obstacleTree;
    MotionGenerator* _motionGenerator;
    PathPlanner* _pathPlanner;
    RobotManager _robotManager;

    // Shared SDF data to avoid recomputation across multiple MotionGenerators
    std::vector<std::vector<std::vector<double>>> _sharedSdf;
    Eigen::Vector3d _sharedSdfMinPoint;
    Eigen::Vector3d _sharedSdfMaxPoint;
    double _sharedSdfResolution;
    bool _sdfCacheInitialized = false;

    // Hardware-aware configuration
    HardwareConfig _hardwareConfig;
    std::shared_ptr<boost::asio::thread_pool> _sharedThreadPool;

    // Trajectory planning results
    std::vector<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>> _trajectories;
    
    /**
     * @brief Initialize thread pool
     */
    void initializeThreadPool();
    
    /**
     * @brief Validate trajectory continuity
     */
    void validateTrajectoryDiscontinuities() const;
    
    /**
     * @brief Fix trajectory discontinuities
     * @return True if discontinuities were corrected
     */
    bool fixTrajectoryDiscontinuities();
    
    /**
     * @brief Add correction segment between trajectory points
     * @param segmentIndex Segment index for insertion
     * @param fromPoint Start point
     * @param toPoint Target point
     * @return True if correction added
     */
    bool addCorrectionSegment(
        size_t segmentIndex,
        const MotionGenerator::TrajectoryPoint& fromPoint,
        const MotionGenerator::TrajectoryPoint& toPoint
    );
    
    /**
     * @brief Initialize shared SDF cache
     */
    void initializeSharedSdf();
    
    /**
     * @brief Create motion generator with shared SDF
     * @param arm Robot arm configuration
     * @return Motion generator with shared SDF
     */
    std::unique_ptr<MotionGenerator> createMotionGeneratorWithSharedSdf(const RobotArm& arm);
    
    /**
     * @brief Determine optimal parallelization strategy based on workload
     * @param numTrajectories Number of trajectories to plan
     * @return True if flat parallelization should be used, false for hierarchical
     */
    bool shouldUseFlatParallelization(size_t numTrajectories) const;
    
    /**
     * @brief Analyze segment durations to filter out short segments
     * @param validArms Valid robot arm configurations for each pose
     * @param validSegments Valid trajectory segments 
     * @param minDuration Minimum duration threshold in seconds
     * @return Filtered segments that meet duration criteria
     */
    std::vector<std::pair<size_t, size_t>> analyzeAndFilterSegments(
        const std::vector<RobotArm>& validArms,
        const std::vector<std::pair<size_t, size_t>>& validSegments,
        double minDuration) const;
        
    /**
     * @brief Estimate duration for a trajectory segment
     * @param startArm Starting robot configuration
     * @param endArm Ending robot configuration  
     * @return Estimated trajectory duration in seconds
     */
    double estimateSegmentDuration(const RobotArm& startArm, const RobotArm& endArm) const;
    
    /**
     * @brief Try planning trajectories starting from a specific segment with fallback
     * @param validArms Valid robot arm configurations
     * @param filteredSegments Segments that passed duration filtering
     * @param validPoseIndices Mapping to original pose indices
     * @param startSegmentIdx Index of segment to start planning from
     * @param useHauserForRepositioning Use RRT+Hauser instead of STOMP
     * @param enableShortcutting Apply path shortcutting
     * @return True if planning succeeded from this starting segment
     */
    bool tryPlanningFromSegment(
        const std::vector<RobotArm>& validArms,
        const std::vector<std::pair<size_t, size_t>>& filteredSegments,
        const std::vector<size_t>& validPoseIndices,
        size_t startSegmentIdx,
        bool useHauserForRepositioning,
        bool enableShortcutting);
};