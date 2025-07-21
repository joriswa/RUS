# Enhanced Trajectory Planning Implementation Plan

## Executive Summary

This document outlines a comprehensive implementation plan for enhancing the robotic ultrasound trajectory planning system. The enhancement focuses on intelligent segmentation, robust pose validation, quintic spline interpolation, and adaptive STOMP processing with contact force trajectory generation.

## Current System Analysis

### Existing Components Assessment

#### 1. planTrajectories Implementation
**Current State**: 
- Basic waypoint-to-waypoint trajectory generation
- Limited segmentation logic
- Uniform processing regardless of configuration complexity

**Identified Issues**:
- Inefficient processing of simple trajectory segments
- Poor handling of large configuration changes
- No pose validity preprocessing
- Rigid segmentation approach

#### 2. STOMP Integration
**Current State**:
- Robust obstacle avoidance optimization
- Parallel trajectory sampling
- Cost-based trajectory selection
- Time-optimal scaling post-processing

**Identified Issues**:
- Single-shot planning approach (no retry mechanisms)
- Fixed parameter sets regardless of trajectory difficulty
- No adaptive segmentation support
- Limited failure recovery options

#### 3. Contact Force Trajectory Generation
**Current State**:
- Basic end-effector trajectory tracking via TaskSpacePathTrackingCostCalculator
- Obstacle avoidance through ObstacleCostCalculator
- Joint constraint enforcement via ConstraintCostCalculator

**Gap Analysis**:
- No explicit contact force control integration
- Missing force trajectory generation for ultrasound scanning
- No compliance modeling for patient interaction

## Technical Architecture

### Enhanced Trajectory Planning Pipeline

```
Input Waypoints
       ↓
[1] Configuration Analysis & Segmentation Detection
       ↓
[2] Pose Validation & Filtering
       ↓
[3] Intelligent Segmentation with Continuity Preservation
       ↓
[4] Quintic Spline Interpolation Through Valid Poses
       ↓
[5] Contact Force Trajectory Generation (if applicable)
       ↓
[6] Robust STOMP Processing with Adaptive Parameters
       ↓
[7] Collision-Free Solution Validation & Retry
       ↓
Output Trajectory
```

## Phase 1: Configuration Analysis & Segmentation Detection

### 1.1 Configuration Change Metrics

Implement multi-criteria assessment for detecting significant configuration changes:

#### Joint Space Metrics
```cpp
struct ConfigurationChangeMetrics {
    double jointAngleThreshold = 0.5;     // radians
    double jointVelocityThreshold = 1.0;   // rad/s
    double cumulativeChangeThreshold = 1.5; // radians
};

bool detectStarkConfigurationChange(
    const Eigen::VectorXd& config1,
    const Eigen::VectorXd& config2,
    const ConfigurationChangeMetrics& metrics
);
```

#### Task Space Metrics
```cpp
struct TaskSpaceChangeMetrics {
    double positionThreshold = 0.1;       // meters
    double orientationThreshold = 0.3;    // radians
    double manipulabilityThreshold = 0.5; // relative change
};

bool detectTaskSpaceDiscontinuity(
    const Eigen::Affine3d& pose1,
    const Eigen::Affine3d& pose2,
    const TaskSpaceChangeMetrics& metrics
);
```

### 1.2 Segmentation Decision Algorithm

```cpp
class IntelligentSegmentationAnalyzer {
public:
    struct SegmentationResult {
        std::vector<int> segmentBoundaries;
        std::vector<SegmentType> segmentTypes;
        std::vector<double> complexityScores;
    };
    
    enum class SegmentType {
        SIMPLE_LINEAR,        // Direct interpolation sufficient
        MODERATE_COMPLEX,     // Standard STOMP processing
        HIGH_COMPLEX,         // Enhanced STOMP with retry
        CRITICAL_TRANSITION   // Special handling required
    };
    
    SegmentationResult analyzeTrajectoryComplexity(
        const std::vector<Eigen::VectorXd>& waypoints,
        const RobotArm& arm
    );
    
private:
    double computeSegmentComplexity(
        const Eigen::VectorXd& start,
        const Eigen::VectorXd& end,
        const RobotArm& arm
    );
};
```

## Phase 2: Enhanced Pose Validation & Filtering

### 2.1 Comprehensive Pose Validation

```cpp
class RobustPoseValidator {
public:
    enum class ValidationResult {
        VALID,
        JOINT_LIMITS_VIOLATED,
        SINGULARITY_DETECTED,
        COLLISION_DETECTED,
        IK_FAILED,
        FORCE_CONSTRAINTS_VIOLATED
    };
    
    struct ValidationReport {
        ValidationResult result;
        std::string errorMessage;
        double confidence;
        Eigen::VectorXd suggestedCorrection;
    };
    
    ValidationReport validatePose(
        const Eigen::Affine3d& targetPose,
        const RobotArm& arm,
        const std::shared_ptr<BVHTree>& obstacleTree
    );
    
    std::vector<Eigen::VectorXd> filterValidPoses(
        const std::vector<Eigen::Affine3d>& poses,
        const RobotArm& arm,
        const std::shared_ptr<BVHTree>& obstacleTree
    );
    
private:
    bool checkJointLimits(const Eigen::VectorXd& jointConfig);
    bool checkSingularityProximity(const Eigen::VectorXd& jointConfig, const RobotArm& arm);
    bool checkCollisionStatus(const Eigen::VectorXd& jointConfig, const RobotArm& arm, const std::shared_ptr<BVHTree>& obstacleTree);
    double computeManipulability(const Eigen::VectorXd& jointConfig, const RobotArm& arm);
};
```

### 2.2 Invalid Pose Handling Strategy

```cpp
class PoseRecoveryManager {
public:
    struct RecoveryOptions {
        bool allowInterpolation = true;
        bool allowOptimization = true;
        double maxDeviation = 0.05;  // meters
        int maxIterations = 10;
    };
    
    std::vector<Eigen::VectorXd> recoverInvalidPoses(
        const std::vector<Eigen::Affine3d>& originalPoses,
        const std::vector<bool>& validityMask,
        const RobotArm& arm,
        const RecoveryOptions& options
    );
    
private:
    Eigen::VectorXd optimizePoseForValidity(
        const Eigen::Affine3d& targetPose,
        const RobotArm& arm,
        const RecoveryOptions& options
    );
};
```

## Phase 3: Quintic Spline Integration with Continuity Preservation

### 3.1 Enhanced Spline Fitting

```cpp
class ContinuityPreservingSplineGenerator {
public:
    struct SplineParameters {
        double timeStep = 0.01;           // seconds
        bool enforceVelocityContinuity = true;
        bool enforceAccelerationContinuity = true;
        double maxVelocity = 1.0;         // rad/s per joint
        double maxAcceleration = 2.0;     // rad/s² per joint
    };
    
    std::vector<TrajectoryPoint> generateSplineTrajectory(
        const std::vector<Eigen::VectorXd>& validWaypoints,
        const std::vector<double>& timeStamps,
        const SplineParameters& params
    );
    
    // Segment-aware spline generation with boundary condition matching
    std::vector<TrajectoryPoint> generateSegmentedSplineTrajectory(
        const std::vector<std::vector<Eigen::VectorXd>>& segments,
        const SplineParameters& params
    );
    
private:
    std::vector<Eigen::VectorXd> computeQuinticCoefficients(
        const std::vector<Eigen::VectorXd>& waypoints,
        const std::vector<double>& times,
        const SplineParameters& params
    );
    
    void enforceBoundaryConditions(
        std::vector<TrajectoryPoint>& segment1,
        std::vector<TrajectoryPoint>& segment2
    );
};
```

## Phase 4: Contact Force Trajectory Generation

### 4.1 Force-Aware Trajectory Planning

```cpp
class ContactForceTrajectoryGenerator {
public:
    struct ForceParameters {
        double targetNormalForce = 5.0;      // Newtons
        double maxTangentialForce = 2.0;     // Newtons
        double forceControlBandwidth = 20.0; // Hz
        Eigen::Vector3d surfaceNormal = Eigen::Vector3d(0, 0, 1);
    };
    
    struct ForceTrajectoryPoint {
        TrajectoryPoint motion;
        Eigen::Vector3d targetForce;
        Eigen::Vector3d forceDirection;
        double stiffness;
        double damping;
    };
    
    std::vector<ForceTrajectoryPoint> generateForceAwareTrajectory(
        const std::vector<TrajectoryPoint>& motionTrajectory,
        const ForceParameters& forceParams,
        const SurfaceModel& patientSurface
    );
    
private:
    Eigen::Vector3d computeContactForce(
        const Eigen::Affine3d& endEffectorPose,
        const Eigen::Vector3d& surfaceNormal,
        const ForceParameters& params
    );
    
    void optimizeForceCompliance(
        std::vector<ForceTrajectoryPoint>& trajectory,
        const ForceParameters& params
    );
};
```

### 4.2 Surface Contact Modeling

```cpp
class SurfaceContactModel {
public:
    struct ContactState {
        bool inContact;
        Eigen::Vector3d contactPoint;
        Eigen::Vector3d surfaceNormal;
        double penetrationDepth;
        double contactForce;
    };
    
    ContactState evaluateContact(
        const Eigen::Affine3d& endEffectorPose,
        const SurfaceModel& surface
    );
    
    Eigen::Vector3d computeRequiredForce(
        const ContactState& currentContact,
        const ForceParameters& targetParams
    );
};
```

## Phase 5: Robust STOMP Processing with Adaptive Parameters

### 5.1 Enhanced STOMP Configuration

```cpp
class AdaptiveStompProcessor {
public:
    struct AdaptiveStompConfig {
        StompConfig baseConfig;
        
        // Adaptive parameters
        double difficultyMultiplier = 1.0;
        int maxRetryAttempts = 5;
        bool enableParameterScaling = true;
        bool enableProgressiveRefinement = true;
        
        // Segment-specific tuning
        double simpleSegmentSpeedup = 2.0;
        double complexSegmentSlowdown = 0.5;
        int criticalSegmentIterations = 200;
    };
    
    struct ProcessingResult {
        bool success;
        std::vector<TrajectoryPoint> trajectory;
        int attemptsUsed;
        double finalCost;
        std::string statusMessage;
    };
    
    ProcessingResult processTrajectorySegment(
        const std::vector<Eigen::VectorXd>& waypoints,
        const IntelligentSegmentationAnalyzer::SegmentType& segmentType,
        const AdaptiveStompConfig& config,
        std::shared_ptr<boost::asio::thread_pool> sharedPool
    );
    
private:
    StompConfig adaptConfigForSegment(
        const StompConfig& baseConfig,
        const IntelligentSegmentationAnalyzer::SegmentType& segmentType,
        int attemptNumber
    );
    
    bool validateSolution(
        const std::vector<TrajectoryPoint>& trajectory,
        const std::shared_ptr<BVHTree>& obstacleTree
    );
};
```

### 5.2 Retry Mechanism with Progressive Parameter Adjustment

```cpp
class ProgressiveRetryManager {
public:
    struct RetryStrategy {
        std::vector<double> temperatureSequence = {10.0, 15.0, 25.0, 40.0, 60.0};
        std::vector<int> iterationSequence = {50, 100, 150, 200, 300};
        std::vector<double> learningRateSequence = {0.1, 0.05, 0.025, 0.01, 0.005};
        bool enableNoiseAdaptation = true;
        bool enableCostCalculatorTuning = true;
    };
    
    ProcessingResult retryUntilSuccess(
        const std::vector<Eigen::VectorXd>& waypoints,
        const AdaptiveStompConfig& baseConfig,
        const RetryStrategy& strategy,
        std::shared_ptr<boost::asio::thread_pool> sharedPool
    );
    
private:
    StompConfig createRetryConfig(
        const StompConfig& baseConfig,
        const RetryStrategy& strategy,
        int retryAttempt
    );
};
```

## Phase 6: Integration with Existing System

### 6.1 Enhanced MotionGenerator Interface

```cpp
class EnhancedMotionGenerator : public MotionGenerator {
public:
    struct EnhancedPlanningConfig {
        bool enableIntelligentSegmentation = true;
        bool enablePoseValidation = true;
        bool enableForceTrajectories = false;
        bool enableAdaptiveSTOMP = true;
        bool enableProgressiveRetry = true;
        
        ConfigurationChangeMetrics configChangeMetrics;
        TaskSpaceChangeMetrics taskSpaceMetrics;
        SplineParameters splineParams;
        ForceParameters forceParams;
        AdaptiveStompConfig stompConfig;
        RetryStrategy retryStrategy;
    };
    
    bool performEnhancedSTOMP(
        const EnhancedPlanningConfig& config,
        std::shared_ptr<boost::asio::thread_pool> sharedPool = nullptr
    );
    
    // Contact force trajectory planning
    bool planContactForceTrajectory(
        const std::vector<Eigen::Affine3d>& scanPoses,
        const SurfaceModel& patientSurface,
        const EnhancedPlanningConfig& config
    );
    
private:
    IntelligentSegmentationAnalyzer _segmentationAnalyzer;
    RobustPoseValidator _poseValidator;
    PoseRecoveryManager _poseRecovery;
    ContinuityPreservingSplineGenerator _splineGenerator;
    ContactForceTrajectoryGenerator _forceGenerator;
    AdaptiveStompProcessor _stompProcessor;
    ProgressiveRetryManager _retryManager;
    
    std::vector<std::vector<Eigen::VectorXd>> segmentTrajectory(
        const std::vector<Eigen::VectorXd>& waypoints,
        const EnhancedPlanningConfig& config
    );
    
    std::vector<Eigen::VectorXd> validateAndFilterPoses(
        const std::vector<Eigen::VectorXd>& waypoints,
        const EnhancedPlanningConfig& config
    );
    
    bool processSegmentWithRetry(
        const std::vector<Eigen::VectorXd>& segmentWaypoints,
        const IntelligentSegmentationAnalyzer::SegmentType& segmentType,
        const EnhancedPlanningConfig& config,
        std::vector<TrajectoryPoint>& outputTrajectory
    );
};
```

## Implementation Roadmap

### Phase 1: Foundation (Week 1-2)
- [ ] Implement `IntelligentSegmentationAnalyzer`
- [ ] Create `RobustPoseValidator` with comprehensive validation
- [ ] Develop configuration change detection metrics
- [ ] Unit tests for segmentation and validation components

### Phase 2: Spline Integration (Week 3)
- [ ] Enhance `ContinuityPreservingSplineGenerator` 
- [ ] Implement boundary condition enforcement between segments
- [ ] Integration tests with existing quintic spline code
- [ ] Performance optimization for spline generation

### Phase 3: Force Trajectories (Week 4)
- [ ] Implement `ContactForceTrajectoryGenerator`
- [ ] Develop `SurfaceContactModel` for ultrasound applications
- [ ] Create force-aware cost calculators for STOMP
- [ ] Validation with simulated ultrasound scanning scenarios

### Phase 4: Adaptive STOMP (Week 5-6)
- [ ] Implement `AdaptiveStompProcessor`
- [ ] Create `ProgressiveRetryManager` with parameter adaptation
- [ ] Integration with existing STOMP implementation
- [ ] Performance benchmarking and optimization

### Phase 5: System Integration (Week 7)
- [ ] Integrate all components into `EnhancedMotionGenerator`
- [ ] Comprehensive testing with real ultrasound scanning trajectories
- [ ] Performance validation and optimization
- [ ] Documentation and API finalization

### Phase 6: Validation & Deployment (Week 8)
- [ ] Clinical workflow validation
- [ ] Safety validation for medical applications
- [ ] Performance benchmarking against current system
- [ ] Production deployment preparation

## Performance Considerations

### Optimization Strategies

1. **Lazy Evaluation**: Only perform expensive computations when necessary
2. **Caching**: Cache IK solutions, collision checks, and complexity scores
3. **Parallel Processing**: Leverage existing thread pool for segment processing
4. **Early Termination**: Stop processing when collision-free solutions are found
5. **Adaptive Resolution**: Use coarse planning followed by refinement

### Memory Management

```cpp
class TrajectoryPlanningCache {
public:
    struct CacheEntry {
        Eigen::VectorXd jointConfig;
        bool isValid;
        double manipulability;
        std::chrono::steady_clock::time_point timestamp;
    };
    
    void cacheValidationResult(const Eigen::Affine3d& pose, const CacheEntry& result);
    std::optional<CacheEntry> getCachedResult(const Eigen::Affine3d& pose, double tolerance = 1e-3);
    void clearExpiredEntries(std::chrono::milliseconds maxAge);
    
private:
    std::unordered_map<size_t, CacheEntry> _cache;
    std::mutex _cacheMutex;
};
```

## Testing Strategy

### Unit Tests
- Configuration change detection accuracy
- Pose validation reliability
- Spline continuity verification
- Force trajectory generation correctness

### Integration Tests
- End-to-end trajectory planning with various waypoint sets
- Performance comparison with existing system
- Robustness testing with challenging configurations

### Clinical Validation
- Ultrasound scanning accuracy with force control
- Patient safety validation
- Medical workflow integration testing

## Conclusion

This enhanced trajectory planning system provides:

1. **Intelligent Processing**: Only applies complex algorithms where needed
2. **Robust Validation**: Comprehensive pose checking and error recovery
3. **Smooth Trajectories**: Quintic splines with continuity preservation
4. **Force Control**: Contact force trajectories for ultrasound applications
5. **Adaptive Optimization**: STOMP with retry mechanisms and parameter adaptation
6. **Clinical Safety**: Medical-grade validation and safety checks

The implementation maintains backward compatibility while providing significant improvements in trajectory quality, planning reliability, and system performance for robotic ultrasound applications.
