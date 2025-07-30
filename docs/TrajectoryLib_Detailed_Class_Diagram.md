# TrajectoryLib planTrajectories Implementation Details

This document provides detailed class diagrams and implementation analysis specifically focused on the components used by the `UltrasoundScanTrajectoryPlanner::planTrajectories()` method.

## Core Implementation Classes

### 1. UltrasoundScanTrajectoryPlanner Class Detail

```mermaid
classDiagram
    class UltrasoundScanTrajectoryPlanner {
        -Eigen::VectorXd _currentJoints
        -string _environment  
        -vector~Eigen::Affine3d~ _poses
        -RobotArm* _arm
        -shared_ptr~BVHTree~ _obstacleTree
        -MotionGenerator* _motionGenerator
        -PathPlanner* _pathPlanner
        -RobotManager _robotManager
        -HardwareConfig _hardwareConfig
        -shared_ptr~boost::asio::thread_pool~ _sharedThreadPool
        -vector~vector~vector~double~~~ _sharedSdf
        -Eigen::Vector3d _sharedSdfMinPoint
        -Eigen::Vector3d _sharedSdfMaxPoint
        -double _sharedSdfResolution
        -bool _sdfCacheInitialized
        -vector~pair~vector~TrajectoryPoint~, bool~~ _trajectories
        
        +UltrasoundScanTrajectoryPlanner(environmentString)
        +~UltrasoundScanTrajectoryPlanner()
        +setPoses(poses) void
        +setCurrentJoints(joints) void
        +setEnvironment(environment) void
        +planTrajectories(useHauser, enableShortcutting) bool
        +getTrajectories() vector~pair~vector~TrajectoryPoint~, bool~~
        +getScanPoses() vector~Eigen::Affine3d~
        +printSegmentTimes() void
        
        -initializeThreadPool() void
        -validateTrajectoryDiscontinuities() void
        -fixTrajectoryDiscontinuities() bool
        -addCorrectionSegment(segmentIndex, fromPoint, toPoint) bool
        -initializeSharedSdf() void
        -createMotionGeneratorWithSharedSdf(arm) unique_ptr~MotionGenerator~
        -planTrajectoryBatch(requests, descriptions, enableShortcutting) vector~Trajectory~
        -planTrajectoryBatchHauser(requests, descriptions, enableShortcutting) vector~Trajectory~
    }
    
    class HardwareConfig {
        +unsigned int logicalCores
        +unsigned int physicalCores  
        +size_t batchSize
        +static detect() HardwareConfig
    }
    
    class TrajectoryPoint {
        +vector~double~ position
        +vector~double~ velocity
        +vector~double~ acceleration
        +double time
    }
    
    UltrasoundScanTrajectoryPlanner --> HardwareConfig : uses
    UltrasoundScanTrajectoryPlanner --> TrajectoryPoint : produces
```

### 2. PathPlanner Component

```mermaid
classDiagram
    class PathPlanner {
        -shared_ptr~BVHTree~ _obstacleTree
        -RobotArm _startConfiguration
        -Params _params
        -Algorithm _currentAlgorithm
        -NodePtr _rootNode
        -vector~NodePtr~ _nodes
        -default_random_engine _generator
        
        +PathPlanner()
        +~PathPlanner()
        +setObstacleTree(tree) void
        +setStartPose(robotArm) void
        +setParams(params) void
        +planCheckpoints(poses, currentJoints) CheckpointPlanResult
        +runPathFinding() bool
        +shortcutPath(iterations, maxAttempts) void
        +getAnglesPath() Eigen::MatrixXd
        +setGoalConfiguration(robotArm) void
        
        -solveInverseKinematics(pose) RobotArm
        -isConfigurationValid(robotArm) bool
        -groupValidSegments(checkpoints) vector~pair~size_t, size_t~~
        -findFirstValidIndex(checkpoints) size_t
        -densifyTaskSpacePath(originalPoses) vector~Eigen::Affine3d~
    }
    
    class CheckpointPlanResult {
        +vector~pair~RobotArm, bool~~ checkpoints
        +vector~pair~size_t, size_t~~ validSegments  
        +vector~pair~size_t, size_t~~ jumpPairs
        +size_t firstValidIndex
    }
    
    class Params {
        +Algorithm algo
        +double stepSize
        +double goalBiasProbability
        +bool customCost
        +int maxIterations
    }
    
    class PathNode {
        +State _state
        +NodePtr _parent
        +double _cost
        +double _costToGoal
        +RobotArm _arm
        
        +PathNode(state, arm, parent, cost, costToGoal)
        +createChild(point, arm) NodePtr
        +estimatedTotalCost() double
    }
    
    PathPlanner --> CheckpointPlanResult : returns
    PathPlanner --> Params : configured by
    PathPlanner --> PathNode : uses
    CheckpointPlanResult --> RobotArm : contains
```

### 3. MotionGenerator Component

```mermaid
classDiagram
    class MotionGenerator {
        -RobotArm _arm
        -shared_ptr~BVHTree~ _obstacleTree
        -Eigen::MatrixXd _waypoints
        -vector~TrajectoryPoint~ _path
        -vector~vector~vector~double~~~ _sdf
        -Eigen::Vector3d _sdfMinPoint
        -Eigen::Vector3d _sdfMaxPoint
        -double _sdfResolution
        -bool _sdfInitialized
        -ArmFeasibilityChecker* _feasibilityChecker
        
        +MotionGenerator(arm)
        +MotionGenerator(arm, sharedSdf, minPoint, maxPoint, resolution, obstacleTree)
        +~MotionGenerator()
        +setObstacleTree(tree) void
        +setWaypoints(waypoints) void
        +performSTOMP(config, threadPool, threadId) bool
        +performHauser(maxIterations) void
        +generateTrajectoryFromCheckpoints(waypoints) vector~TrajectoryPoint~
        +getPath() vector~TrajectoryPoint~
        +createSDF() void
        +isSdfInitialized() bool
        +getSdf() vector~vector~vector~double~~~
        +getSdfMinPoint() Eigen::Vector3d
        +getSdfMaxPoint() Eigen::Vector3d
        +getSdfResolution() double
        +computeTimeOptimalSegment(start, end, startTime) vector~TrajectoryPoint~
        
        -initializeTrajectory() void
        -generateNoisySamples(config) vector~Eigen::MatrixXd~
        -evaluateCosts(samples, config) vector~double~
        -updateTrajectory(samples, costs, config) void
        -checkConvergence(costs) bool
        -validateCollisionFree() bool
        -computeCollisionCost(trajectory) double
        -computeSmoothnessCost(trajectory) double
    }
    
    class StompConfig {
        +int numNoisyTrajectories
        +int numBestSamples
        +int maxIterations
        +int N
        +double dt
        +double learningRate
        +double temperature
        +double velocityWeight
        +double accelerationWeight
        +double jerkWeight
        +double collisionWeight
        +bool enableEarlyStopping
        +double convergenceThreshold
        +int earlyStoppingPatience
        +double outputFrequency
        +double maxVelocity
        +double maxAcceleration
        +double maxJerk
        
        +static optimized() StompConfig
        +static fast() StompConfig
        +static quality() StompConfig
        +toString() string
    }
    
    class ArmFeasibilityChecker {
        -shared_ptr~BVHTree~ _obstacleTree
        -RobotArm _arm
        
        +ArmFeasibilityChecker(arm, obstacleTree)
        +ConfigFeasible(x) bool
        +SegmentFeasible(a, b) bool
    }
    
    MotionGenerator --> StompConfig : configured by
    MotionGenerator --> ArmFeasibilityChecker : uses
    MotionGenerator --> TrajectoryPoint : produces
```

### 4. Robot Component Hierarchy

```mermaid
classDiagram
    class RobotArm {
        -KDL::Chain _kinematicChain
        -Eigen::VectorXd _jointAngles
        -Eigen::VectorXd _jointLimitsMin
        -Eigen::VectorXd _jointLimitsMax
        -Eigen::Affine3d _endEffectorPose
        -shared_ptr~BVHTree~ _obstacleTree
        -bool _collisionCheckEnabled
        
        +RobotArm(urdfString)
        +RobotArm(RobotArm& other)
        +~RobotArm()
        +setJointAngles(angles) void
        +getJointAngles() Eigen::VectorXd
        +forwardKinematics() Eigen::Affine3d
        +inverseKinematics(pose) bool
        +isInCollision(obstacleTree) bool
        +isConfigurationValid() bool
        +getEndEffectorPose() Eigen::Affine3d
        +getJointLimits() pair~VectorXd, VectorXd~
        +getNumJoints() int
        +clone() RobotArm
        
        -updateEndEffectorPose() void
        -checkJointLimits() bool
        -performCollisionCheck() bool
    }
    
    class RobotManager {
        -string _urdfContent
        -vector~Obstacle~ _obstacles
        -tinyxml2::XMLDocument _urdfDoc
        -bool _parsed
        
        +RobotManager()
        +~RobotManager()
        +parseURDF(urdfString) bool
        +getTransformedObstacles() vector~Obstacle~
        +getRobotLinks() vector~Link~
        +getJointInformation() vector~Joint~
        +isValidConfiguration(robotArm) bool
        
        -extractObstacles() void
        -parseLinks() void
        -parseJoints() void
        -transformObstacles() void
    }
    
    RobotArm --> RobotManager : configured by
    UltrasoundScanTrajectoryPlanner --> RobotArm : uses
    UltrasoundScanTrajectoryPlanner --> RobotManager : uses
```

### 5. Collision Detection System

```mermaid
classDiagram
    class BVHTree {
        -vector~BVHNode~ _nodes
        -vector~Obstacle~ _obstacles
        -AABB _rootBoundingBox
        -int _maxDepth
        -int _leafSize
        
        +BVHTree(obstacles)
        +~BVHTree()
        +queryCollision(point) bool
        +queryCollision(robotArm) bool
        +nearestDistance(point) double
        +raycast(origin, direction) RaycastResult
        +getBoundingBox() AABB
        +getObstacles() vector~Obstacle~
        
        -buildTree(obstacles, depth) int
        -splitObstacles(obstacles) pair~vector~Obstacle~, vector~Obstacle~~
        -computeAABB(obstacles) AABB
        -traverseTree(point, nodeIndex) bool
    }
    
    class BVHNode {
        +AABB boundingBox
        +int leftChild
        +int rightChild  
        +vector~int~ obstacleIndices
        +bool isLeaf
    }
    
    class Obstacle {
        +GeometryType type
        +Eigen::Vector3d position
        +Eigen::Vector3d dimensions
        +Eigen::Quaterniond orientation
        +string materialProperties
        
        +containsPoint(point) bool
        +distanceToPoint(point) double
        +getBoundingBox() AABB
    }
    
    BVHTree --> BVHNode : contains
    BVHTree --> Obstacle : organizes
    MotionGenerator --> BVHTree : queries
    RobotArm --> BVHTree : collision checks
```

## Data Flow Implementation

### 1. planTrajectories Method Flow

```mermaid
graph TD
    subgraph "Method Entry"
        Validate[Validate Input Data<br/>- Environment not empty<br/>- Current joints = 7 DOF<br/>- Poses not empty]
        ClearTraj[Clear Previous Trajectories<br/>_trajectories.clear()]
    end
    
    subgraph "Single Pose Handling" 
        SingleCheck{poses.size() == 1?}
        SinglePlan[Plan Direct Movement<br/>- planCheckpoints(poses, currentJoints)<br/>- Use first valid checkpoint<br/>- Single trajectory request]
        SingleResult[Store Single Trajectory<br/>- contactFlag = false<br/>- Return early]
    end
    
    subgraph "Multi-Pose Processing"
        MultiPlan[Plan All Checkpoints<br/>- planCheckpoints(poses, currentJoints)<br/>- Extract checkpoints & validSegments<br/>- Identify firstValidIndex]
        BuildRequests[Build Repositioning Requests<br/>- currentJoints → firstValidCheckpoint<br/>- Inter-segment transitions<br/>- Store descriptions for logging]
    end
    
    subgraph "Batch Processing"
        ChooseAlg{useHauserForRepositioning?}
        BatchSTOMP[planTrajectoryBatch<br/>- STOMP optimization<br/>- Parallel processing<br/>- Shared SDF cache]
        BatchHauser[planTrajectoryBatchHauser<br/>- RRT Connect + Hauser<br/>- Physics-aware optimization<br/>- Parallel processing]
    end
    
    subgraph "Trajectory Assembly"
        AddInitial[Add Initial Repositioning<br/>- currentJoints → firstValidCheckpoint<br/>- contactFlag = false]
        
        loop ContactSegments[For Each Valid Segment]
            CheckSingle{start == end?}
            SinglePoint[Create Single Point<br/>- Zero velocity/acceleration<br/>- contactFlag = true]
            MultiPoint[Generate Contact Trajectory<br/>- generateTrajectoryFromCheckpoints<br/>- Time-optimal through waypoints<br/>- contactFlag = true]
            AddInter[Add Inter-segment Repositioning<br/>- If not last segment<br/>- contactFlag = false]
        end
    end
    
    subgraph "Final Processing"
        FixDisc[Fix Discontinuities<br/>- fixTrajectoryDiscontinuities()<br/>- Add correction segments if needed]
        PrintTiming[Print Segment Timing<br/>- printSegmentTimes()<br/>- Performance analysis]
        ReturnTrue[Return true<br/>- Success indication]
    end
    
    Validate --> ClearTraj
    ClearTraj --> SingleCheck
    SingleCheck -->|Yes| SinglePlan
    SingleCheck -->|No| MultiPlan
    SinglePlan --> SingleResult
    MultiPlan --> BuildRequests
    BuildRequests --> ChooseAlg
    ChooseAlg -->|STOMP| BatchSTOMP
    ChooseAlg -->|Hauser| BatchHauser
    BatchSTOMP --> AddInitial
    BatchHauser --> AddInitial
    AddInitial --> ContactSegments
    ContactSegments --> CheckSingle
    CheckSingle -->|Yes| SinglePoint
    CheckSingle -->|No| MultiPoint
    SinglePoint --> AddInter
    MultiPoint --> AddInter
    AddInter --> FixDisc
    FixDisc --> PrintTiming
    PrintTiming --> ReturnTrue
    
    classDef validation fill:#ffebee
    classDef singlePath fill:#e8f5e8
    classDef multiPath fill:#e3f2fd
    classDef batch fill:#f3e5f5
    classDef assembly fill:#fff3e0
    classDef final fill:#f1f8e9
    
    class Validate,ClearTraj validation
    class SingleCheck,SinglePlan,SingleResult singlePath
    class MultiPlan,BuildRequests multiPath
    class ChooseAlg,BatchSTOMP,BatchHauser batch
    class AddInitial,ContactSegments,CheckSingle,SinglePoint,MultiPoint,AddInter assembly
    class FixDisc,PrintTiming,ReturnTrue final
```

### 2. Batch Processing Implementation

```mermaid
sequenceDiagram
    participant PT as planTrajectories
    participant TP as ThreadPool
    participant SDF as SDF Cache
    participant MG as MotionGenerator
    participant STOMP as STOMP Algorithm
    participant BVH as BVHTree
    
    Note over PT,BVH: Initialization Phase
    PT->>PT: initializeSharedSdf()
    PT->>SDF: Create 3D voxel grid
    SDF->>BVH: Query obstacle distances
    BVH-->>SDF: Distance values
    SDF-->>PT: Cached SDF ready
    
    PT->>TP: Initialize thread pool (physicalCores)
    
    Note over PT,BVH: Batch Processing Phase
    loop For each batch (batchSize = physicalCores/2)
        PT->>TP: Post trajectory planning tasks
        
        par Parallel Trajectory Planning
            TP->>MG: Create with shared SDF
            MG->>MG: setWaypoints(startJoints, targetJoints)
            MG->>STOMP: performSTOMP(config, threadPool, threadId)
            
            loop STOMP Optimization
                STOMP->>STOMP: Generate noisy samples
                STOMP->>SDF: Query collision costs (cached)
                STOMP->>STOMP: Evaluate costs & update trajectory
            end
            
            STOMP-->>MG: Optimized trajectory
            MG-->>TP: TrajectoryPoint vector
        and
            Note over TP,BVH: Additional parallel trajectories...
        end
        
        TP-->>PT: Completed batch results
    end
    
    Note over PT,BVH: Results Assembly
    PT->>PT: Collect all trajectory results
    PT->>PT: Handle failed trajectories (two-step fallback)
    PT-->>PT: Complete repositioning trajectories
```

### 3. Trajectory Validation and Correction

```mermaid
graph TB
    subgraph "Discontinuity Detection Flow"
        Start[Input: Trajectory Segments]
        
        subgraph "Analysis Loop"
            GetSegments[Get Adjacent Segments<br/>segment[i] and segment[i+1]]
            CheckEmpty{Empty segments?}
            GetEndpoints[Extract Endpoints<br/>currentEnd = segment[i].back()<br/>nextStart = segment[i+1].front()]
            CheckSizes{Position vector<br/>sizes match?}
            CalcDiffs[Calculate Joint Differences<br/>maxJointDiff = max(|nextStart[j] - currentEnd[j]|)<br/>worstJoint = argmax(differences)]
            CheckThreshold{maxJointDiff > 0.01 rad?}
            LogDisc[Log Discontinuity<br/>- Segment indices<br/>- Worst joint<br/>- Angle difference in degrees]
            AddCorrection[Call addCorrectionSegment<br/>- segmentIndex<br/>- currentEnd<br/>- nextStart]
        end
        
        CheckComplete{All segments<br/>checked?}
        Summary[Generate Summary<br/>- Total discontinuities found<br/>- Classification (minor/major)<br/>- Maximum discontinuity angle]
    end
    
    subgraph "Correction Generation Flow" 
        CorrStart[addCorrectionSegment Entry]
        CreateMG[Create MotionGenerator<br/>with shared SDF]
        ResetTimes[Reset trajectory times<br/>correctionStart.time = 0.0<br/>correctionEnd.time = 0.0]
        GenTraj[Generate Time-Optimal Segment<br/>computeTimeOptimalSegment(<br/>  correctionStart,<br/>  correctionEnd,<br/>  fromPoint.time)]
        ValidateResult{Correction trajectory<br/>not empty?}
        AdjustTiming[Adjust Timing<br/>point.time += currentSegmentEndTime]
        AppendPoints[Append to Current Segment<br/>Skip first point to avoid duplication<br/>Ensure exact endpoint matching]
        CorrSuccess[Return Success]
        CorrFailed[Return Failed]
    end
    
    Start --> GetSegments
    GetSegments --> CheckEmpty
    CheckEmpty -->|Yes| CheckComplete
    CheckEmpty -->|No| GetEndpoints
    GetEndpoints --> CheckSizes
    CheckSizes -->|No| CheckComplete
    CheckSizes -->|Yes| CalcDiffs
    CalcDiffs --> CheckThreshold
    CheckThreshold -->|No| CheckComplete
    CheckThreshold -->|Yes| LogDisc
    LogDisc --> AddCorrection
    AddCorrection --> CorrStart
    CorrStart --> CreateMG
    CreateMG --> ResetTimes
    ResetTimes --> GenTraj
    GenTraj --> ValidateResult
    ValidateResult -->|Yes| AdjustTiming
    ValidateResult -->|No| CorrFailed
    AdjustTiming --> AppendPoints
    AppendPoints --> CorrSuccess
    CorrSuccess --> CheckComplete
    CorrFailed --> CheckComplete
    CheckComplete -->|No| GetSegments
    CheckComplete -->|Yes| Summary
    
    classDef detection fill:#e3f2fd
    classDef analysis fill:#f3e5f5
    classDef correction fill:#fff3e0
    classDef result fill:#e8f5e8
    
    class Start,GetSegments,CheckEmpty,GetEndpoints detection
    class CheckSizes,CalcDiffs,CheckThreshold,LogDisc,AddCorrection analysis
    class CorrStart,CreateMG,ResetTimes,GenTraj,ValidateResult,AdjustTiming,AppendPoints correction
    class CheckComplete,Summary,CorrSuccess,CorrFailed result
```

## Performance Implementation Details

### Thread Pool and SDF Caching

```mermaid
classDiagram
    class ThreadPoolManager {
        -shared_ptr~boost::asio::thread_pool~ _sharedThreadPool
        -HardwareConfig _hardwareConfig
        -atomic~size_t~ completedTrajectories
        -atomic~size_t~ successfulTrajectories
        
        +initializeThreadPool() void
        +getOptimalThreadCount() size_t
        +getBatchSize() size_t
        +preWarmThreads() void
        +submitBatch(tasks) vector~future~void~~
        +waitForCompletion(futures) void
        
        -detectHardware() HardwareConfig
    }
    
    class SDFCacheManager {
        -vector~vector~vector~double~~~ _sharedSdf
        -Eigen::Vector3d _sharedSdfMinPoint
        -Eigen::Vector3d _sharedSdfMaxPoint  
        -double _sharedSdfResolution
        -bool _sdfCacheInitialized
        -mutex _sdfMutex
        
        +initializeSharedSdf() void
        +createMotionGeneratorWithSharedSdf(arm) unique_ptr~MotionGenerator~
        +isSdfInitialized() bool
        +getSdfMemoryUsage() size_t
        
        -computeSDF(obstacleTree) void
        -validateSdfCache() bool
    }
    
    UltrasoundScanTrajectoryPlanner --> ThreadPoolManager : uses
    UltrasoundScanTrajectoryPlanner --> SDFCacheManager : uses
```

### STOMP Implementation Details

```mermaid
classDiagram
    class StompOptimizer {
        -StompConfig _config
        -RobotArm _arm
        -shared_ptr~BVHTree~ _obstacleTree
        -Eigen::MatrixXd _trajectory
        -vector~Eigen::MatrixXd~ _noisySamples
        -vector~double~ _costs
        -default_random_engine _generator
        -normal_distribution~double~ _noise
        
        +performSTOMP(config, threadPool, threadId) bool
        +setInitialTrajectory(waypoints) void
        +generateNoisySamples() void
        +evaluateAllCosts() void
        +updateTrajectoryWeighted() void
        +checkConvergence() bool
        +handleEarlyStopping() bool
        
        -computeCollisionCost(sample) double
        -computeSmoothnessCost(sample) double
        -computeConstraintCost(sample) double
        -applyNoiseToSample(sample, variance) void
        -normalizeWeights(costs) vector~double~
        -validateFinalTrajectory() bool
    }
    
    class CostFunctionSuite {
        +static computeObstacleCost(trajectory, sdf) double
        +static computeVelocityCost(trajectory, maxVel) double
        +static computeAccelerationCost(trajectory, maxAcc) double
        +static computeJerkCost(trajectory) double
        +static computeJointLimitCost(trajectory, limits) double
        +static computeTotalCost(trajectory, weights) double
        
        -static sdfQuery(position, sdf, minPoint, maxPoint, resolution) double
        -static finiteDifference(trajectory, order) Eigen::MatrixXd
        -static sigmoid(x, steepness) double
    }
    
    MotionGenerator --> StompOptimizer : uses
    StompOptimizer --> CostFunctionSuite : evaluates costs with
```

## Error Handling and Fallback Mechanisms

### Exception Hierarchy

```mermaid
classDiagram
    class TrajectoryPlanningException {
        <<abstract>>
        +TrajectoryPlanningException(message)
        +what() string
        #message string
    }
    
    class StompFailedException {
        +StompFailedException(message)
        +getIterationCount() int
        +getLastCost() double
        -iterationCount int
        -lastCost double
    }
    
    class StompTimeoutException {
        +StompTimeoutException(message, timeoutSeconds)
        +getTimeoutDuration() double
        -timeoutSeconds double
    }
    
    class CheckpointPlanningException {
        +CheckpointPlanningException(message, poseIndex)
        +getPoseIndex() int
        -poseIndex int
    }
    
    class CollisionDetectionException {
        +CollisionDetectionException(message, configuration)
        +getConfiguration() Eigen::VectorXd
        -configuration Eigen::VectorXd
    }
    
    TrajectoryPlanningException <|-- StompFailedException
    TrajectoryPlanningException <|-- StompTimeoutException
    TrajectoryPlanningException <|-- CheckpointPlanningException
    TrajectoryPlanningException <|-- CollisionDetectionException
```

### Fallback Strategy Implementation

```mermaid
graph TB
    subgraph "STOMP Fallback Strategy"
        STOMPAttempt[Primary STOMP Attempt<br/>- Optimized configuration<br/>- Single trajectory request]
        STOMPSuccess{STOMP Success?}
        
        subgraph "Two-Step Fallback"
            Step1[Step 1: startJoints → currentJoints<br/>- Generate intermediate trajectory<br/>- Come to rest at current position]
            Step1Success{Step 1 Success?}
            Step2[Step 2: currentJoints → targetJoints<br/>- Generate final trajectory<br/>- Start from rest]
            Step2Success{Step 2 Success?}
            CombineTraj[Combine Trajectories<br/>- Concatenate step1 + step2<br/>- Skip duplicate point<br/>- Maintain timing continuity]
        end
        
        FallbackFailed[Log Complete Failure<br/>- Both STOMP attempts failed<br/>- Return empty trajectory]
    end
    
    STOMPAttempt --> STOMPSuccess
    STOMPSuccess -->|Yes| Success[Return Optimized Trajectory]
    STOMPSuccess -->|No| Step1
    Step1 --> Step1Success
    Step1Success -->|No| FallbackFailed
    Step1Success -->|Yes| Step2
    Step2 --> Step2Success
    Step2Success -->|No| FallbackFailed
    Step2Success -->|Yes| CombineTraj
    CombineTraj --> Success
    
    classDef primary fill:#e3f2fd
    classDef fallback fill:#fff3e0
    classDef success fill:#e8f5e8
    classDef failure fill:#ffebee
    
    class STOMPAttempt primary
    class Step1,Step1Success,Step2,Step2Success,CombineTraj fallback
    class Success success
    class FallbackFailed failure
```

This detailed implementation analysis shows how `planTrajectories` coordinates multiple sophisticated components to achieve robust, high-performance trajectory planning for medical ultrasound applications. The modular design enables both performance optimization through parallel processing and reliability through comprehensive fallback mechanisms.