# Minimal Trajectory Planning Enhancement Plan

## Current System Issue

The current system creates **too many short trajectory segments** because:

1. **Every invalid pose creates a segment boundary** → Many small fragments
2. **Conservative segmentation** → Unnecessary breaks in scan trajectories  
3. **Single-pose segments** → Inefficient and jerky motion

### Current Workflow Problem:
```
Scan Poses: [P1, P2_invalid, P3, P4_invalid, P5, P6, P7_invalid, P8]
                ↓
Current Result: [P1] → [P3] → [P5,P6] → [P8]  (4 separate short trajectories)
```

## Proposed Enhancement

**Create longer, smoother scan trajectories** by:

1. **Skip invalid poses entirely** (don't create boundaries)
2. **Only segment on stark configuration changes** (not pose validity)
3. **Generate quintic splines** through longer sequences of valid poses
4. **Use STOMP variant** that terminates once collision-free solution found

### Enhanced Workflow:
```
Scan Poses: [P1, P2_invalid, P3, P4_invalid, P5, P6, P7_invalid, P8]
                ↓
Filter Valid: [P1, P3, P5, P6, P8]
                ↓
Check Config Changes: P1→P3 (OK), P3→P5 (OK), P5→P6 (OK), P6→P8 (STARK!)
                ↓
Enhanced Result: [P1,P3,P5,P6] → [P8]  (2 longer, smoother trajectories)
                ↓
For each segment: Generate Quintic Polynomial → Apply STOMP until collision-free
```

## Two-Stage Trajectory Generation Approach

### Stage 1: Quintic Polynomial Initial Trajectory
- Create smooth initial trajectory through valid poses using existing `generateTrajectoryFromCheckpoints()`
- This provides an excellent starting point that respects velocity/acceleration limits

### Stage 2: STOMP Optimization Until Collision-Free  
- Apply STOMP to the quintic trajectory to handle obstacles
- **Terminate as soon as collision-free solution is found** (don't waste iterations)
- If quintic trajectory is already collision-free, skip STOMP entirely

## Minimal Enhancements Needed

### 1. **Skip Invalid Poses Instead of Segmenting**
Don't create boundaries at invalid poses - just omit them from the sequence.

### 2. **Smarter Configuration Change Detection** 
Only segment when configuration changes are truly "stark" - be more permissive to create longer sequences.

### 3. **Two-Stage Trajectory Generation**
- **Stage 1**: Generate quintic polynomial through valid poses
- **Stage 2**: Apply STOMP optimization until collision-free (or skip if already collision-free)

## Implementation Plan

### Phase 1: Modified Pose Filtering (1 day)

**File: `libs/TrajectoryLib/src/Planning/PathPlanner.cpp`**

Modify `planCheckpoints()` to skip invalid poses instead of creating boundaries:

```cpp
PathPlanner::CheckpointPlanResult PathPlanner::planCheckpoints(
    const std::vector<Eigen::Affine3d> &originalScanPoses,
    const Eigen::VectorXd &currentJoints)
{
    CheckpointPlanResult result;
    if (originalScanPoses.empty()) return result;

    std::vector<RobotArm> validArms;
    std::vector<size_t> validPoseIndices;
    
    // First, filter to only valid poses (skip invalid ones entirely)
    for (size_t poseIdx = 0; poseIdx < originalScanPoses.size(); ++poseIdx) {
        auto [arm, isValid] = selectGoalPose(originalScanPoses[poseIdx]);
        
        if (isValid && !armHasCollision(arm)) {
            validArms.push_back(arm);
            validPoseIndices.push_back(poseIdx);
            result.checkpoints.push_back({arm, true});
        }
        // Skip invalid poses - don't add them to checkpoints at all
    }
    
    if (validArms.empty()) {
        return result; // No valid poses found
    }
    
    // Now create segments based only on stark configuration changes
    std::vector<size_t> segmentBoundaries;
    segmentBoundaries.push_back(0); // Always start with first valid pose
    
    for (size_t i = 1; i < validArms.size(); ++i) {
        if (hasStarkConfigurationChange(validArms[i-1], validArms[i])) {
            segmentBoundaries.push_back(i);
        }
    }
    
    // Create validSegments from boundaries
    for (size_t i = 0; i < segmentBoundaries.size(); ++i) {
        size_t segStart = segmentBoundaries[i];
        size_t segEnd = (i + 1 < segmentBoundaries.size()) ? 
                       segmentBoundaries[i + 1] - 1 : 
                       validArms.size() - 1;
        
        result.validSegments.push_back({segStart, segEnd});
    }
    
    result.firstValidIndex = 0; // Always valid since we filtered
    return result;
}
```

### Phase 2: Enhanced Configuration Change Detection (1 day)

Add the stark configuration change detection function with **more permissive thresholds**:

```cpp
bool PathPlanner::hasStarkConfigurationChange(const RobotArm& arm1, const RobotArm& arm2) {
    // More permissive joint space thresholds to create longer scan sequences
    const double MAX_JOINT_DISTANCE = M_PI_2 * 2.0;  // Much more permissive (π instead of π/2)
    const double MAX_SINGLE_JOINT = M_PI_4 * 1.5;    // More permissive (~67° instead of 45°)
    
    auto joints1 = arm1.getJointAngles();
    auto joints2 = arm2.getJointAngles();
    
    double totalDist = 0, maxDiff = 0;
    for (int i = 0; i < 7; i++) {
        double diff = std::abs(joints2[i] - joints1[i]);
        totalDist += diff * diff;
        maxDiff = std::max(maxDiff, diff);
    }
    totalDist = std::sqrt(totalDist);
    
    // Check for truly stark changes (must be quite large)
    if (totalDist > MAX_JOINT_DISTANCE || maxDiff > MAX_SINGLE_JOINT) {
        return true;
    }
    
    // Cartesian space checks for major discontinuities
    auto pose1 = arm1.getEndeffectorPose();
    auto pose2 = arm2.getEndeffectorPose();
    
    double cartesianDist = (pose2.translation() - pose1.translation()).norm();
    if (cartesianDist > 0.25) return true; // 25cm jump (quite large)
    
    Eigen::Matrix3d orientDiff = pose2.linear() * pose1.linear().transpose();
    Eigen::AngleAxisd angleAxis(orientDiff);
    if (std::abs(angleAxis.angle()) > 1.0) return true; // ~60 degree rotation (quite large)
    
    return false;
}
```

### Phase 3: Two-Stage Trajectory Generation (2 days)

**File: `libs/USLib/src/USTrajectoryPlanner.cpp`**

Implement the quintic → STOMP approach for scan trajectories:

```cpp
// Enhanced trajectory generation: Quintic first, then STOMP if needed
std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>
UltrasoundScanTrajectoryPlanner::generateScanTrajectoryTwoStage(
    const std::vector<Eigen::VectorXd>& validCheckpoints,
    const StompConfig& config)
{
    try {
        auto motionGen = new MotionGenerator(*_arm);
        motionGen->setObstacleTree(_obstacleTree);
        
        // Stage 1: Generate quintic polynomial trajectory through checkpoints
        auto quinticTrajectory = motionGen->generateTrajectoryFromCheckpoints(validCheckpoints);
        
        // Check if quintic trajectory is already collision-free
        if (isTrajectoryCollisionFree(quinticTrajectory)) {
            qDebug() << "Quintic trajectory is collision-free, skipping STOMP optimization";
            auto result = std::make_pair(quinticTrajectory, true);
            delete motionGen;
            return result;
        }
        
        // Stage 2: Apply STOMP optimization starting from quintic trajectory
        qDebug() << "Quintic trajectory has collisions, applying STOMP optimization";
        
        // Set the quintic trajectory as the starting point for STOMP
        Eigen::MatrixXd waypoints(validCheckpoints.size(), validCheckpoints[0].size());
        for (size_t i = 0; i < validCheckpoints.size(); ++i) {
            waypoints.row(i) = validCheckpoints[i].transpose();
        }
        motionGen->setWaypoints(waypoints);
        
        // Apply STOMP with early termination when collision-free
        bool success = performSTOMPUntilCollisionFree(motionGen, config);
        
        if (!success) {
            delete motionGen;
            throw std::runtime_error("STOMP failed to find collision-free solution");
        }
        
        auto result = std::make_pair(motionGen->getPath(), true);
        delete motionGen;
        return result;
        
    } catch (const std::exception &e) {
        throw std::runtime_error("Two-stage trajectory generation failed: " + std::string(e.what()));
    }
}

bool UltrasoundScanTrajectoryPlanner::performSTOMPUntilCollisionFree(
    MotionGenerator* motionGen, 
    const StompConfig& config)
{
    const int MAX_TOTAL_ITERATIONS = 300;
    const int CHECK_FREQUENCY = 10; // Check collision every 10 iterations
    
    StompConfig iterativeConfig = config;
    iterativeConfig.maxIterations = CHECK_FREQUENCY;
    
    for (int totalIterations = 0; totalIterations < MAX_TOTAL_ITERATIONS; totalIterations += CHECK_FREQUENCY) {
        unsigned int numThreads = std::thread::hardware_concurrency();
        auto threadPool = std::make_shared<boost::asio::thread_pool>(numThreads);
        
        bool success = motionGen->performSTOMP(iterativeConfig, threadPool);
        threadPool->join();
        
        if (success) {
            auto trajectory = motionGen->getPath();
            if (isTrajectoryCollisionFree(trajectory)) {
                qDebug() << "STOMP found collision-free solution after" << (totalIterations + CHECK_FREQUENCY) << "iterations";
                return true;
            }
        }
        
        // Continue optimizing if still has collisions
        qDebug() << "Iteration" << (totalIterations + CHECK_FREQUENCY) << "- still has collisions, continuing...";
    }
    
    qDebug() << "STOMP failed to find collision-free solution after" << MAX_TOTAL_ITERATIONS << "iterations";
    return false;
}
```

### Phase 4: Integration with Main Planning Loop (1 day)

**File: `libs/USLib/src/USTrajectoryPlanner.cpp`**

Modify the main `planTrajectories()` to use the new approach:

```cpp
bool UltrasoundScanTrajectoryPlanner::planTrajectories()
{
    // ... existing validation code ...
    
    _trajectories.clear();
    
    // Use enhanced planCheckpoints that skips invalid poses
    auto checkpointResult = _pathPlanner->planCheckpoints(_poses, _currentJoints);
    auto &checkpoints = checkpointResult.checkpoints;
    auto &validSegments = checkpointResult.validSegments;
    
    qDebug() << "Enhanced segmentation created" << validSegments.size() << "scan segments";
    for (const auto &segment : validSegments) {
        qDebug() << "Segment: poses" << segment.first << "to" << segment.second;
    }
    
    // Handle repositioning to first valid pose (if needed)
    if (!checkpoints.empty()) {
        auto repositionTrajectory = planSingleStompTrajectory(
            _currentJoints, 
            checkpoints[0].first.getJointAngles(), 
            StompConfig()
        );
        _trajectories.push_back(repositionTrajectory);
    }
    
    // Process each scan segment with enhanced trajectory generation
    for (const auto &segment : validSegments) {
        std::vector<Eigen::VectorXd> segmentCheckpoints;
        for (size_t i = segment.first; i <= segment.second; ++i) {
            segmentCheckpoints.push_back(checkpoints[i].first.getJointAngles());
        }
        
        // Generate smooth scan trajectory through this segment
        auto scanTrajectory = generateScanTrajectoryWithSTOMP(segmentCheckpoints, StompConfig());
        _trajectories.push_back(scanTrajectory);
    }
    
    return true;
}
```

## Modified Files Summary

### 1. **PathPlanner.cpp** (2 new functions + modified planCheckpoints)
- Modify `planCheckpoints()` to skip invalid poses instead of creating boundaries
- Add `hasStarkConfigurationChange()` for smarter segmentation
- **Result**: Longer valid segments, fewer trajectory breaks

### 2. **USTrajectoryPlanner.cpp** (2 new functions + modified planTrajectories)  
- Add `generateScanTrajectoryWithSTOMP()` for enhanced scan trajectory generation
- Add `performSTOMPWithEarlyTermination()` for efficient optimization
- Modify `planTrajectories()` to use the new approach
- **Result**: Smoother scan trajectories, faster optimization

### 3. **USTrajectoryPlanner.h** (2 method declarations)
- Declare the new scan trajectory generation methods

## Expected Results

### Before Enhancement:
```
Input: 20 scan poses (5 invalid)
Current Result: 8-12 short trajectory segments with many repositioning moves
```

### After Enhancement:  
```
Input: 20 scan poses (5 invalid) 
Enhanced Result: 2-3 long, smooth scan trajectories (invalid poses skipped)
```

### Benefits:
- ✅ **Fewer trajectory segments** (longer scan sequences)
- ✅ **Smoother scanning motion** (quintic splines through more poses)
- ✅ **Faster optimization** (early STOMP termination)
- ✅ **Invalid poses handled gracefully** (skipped, not segmented)
- ✅ **Natural scanning flow** (only break on stark configuration changes)

## Timeline: 5 days total
- Day 1: Modified pose filtering (skip invalid poses)
- Day 2: Enhanced configuration change detection  
- Day 3-4: STOMP with early termination
- Day 5: Integration and testing

This approach creates the smooth, continuous scan trajectories you're looking for while maintaining efficiency and collision safety.
