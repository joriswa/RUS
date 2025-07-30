# TrajectoryLib planTrajectories Visual Summary

This document provides quick visual references and ASCII-based diagrams specifically focused on the `planTrajectories` functionality, suitable for terminal viewing and rapid understanding.

## planTrajectories Quick Reference

### Function Signature and Key Parameters
```
bool planTrajectories(bool useHauserForRepositioning = false, 
                     bool enableShortcutting = true)

Input Requirements:
- _environment: URDF string (not empty)
- _currentJoints: 7-DOF joint configuration
- _poses: vector<Affine3d> scan poses (not empty)

Output:
- _trajectories: vector<pair<TrajectoryPoint[], bool>>
  - bool flag: false=repositioning, true=contact_force
```

### Core Data Flow (ASCII Diagram)

```
INPUT PHASE:
┌─────────────────────┐    ┌──────────────────────┐    ┌─────────────────────┐
│   Scan Poses        │    │   Current Joints     │    │   URDF Environment  │
│ vector<Affine3d>    │    │   VectorXd (7-DOF)   │    │   string            │
└─────────┬───────────┘    └──────────┬───────────┘    └─────────┬───────────┘
          │                           │                          │
          └─────────────┐             │             ┌────────────┘
                        │             │             │
                        ▼             ▼             ▼
                  ┌─────────────────────────────────────────┐
                  │         planTrajectories()              │
                  │                                         │
                  │  ┌─────────────────────────────────┐    │
                  │  │    ENVIRONMENT SETUP            │    │
                  │  │  • RobotManager.parseURDF()     │    │
                  │  │  • BVHTree construction         │    │
                  │  │  • Obstacle tree setup          │    │
                  │  └─────────────────────────────────┘    │
                  │                                         │
                  │  ┌─────────────────────────────────┐    │
                  │  │    CHECKPOINT PLANNING          │    │
                  │  │  • PathPlanner.planCheckpoints()│    │
                  │  │  • IK solving for each pose     │    │
                  │  │  • Validity checking            │    │
                  │  │  • Segment grouping             │    │
                  │  └─────────────────────────────────┘    │
                  │                                         │
                  │  ┌─────────────────────────────────┐    │
                  │  │    BATCH PLANNING               │    │
                  │  │  • Build repositioning requests │    │
                  │  │  • STOMP vs Hauser selection    │    │
                  │  │  • Multi-threaded optimization  │    │
                  │  │  • Shared SDF caching           │    │
                  │  └─────────────────────────────────┘    │
                  │                                         │
                  │  ┌─────────────────────────────────┐    │
                  │  │    TRAJECTORY ASSEMBLY          │    │
                  │  │  • Initial repositioning        │    │
                  │  │  • Contact force segments       │    │
                  │  │  • Inter-segment repositioning  │    │
                  │  │  • Discontinuity correction     │    │
                  │  └─────────────────────────────────┘    │
                  └─────────────────────────────────────────┘
                                        │
                                        ▼
OUTPUT PHASE:
┌─────────────────────────────────────────────────────────────────────────┐
│                        Final Trajectories                               │
│  vector<pair<vector<TrajectoryPoint>, bool>>                           │
│                                                                         │
│  Segment 0: [TrajectoryPoints...] → false (repositioning)              │
│  Segment 1: [TrajectoryPoints...] → true  (contact force)              │
│  Segment 2: [TrajectoryPoints...] → false (repositioning)              │
│  Segment 3: [TrajectoryPoints...] → true  (contact force)              │
│  ...                                                                    │
└─────────────────────────────────────────────────────────────────────────┘
```

## Component Interaction Matrix

```
                    │ Robot │ Robot │ Path  │Motion │ BVH  │Thread│ SDF  │
                    │ Arm   │ Mgr   │Planner│  Gen  │ Tree │ Pool │Cache │
────────────────────┼───────┼───────┼───────┼───────┼──────┼──────┼──────┤
planTrajectories    │   ●   │   ●   │   ●   │   ●   │  ●   │  ●   │  ●   │
────────────────────┼───────┼───────┼───────┼───────┼──────┼──────┼──────┤
Environment Setup   │   ●   │   ●   │   ○   │   ○   │  ●   │  ○   │  ○   │
Checkpoint Planning │   ●   │   ○   │   ●   │   ○   │  ●   │  ○   │  ○   │
Batch Planning      │   ●   │   ○   │   ○   │   ●   │  ●   │  ●   │  ●   │
Contact Generation  │   ●   │   ○   │   ○   │   ●   │  ●   │  ○   │  ●   │
Validation          │   ●   │   ○   │   ○   │   ●   │  ●   │  ○   │  ●   │
────────────────────┼───────┼───────┼───────┼───────┼──────┼──────┼──────┤

Legend: ● = Direct usage, ○ = Indirect usage
```

## Algorithm Decision Tree

```
Algorithm Selection for planTrajectories:

useHauserForRepositioning?
├─ TRUE: RRT+Hauser Pipeline
│  ├─ RRT Connect Path Planning
│  │  ├─ Success → Path Shortcutting (if enabled)
│  │  └─ Failure → Retry once
│  ├─ Hauser Optimization
│  │  ├─ DynamicPath initialization
│  │  ├─ Time-optimal planning
│  │  └─ Feasibility checking
│  └─ Result: Physics-aware trajectory
│
└─ FALSE: STOMP Pipeline (Default)
   ├─ Primary STOMP Attempt
   │  ├─ Success → Return optimized trajectory
   │  └─ Failure → Two-step fallback
   ├─ Two-step Fallback
   │  ├─ Step 1: start → current (rest)
   │  ├─ Step 2: current → target
   │  └─ Combine trajectories
   └─ Result: Smooth, collision-free trajectory
```

## Performance Characteristics (ASCII Charts)

### Execution Time by Component

```
Component Performance (typical 10-pose sequence):
                                                                        
Environment Setup    ████                                    ~50ms
Checkpoint Planning  ████████████                            ~150ms  
Batch Planning       ████████████████████████████████████    ~3000ms
Contact Generation   ████████                                ~100ms
Validation           ██                                      ~20ms
                                                                        
                     0    500   1000  1500  2000  2500  3000ms
```

### Memory Usage by Component

```
Memory Allocation (typical workspace):
                                                                        
SDF Cache           ████████████████████████████████████████  ~40MB
BVH Tree           ████████                                   ~8MB
Trajectory Storage ████                                      ~4MB  
Thread Pool        ██                                        ~2MB
Working Memory     ████████                                  ~8MB
                                                                        
                   0    10    20    30    40    50MB
```

### Thread Utilization Pattern

```
CPU Core Utilization During Batch Planning:
Time →  0s    1s    2s    3s    4s    5s    6s    7s    8s
Core 0: ████████████████████████████████████████████████████
Core 1: ████████████████████████████████████████████████████  
Core 2: ████████████████████████████████████████████████████
Core 3: ████████████████████████████████████████████████████
Core 4: ████████████    ████████████    ████████████████████
Core 5: ████████████    ████████████    ████████████████████
Core 6: ████████████    ████████████    ████████████████████
Core 7: ████████████    ████████████    ████████████████████

Legend: █ = Active, ░ = Idle
Pattern: High utilization during parallel trajectory optimization
```

## Data Structure Quick Reference

### TrajectoryPoint Structure
```cpp
struct TrajectoryPoint {
    vector<double> position;     // 7 joint angles [rad]
    vector<double> velocity;     // 7 joint velocities [rad/s]  
    vector<double> acceleration; // 7 joint accelerations [rad/s²]
    double time;                 // Timestamp [s]
};

Typical memory: 7×3×8 + 8 = 176 bytes per point
Full trajectory: 75 points × 176 bytes = ~13KB per trajectory
```

### CheckpointPlanResult Structure  
```cpp
struct CheckpointPlanResult {
    vector<pair<RobotArm, bool>> checkpoints;      // IK solutions + validity
    vector<pair<size_t, size_t>> validSegments;    // Continuous path segments
    vector<pair<size_t, size_t>> jumpPairs;        // Large discontinuities
    size_t firstValidIndex;                        // Starting checkpoint
};

Example for 10 poses:
checkpoints: 10 entries (~2KB)
validSegments: 2-4 entries (~64 bytes)  
jumpPairs: 0-2 entries (~32 bytes)
firstValidIndex: 1 value (8 bytes)
```

### STOMP Configuration Summary
```cpp
StompConfig::optimized() parameters:
┌─────────────────────┬─────────┬──────────────────────────────┐
│ Parameter           │ Value   │ Purpose                      │
├─────────────────────┼─────────┼──────────────────────────────┤
│ numNoisyTrajectories│    12   │ Parallel sampling diversity  │
│ numBestSamples      │     6   │ Selection for updates        │
│ maxIterations       │   250   │ Optimization iterations      │
│ N (trajectory points)│    75   │ Temporal resolution          │
│ dt (time step)      │   0.1s  │ Discretization               │
│ learningRate        │   0.1   │ Update step size             │
│ temperature         │ 15.9079 │ Sample weighting             │
└─────────────────────┴─────────┴──────────────────────────────┘
```

## Common Usage Patterns

### Basic Usage Pattern
```cpp
// 1. Setup
UltrasoundScanTrajectoryPlanner planner(urdfEnvironment);
planner.setCurrentJoints(currentJointConfig);
planner.setPoses(scanPoseSequence);

// 2. Planning
bool success = planner.planTrajectories(false, true); // STOMP + shortcutting

// 3. Execution
if (success) {
    auto trajectories = planner.getTrajectories();
    for (const auto& [points, isContactForce] : trajectories) {
        if (isContactForce) {
            executeContactTrajectory(points);
        } else {
            executeRepositioningTrajectory(points);
        }
    }
}
```

### Error Handling Pattern
```cpp
try {
    bool success = planner.planTrajectories();
    if (!success) {
        // Handle planning failure
        LOG_ERROR << "Trajectory planning failed";
        return false;
    }
} catch (const StompFailedException& e) {
    // STOMP optimization failed
    LOG_WARNING << "STOMP failed: " << e.what();
    // Try Hauser fallback
    success = planner.planTrajectories(true, true);
} catch (const std::runtime_error& e) {
    // Environment or configuration issues
    LOG_ERROR << "Planning error: " << e.what();
    return false;
}
```

## Troubleshooting Quick Guide

### Common Issues and Solutions

```
Issue: "Environment string is not populated"
┌─────────────────────────────────────────────────────────┐
│ Cause: setEnvironment() not called or empty URDF       │
│ Solution: Call setEnvironment(validURDFString)          │  
│ Check: Verify URDF parsing with parser validation      │
└─────────────────────────────────────────────────────────┘

Issue: "Current joints are not populated or are not = 7"
┌─────────────────────────────────────────────────────────┐
│ Cause: setCurrentJoints() not called or wrong size     │
│ Solution: Call setCurrentJoints(7DOF_VectorXd)         │
│ Check: Verify vector size equals robot DOF             │
└─────────────────────────────────────────────────────────┘

Issue: "Target poses are not populated"  
┌─────────────────────────────────────────────────────────┐
│ Cause: setPoses() not called or empty vector           │
│ Solution: Call setPoses(vector<Affine3d>)               │
│ Check: Verify poses are within robot workspace         │
└─────────────────────────────────────────────────────────┘

Issue: "No valid checkpoint found"
┌─────────────────────────────────────────────────────────┐
│ Cause: All poses have failed IK or collision           │
│ Solution: Adjust poses or environment obstacles        │
│ Check: Use pose validation before planning             │
└─────────────────────────────────────────────────────────┘

Issue: STOMP optimization failures
┌─────────────────────────────────────────────────────────┐
│ Cause: Complex environment or conflicting constraints  │
│ Solution: Try Hauser fallback (useHauserForRepos=true) │
│ Check: Increase maxIterations or adjust cost weights   │
└─────────────────────────────────────────────────────────┘
```

### Performance Optimization Tips

```
Memory Optimization:
├─ Enable SDF caching for batch operations
├─ Use hardware-detected thread count  
├─ Process in optimal batch sizes
└─ Clear old trajectories before new planning

Speed Optimization:
├─ Use STOMP for smooth trajectories (quality)
├─ Use Hauser for fast trajectories (speed)
├─ Enable shortcutting for path reduction
└─ Reduce STOMP iterations for quick results

Quality Optimization:
├─ Increase STOMP maxIterations (250+)
├─ Use fine temporal resolution (dt=0.05-0.1)
├─ Enable trajectory validation
└─ Use correction segments for continuity
```

## Integration Checklist

### Prerequisites
```
□ Environment URDF string available
□ Current robot joint configuration known  
□ Target scan poses defined (Affine3d format)
□ Obstacle information up-to-date
□ Hardware detection completed
□ Thread pool initialized
```

### Planning Phase
```
□ Environment setup completed successfully
□ All poses have valid IK solutions
□ Checkpoint planning identified valid segments  
□ SDF cache initialized (for performance)
□ Algorithm selection made (STOMP vs Hauser)
□ Batch processing parameters configured
```

### Validation Phase
```
□ All requested trajectories generated
□ Trajectory continuity validated
□ Correction segments added if needed
□ Timing information consistent
□ Memory usage within acceptable limits  
□ Performance metrics logged
```

### Execution Phase
```
□ Trajectory format matches robot controller
□ Contact force flags properly interpreted
□ Safety limits verified
□ Real-time execution capability confirmed
□ Error handling mechanisms in place
□ Monitoring and logging active
```

This visual summary provides the essential information needed to understand, implement, and troubleshoot the `planTrajectories` functionality efficiently.