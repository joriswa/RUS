# USTrajectoryPlanner::planTrajectories Algorithm

## Overview

The `planTrajectories` function serves as the main orchestration engine for ultrasound scanning trajectory generation, coordinating multi-level planning from task specification to executable robot trajectories. The algorithm implements a hierarchical decomposition approach with parallel execution optimization.

## Algorithm Architecture

### Input Validation and Initialization

The function validates three critical inputs:
- **Environment representation**: Obstacle geometry for collision avoidance
- **Current joint configuration**: 7-DOF robot state (Franka robot)
- **Target poses**: Sequence of desired end-effector poses for ultrasound scanning

Thread pool initialization creates `2 × hardware_concurrency` worker threads using `boost::asio::thread_pool`, indicating optimized resource utilization for I/O-bound operations and nested parallelism.

### Hierarchical Planning Decomposition

#### Level 1: Checkpoint Planning
Invokes `planCheckpoints()` to transform Cartesian pose sequences into feasible joint configurations:
- **Input**: Target poses $\mathbf{T}_i \in SE(3)$, current joints $\mathbf{q}_0$
- **Output**: Joint configurations $\{\mathbf{q}_i\}$, validity flags, motion segments
- **Result**: Decomposition into continuous motion regions and transition boundaries

#### Level 2: Segmentation Analysis
Extracts three key planning components:
- **Checkpoints**: Array of `(RobotArm, validity_flag)` pairs
- **Valid segments**: Index ranges `[start, end]` for continuous motion regions  
- **Transition points**: Boundaries requiring discrete repositioning maneuvers

#### Level 3: Trajectory Generation
Creates three distinct trajectory types for parallel execution:

1. **Initial trajectory**: Current position → first valid checkpoint
2. **Segment trajectories**: Multi-waypoint smooth motion within continuous regions
3. **Transition trajectories**: Point-to-point motion between segment boundaries

### Parallel Execution Strategy

#### Task Decomposition
Total parallel tasks: $N_{tasks} = 1 + N_{segments} + (N_{segments} - 1)$

- 1 initial trajectory task
- $N_{segments}$ continuous motion tasks  
- $(N_{segments} - 1)$ inter-segment transition tasks

#### Resource Management
Each parallel task receives:
- **Dedicated MotionGenerator instance**: Prevents state conflicts
- **Local PathPlanner instance** (Hauser algorithm): Thread-safety isolation
- **Shared obstacle tree**: Read-only collision environment
- **Algorithm configuration**: Immutable planning parameters

#### Synchronization Pattern
Uses promise/future mechanism for result collection:
```cpp
std::promise<TrajectoryResult> promise;
std::future<TrajectoryResult> future = promise.get_future();
```

### Algorithm-Specific Processing

#### STOMP Algorithm Path
- **Point-to-point**: 2-waypoint matrix (start, goal) with `performSTOMP()`
- **Multi-checkpoint**: `generateTrajectoryFromCheckpoints()` for smoother multi-point trajectories
- **Nested parallelism**: Passes thread pool to STOMP for internal optimization

#### Hauser Algorithm Path  
- **Preprocessing requirement**: Collision-free waypoints via path planning
- **Point-to-point**: `PathPlanner::runPathFinding()` → `performHauser()`
- **Multi-checkpoint**: Path planning between consecutive pairs → waypoint concatenation
- **Resource isolation**: Local PathPlanner instances prevent thread contention

### Result Collection and Integration

#### Synchronization
- Sequential `future.get()` calls maintain trajectory ordering
- `threadPool->join()` ensures task completion before resource cleanup
- Exception propagation via `std::current_exception()` for robust error handling

#### Output Structure
Each trajectory element: `std::pair<std::vector<TrajectoryPoint>, bool>`
- **TrajectoryPoint**: Joint positions, velocities, accelerations over time
- **Boolean flag**: Segment trajectory (true) vs. point-to-point (false)

## Computational Complexity

- **Time complexity**: $O(N_{segments} \times C_{primitive})$ where $C_{primitive}$ is motion primitive computation cost
- **Space complexity**: $O(N_{checkpoints} + N_{trajectories})$  
- **Parallel efficiency**: Scales with available CPU cores up to trajectory count limit

## Key Algorithmic Contributions

### Multi-Level Planning Framework
Hierarchical decomposition from task-level poses to time-parameterized joint trajectories with intermediate checkpoint and segmentation phases.

### Parallel Trajectory Orchestration
Thread pool-based parallel execution with algorithm-agnostic interface supporting both STOMP and Hauser motion primitives.

### Medical Robotics Specialization
Ultrasound-specific trajectory requirements including orientation-aware repositioning and collision-safe motion in clinical environments.

### Resource-Safe Parallel Programming
Sophisticated thread safety patterns with local resource instantiation, promise/future synchronization, and explicit memory management for real-time robotics applications.

This algorithm enables efficient generation of complex ultrasound scanning trajectories while exploiting modern multi-core hardware for computational performance suitable for clinical workflows.
