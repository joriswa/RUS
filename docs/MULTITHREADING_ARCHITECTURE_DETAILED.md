# Multithreading and Worker Pool Architecture in USTrajectoryPlanner

## Thread Pool Architecture

### Configuration and Initialization

The USTrajectoryPlanner implements a sophisticated multithreading architecture using `boost::asio::thread_pool`:

```cpp
unsigned int numThreads = std::thread::hardware_concurrency();
std::shared_ptr<boost::asio::thread_pool> threadPool = 
    std::make_shared<boost::asio::thread_pool>(2 * numThreads);
```

**Key Design Decisions:**
- **Pool Size**: `2 × hardware_concurrency` threads
- **Rationale**: Accounts for I/O-bound operations and nested parallelism within motion primitives
- **Type**: `boost::asio::thread_pool` provides work-stealing queue implementation for load balancing

### Thread Pool Pattern Implementation

#### Work Distribution Mechanism
Uses `boost::asio::post()` for non-blocking task submission:

```cpp
boost::asio::post(*threadPool, [captured_data]() {
    // Trajectory computation task
    try {
        auto result = computeTrajectory(/* parameters */);
        promise.set_value(result);
    } catch (...) {
        promise.set_exception(std::current_exception());
    }
});
```

**Advantages:**
- **Asynchronous submission**: Main thread doesn't block during task creation
- **Work-stealing**: Automatic load balancing across worker threads
- **Exception safety**: Proper exception propagation through promise/future

#### Task Granularity Strategy

The algorithm creates three categories of parallel tasks:

1. **Initial trajectory task** (1 task):
   - Current joints → first valid checkpoint
   - Single point-to-point motion

2. **Segment trajectory tasks** (N tasks):
   - Multi-waypoint trajectories within continuous motion regions
   - Most computationally intensive tasks

3. **Transition trajectory tasks** (N-1 tasks):
   - Point-to-point motions between segment boundaries
   - Lighter computational load

**Load Balancing Considerations:**
- Segment tasks have variable complexity (depends on waypoint count)
- Transition tasks are relatively uniform (2-waypoint trajectories)
- Work-stealing queue automatically redistributes load imbalances

## Resource Management and Thread Safety

### Shared Resource Strategy

#### Read-Only Shared Resources
Resources safely shared across threads without synchronization:

```cpp
// Obstacle environment (read-only after initialization)
std::shared_ptr<BVHTree> _obstacleTree;

// Robot model template (immutable)
RobotArm* _arm;

// Planning configuration (immutable during execution)
TrajectoryPlanningConfig _planningConfig;
```

#### Per-Thread Resource Instantiation
Critical mutable resources created locally for each task:

```cpp
// Each task creates its own MotionGenerator
auto motionGen = new MotionGenerator(*_arm);
motionGen->setObstacleTree(_obstacleTree);

// Local PathPlanner for thread safety (Hauser algorithm)
auto localPathPlanner = std::make_unique<PathPlanner>();
localPathPlanner->setObstacleTree(_obstacleTree);
```

**Thread Safety Benefits:**
- **No state conflicts**: Each thread operates on independent objects
- **No synchronization overhead**: Eliminates mutex contention
- **Scalability**: Performance scales linearly with core count

### Memory Management in Multithreaded Context

#### RAII and Smart Pointers
```cpp
// Automatic cleanup with smart pointers
auto localPathPlanner = std::make_unique<PathPlanner>();

// Manual cleanup for legacy interfaces
auto motionGen = new MotionGenerator(*_arm);
// ... use motionGen ...
delete motionGen;  // Explicit cleanup before task completion
```

#### Exception Safety
Each task wrapped in comprehensive exception handling:

```cpp
try {
    auto result = performTrajectoryComputation();
    promises[taskIndex].set_value(result);
} catch (...) {
    promises[taskIndex].set_exception(std::current_exception());
}
```

**Guarantees:**
- **Resource cleanup**: Objects properly destroyed even on exceptions
- **Promise resolution**: Every promise receives either value or exception
- **No deadlocks**: Main thread never blocks indefinitely

## Nested Parallelism Architecture

### Two-Level Parallel Hierarchy

#### Level 1: Trajectory-Level Parallelism
Main thread pool distributes trajectory computation tasks:
- Each trajectory segment computed independently
- Parallel execution across available CPU cores
- Managed by boost::asio::thread_pool

#### Level 2: Motion Primitive Parallelism
STOMP algorithm receives shared thread pool for internal optimization:

```cpp
// Pass thread pool down to motion primitive
motionGen->performSTOMP(_planningConfig.stompConfig, threadPool);
```

**Benefits:**
- **Resource sharing**: Same threads used for both levels
- **Efficiency**: No thread creation overhead for nested parallelism
- **Scalability**: Adapts to both few large segments and many small segments

### Algorithm-Specific Thread Utilization

#### STOMP Algorithm
- **Supports nested parallelism**: Uses passed thread pool for internal optimization
- **Parallel sampling**: Multiple trajectory candidates evaluated simultaneously
- **Gradient computation**: Parallelized cost function evaluations

#### Hauser Algorithm
- **Sequential execution**: No internal parallelism support
- **Thread pool usage**: Only at trajectory-level granularity
- **Path planning**: Sequential RRT/RRT* algorithms per task

## Synchronization and Result Collection

### Promise/Future Synchronization Pattern

#### Task Result Management
```cpp
std::vector<std::promise<TrajectoryResult>> promises(numTasks);
std::vector<std::future<TrajectoryResult>> futures;

for (auto& promise : promises) {
    futures.push_back(promise.get_future());
}

// Collect results in order
for (auto& future : futures) {
    _trajectories.push_back(future.get());  // Blocking wait
}
```

**Synchronization Properties:**
- **Order preservation**: Results collected in task submission order
- **Blocking collection**: Main thread waits for each result sequentially
- **Exception propagation**: Failed tasks throw exceptions during collection

#### Thread Pool Lifecycle Management
```cpp
// Ensure all tasks complete before destruction
threadPool->join();
```

**Resource Guarantees:**
- **Complete execution**: All submitted tasks finish before thread pool destruction
- **Resource cleanup**: Proper cleanup of thread pool resources
- **Deterministic completion**: Main thread waits for all parallel work

## Performance Characteristics and Optimization

### Scalability Analysis

#### Parallel Efficiency Factors
1. **Task granularity**: Segment size affects load balancing
2. **Thread count**: Optimal at 2× hardware threads for this workload
3. **Memory bandwidth**: Collision checking can become memory-bound
4. **Algorithm choice**: STOMP benefits more from parallelism than Hauser

#### Performance Bottlenecks
- **Collision checking**: Shared obstacle tree access pattern
- **Memory allocation**: Dynamic trajectory point allocation
- **Load imbalance**: Variable segment complexity

### Optimization Strategies

#### Work-Stealing Queue Benefits
- **Dynamic load balancing**: Threads automatically share work
- **Cache locality**: Work-stealing preserves spatial locality when possible
- **Minimal synchronization**: Lock-free queue operations

#### Resource Pool Optimization
```cpp
// Future optimization opportunity: Object pools
// ThreadLocal<MotionGenerator> generatorPool;
// ThreadLocal<PathPlanner> plannerPool;
```

**Potential Improvements:**
- **Object pooling**: Reuse MotionGenerator/PathPlanner instances
- **Memory pools**: Pre-allocated trajectory point arrays
- **NUMA awareness**: Thread affinity for large-scale systems

## Clinical Application Considerations

### Real-Time Constraints
- **Deterministic completion**: Thread pool ensures bounded execution time
- **Memory management**: Explicit cleanup prevents memory leaks in long-running applications
- **Exception handling**: Robust error recovery for clinical reliability

### Computational Resource Management
- **CPU utilization**: 2× thread pool maximizes throughput without overwhelming system
- **Memory footprint**: Per-thread resource instantiation minimizes peak memory usage
- **System responsiveness**: Cooperative multitasking doesn't starve other processes

This multithreading architecture enables the USTrajectoryPlanner to efficiently exploit modern multi-core hardware while maintaining the reliability and deterministic behavior required for medical robotics applications.
