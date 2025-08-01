# Downstream Parallelization Optimization for planTrajectories

## Summary of Changes

This optimization addresses the performance bottlenecks in the downstream parallelization of `planTrajectories` in the USPlanner to ensure efficient coordination with STOMP for maximum speed.

## Problem Analysis

### Original Architecture Issues
1. **Nested Parallelization**: Two-level parallelization caused thread contention
   - Outer level: Multiple trajectories running STOMP in parallel
   - Inner level: Each STOMP run parallelizes sampling and evaluation
   - Result: Thread pool exhaustion and reduced efficiency

2. **Suboptimal Resource Utilization**: Fixed parallelization strategy regardless of workload size

3. **Memory Overhead**: Excessive creation of MotionGenerator instances

## Optimization Strategy: Workload-Aware Flat Parallelization

### Key Improvements

#### 1. Workload-Aware Parallelization Selection
- **Small batches (≤ physicalCores)**: Use hierarchical parallelization (original STOMP internal parallelization)
- **Large batches (> physicalCores)**: Use flat parallelization (trajectory-level parallelization only)
- Dynamic strategy selection based on `shouldUseFlatParallelization()`

#### 2. Enhanced STOMP Configuration
- Added `disableInternalParallelization` parameter to `StompConfig`
- Conditional parallelization in `generateNoisySamples()` and `evaluateTrajectories()`
- Sequential processing within STOMP when in flat mode

#### 3. Optimized Hardware Configuration
- Updated `HardwareConfig` with `optimalThreadsForFlatParallelization`
- Better batch size calculation: `physicalCores / 4` instead of `physicalCores / 2`
- Dedicated thread pool sizing for flat parallelization

#### 4. Performance Monitoring
- Real-time throughput measurement (trajectories/sec)
- Average time per trajectory tracking
- Parallelization strategy logging
- Success rate monitoring

#### 5. Shared Resource Optimization
- Enhanced SDF caching to avoid recomputation
- Improved memory allocation patterns
- Better thread pool pre-warming

## Technical Implementation

### New Methods
- `shouldUseFlatParallelization(size_t numTrajectories)`: Workload analysis
- Enhanced `initializeThreadPool()`: Optimal thread configuration
- Performance metrics in batch planning methods

### Modified Functions
- `planTrajectoryBatch()`: Workload-aware strategy selection
- `planTrajectoryBatchHauser()`: Consistent optimization approach
- `generateNoisySamples()`: Conditional parallelization
- `evaluateTrajectories()`: Sequential processing option

## Performance Benefits

### Expected Improvements
1. **Eliminated Thread Contention**: No competing nested parallelization
2. **Better Cache Utilization**: Sequential STOMP operations improve data locality
3. **Scalable Performance**: Works optimally with both small and large batches
4. **Predictable Behavior**: Easier tuning and optimization
5. **Reduced Memory Pressure**: Less simultaneous object creation

### Benchmark Integration
- Created `ParallelizationBenchmark` application
- Tests various batch sizes (1, 2, 4, 8, 16 trajectories)
- Measures throughput and success rates
- Validates workload-aware strategy effectiveness

## Usage

The optimization is automatic and requires no changes to existing code:

```cpp
// Existing code continues to work
UltrasoundScanTrajectoryPlanner planner(environmentString);
planner.setCurrentJoints(joints);
planner.setPoses(poses);
bool success = planner.planTrajectories(); // Automatically optimized

// Batch planning also optimized
auto results = planner.planTrajectoryBatch(requests, descriptions);
```

## Configuration

Hardware configuration is automatically detected:
- Physical cores detected at runtime
- Optimal thread pool size calculated
- Batch size optimized for the system
- Strategy selection based on workload

## Monitoring

Performance metrics are logged automatically:
```
Parallelization strategy for 8 trajectories: FLAT (threshold: 4 cores)
Thread pool initialized with 4 threads for flat parallelization
Batch planning complete (FLAT): 8/8 (100.0%) in 1250ms
Performance: 6.40 trajectories/sec, 156.25ms avg/trajectory
```

## Future Enhancements

1. **Adaptive Thresholds**: Machine learning-based threshold optimization
2. **Pipeline Parallelization**: Overlap different STOMP phases
3. **NUMA Awareness**: CPU topology-aware thread placement
4. **Dynamic Load Balancing**: Work-stealing for uneven trajectory complexity

## Conclusion

This optimization provides a comprehensive solution for efficient STOMP coordination in batch trajectory planning, yielding the fastest results through intelligent workload-aware parallelization strategy selection.