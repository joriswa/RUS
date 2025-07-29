# Trajectory Validation and Constraint Cost Usage

This document explains how to use the trajectory validation and configurable constraint cost features in the STOMP trajectory planner.

## Features Added

1. **Constraint Cost Calculator**: Penalizes trajectories that violate velocity/acceleration limits during optimization
2. **Trajectory Validation**: Hard constraint checking that rejects invalid trajectories
3. **Configurable Weights**: Control the importance of constraint violations through `StompConfig`
4. **Fixed Trajectory Points**: Use fixed N=75 trajectory points instead of variable dt-based discretization

## Updated Default Parameters

The default STOMP parameters have been optimized based on research findings:

```cpp
// New optimized defaults
numNoisyTrajectories = 28
numBestSamples = 4  
maxIterations = 300
temperature = 23.756654804919137
learningRate = 0.4426954605506927
dt = 0.07393752094959168
N = 75  // Fixed number of trajectory points
```

## Usage Examples

### Basic Usage with New Defaults

```cpp
// Uses optimized parameters automatically
auto config = StompConfig::optimized();
bool success = motionGenerator.performSTOMP(config);
```

### Custom Configuration with Fixed N

```cpp
// Custom config with fixed trajectory points
auto config = StompConfig::custom(
    30,    // numNoisy
    5,     // numBest  
    200,   // maxIter
    0.3,   // learningRate
    20.0,  // temperature
    100    // N (trajectory points)
);
bool success = motionGenerator.performSTOMP(config);
```

### Preset Configurations

```cpp
// Speed-optimized (N=50, fewer iterations)
auto fastConfig = StompConfig::fast();

// Quality-optimized (N=100, more iterations) 
auto qualityConfig = StompConfig::quality();

// Hybrid planning (N=60, time-limited)
auto hybridConfig = StompConfig::hybrid();
```

### Custom Constraint Weight

```cpp
// Higher penalty for constraint violations
auto config = StompConfig::optimized();
config.constraintCostWeight = 0.5;  // Increase penalty
bool success = motionGenerator.performSTOMP(config);

// Disable constraint cost (only hard validation remains)
config.constraintCostWeight = 0.0;  // Disable soft constraint cost
bool success = motionGenerator.performSTOMP(config);
```

## Key Changes

1. **Fixed N Parameter**: Trajectories now use a fixed number of points (default N=75) instead of variable discretization
2. **Adaptive dt**: Time step is calculated as `dt = estimatedTime / (N-1)` to maintain trajectory duration
3. **Simplified USTrajectoryPlanner**: Removed custom parameter overrides, now uses optimized defaults
4. **Research-Optimized Defaults**: Parameters tuned for best performance based on empirical studies

## Benefits of Fixed N

- **Consistent Discretization**: All trajectories have the same resolution regardless of duration
- **Predictable Performance**: Computation time scales predictably with fixed point count
- **Better Optimization**: Fixed problem size enables more stable convergence
- **Simplified Configuration**: Fewer parameters to tune (N instead of dt)

## Migration from Previous Version

Old code using custom parameters:
```cpp
// OLD: Manual parameter tuning
auto config = StompConfig::optimized();
config.temperature = 100.7;
config.learningRate = 0.44;
config.dt = 0.25;
config.numNoisyTrajectories = 4;
```

New code with optimized defaults:
```cpp  
// NEW: Just use optimized defaults
auto config = StompConfig::optimized();
// Or customize specific aspects if needed
config.constraintCostWeight = 0.3;
```
