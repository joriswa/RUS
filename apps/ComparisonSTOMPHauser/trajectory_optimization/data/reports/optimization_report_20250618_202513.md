# Trajectory Planning Algorithm Optimization Report

**Generated**: 2025-06-18 20:25:13
**Study**: stomp_focused_optimization
**Algorithms**: 1

## Executive Summary

**Best Performing Algorithm**: stomp
**Best Objective Value**: 0.5445

## Algorithm Performance Comparison

| Algorithm | Best Value | Mean ± Std | Median | Trials |
|-----------|------------|-------------|---------|--------|
| stomp | 0.5445 | 0.8298 ± 0.2453 | 0.7243 | 400 |

## Optimal Configurations

### stomp

**Objective Value**: 0.5445
**Trial Number**: 373

**Parameters**:

- stomp_max_iterations: 70
- stomp_num_noisy_trajectories: 10
- stomp_num_best_samples: 13
- stomp_learning_rate: 0.0317
- stomp_temperature: 43.6089
- stomp_dt: 0.1449
- stomp_noise_stddev: 0.2022
- stomp_control_cost_weight: 8.8363
- stomp_smoothness_cost_weight: 1.0024
- stomp_collision_cost_weight: 27.9573
- stomp_use_finite_differences: True
- stomp_trajectory_length: 89
- stomp_convergence_threshold: 0.0006

## Statistical Analysis

### Parameter Importance

**stomp - Top 5 Parameters**:

1. stomp_smoothness_cost_weight: 0.6772
2. stomp_num_noisy_trajectories: 0.2390
3. stomp_temperature: 0.0386
4. stomp_noise_stddev: 0.0182
5. stomp_num_best_samples: 0.0112
