# STOMP Optimization Visualization Report

**Generated**: 2025-06-18 20:29:44
**Total Trials**: 400
**Best Objective Value**: 0.5445

## Overview

This report contains comprehensive visualizations of the STOMP optimization results, 
including convergence analysis, parameter relationships, and performance metrics.

## Visualizations

### 1. Convergence Analysis
![Convergence Analysis](stomp_convergence_analysis.png)

Shows the optimization convergence over trials, including:
- Objective value progression
- Running best value
- Distribution of results
- Time-based analysis

### 2. Parameter Analysis
![Parameter Analysis](stomp_parameter_analysis.png)

Individual parameter relationships with objective value:
- Scatter plots for each parameter
- Best configuration markers
- Trend lines where applicable

### 3. Correlation Analysis
![Correlation Analysis](stomp_correlation_analysis.png)

Parameter correlation and importance analysis:
- Parameter correlation matrix
- Parameter importance ranking
- Best vs worst trials comparison

### 4. Performance Analysis
![Performance Analysis](stomp_performance_analysis.png)

Performance metrics analysis including planning time, success rates, and safety measures.

### 5. Comprehensive Summary
![Comprehensive Summary](stomp_comprehensive_summary.png)

Complete optimization overview with statistics, parameter distributions, and performance evolution.

## Key Findings

### Best Configuration
- **Max Iterations**: 70
- **Num Noisy Trajectories**: 10
- **Num Best Samples**: 13
- **Learning Rate**: 0.031730693138185544
- **Temperature**: 43.60889870960598
- **Dt**: 0.1449303599727216
- **Noise Stddev**: 0.20215278631604358
- **Control Cost Weight**: 8.836273300427987
- **Smoothness Cost Weight**: 1.0024247421993808
- **Collision Cost Weight**: 27.95725993656989
- **Use Finite Differences**: True
- **Trajectory Length**: 89
- **Convergence Threshold**: 0.0006105091113061596

### Parameter Importance (Top 5)
1. **Smoothness Cost Weight**: 0.6863
2. **Num Noisy Trajectories**: 0.6334
3. **Temperature**: 0.6188
4. **Max Iterations**: 0.5761
5. **Trajectory Length**: 0.5403

### Statistics
- **Mean Objective Value**: 0.8298
- **Standard Deviation**: 0.2453
- **Best Trial Number**: 373
- **Improvement from First Trial**: 60.0%

## Files Generated
- `stomp_convergence_analysis.png`
- `stomp_parameter_analysis.png`
- `stomp_correlation_analysis.png`
- `stomp_performance_analysis.png`
- `stomp_comprehensive_summary.png`
