# IK Methods Comparison - Cleaned Up Structure

## Overview
This directory contains a comprehensive comparison of three IK (Inverse Kinematics) methods:
- **Newton-Raphson**: Iterative gradient-based solver
- **SA-Optimized**: Simulated Annealing optimization 
- **Grid-Search**: Exhaustive search with 14-bit encoder precision

## File Organization

### Core Implementation
- `core/newton_raphson_ik.h/cpp` - Newton-Raphson IK solver
- `core/grid_search_ik.h/cpp` - Grid search IK solver
- `core/ik_cost_functions.h/cpp` - **Shared cost functions for fair comparison**

### Main Comparison
- `three_method_comparison.cpp` - Main comparison executable
- `CMakeLists.txt` - Build configuration

### Analysis & Plotting
- `comparison_methods/` - **All plotting and analysis scripts**
  - `master_plot_generator.py` - **Main plotting script with grey styling**
  - Legacy plotting scripts (moved from main directory)

## Key Improvements

### 1. Shared Cost Functions ✅
All IK methods now use the same cost function utilities from `core/ik_cost_functions.cpp`:
- `computeComprehensiveCost()` - Full cost function (pose + clearance + manipulability)
- `computePoseErrorMagnitude()` - Position and orientation error
- `computeClearancePenalty()` - Obstacle avoidance penalty
- `computeJointLimitDistances()` - Safety distance calculations

### 2. Organized Plotting ✅
- All 46+ plotting scripts moved to `comparison_methods/`
- New master plotting script with consistent **AAAAA, CCCCC grey styling**
- Professional publication-ready plots

### 3. Consistent Styling ✅
Grey color scheme as requested:
- Newton-Raphson: `#AAAAAA`
- SA-Optimized: `#CCCCCC` 
- Grid-Search: `#888888`
- Background: `#F5F5F5`
- Text: `#333333`

## Usage

### 1. Build and Run Comparison
```bash
cd /Users/joris/Uni/MA/Code/PathPlanner_US_wip
make TwoMethodComparison
./apps/ComparisonIK/TwoMethodComparison
```

### 2. Generate Plots
```bash
cd apps/ComparisonIK
python3 comparison_methods/master_plot_generator.py
```

Plots will be saved to `comparison_methods/plots/`:
- `success_rates_comparison.png`
- `execution_time_boxplot.png` 
- `clearance_comparison.png`
- `comparison_dashboard.png` (comprehensive overview)

## Results Format
The comparison generates `three_method_comparison_results.csv` with columns:
- `pose_id, run_id, method, success, time_ms`
- `pos_error, ori_error, iterations, collision_free`
- `min_clearance, avg_clearance, num_links_checked`
- `q1, q2, q3, q4, q5, q6, q7` (joint angles)

## Benefits of Cleanup

1. **Code Reusability**: Shared cost functions ensure fair comparison
2. **Organization**: Clean separation of implementation vs analysis
3. **Consistency**: Unified styling across all plots  
4. **Maintainability**: Single source of truth for cost calculations
5. **Professional**: Publication-ready grey-tone styling

## Technical Notes

- **Grid Search**: Uses 14-bit encoder precision (16,384 discrete q7 values)
- **Cost Function**: Weighted combination of pose error (100x), clearance penalty, and manipulability
- **Safety Analysis**: Joint limit distance tracking with warning thresholds
- **Collision Detection**: Full environment collision checking with clearance metrics
