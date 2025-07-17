# ComparisonIK - Inverse Kinematics Method Comparison

This directory contains a clean, focused implementation for comparing three inverse kinematics methods for the Franka Panda robot.

## Overview

The ComparisonIK module provides:
1. **Three-method comparison**: Newton-Raphson vs SA-Optimized vs Grid-Search IK solvers
2. **Shared cost functions**: Fair comparison using identical cost calculations
3. **Professional visualization**: Grey-styled plots with AAAAA/CCCCC color scheme

## File Structure

```
ComparisonIK/
├── core/                          # Core IK implementations
│   ├── newton_raphson_ik.h/cpp   # Newton-Raphson solver
│   ├── grid_search_ik.h/cpp      # Grid search solver  
│   └── ik_cost_functions.h/cpp   # Shared cost functions
├── comparison_methods/            # Analysis & plotting scripts
│   ├── master_plot_generator.py  # Main plotting script
│   └── plot_*.py                 # Specific analysis scripts
├── three_method_comparison.cpp   # Main comparison executable
├── run_complete_analysis.sh      # Complete workflow script
└── README.md                     # This file
```

## Usage

### Quick Start
```bash
# Run complete analysis pipeline
./run_complete_analysis.sh
```

### Manual Steps
```bash
# 1. Build
make ThreeMethodComparison

# 2. Run comparison 
./ThreeMethodComparison

# 3. Generate plots
cd comparison_methods
python3 master_plot_generator.py
```

## Key Features

- **Fair comparison**: All methods use identical cost functions from `core/ik_cost_functions.cpp`
- **Professional plots**: Consistent grey styling (AAAAA, CCCCC, 888888)
- **Comprehensive analysis**: Success rates, execution time, clearance, joint limits
- **Real robot data**: Uses actual Franka Panda poses from ultrasound scanning

## Results

The analysis generates:
- `three_method_comparison_results.csv` - Raw numerical data
- Professional plots in `comparison_methods/plots/` directory
- Console output with key findings and statisticsverse Kinematics Method Comparison

This directory contains a clean, focused implementation for comparing different inverse kinematics methods for the Franka Panda robot, along with cost function analysis and visualization tools.

## Overview

The ComparisonIK module provides:
1. **Three-method comparison**: Newton-Raphson, SA-Optimized, and Grid Search IK solvers
2. **Cost function analysis**: Underlying cost function landscape visualization
3. **Python analysis tools**: Comprehensive plotting and statistical analysis

## Essential Components

### C++ Executables
- **`ThreeMethodComparison`**: Main comparison between Newton-Raphson, SA-Optimized, and Grid Search IK methods
- **`UnderlyingCostFunctionGenerator`**: Generates actual PathPlanner cost function data across different q7 values and poses

### Core IK Implementation
- `core/newton_raphson_ik.cpp`: Newton-Raphson IK solver implementation
- `core/grid_search_ik.cpp`: Grid search IK solver implementation  
- `core/ik_cost_functions.cpp`: Shared cost functions for fair comparison

### Python Analysis Tools
- `generate_complete_analysis.py`: Comprehensive analysis with statistical summaries
- `plot_underlying_cost_function.py`: Cost function landscape visualization
- `plot_clearance_boxplots.py`: Obstacle clearance analysis
- `plot_execution_time_boxplots.py`: Performance timing analysis
- `plot_joint_limits_boxplots.py`: Joint limit proximity analysis

### Data Files
- `two_method_comparison_results.csv`: Results from two-method comparison
- `underlying_cost_function_data.csv`: Real PathPlanner cost function landscape data

## Usage

### 1. Build the System
```bash
cd /path/to/PathPlanner_US_wip
mkdir -p build && cd build
cmake ..
make -j4
```

### 2. Run Two-Method Comparison
```bash
cd apps/ComparisonIK
../../build/apps/ComparisonIK/TwoMethodComparison
```

### 3. Generate Real PathPlanner Cost Function Data
```bash
../../build/apps/ComparisonIK/UnderlyingCostFunctionGenerator
```

This uses the actual cost function from PathPlanner's `selectGoalPoseSimulatedAnnealing` method.

### 4. Generate Complete Analysis
```bash
python3 generate_complete_analysis.py
```

### 5. Individual Visualizations
```bash
# Cost function analysis
python3 plot_underlying_cost_function.py

# Specific analyses
python3 plot_clearance_boxplots.py
python3 plot_execution_time_boxplots.py
python3 plot_joint_limits_boxplots.py
```

## Output Files

### Generated Visualizations
All plots are saved to the `plots/` directory:
- `COMPLETE_COMPARISONIK_ANALYSIS.png`: Comprehensive analysis dashboard
- `cost_function_all_poses.png`: Cost function landscapes
- `ENHANCED_JOINT_LIMITS_BOXPLOTS.png`: Joint limit analysis
- And additional specialized plots

### Data Format
- **CSV data**: Contains pose information, execution times, clearances, and joint configurations
- **Cost data**: Includes pose positions, orientations, q7 values, and corresponding costs

## Key Features

1. **Real scan poses**: Uses actual ultrasound scanning poses from medical scenarios
2. **Safety analysis**: Evaluates obstacle clearance and joint limit proximity
3. **Performance metrics**: Execution time comparison between methods
4. **Real cost function insights**: Direct access to PathPlanner's actual optimization landscape used by `selectGoalPoseSimulatedAnnealing`
5. **Statistical analysis**: Comprehensive success rates and safety compliance metrics

## Results Summary

The cleaned-up system now provides:
- ✅ Two working IK comparison methods
- ✅ Cost function analysis with synthetic data generation
- ✅ Complete Python visualization pipeline
- ✅ Statistical analysis and reporting
- ✅ Clean, maintainable codebase with essential functionality

## Build Status
- **TwoMethodComparison**: ✅ Built and tested
- **UnderlyingCostFunctionGenerator**: ✅ Built and tested  
- **Python Analysis**: ✅ All scripts working
- **Visualization**: ✅ Plots generated successfully

## File Count Summary
- **Before cleanup**: 70+ files (redundant docs, tests, executables)
- **After cleanup**: 16 essential files
- **Functionality preserved**: Core IK comparison and cost analysis