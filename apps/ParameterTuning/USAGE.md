# Parameter Tuning - Quick Usage Guide

## Overview
This system optimizes STOMP and Hauser trajectory planning parameters using real motion planning libraries and actual ultrasound scanning scenarios.

## Core Components

### 1. C++ Evaluator
- **File**: `enhanced_parameter_evaluator.cpp`
- **Purpose**: Evaluates trajectory planning parameters using real motion libraries
- **Input**: YAML configuration files with algorithm parameters
- **Output**: JSON results with comprehensive metrics

### 2. Python Optimizer
- **File**: `enhanced_parameter_optimizer.py`
- **Purpose**: Advanced parameter optimization using Optuna, scikit-optimize, etc.
- **Features**: Bayesian optimization, distributed tuning, convergence analysis

### 3. Main Execution Script
- **File**: `run_parameter_optimization.py`
- **Purpose**: Simple interface for running parameter optimization

## Quick Start

### Prerequisites
1. Build the C++ evaluator:
   ```bash
   cd ../../build
   make EnhancedParameterEvaluator
   ```

2. Install Python dependencies:
   ```bash
   pip install optuna pandas numpy matplotlib seaborn pyyaml scipy
   ```

3. Verify scenario_1 data exists in `../../res/scenario_1/`:
   - `obstacles.xml` - Environment obstacles
   - `panda_US.urdf` - Robot model  
   - `scan_poses.csv` - Ultrasound scanning poses

### Usage Examples

```bash
# Quick optimization (20 trials, basic scenarios)
python run_parameter_optimization.py --quick

# Full optimization (100 trials, all scenarios)  
python run_parameter_optimization.py --full

# Optimize STOMP only
python run_parameter_optimization.py --algorithm STOMP --trials 50

# Create visualization plots
python create_comprehensive_plots.py

# Debug parameter sensitivity
python diagnose_parameter_sensitivity.py
```

## How It Works

1. **Parameter Space Definition**: Python defines parameter ranges for STOMP/Hauser
2. **YAML Generation**: Python creates YAML config files with trial parameters
3. **C++ Evaluation**: C++ evaluator loads real scenarios, runs trajectory planning
4. **Metrics Collection**: C++ outputs comprehensive performance metrics (JSON)
5. **Optimization Loop**: Python optimization library suggests next parameters
6. **Convergence**: Process continues until optimal parameters found

## Results

The system produces optimized parameters like:

**STOMP Parameters:**
- exploration_constant: 0.035570
- num_noisy_trajectories: 22
- max_iterations: 108
- learning_rate: 0.354020

**Hauser Parameters:**
- max_deviation: 1.557777
- max_iterations: 1126
- acceleration_limit: 0.523148

## File Structure

```
ParameterTuning/
├── enhanced_parameter_evaluator.cpp    # C++ evaluation engine
├── enhanced_parameter_optimizer.py     # Python optimization framework  
├── run_parameter_optimization.py       # Main execution script
├── create_comprehensive_plots.py       # Visualization tools
├── diagnose_parameter_sensitivity.py   # Analysis tools
├── debug_raw_outputs.py               # Debugging utilities
├── archive/                            # Legacy implementations
├── scripts/                            # Additional utilities
├── results/                            # Optimization results
└── plots/                             # Generated visualizations
```

## Advanced Usage

For custom scenarios or advanced optimization, see:
- `enhanced_parameter_optimizer.py` - Modify parameter spaces, optimization algorithms
- `enhanced_parameter_evaluator.cpp` - Add new evaluation metrics or scenarios
- `PARAMETER_TUNING_COMPLETION_SUMMARY.md` - Detailed technical documentation
