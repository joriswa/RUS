# Parameter Tuning Application

A comprehensive parameter optimization system for trajectory planning algorithms in ultrasound scanning applications. This application optimizes STOMP and Hauser trajectory planners using real motion generation libraries and actual scenario data.

## Overview

The Parameter Tuning application provides:
- **Real-world optimization** using actual C++ motion planning libraries
- **Multi-algorithm support** for STOMP and Hauser trajectory planners
- **Comprehensive evaluation** with composite objective functions
- **Rich visualizations** and detailed analysis reports
- **Production-ready results** with optimized parameter configurations

## Quick Start

### Prerequisites

1. **Build the C++ evaluator**:
   ```bash
   cd PathPlanner_US_wip/build
   make EnhancedParameterEvaluator
   ```

2. **Install Python dependencies**:
   ```bash
   pip install numpy pandas matplotlib seaborn optuna pyyaml scipy
   ```

3. **Verify scenario data** (should exist in `../../res/scenario_1/`):
   - `obstacles.xml` - Environment obstacles
   - `panda_US.urdf` - Robot model
   - `scan_poses.csv` - Ultrasound scanning poses

### Basic Usage

```bash
# Quick optimization (20 trials each, simple scenarios)
python run_parameter_optimization.py --quick

# Full optimization (100 trials each, all scenarios)
python run_parameter_optimization.py --full

# Optimize specific algorithm
python run_parameter_optimization.py --algorithm STOMP --trials 50

# Create comprehensive visualizations
python create_comprehensive_plots.py
```

## File Structure

```
ParameterTuning/
├── README.md                          # This file
├── CMakeLists.txt                     # Build configuration
├── run_parameter_optimization.py      # Main optimization script
├── create_comprehensive_plots.py      # Visualization suite
├── enhanced_parameter_evaluator.cpp   # C++ evaluation engine
├── enhanced_parameter_optimizer.py    # Core optimization logic
├── enhanced_parameter_tuning.h        # C++ headers
├── parameter_tuning_main.cpp          # Legacy main (archived)
├── create_tuning_plots.py            # Basic plotting utilities
├── PARAMETER_TUNING_COMPLETION_SUMMARY.md  # Implementation summary
├── results/                           # Optimization results
│   └── simplified_tuning_results/
│       ├── stomp_results.json
│       └── hauser_results.json
├── plots/                            # Generated visualizations
│   ├── tuning_analysis/
│   └── comprehensive_analysis/
├── scripts/                          # Utility scripts
│   ├── run_parameter_tuning.py      # Alternative runner
│   ├── analyze_results.py           # Result analysis
│   └── test_yaml_generation.py      # YAML testing
└── archive/                          # Archived/legacy files
    ├── advanced_parameter_analysis.py
    ├── run_enhanced_tuning.py
    └── run_actual_parameter_tuning.py
```

## Detailed Usage

### Command Line Options

```bash
python run_parameter_optimization.py [options]

Options:
  --algorithm {STOMP,Hauser,both}  Algorithm to optimize (default: both)
  --trials N                       Number of trials per algorithm (default: 50)
  --scenarios {quick,standard,full,all}  Test scenarios (default: standard)
  --output-dir DIR                 Output directory (default: results/optimization_results)
  --quick                          Quick mode: 20 trials, simple scenarios
  --full                           Full mode: 100 trials, all scenarios
  --log-file FILE                  Log file path (default: parameter_optimization.log)
```

### Scenario Types

- **quick**: 3 poses, minimal complexity
- **standard**: 3 and 8 pose scenarios
- **full**: 3, 8, and all pose scenarios
- **all**: Complete test suite including contact-based poses

### Algorithm Parameters

#### STOMP Parameters
- `exploration_constant`: Exploration vs exploitation balance (0.001-0.5)
- `num_noisy_trajectories`: Number of noisy samples (5-100)
- `num_best_samples`: Best samples to keep (3-20)
- `max_iterations`: Maximum optimization iterations (50-1000)
- `learning_rate`: Learning rate for updates (0.1-0.7)
- `temperature`: Temperature for sampling (5.0-50.0)
- `dt`: Time discretization (0.01-0.2)
- `adaptive_sampling`: Enable adaptive sampling (True/False)
- `early_termination`: Enable early termination (True/False)

#### Hauser Parameters
- `max_deviation`: Maximum trajectory deviation (0.1-2.0)
- `time_step`: Time step for planning (0.01-0.5)
- `max_iterations`: Maximum planning iterations (100-2000)
- `tolerance`: Convergence tolerance (1e-6 to 1e-3)
- `acceleration_limit`: Maximum acceleration (0.5-5.0)
- `velocity_limit`: Maximum velocity (0.5-3.0)
- `interpolation_dt`: Interpolation time step (0.01-0.1)

## Optimization Process

### Objective Function

The optimization minimizes a composite objective that balances:

- **Planning Time** (30% weight): Faster planning preferred
- **Success Rate** (25% weight): Higher success rates preferred
- **Path Length** (20% weight): Shorter, efficient paths preferred
- **Safety Score** (15% weight): Higher safety margins preferred
- **Smoothness Score** (10% weight): Smoother trajectories preferred

### Evaluation Pipeline

1. **Parameter Sampling**: Optuna TPE optimizer samples parameters
2. **Configuration Generation**: Creates YAML config for C++ evaluator
3. **Trajectory Planning**: C++ evaluator runs actual motion planning
4. **Metrics Collection**: Collects planning time, success rate, path quality
5. **Objective Computation**: Combines metrics into composite objective
6. **Optimization Update**: Updates parameter search based on results

## Results and Analysis

### Generated Files

After optimization, the following files are created:

- `{algorithm}_results.json`: Complete optimization history
- `parameter_optimization.log`: Detailed execution log
- Various visualization PNG files (when running plotting script)
- `RESULTS_SUMMARY.md`: Concise results summary

### Key Metrics

- **Best Objective**: Lower values indicate better overall performance
- **Success Rate**: Percentage of successful trajectory plans
- **Planning Time**: Average time to compute trajectories
- **Path Length**: Total joint space distance
- **Safety/Smoothness**: Quality scores for trajectories

### Visualization Suite

The comprehensive plotting script generates:

1. **Performance Comparison**: Direct algorithm comparison
2. **Convergence Analysis**: Optimization progress over trials
3. **Parameter Sensitivity**: Impact of each parameter on performance
4. **Optimization Dashboard**: Complete results overview
5. **Distribution Analysis**: Statistical analysis of results

## Recent Results Summary

Based on the latest optimization run:

| Algorithm | Best Objective | Winner | Parameters | Trials |
|-----------|---------------|---------|------------|---------|
| STOMP     | 0.500067      | ❌      | 9          | 50     |
| Hauser    | 0.500066      | ✅      | 7          | 50     |

**Winner**: Hauser (by 0.0003% - extremely close performance)

### Optimal STOMP Configuration
```json
{
  "exploration_constant": 0.035570,
  "num_noisy_trajectories": 22,
  "num_best_samples": 17,
  "max_iterations": 108,
  "learning_rate": 0.354020,
  "temperature": 28.266176,
  "dt": 0.091734,
  "adaptive_sampling": false,
  "early_termination": false
}
```

### Optimal Hauser Configuration
```json
{
  "max_deviation": 1.557777,
  "time_step": 0.193934,
  "max_iterations": 1126,
  "tolerance": 0.000311,
  "acceleration_limit": 0.523148,
  "velocity_limit": 1.930301,
  "interpolation_dt": 0.048637
}
```

## Technical Implementation

### C++ Evaluator Features

- **Real Library Integration**: Uses TrajectoryLib, USLib, GeometryLib, Hauser10
- **YAML Configuration**: Flexible parameter and scenario specification
- **JSON Output**: Clean integration with Python optimization
- **Robust Error Handling**: Timeout protection and graceful degradation
- **Scenario Support**: Multiple test scenarios with varying complexity

### Python Optimization Features

- **Optuna Integration**: State-of-the-art Bayesian optimization
- **Multi-Algorithm Support**: Parallel optimization of different planners
- **Flexible Scenarios**: Configurable test scenarios and difficulty levels
- **Comprehensive Logging**: Detailed execution tracking and debugging
- **Result Persistence**: JSON serialization of complete optimization history

## Troubleshooting

### Common Issues

1. **C++ Evaluator Not Found**
   ```bash
   # Build the evaluator
   cd PathPlanner_US_wip/build
   cmake .. && make EnhancedParameterEvaluator
   ```

2. **Missing Dependencies**
   ```bash
   # Install required packages
   pip install numpy pandas matplotlib seaborn optuna pyyaml scipy
   ```

3. **Scenario Data Missing**
   ```bash
   # Verify files exist in res/scenario_1/
   ls ../../res/scenario_1/
   # Should show: obstacles.xml, panda_US.urdf, scan_poses.csv
   ```

4. **YAML Parsing Errors**
   ```bash
   # Test YAML generation
   python scripts/test_yaml_generation.py
   ```

### Debugging

- Enable verbose logging by checking `parameter_optimization.log`
- Use `--quick` mode for faster debugging iterations
- Test individual components with scripts in `scripts/` directory
- Verify C++ evaluator with manual YAML files

## Development and Extension

### Adding New Algorithms

1. Extend parameter space in `get_parameter_space()`
2. Add algorithm case in C++ evaluator
3. Update optimization loop in `optimize_algorithm()`
4. Add visualization support in plotting scripts

### Custom Scenarios

1. Modify `create_test_scenarios()` function
2. Add new scenario types and configurations
3. Update YAML generation for new scenario formats
4. Test with `scripts/test_yaml_generation.py`

### New Optimization Methods

1. Add optimizer in `optimize_algorithm()` method
2. Implement parameter sampling logic
3. Update results serialization format
4. Add visualization support for new optimizer

## Academic Applications

This system supports research in:

- **Trajectory Planning**: Comparative analysis of planning algorithms
- **Parameter Optimization**: Multi-objective optimization in robotics
- **Medical Robotics**: Ultrasound scanning trajectory generation
- **Motion Planning**: Real-world validation of planning algorithms

## Citation

If you use this parameter tuning system in academic work, please reference:

```
Parameter Optimization for Trajectory Planning in Ultrasound Scanning Applications
PathPlanner_US_wip Parameter Tuning System
[Include appropriate institutional and author information]
```

## License

[Include appropriate license information]

---

**Status**: ✅ Production Ready  
**Last Updated**: [Current Date]  
**Validation**: ✅ Tested with real motion libraries  
**Documentation**: ✅ Comprehensive