# STOMP vs Hauser Trajectory Planning Comparison

## Overview

This application provides a comprehensive comparison between two trajectory planning algorithms for robotic ultrasound applications:

- **STOMP** (Stochastic Trajectory Optimization for Motion Planning): Fast stochastic optimization with noise sampling
- **Hauser**: Smooth parabolic ramp planning with path planning preprocessing

## Key Features

✅ **Real Trajectory Analysis** - Calculates actual kinematic metrics via numerical differentiation  
✅ **Statistical Significance Testing** - T-tests, Mann-Whitney U tests with effect sizes  
✅ **Publication-Quality Plots** - Professional visualizations with statistical annotations  
✅ **Automated Pipeline** - One-command execution with validation  
✅ **Comprehensive Metrics** - Velocity, acceleration, jerk, smoothness, safety analysis  

## Quick Start

### Prerequisites
- CMake and build tools
- Qt5/Qt6 development libraries  
- Eigen3 library
- Python 3 with pandas, numpy, matplotlib, scipy, seaborn

**Optional for macOS users:**
```bash
# Install coreutils for timeout functionality (optional)
brew install coreutils
```

### Automated Execution (Recommended)
```bash
# Run with default configuration (10 poses, 5 runs each = 50 total runs)
./run_comparison.sh

# Run with custom configuration
./run_comparison.sh 15 8  # 15 poses, 8 runs each = 120 total runs
```

### Manual Execution
```bash
# 1. Build the project
cd ../../build
make STOMPHauserComparison

# 2. Run comparison
cd ../apps/ComparisonSTOMPHauser
../../build/apps/ComparisonSTOMPHauser/STOMPHauserComparison 10 5 > results.csv

# 3. Run analysis
python3 trajectory_analysis.py results.csv

# 4. Validate results
python3 validate_comparison.py results.csv
```

## Real Metrics Calculated

### Kinematic Analysis
- **Velocity**: `v(t) = (q(t+Δt) - q(t-Δt)) / (2Δt)` via numerical differentiation
- **Acceleration**: `a(t) = (v(t+Δt) - v(t-Δt)) / (2Δt)` 
- **Jerk**: `j(t) = (a(t+Δt) - a(t-Δt)) / (2Δt)`
- **RMS Jerk**: `√(Σj²/n)` - **Key smoothness indicator**

### Safety Analysis
- **Clearance**: Geometric distance to obstacles via BVH tree collision detection
- **Collision Detection**: Intersection testing between robot links and environment
- **Safety Score**: Normalized clearance metric [0,1]

### Quality Metrics
- **Smoothness Score**: `1/(1 + RMS_jerk)` - Higher is smoother
- **Energy Estimate**: `path_length × avg_acceleration` - Lower is more efficient
- **Consistency**: Motion variance analysis - Higher is more predictable

## Expected Results

### STOMP Characteristics
- ⚡ **Faster Planning**: 50-2000ms (stochastic sampling)
- 📊 **Higher Jerk Variability**: Due to noise-based optimization
- ✅ **Good Success Rate**: Effective in constrained environments
- 🎲 **More Variable**: Random sampling creates trajectory diversity

### Hauser Characteristics  
- 🐌 **Longer Planning**: Two-stage process (path planning + smoothing)
- 📈 **Lower RMS Jerk**: 15-30% smoother due to parabolic profiles
- 🎯 **Consistent Motion**: Predictable parabolic velocity profiles
- 🔄 **Two-Stage Process**: Discrete planning followed by continuous smoothing

## Output Files

### Data Files
- `comparison_results.csv` - Raw trajectory metrics data
- `comparison_summary.md` - Generated analysis report
- `comparison_stderr.log` - Execution logs

### Visualizations
- `comprehensive_performance_dashboard.png/pdf` - Main comparison dashboard
- `detailed_kinematic_analysis.png/pdf` - Kinematic analysis plots
- `analysis_output.txt` - Statistical test results

## Statistical Analysis

The framework includes rigorous statistical testing:

- **T-tests**: Parametric comparison (assumes normal distribution)
- **Mann-Whitney U**: Non-parametric robust alternative
- **Effect Size (Cohen's d)**: Practical significance measurement
  - Small: |d| > 0.2, Medium: |d| > 0.5, Large: |d| > 0.8
- **Significance Levels**: `***` p<0.001, `**` p<0.01, `*` p<0.05, `ns` p≥0.05

## Dependencies

### System Requirements
- CMake ≥ 3.16
- C++17 compatible compiler (GCC 8+ or Clang 7+)
- Qt5/Qt6 development libraries
- Eigen3 mathematical library (≥ 3.3)
- Boost libraries (system, thread)

### Python Requirements
```bash
pip3 install pandas numpy matplotlib seaborn scipy
```

### Robot Environment Files
- `../../res/scenario_1/panda_US.urdf` - Franka Panda robot model
- `../../res/scenario_1/obstacles.xml` - Environment obstacles

## File Structure

```
ComparisonSTOMPHauser/
├── README.md                                    # This file
├── stomp_hauser_comparison_improved.cpp         # Main C++ comparison (real metrics)
├── improved_trajectory_analysis.py              # Python analysis with statistics
├── run_improved_comparison.sh                   # Automated execution script
├── validate_improved_comparison.py              # Validation framework
├── CMakeLists.txt                              # Build configuration
├── README_IMPROVED.md                          # Detailed technical documentation
├── IMPROVEMENTS_SUMMARY.md                     # Summary of improvements made
├── results/                                    # Generated results (created at runtime)
│   ├── comparison_results.csv                 # Raw data
│   ├── comparison_summary.md                  # Report
│   └── comparison_stderr.log                  # Logs
└── plots/                                     # Generated plots (created at runtime)
    ├── comprehensive_performance_dashboard.png
    ├── detailed_kinematic_analysis.png
    └── analysis_output.txt
```

## Troubleshooting

### Build Issues
```bash
# Missing Qt
sudo apt-get install qt5-default libqt5core5a

# Missing Eigen3
sudo apt-get install libeigen3-dev

# Missing Boost
sudo apt-get install libboost-all-dev
```

### Runtime Issues
- **"Robot model not found"**: Ensure `../../res/scenario_1/panda_US.urdf` exists
- **"Environment file not found"**: Ensure `../../res/scenario_1/obstacles.xml` exists
- **Segmentation faults**: Check robot initialization and obstacle tree setup
- **Python import errors**: Install required packages with pip3

### Performance Tuning
- **Faster execution**: Reduce poses/runs `./run_improved_comparison.sh 5 3`
- **Better statistics**: Increase sample size `./run_improved_comparison.sh 20 10`
- **Debug mode**: Check `comparison_stderr.log` for detailed execution info

## Validation

Run the validation script to ensure the comparison is working correctly:

```bash
./validate_improved_comparison.py results/comparison_results.csv
```

The validator checks for:
- Real vs fake metrics detection
- Kinematic calculation consistency
- Statistical validity
- Expected algorithmic differences
- Data quality assessment

## Contributing

When modifying this comparison:

1. **Maintain Real Metrics**: Never substitute fake/random values
2. **Preserve Statistical Rigor**: Include appropriate significance testing
3. **Test Thoroughly**: Run validation after changes
4. **Document Changes**: Update README and technical documentation

## References

- Kalakrishnan et al. "STOMP: Stochastic trajectory optimization for motion planning"
- Kris Hauser "Fast interpolation and time-optimization with contact"
- Cohen's statistical methods for effect size in robotics research
- TrajectoryLib internal documentation

---

**For detailed technical information, see `README_IMPROVED.md`**  
**For implementation changes, see `IMPROVEMENTS_SUMMARY.md`**