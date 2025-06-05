# Scan Process Test Plotting Tools

This directory contains tools for generating comprehensive, publication-quality plots from Scan Process Test results.

## 🚀 Quick Start

### Automated Workflow (Recommended)

```bash
# Run complete workflow: build, test, and generate plots
./run_scan_analysis.sh

# Run only plotting from existing CSV data
./run_scan_analysis.sh --plot-only

# Run single test instead of full test suite
./run_scan_analysis.sh --single

# Show all options
./run_scan_analysis.sh --help
```

### Manual Workflow

```bash
# 1. Build the test application
cd /path/to/PathPlanner_US_wip/build
make ScanProcessTest

# 2. Run tests to generate CSV data
./apps/ScanProcessTest/ScanProcessTest

# 3. Generate plots
cd /path/to/PathPlanner_US_wip/apps/ScanProcessTest
source plot_env/bin/activate  # or create: python3 -m venv plot_env
pip install -r requirements.txt
python plot_scan_results.py path/to/scan_test_results_*.csv
```

## Files

- `scan_process_test.cpp` - Main C++ test application with modular testing capabilities
- `plot_scan_results.py` - Python script for generating comprehensive analysis plots
- `requirements.txt` - Python dependencies
- `CMakeLists.txt` - Build configuration

## Generated Plots

The plotting script generates 5 comprehensive analysis plots optimized for printing:

### 1. Performance Overview (`performance_overview.png`)
- Planning time vs noise level (box plots)
- Success rate by noise level 
- Total time breakdown (pie chart)
- Trajectory count distribution
- Planning efficiency trends
- Average trajectory size analysis

### 2. Noise Impact Analysis (`noise_impact_analysis.png`)
- Planning time distribution by noise level
- Trajectory type distribution (contact vs free)
- Performance metrics correlation matrix
- Statistical summary table

### 3. Timing Analysis (`timing_analysis.png`)
- Time component breakdown (CSV load, init, planning)
- Planning vs total time correlation
- Planning efficiency trends by trial
- Time variability analysis (coefficient of variation)

### 4. Trajectory Analysis (`trajectory_analysis.png`)
- Trajectory count vs planning time correlation
- Average trajectory size distributions
- Contact vs free trajectory counts
- Trajectory size efficiency metrics

### 5. Comprehensive Summary (`comprehensive_summary.png`)
- Key performance indicators
- Performance trends summary
- Detailed statistics table
- Planning time distributions
- Correlation heatmap
- Test metadata and configuration

## Plot Features

All plots are optimized for printing with:
- High DPI (300 DPI) for crisp printing
- Publication-quality fonts (Times New Roman/serif)
- Consistent color scheme
- Clear legends and labels
- Statistical annotations
- Grid lines for easy reading
- Tight layout with minimal whitespace

## Configuration

The test application can be configured via command line:

```bash
# Run single test (original behavior)
./ScanProcessTest --single

# Reduce output verbosity
./ScanProcessTest --quiet

# Show help
./ScanProcessTest --help
```

## Test Configuration

The modular test runs with the following noise levels by default:
- 0.000 rad (0.000°) - Baseline, no noise
- 0.001 rad (0.057°) - Very small noise
- 0.005 rad (0.286°) - Small noise  
- 0.010 rad (0.573°) - Medium-small noise
- 0.020 rad (1.146°) - Medium noise
- 0.050 rad (2.865°) - Large noise
- 0.100 rad (5.730°) - Very large noise

Each noise level is tested with 3 trials for statistical significance.

## Data Analysis

The CSV output includes:
- Trial metadata (number, noise level)
- Pose and trajectory counts
- Detailed timing breakdown
- Success/failure status
- Trajectory size and type information
- Error messages (if any)

## Customization

To customize the plots:

1. **Modify noise levels**: Edit the `noise_levels` vector in `scan_process_test.cpp`
2. **Change trial count**: Modify `config.num_trials` in the main function
3. **Adjust plot styling**: Edit the `plt.rcParams` configuration in `plot_scan_results.py`
4. **Add new analysis**: Extend the `ScanResultsPlotter` class with additional methods

## Dependencies

### C++ (scan_process_test.cpp)
- Eigen3 (matrix operations)
- USLib (ultrasound trajectory planner)
- Standard C++17 libraries

### Python (plot_scan_results.py)
- pandas (data manipulation)
- numpy (numerical operations)
- matplotlib (plotting)
- seaborn (statistical visualization)
- pathlib (file operations)

## Output

The plotting script creates a `plots/` directory containing all generated PNG files. Each plot is saved at 300 DPI with tight bounding boxes for optimal printing quality.

Example output files:
- `plots/performance_overview.png`
- `plots/noise_impact_analysis.png`
- `plots/timing_analysis.png`
- `plots/trajectory_analysis.png`
- `plots/comprehensive_summary.png`

## Troubleshooting

### Common Issues

1. **"No CSV files found"**: Run the scan process test first to generate CSV data
2. **Import errors**: Install required Python packages with `pip install -r requirements.txt`
3. **Permission errors**: Ensure write permissions in the output directory
4. **Memory errors**: Reduce plot complexity for very large datasets

### Performance Tips

- Use `--quiet` flag for faster test execution when focusing on results
- Run tests on a dedicated machine for consistent timing measurements
- Use multiple trials for statistical significance
- Monitor system load during testing for consistent results

## ✅ Current Status

**SUCCESS**: The plotting system has been successfully implemented and tested!

- ✅ **Enhanced C++ test application** with modular testing, noise application, timing, and CSV export
- ✅ **Comprehensive Python plotting script** generating 5 publication-quality analysis plots
- ✅ **Complete automation workflow** for seamless testing and analysis
- ✅ **Successfully tested** with 21 trials across 7 noise levels (100% success rate)

### Generated Analysis Plots

The system generates 5 comprehensive analysis plots optimized for printing:

1. **Performance Overview** - Main performance metrics and trends
2. **Noise Impact Analysis** - Detailed noise effect analysis with statistical summaries  
3. **Timing Analysis** - Comprehensive timing breakdown and efficiency metrics
4. **Trajectory Analysis** - Trajectory characteristics and planning efficiency
5. **Comprehensive Summary** - Publication-ready summary report with all key findings

All plots are saved at 300 DPI with publication-quality formatting using Times New Roman fonts and consistent color schemes.
