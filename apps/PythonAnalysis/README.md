# Python Analysis Application

This application provides comprehensive analysis tools for trajectory planning and execution time evaluation.

## Structure

- `scripts/` - Analysis scripts for different evaluation types
- `results/` - Output directory for generated analysis results  
- `data/` - Input data files and datasets
- `docs/` - Documentation and analysis reports

## Available Analysis Tools

### 1. IK Method Comparison Analysis (`ik_execution_time_analysis.py`)
- Analyzes execution times for different IK methods (Newton-Raphson, Damped_LS, selectGoalPose, SA-Optimized)
- Focuses on middle 10 poses for targeted evaluation
- Generates comprehensive boxplots and performance statistics

### 2. SinglePose Evaluator Analysis (`single_pose_execution_analysis.py`)
- Evaluates trajectory planning algorithms (STOMP, HAUSER variants)
- Compares execution times, computation efficiency, and success rates
- Provides algorithm recommendations based on performance metrics

### 3. Quick Dashboard (`quick_dashboard.py`)
- Interactive dashboard for trajectory analysis metrics
- Visualizes success rates, planning times, clearance, and safety metrics
- Can be used with any CSV output from evaluation tools

### 4. Trajectory Visualization (`visualize_trajectory_analysis.py`)
- Comprehensive trajectory analysis and visualization
- Safety analysis including clearance and collision metrics
- Performance dashboards with statistical summaries

## Usage

### IK Method Analysis
```bash
cd /Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/PythonAnalysis
source ../../venv_analysis/bin/activate
python scripts/ik_execution_time_analysis.py ../../four_method_comparison_results.csv
```

### SinglePose Analysis
```bash
python scripts/single_pose_execution_analysis.py
```

### Quick Dashboard
```bash
python scripts/quick_dashboard.py <path_to_csv_file>
```

### Trajectory Visualization
```bash
python scripts/visualize_trajectory_analysis.py
```

## Dependencies

All scripts use the virtual environment at `../../venv_analysis/` with the following packages:
- pandas
- matplotlib  
- numpy
- seaborn (optional)
- pathlib

## Output

Results are automatically saved to the `results/` directory with timestamps for version control.
Generated files include:
- PNG visualization files
- CSV performance summaries
- TXT statistical reports
- JSON data exports
