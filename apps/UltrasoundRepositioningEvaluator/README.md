# Ultrasound Repositioning Evaluator

A specialized evaluation tool for assessing the performance of the UltrasoundTrajectoryPlanner in repositioning scenarios between two scan poses.

## Overview

The UltrasoundRepositioningEvaluator is designed to evaluate trajectory planning performance specifically for ultrasound medical robotics applications. It focuses on the critical repositioning task between two scan poses, which is fundamental for ultrasound scanning procedures.

## Features

### Evaluation Metrics
- **Planning Success Rate**: Percentage of successful trajectory plans
- **Planning Time**: Time required to generate trajectories
- **Trajectory Quality**: Length, smoothness, and energy consumption
- **Motion Characteristics**: Maximum velocities and accelerations
- **Collision Safety**: Verification of collision-free trajectories

### Supported Algorithms
- STOMP (Stochastic Trajectory Optimization for Motion Planning)
- Configurable parameters for comprehensive testing

### Output Formats
- **CSV Results**: Detailed trial-by-trial results
- **Statistics Summary**: Aggregate performance metrics
- **Timestamped Files**: Organized output with timestamps

## Usage

### Basic Usage
```bash
./UltrasoundRepositioningEvaluator
```

### With Custom Parameters
```bash
./UltrasoundRepositioningEvaluator [robot_urdf] [environment_xml] [scan_poses_csv] [output_dir]
```

### Example
```bash
./UltrasoundRepositioningEvaluator \
    ../../res/robot/panda.urdf \
    ../../res/scenario_1/obstacles.xml \
    ./my_scan_poses.csv \
    ./my_results
```

## Input Files

### Robot URDF
- Path to the robot description file
- Default: `../../../res/robot/panda.urdf`
- Example: Franka Emika Panda robot

### Environment XML
- Path to the obstacle environment description
- Default: `../../../res/scenario_1/obstacles.xml`
- Optional: Can run without obstacles

### Scan Poses CSV
- Contains the scan poses for repositioning evaluation
- Format: `x,y,z,qw,qx,qy,qz` (position and quaternion)
- Default: `./sample_scan_poses.csv` (auto-generated if missing)

#### CSV Format Example
```csv
0.5,0.2,0.6,1.0,0.0,0.0,0.0
0.4,-0.1,0.5,0.9659,0.0,0.0,0.2588
```

## Output Files

### Results CSV
- File: `repositioning_results_[timestamp].csv`
- Contains detailed results for each trial:
  - Trial number
  - Success/failure
  - Planning time
  - Trajectory metrics
  - Motion characteristics

### Statistics Summary
- File: `repositioning_statistics_[timestamp].txt`
- Contains aggregate statistics:
  - Success rates
  - Average performance metrics
  - Standard deviations
  - Overall assessment

## Configuration

### Default Parameters
- **Trials**: 5 (configurable in code)
- **Initial Joint Config**: Panda home position
- **STOMP Parameters**:
  - Max iterations: 50
  - Noisy trajectories: 5
  - Learning rate: 0.1
  - Temperature: 10.0

### Customization
Modify the configuration in `main.cpp`:
```cpp
config.num_trials = 10;  // Number of evaluation trials
config.max_stomp_iterations = 100;  // STOMP iterations
config.num_noisy_trajectories = 10;  // STOMP samples
```

## Building

The evaluator is built as part of the main project using CMake:

```bash
mkdir build && cd build
cmake ..
make UltrasoundRepositioningEvaluator
```

## Dependencies

- **TrajectoryLib**: Core trajectory planning library
- **USLib**: Ultrasound-specific trajectory planning
- **GeometryLib**: Geometric computations
- **Eigen3**: Linear algebra
- **Boost**: Threading and system utilities

## Evaluation Scenarios

### Standard Repositioning
- Start: First scan pose
- Target: Second scan pose
- Evaluation: Single repositioning trajectory

### Future Extensions
- Multi-pose sequences
- Different obstacle environments
- Algorithm comparisons (RRT vs STOMP)
- Real-time performance assessment

## Metrics Explained

### Planning Time
- Time to generate collision-free trajectory
- Measured in milliseconds
- Critical for real-time applications

### Trajectory Length
- Total distance in joint space
- Indicates movement efficiency
- Shorter is generally better

### Smoothness
- Based on trajectory jerk (acceleration changes)
- Lower values indicate smoother motion
- Important for precise medical procedures

### Energy Consumption
- Estimated based on joint accelerations
- Proxy for actual robot energy usage
- Lower values indicate more efficient trajectories

## Troubleshooting

### Common Issues

1. **File Not Found Errors**
   - Check URDF and environment file paths
   - Use absolute paths if relative paths fail

2. **Planning Failures**
   - Verify poses are reachable
   - Check for obstacle collisions
   - Adjust STOMP parameters

3. **Build Errors**
   - Ensure all dependencies are installed
   - Check CMake configuration
   - Verify library paths

### Debug Mode
Enable verbose output for detailed logging:
```cpp
config.verbose = true;
```

## Contributing

When extending the evaluator:
1. Add new metrics to `RepositioningResult` struct
2. Implement calculation methods in `RepositioningEvaluator`
3. Update CSV export format
4. Update statistics computation

## Example Results

Typical output structure:
```
evaluation_results/
├── repositioning_results_1672531200.csv
└── repositioning_statistics_1672531200.txt
```

### Sample Statistics Output
```
Ultrasound Repositioning Evaluation Statistics
=============================================

Overall Performance:
  Total Trials: 5
  Successful Trials: 4
  Success Rate: 80.00%

Planning Time Statistics:
  Average: 245.67 ms
  Std Dev: 89.23 ms

Trajectory Quality Statistics:
  Average Length: 2.34 rad
  Average Smoothness: 0.0156
```

This evaluator provides comprehensive assessment capabilities specifically tailored for ultrasound medical robotics applications.
