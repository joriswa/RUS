# UltrasoundRepositioningEvaluator Configuration System

## Overview

The UltrasoundRepositioningEvaluator now supports a comprehensive configuration system that allows users to specify different trajectory planning algorithms with their respective parameters. This enables systematic comparison of STOMP variants and Hauser algorithms for ultrasound repositioning tasks.

## Supported Algorithms

### Trajectory Planning Algorithms

1. **STOMP** - Standard Stochastic Trajectory Optimization for Motion Planning
2. **STOMP_WITH_CHECKPOINTS** - STOMP variant that enforces intermediate waypoints
3. **STOMP_WITH_EARLY_TERMINATION** - STOMP with convergence-based early stopping
4. **HAUSER** - Hauser algorithm for motion generation (requires path planning phase)

### Path Planning Algorithms (for Hauser)

1. **RRT** - Rapidly-exploring Random Tree
2. **RRT_STAR** - Optimal variant of RRT
3. **INFORMED_RRT_STAR** - RRT* with ellipsoidal sampling
4. **RRT_CONNECT** - Bidirectional RRT

## Configuration Structure

### Main Configuration (`RepositioningEvalConfig`)

```cpp
struct RepositioningEvalConfig {
    // File paths
    std::string robot_urdf_path;
    std::string environment_xml_path;
    std::string scan_poses_csv_path;
    std::string output_directory;
    
    // Robot configuration
    Eigen::VectorXd initial_joint_config;
    
    // Evaluation parameters
    int num_trials = 10;
    bool verbose = true;
    
    // Algorithm selection
    TrajectoryAlgorithm trajectory_algorithm = TrajectoryAlgorithm::STOMP;
    
    // Algorithm-specific configurations
    StompAlgorithmConfig stomp_config;
    HauserAlgorithmConfig hauser_config;
    PathPlanningConfig path_planning_config;
};
```

### STOMP Configuration (`StompAlgorithmConfig`)

```cpp
struct StompAlgorithmConfig {
    int max_iterations = 100;
    int num_noisy_trajectories = 10;
    int num_best_samples = 4;
    double learning_rate = 0.1;
    double temperature = 10.0;
    double dt = 0.1;
    int num_joints = 7;
    Eigen::VectorXd joint_std_devs = Eigen::VectorXd::Constant(7, 0.1);
};
```

### Hauser Configuration (`HauserAlgorithmConfig`)

```cpp
struct HauserAlgorithmConfig {
    int max_iterations = 500;
    std::string output_file = ""; // Optional timing output file
};
```

### Path Planning Configuration (`PathPlanningConfig`)

```cpp
struct PathPlanningConfig {
    PathPlanningAlgorithm algorithm = PathPlanningAlgorithm::RRT;
    int max_iterations = 5000;
    double step_size = 0.1;
    double goal_bias = 0.1;
    bool custom_cost = false;
};
```

## Usage Examples

### Basic STOMP Configuration

```cpp
RepositioningEvalConfig config;
config.robot_urdf_path = "/path/to/robot.urdf";
config.environment_xml_path = "/path/to/environment.xml";
config.scan_poses_csv_path = "/path/to/scan_poses.csv";
config.output_directory = "./results";

// Initial joint configuration
config.initial_joint_config = Eigen::VectorXd(7);
config.initial_joint_config << -M_PI/4, M_PI/8, -M_PI/8, -M_PI/3, 0, M_PI/4, 0;

// Use STOMP algorithm
config.trajectory_algorithm = TrajectoryAlgorithm::STOMP;

// Customize STOMP parameters
config.stomp_config.max_iterations = 200;
config.stomp_config.num_noisy_trajectories = 15;
config.stomp_config.learning_rate = 0.15;

RepositioningEvaluator evaluator(config);
evaluator.runEvaluation();
```

### Hauser with RRT* Configuration

```cpp
RepositioningEvalConfig config;
// ... set basic paths and parameters ...

// Use Hauser algorithm
config.trajectory_algorithm = TrajectoryAlgorithm::HAUSER;

// Configure path planning phase (RRT*)
config.path_planning_config.algorithm = PathPlanningAlgorithm::RRT_STAR;
config.path_planning_config.max_iterations = 10000;
config.path_planning_config.step_size = 0.1;
config.path_planning_config.goal_bias = 0.15;

// Configure motion generation phase (Hauser)
config.hauser_config.max_iterations = 800;
config.hauser_config.output_file = "./hauser_timing.csv";

RepositioningEvaluator evaluator(config);
evaluator.runEvaluation();
```

### STOMP with Early Termination

```cpp
RepositioningEvalConfig config;
// ... set basic configuration ...

config.trajectory_algorithm = TrajectoryAlgorithm::STOMP_WITH_EARLY_TERMINATION;

// Allow more iterations since early termination will stop when converged
config.stomp_config.max_iterations = 500;
config.stomp_config.learning_rate = 0.1; // Conservative learning rate
config.stomp_config.temperature = 12.0;

RepositioningEvaluator evaluator(config);
evaluator.runEvaluation();
```

## Algorithm Selection Guidelines

### When to Use STOMP Variants

- **STOMP**: General-purpose trajectory optimization, good balance of speed and quality
- **STOMP_WITH_CHECKPOINTS**: When you need to enforce specific intermediate poses
- **STOMP_WITH_EARLY_TERMINATION**: When computational efficiency is important and you want automatic convergence detection

### When to Use Hauser Algorithm

- **HAUSER**: When you need explicit path planning followed by motion generation
- Useful for complex environments where path planning and trajectory optimization should be separated
- Can leverage different RRT variants for path planning phase

### Path Planning Algorithm Selection (for Hauser)

- **RRT**: Fastest, suitable for simple environments
- **RRT_STAR**: Better path quality, more computational cost
- **INFORMED_RRT_STAR**: Best path quality, highest computational cost
- **RRT_CONNECT**: Good for point-to-point planning in complex environments

## Parameter Tuning Guidelines

### STOMP Parameters

- **max_iterations**: Start with 100-200, increase for complex problems
- **num_noisy_trajectories**: 10-20, more samples = better exploration but slower
- **learning_rate**: 0.1-0.2, higher = faster convergence but less stable
- **temperature**: 8-15, higher = more exploration
- **dt**: 0.05-0.1, smaller = smoother trajectories but more waypoints

### Path Planning Parameters

- **max_iterations**: 5000-15000 depending on complexity
- **step_size**: 0.05-0.15, smaller = more precise but slower
- **goal_bias**: 0.1-0.3, higher = faster convergence to goal

### Hauser Parameters

- **max_iterations**: 500-1500, depends on trajectory complexity

## Output and Analysis

### CSV Results

The evaluator exports detailed results including:

- Algorithm-specific timing information
- Path planning vs motion generation breakdown (for Hauser)
- Trajectory quality metrics
- Success rates and performance statistics

### Result Fields

- `algorithm`: Primary algorithm used (STOMP, HAUSER, etc.)
- `path_planning_algorithm`: Path planning algorithm used (for Hauser)
- `planning_time_ms`: Total planning time
- `path_planning_time_ms`: Time spent on path planning (Hauser only)
- `motion_generation_time_ms`: Time spent on motion generation (Hauser only)
- `iterations_used`: Number of iterations used
- `trajectory_length`: Total joint space distance
- `smoothness`: Trajectory smoothness metric (lower is better)
- `collision_free`: Whether trajectory is collision-free

## Backward Compatibility

The system maintains backward compatibility through legacy parameters:

```cpp
// Legacy parameters are automatically mapped to new configuration structures
config.max_stomp_iterations = 150;  // → stomp_config.max_iterations
config.stomp_learning_rate = 0.12;  // → stomp_config.learning_rate
config.max_rrt_iterations = 8000;   // → path_planning_config.max_iterations
```

## Performance Recommendations

### For Speed-Critical Applications

```cpp
config.trajectory_algorithm = TrajectoryAlgorithm::STOMP;
config.stomp_config.max_iterations = 50;
config.stomp_config.num_noisy_trajectories = 8;
config.stomp_config.learning_rate = 0.2;
```

### For Quality-Critical Applications

```cpp
config.trajectory_algorithm = TrajectoryAlgorithm::HAUSER;
config.path_planning_config.algorithm = PathPlanningAlgorithm::INFORMED_RRT_STAR;
config.path_planning_config.max_iterations = 15000;
config.hauser_config.max_iterations = 1500;
```

## Example Files

- `example_configs.cpp`: Comprehensive examples of different configurations
- `sample_scan_poses.csv`: Example scan poses for testing
- Results are saved to specified output directories with timestamped filenames

## Troubleshooting

### Common Issues

1. **Path Planning Failure**: Increase `max_iterations` or adjust `step_size`
2. **STOMP Convergence Issues**: Adjust `learning_rate` and `temperature`
3. **Memory Issues**: Reduce `num_noisy_trajectories` or `max_iterations`
4. **Slow Performance**: Use speed-optimized configurations or reduce problem complexity

### Debug Recommendations

1. Enable verbose logging: `config.verbose = true`
2. Start with simpler algorithms (STOMP, RRT) before complex variants
3. Verify file paths and initial joint configurations
4. Check that scan poses are reachable by the robot
