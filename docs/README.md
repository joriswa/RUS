# TrajectoryLib planTrajectories Documentation

This directory contains comprehensive architecture documentation specifically focused on the functionality required by the `UltrasoundScanTrajectoryPlanner::planTrajectories()` method, which serves as the primary entry point for ultrasound scan trajectory planning.

## planTrajectories-Focused Documentation

### 📐 [TrajectoryLib Architecture Diagram](TrajectoryLib_Architecture_Diagram.md)
planTrajectories-specific architectural analysis showing:
- Core functionality requirements and data flow
- Required component dependencies (RobotManager, BVHTree, PathPlanner, MotionGenerator)
- Complete planTrajectories data flow with sequence diagrams
- Algorithm integration (STOMP optimization pipeline, RRT+Hauser pipeline)
- Performance optimizations (SDF caching, batch processing)
- System integration requirements and external library dependencies

### 🏗️ [Detailed Implementation Analysis](TrajectoryLib_Detailed_Class_Diagram.md)
Comprehensive implementation documentation including:
- UltrasoundScanTrajectoryPlanner class detailed breakdown
- PathPlanner component with CheckpointPlanResult structure
- MotionGenerator component with STOMP and Hauser integration
- Robot component hierarchy (RobotArm, RobotManager)
- Collision detection system (BVHTree, ArmFeasibilityChecker)
- Data flow implementation with method-level detail
- Performance implementation (ThreadPoolManager, SDFCacheManager)
- Error handling and fallback mechanisms

### 📊 [Visual Summary and Quick Reference](TrajectoryLib_Visual_Summary.md)
Quick reference and troubleshooting guide including:
- ASCII-based data flow diagrams suitable for terminal viewing
- Component interaction matrix showing planTrajectories dependencies
- Algorithm decision tree for STOMP vs Hauser selection
- Performance characteristics with visual charts
- Data structure quick reference (TrajectoryPoint, CheckpointPlanResult, StompConfig)
- Troubleshooting guide with common issues and solutions
- Integration checklist for planTrajectories implementation

## planTrajectories Core Functionality

### Function Overview
The `planTrajectories` method orchestrates multiple TrajectoryLib components to generate collision-free, time-optimal paths for both repositioning movements and contact-force scanning operations.

```cpp
bool planTrajectories(bool useHauserForRepositioning = false, 
                     bool enableShortcutting = true)

// Input Requirements:
// - Environment URDF string (setEnvironment)
// - Current 7-DOF joint configuration (setCurrentJoints)  
// - Target scan poses sequence (setPoses)

// Output:
// - Complete trajectory sequence with contact force flags
// - Optimized for hardware performance and safety
```

### Key Implementation Features

#### Core Functionality Chain
1. **Environment Processing**: URDF parsing → BVHTree construction → Obstacle management
2. **Checkpoint Planning**: Pose sequence → IK solving → Validity checking → Segment grouping
3. **Batch Trajectory Planning**: Request building → Algorithm selection → Parallel optimization
4. **Contact Generation**: Valid segments → Time-optimal trajectories → Contact force flags
5. **Validation**: Discontinuity detection → Correction segments → Final validation

#### Performance Optimizations
- **Shared SDF Caching**: Memory-efficient collision detection across threads
- **Hardware-Aware Batching**: CPU core detection and optimal batch size calculation
- **Parallel Processing**: Multi-threaded STOMP/Hauser optimization
- **Two-Step Fallback**: Robust error recovery for failed optimizations

#### Algorithm Integration
- **STOMP Pipeline**: Stochastic optimization with research-tuned parameters
- **RRT+Hauser Pipeline**: Sampling-based planning with physics-aware smoothing
- **Runtime Selection**: Algorithm choice based on quality vs speed requirements

## Component Dependencies

### Required Components for planTrajectories

| Component | Purpose | Key Usage |
|-----------|---------|-----------|
| **RobotManager** | URDF parsing and obstacle extraction | Environment setup phase |
| **BVHTree** | Collision detection and spatial queries | Real-time collision checking |
| **PathPlanner** | Checkpoint planning and IK solving | Pose sequence processing |
| **MotionGenerator** | STOMP/Hauser trajectory optimization | Batch trajectory planning |
| **RobotArm** | Kinematics and configuration management | Throughout all phases |
| **ThreadPool** | Parallel processing coordination | Batch optimization |
| **SDF Cache** | Shared collision distance queries | Performance optimization |

### External Dependencies

| Library | Usage in planTrajectories | Critical Functions |
|---------|---------------------------|-------------------|
| **Eigen3** | Linear algebra, pose transformations | VectorXd, Affine3d operations |
| **Boost** | Thread pool, mathematical functions | asio::thread_pool, geometry |
| **orocos_kdl** | Kinematics calculations | Forward/inverse kinematics |
| **Hauser10** | Time-optimal trajectory generation | DynamicPath, ParabolicRamp |
| **GeometryLib** | Collision detection infrastructure | BVHTree, ObstacleTree |
| **Qt Framework** | Logging and system utilities | QDebug, QObject |

## Integration Examples

### Basic planTrajectories Usage
```cpp
// Setup
UltrasoundScanTrajectoryPlanner planner(urdfEnvironment);
planner.setCurrentJoints(currentJointConfig);
planner.setPoses(scanPoseSequence);

// Planning
bool success = planner.planTrajectories(false, true); // STOMP + shortcutting

// Execution
if (success) {
    auto trajectories = planner.getTrajectories();
    for (const auto& [points, isContactForce] : trajectories) {
        if (isContactForce) {
            executeContactTrajectory(points);
        } else {
            executeRepositioningTrajectory(points);
        }
    }
}
```

### Error Handling Pattern
```cpp
try {
    bool success = planner.planTrajectories();
    if (!success) {
        LOG_ERROR << "Trajectory planning failed";
        return false;
    }
} catch (const StompFailedException& e) {
    LOG_WARNING << "STOMP failed: " << e.what();
    // Try Hauser fallback
    success = planner.planTrajectories(true, true);
} catch (const std::runtime_error& e) {
    LOG_ERROR << "Planning error: " << e.what();
    return false;
}
```

## Documentation Navigation

For detailed implementation guidance, follow the role-based reading paths in [ARCHITECTURE_INDEX.md](ARCHITECTURE_INDEX.md):

- **Developers**: Implementation details and code examples
- **System Integrators**: API usage and integration patterns  
- **Performance Engineers**: Optimization strategies and tuning

This documentation suite provides **1,700+ lines** of comprehensive planTrajectories-focused analysis, enabling effective development, integration, and optimization of ultrasound scan trajectory planning systems.
- **Eigen3**: Linear algebra and matrix operations
- **Boost**: Mathematical functions and threading
- **orocos_kdl**: Kinematics and dynamics library
- **GeometryLib**: BVH trees and collision detection
- **Hauser10**: Dynamic path representation

### Integration
- **USLib**: Medical ultrasound trajectory planning
- **Parameter Tuning Tools**: STOMP optimization configuration
- **Visualization Applications**: Real-time trajectory display

## Performance Characteristics

- **Real-time Capability**: Sub-second trajectory generation
- **Multi-threaded STOMP**: Parallel optimization across CPU cores
- **Memory Efficient**: Optimized data structures and memory pools
- **Scalable**: Handles complex 7-DOF robot configurations

## Safety and Reliability

- **Collision Avoidance**: Multi-level collision detection system
- **Joint Limit Enforcement**: Hardware constraint validation
- **Exception Handling**: Robust error recovery mechanisms
- **Trajectory Validation**: Comprehensive quality assurance

## Getting Started

1. **Review Architecture**: Start with the [main architecture diagram](TrajectoryLib_Architecture_Diagram.md)
2. **Understand Classes**: Examine the [detailed class documentation](TrajectoryLib_Detailed_Class_Diagram.md)
3. **Explore Code**: Navigate the TrajectoryLib source code with architectural context
4. **Integration**: Use the API documentation for integrating with other systems

## Future Enhancements

The architecture is designed for extensibility:
- **New Planning Algorithms**: Plugin-based algorithm framework
- **Custom Cost Functions**: Extensible cost calculator system
- **Additional Robot Models**: Modular robot definition support
- **Advanced Visualization**: Qt-based rendering extensions

## Contributing

When contributing to TrajectoryLib, please:
1. Follow the modular architecture principles
2. Maintain thread safety in shared components
3. Add appropriate error handling and validation
4. Update architecture documentation for significant changes

## Related Documentation

- [RUS System Architecture](../RUS_SYSTEM_ARCHITECTURE_DOCUMENTATION.md)
- [TrajectoryLib Treatment Plan](../TRAJECTORYLIB_FULL_TREATMENT_PLAN.md)
- [STOMP Implementation Guide](../STOMP_INTERFACE_GUIDE.md)