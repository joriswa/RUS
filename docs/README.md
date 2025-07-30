# TrajectoryLib Documentation

This directory contains comprehensive documentation for the TrajectoryLib library, the core trajectory planning and optimization component of the Robotic Ultrasound System (RUS).

## Architecture Documentation

### 📐 [TrajectoryLib Architecture Diagram](TrajectoryLib_Architecture_Diagram.md)
A high-level architectural overview showing:
- Module organization and relationships
- External dependencies and integration points
- Data flow and system interfaces
- Key components and their interactions

### 🏗️ [Detailed Class Architecture](TrajectoryLib_Detailed_Class_Diagram.md)
Comprehensive class-level documentation including:
- Detailed class diagrams with methods and properties
- Design patterns and architectural decisions
- Memory management and thread safety
- Performance optimization features

## Key Features

### Core Capabilities
- **STOMP Optimization**: Stochastic Trajectory Optimization for Motion Planning
- **Path Planning**: RRT and RRT* algorithms for initial path generation
- **Robot Kinematics**: Forward/inverse kinematics with multiple robot support
- **Collision Detection**: Efficient spatial indexing with BVH trees
- **Multi-threading**: Parallel optimization for real-time performance

### Architecture Highlights
- **Modular Design**: Six distinct modules (Core, Motion, Planning, Robot, Utils, Visualization)
- **Plugin Architecture**: Extensible cost functions and planning algorithms  
- **Safety-Critical**: Medical-grade reliability and error handling
- **High Performance**: Multi-threaded STOMP with SIMD optimization

## Module Overview

| Module | Purpose | Key Components |
|--------|---------|----------------|
| **Core** | Utilities and basic algorithms | Util, spline, SimulatedAnnealing |
| **Motion** | Trajectory optimization | MotionGenerator (STOMP), CostCalculators |  
| **Planning** | Path planning algorithms | PathPlanner, RRT, RRT* |
| **Robot** | Robot modeling and kinematics | Robot, RobotArm, RobotManager |
| **Utils** | Analysis and evaluation | TrajectoryEvaluator |
| **Visualization** | 3D rendering support | TrackballCameraController |

## Dependencies

### External Libraries
- **Qt Framework**: GUI, OpenGL, 3D rendering
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