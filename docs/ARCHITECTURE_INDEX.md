# TrajectoryLib planTrajectories Documentation Index

This index provides navigation guidance for the TrajectoryLib architecture documentation suite, specifically focused on the functionality required by the `UltrasoundScanTrajectoryPlanner::planTrajectories()` method and its downstream components.

## 📋 Quick Navigation

| Document | Purpose | Lines | Best For |
|----------|---------|-------|----------|
| **[README](README.md)** | Overview and getting started | 98 | Project context |
| **[Visual Summary](TrajectoryLib_Visual_Summary.md)** | planTrajectories quick reference | 380+ | Implementation and troubleshooting |
| **[Architecture Diagram](TrajectoryLib_Architecture_Diagram.md)** | planTrajectories-focused architecture | 580+ | System integration |
| **[Detailed Class Diagram](TrajectoryLib_Detailed_Class_Diagram.md)** | planTrajectories implementation details | 640+ | Development work |

## 🎯 Choose Your Path

### For Developers Implementing planTrajectories Integration
1. Start with **[Architecture Diagram](TrajectoryLib_Architecture_Diagram.md)** for core functionality requirements
2. Deep dive into **[Detailed Class Diagram](TrajectoryLib_Detailed_Class_Diagram.md)** for implementation details
3. Reference **[Visual Summary](TrajectoryLib_Visual_Summary.md)** for troubleshooting and quick lookups

### For System Integrators Working with USTrajectoryPlanner  
1. Begin with **[Visual Summary](TrajectoryLib_Visual_Summary.md)** for usage patterns and API overview
2. Study **[Architecture Diagram](TrajectoryLib_Architecture_Diagram.md)** for system integration requirements
3. Review **[Detailed Class Diagram](TrajectoryLib_Detailed_Class_Diagram.md)** for error handling mechanisms

### For Performance Engineers and Optimization Specialists
1. Analyze **[Detailed Class Diagram](TrajectoryLib_Detailed_Class_Diagram.md)** for performance implementation details
2. Study **[Architecture Diagram](TrajectoryLib_Architecture_Diagram.md)** for optimization strategies
3. Use **[Visual Summary](TrajectoryLib_Visual_Summary.md)** for performance characteristics and metrics

## 📊 Documentation Statistics

- **Total Documentation**: 1,700+ lines across 4 files
- **planTrajectories Focus**: Complete coverage of actual implementation requirements
- **Visual Elements**: 32+ diagrams including Mermaid flowcharts and ASCII art
- **Code Examples**: 50+ code snippets showing actual usage patterns
- **Implementation Depth**: From data flow analysis to method-level implementation details

## 🔍 planTrajectories-Specific Focus Areas

### Core Functionality Chain
✅ **Environment Processing**: URDF parsing → BVHTree construction → Obstacle management  
✅ **Checkpoint Planning**: Pose sequence → IK solving → Validity checking → Segment grouping  
✅ **Batch Trajectory Planning**: Request building → Algorithm selection → Parallel optimization  
✅ **Contact Generation**: Valid segments → Time-optimal trajectories → Contact force flags  
✅ **Validation**: Discontinuity detection → Correction segments → Final validation  

### Key Implementation Details
✅ **Single vs Multi-pose handling**: Special case optimization for single pose requests  
✅ **Two-step STOMP fallback**: Robust error recovery for failed optimizations  
✅ **Shared SDF caching**: Memory-efficient collision detection across threads  
✅ **Hardware-aware batching**: CPU core detection and optimal batch size calculation  
✅ **Trajectory continuity**: Discontinuity detection and correction segment generation  

### Integration Requirements  
✅ **Input validation**: Environment, current joints, and pose sequence validation  
✅ **Memory management**: SDF cache initialization, thread pool management  
✅ **Error handling**: Exception hierarchies and fallback mechanisms  
✅ **Performance monitoring**: Timing analysis and resource utilization tracking  

## 🔗 Implementation Cross-References

### Key Source Files Analyzed
- `libs/USLib/src/USTrajectoryPlanner.cpp` - Main planTrajectories implementation
- `libs/USLib/include/USLib/USTrajectoryPlanner.h` - Public interface and data structures  
- `libs/TrajectoryLib/include/TrajectoryLib/Planning/PathPlanner.h` - Checkpoint planning
- `libs/TrajectoryLib/include/TrajectoryLib/Motion/MotionGenerator.h` - STOMP optimization

### External Dependencies Documented
- **Eigen3**: Linear algebra operations, Affine3d transformations, VectorXd configurations
- **Boost**: asio::thread_pool, geometry library, mathematical functions
- **orocos_kdl**: Forward/inverse kinematics, chain kinematics, dynamics
- **Hauser10**: DynamicPath optimization, ParabolicRamp planning, FeasibilityChecker
- **GeometryLib**: BVHTree spatial acceleration, ObstacleTree management
- **Qt Framework**: QDebug logging, QObject base classes, thread synchronization

## 📈 Diagram Coverage by Component

### planTrajectories Data Flow
- Complete sequence diagrams showing actual method call patterns
- Data structure transformations from input to output
- Error handling and fallback mechanism flows

### Component Dependencies
- Class diagrams showing actual planTrajectories component relationships
- Method-level interaction patterns
- Memory and resource management strategies

### Performance Analysis
- Thread utilization patterns during batch processing
- Memory usage breakdown by component
- SDF caching and shared resource management

### Algorithm Integration
- STOMP optimization pipeline with actual parameter configurations
- RRT+Hauser fallback mechanism implementation
- Trajectory validation and discontinuity correction processes

## 🏗️ Architecture Validation

This documentation suite was created by analyzing the actual `planTrajectories` implementation and tracing all downstream functionality requirements. Every component, data structure, and algorithm documented is directly used by or required for the `planTrajectories` method to function correctly.

### Validation Criteria
✅ All documented components are directly referenced in planTrajectories code  
✅ Data flow diagrams match actual method call sequences  
✅ Performance characteristics reflect actual implementation behavior  
✅ Error handling covers all exception types thrown by planTrajectories  
✅ Memory usage estimates based on actual data structure sizes  
✅ Algorithm parameters match optimized configurations used in production  

## 🚀 Implementation Support

This focused approach ensures the documentation directly supports developers working with the planTrajectories functionality, providing practical, implementation-relevant guidance rather than abstract architectural concepts.

### Ready-to-Use Resources
- Complete usage patterns with error handling
- Troubleshooting guide for common planTrajectories issues
- Performance optimization recommendations based on actual bottlenecks
- Integration checklists for successful planTrajectories deployment
- Memory and CPU utilization analysis for capacity planning

The comprehensive **1,700+ lines** of planTrajectories-focused documentation enable effective development, integration, and optimization of ultrasound scan trajectory planning systems.