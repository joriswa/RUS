# TrajectoryLib Full Treatment Plan

## Executive Summary

This document outlines a comprehensive treatment plan for the entire TrajectoryLib library. The goal is to modernize, optimize, and clean up the library while preserving all functionality required by dependent applications, particularly the USLib `planTrajectories` system.

## Current Library Analysis

### Library Structure
```
TrajectoryLib/
├── Core/           # Utilities, splines, simulated annealing
├── Motion/         # MotionGenerator, STOMP, cost calculators  
├── Planning/       # PathPlanner, RRT variants
├── Robot/          # Robot models, kinematics, managers
├── Utils/          # TrajectoryEvaluator, analysis tools
├── Visualization/  # Camera controllers, rendering support
└── Logger.h        # Custom logging system (recently added)
```

### Key Components Identified

#### 🎯 **Core Motion Planning** (High Priority)
- **MotionGenerator** - STOMP trajectory optimization (2,698 lines)
- **PathPlanner** - RRT/RRT* algorithms 
- **Cost Calculators** - 6 different cost functions
- **Robot Models** - Kinematics and collision detection

#### 🔧 **Supporting Infrastructure** (Medium Priority)  
- **Robot Management** - RobotArm, RobotManager, IK solvers
- **Core Utilities** - Splines, utilities, simulated annealing
- **Trajectory Analysis** - Evaluation and metrics tools

#### 📊 **Auxiliary Components** (Lower Priority)
- **Visualization** - Camera controls, rendering support
- **Logging** - Custom high-performance logging system

### Dependencies and Usage Analysis

#### **External Dependencies**
- Qt (Widgets, OpenGL, 3D) - Heavy GUI dependency
- Eigen3 - Linear algebra (essential)
- Boost (Math, System) - Numerical computations
- orocos_kdl - Kinematics and dynamics

#### **Dependent Applications**
- USLib (planTrajectories system) - **CRITICAL**
- Parameter Tuning applications 
- Comparison and evaluation tools
- PathPlanner GUI application

## Treatment Plan Overview

### Phase 1: Code Quality & Dead Code Elimination (1-2 weeks)
**Status: 30% Complete** ✅ MotionGenerator partially cleaned

### Phase 2: Architecture Optimization (2-3 weeks) 
**Goal: Reduce coupling, improve modularity**

### Phase 3: Performance Optimization (1-2 weeks)
**Goal: Eliminate bottlenecks, optimize algorithms**

### Phase 4: Dependency Reduction (2-3 weeks) 
**Goal: Reduce Qt dependency, modernize build system**

### Phase 5: API Standardization (1 week)
**Goal: Consistent interfaces, better documentation**

---

## Phase 1: Code Quality & Dead Code Elimination

### 1.1 MotionGenerator Deep Clean ✅ **In Progress**

#### **Completed:**
- ✅ Removed 3 unimplemented method declarations
- ✅ Fixed Logger.h include path issues
- ✅ Cleaned basic commented code

#### **Remaining Tasks:**
- 🔄 **Remove ~250-line commented STOMP block** in `performSTOMPWithCheckpoints()`
- 🔍 **Cost Calculator Usage Analysis**
  - Verify which of the 6 cost calculators are actually used
  - Remove unused: `EndEffectorMovementCostCalculator`, `MagnetEndPositionCostCalculator`, etc.
- 🧹 **Clean Redundant Methods**
  - Multiple overloaded `computeCost()` methods
  - Unused trajectory processing methods
  - Debug and utility methods not used by planTrajectories

#### **Safety Measures:**
- Preserve all methods used by USLib's planTrajectories
- Maintain function signatures (no breaking changes)
- Incremental removal with compilation verification

### 1.2 PathPlanner Clean Up

#### **Analysis Needed:**
- Which RRT variants are actively used?
- Are all path planning methods necessary?
- Clean up commented algorithm implementations

#### **Tasks:**
```cpp
// Preserve these (confirmed by user requirements):
- RRT (basic)
- RRT* (optimal) 
- InformedRRT* (ellipsoidal)
- RRTConnect (bidirectional)

// Investigate for removal:
- Unused path planning utilities
- Commented experimental algorithms
- Debug/visualization code in planning
```

### 1.3 Robot Module Cleanup

#### **Components to Analyze:**
- **RobotArm.cpp** - Core kinematics (keep)
- **RobotManager.cpp** - Robot instance management 
- **franka_ik_He.cpp** - Franka-specific IK solver
- **Robot.cpp** - Base robot class

#### **Cleanup Tasks:**
- Remove unused robot model methods
- Clean up commented IK algorithms
- Verify all robot methods are needed by planTrajectories

### 1.4 Core Utilities Review

#### **Components:**
- **spline.h** - Mathematical spline functions (likely needed)
- **Util.cpp** - General utilities 
- **SimulatedAnnealing.cpp** - Optimization algorithm

#### **Tasks:**
- Verify spline usage in trajectory generation
- Remove unused utility functions
- Check if simulated annealing is used anywhere

---

## Phase 2: Architecture Optimization

### 2.1 Dependency Injection & Interface Segregation

#### **Current Issues:**
- MotionGenerator has massive public interface (40+ methods)
- Tight coupling between cost calculators and motion generation
- Hard-coded dependencies in constructors

#### **Proposed Changes:**
```cpp
// Instead of monolithic MotionGenerator:
class TrajectoryOptimizer {
    // Core STOMP optimization only
};

class CostFunctionManager {  
    // Manages composite cost calculations
};

class TrajectoryPostProcessor {
    // Time-optimal scaling, resampling, smoothing
};
```

### 2.2 Cost Calculator System Redesign

#### **Current Issues:**
- 6 different cost calculators with unclear usage
- Composite calculator is complex and potentially unused
- Hard to extend or modify cost functions

#### **Proposed Simplification:**
```cpp
// Keep only actively used calculators:
- ObstacleCostCalculator (collision avoidance)
- ConstraintCostCalculator (joint limits) [if used]

// Remove if unused:
- EndEffectorMovementCostCalculator
- MagnetEndPositionCostCalculator  
- TaskSpacePathTrackingCostCalculator
- CheckpointTransitionCostCalculator
- ApproachDepartureCostCalculator
```

### 2.3 Configuration Management

#### **Current Issues:**
- StompConfig struct has many parameters
- Hard-coded constants scattered throughout
- No validation of configuration parameters

#### **Proposed Solution:**
```cpp
class TrajectoryPlanningConfig {
    // Centralized, validated configuration
    // Default values for common use cases
    // Parameter validation and constraints
};
```

---

## Phase 3: Performance Optimization

### 3.1 STOMP Algorithm Optimization

#### **Current Performance Issues:**
- Parallel trajectory generation could be optimized
- Memory allocations in hot loops
- Redundant cost calculations

#### **Optimization Targets:**
```cpp
// Hot paths identified:
1. Trajectory generation loop (performSTOMP)
2. Cost calculation (computeCost methods)
3. Matrix operations in smoothTrajectoryUpdate
4. Time-optimal scaling pipeline
```

### 3.2 Memory Management

#### **Issues:**
- Frequent Eigen matrix allocations
- Large trajectory matrices created/destroyed repeatedly
- Potential memory leaks in thread pools

#### **Solutions:**
- Object pooling for trajectory matrices
- Pre-allocate working memory buffers
- Smart pointer cleanup in parallel sections

### 3.3 Logging System Optimization ✅ **Recently Completed**

#### **Current Status:**
- ✅ Custom high-performance Logger.h implemented
- ✅ Replaced qDebug() calls in MotionGenerator
- ✅ Zero-overhead when disabled

---

## Phase 4: Dependency Reduction

### 4.1 Qt Dependency Analysis

#### **Current Qt Usage:**
```cpp
// Heavy Qt dependencies:
- QElapsedTimer (performance timing)
- Qt 3D rendering (Visualization module)  
- Qt Widgets (GUI components)
- Qt OpenGL (rendering)

// Minimal Qt usage:
- QElapsedTimer → std::chrono replacement
- qDebug() → custom Logger (already done)
```

#### **Reduction Strategy:**
1. **Replace QElapsedTimer** with `std::chrono::high_resolution_clock`
2. **Separate Visualization Module** - make Qt dependency optional
3. **Core Library Qt-Free** - only visualization needs Qt

### 4.2 Boost Dependency Optimization

#### **Current Boost Usage:**
- Boost.Math (splines, interpolation) - **Essential**
- Boost.System (threading) - **Essential** 
- Boost.Asio (thread pools) - **Essential**

#### **Assessment:**
- ✅ Boost dependencies are justified and performant
- Keep current Boost usage - well-established and optimized

### 4.3 Modular Build System

#### **Proposed CMake Structure:**
```cmake
# Core trajectory planning (Qt-free)
TrajectoryLib_Core  

# Visualization extensions (Qt-dependent)  
TrajectoryLib_Visualization

# Full library (backward compatibility)
TrajectoryLib
```

---

## Phase 5: API Standardization

### 5.1 Consistent Naming Conventions

#### **Current Issues:**
- Mixed camelCase/snake_case in some areas
- Inconsistent parameter naming
- Some unclear method names

#### **Standardization:**
```cpp
// Method naming:
compute*()    // for calculations
generate*()   // for creating new data
set*() / get*() // for properties
is*() / has*() // for boolean checks

// Parameter naming:
trajectory    // for std::vector<TrajectoryPoint>
jointConfig   // for Eigen::VectorXd joint angles
taskPose      // for Eigen::Affine3d poses
```

### 5.2 Error Handling Standardization

#### **Current Issues:**
- Mix of return codes, exceptions, and logging
- Inconsistent error messages
- Some silent failures

#### **Proposed Standard:**
```cpp
enum class TrajectoryPlanningResult {
    Success,
    CollisionDetected, 
    JointLimitsExceeded,
    ConfigurationError,
    OptimizationFailed
};

// Consistent error reporting with logging
```

### 5.3 Documentation & API Reference

#### **Current State:**
- Sparse Doxygen comments
- No usage examples
- Missing API documentation

#### **Documentation Plan:**
- Add comprehensive Doxygen comments
- Create usage examples for common scenarios
- Document planTrajectories integration patterns

---

## Implementation Strategy

### Safety-First Approach
1. **No Breaking Changes** - Preserve all function signatures
2. **Incremental Testing** - Compile and test after each change
3. **Usage Verification** - Confirm planTrajectories works after each phase
4. **Rollback Capability** - Git commits for each major change

### Risk Management

#### **Low Risk Changes:**
- Remove clearly dead code (comments, unused methods)
- Add documentation and logging
- Optimize memory usage in non-critical paths

#### **Medium Risk Changes:**  
- Remove unused cost calculators (verify usage first)
- Refactor internal method organization
- Replace Qt timing with std::chrono

#### **High Risk Changes (Avoid):**
- Change public method signatures
- Modify core STOMP algorithm  
- Remove any methods used by planTrajectories

### Validation Plan

#### **After Each Phase:**
1. ✅ Full compilation without errors
2. ✅ USLib planTrajectories functional test
3. ✅ Parameter tuning applications work
4. ✅ No performance regressions

#### **Final Validation:**
- Complete trajectory planning test suite
- Performance benchmarking vs. original
- Memory usage analysis
- Integration test with all dependent applications

---

## Expected Outcomes

### Code Quality Improvements
- **30-40% reduction** in MotionGenerator.cpp file size
- **Elimination of 200+ lines** of commented dead code
- **Removal of 3-5 unused cost calculator classes**
- **Consistent coding standards** throughout library

### Performance Benefits  
- **Reduced compilation time** (fewer unused templates/headers)
- **Lower memory usage** (eliminated redundant allocations)
- **Faster trajectory planning** (optimized hot paths)
- **Optional Qt dependency** (core library Qt-free)

### Maintainability Gains
- **Clearer module boundaries** and responsibilities  
- **Simplified public interfaces** with focused functionality
- **Comprehensive documentation** and usage examples
- **Modern C++20 features** and best practices

### Dependency Optimization
- **Optional Qt dependency** for visualization only
- **Reduced binary size** by eliminating unused components
- **Modular build system** for flexible integration

---

## Timeline & Milestones

### Week 1-2: Code Quality & Dead Code Elimination ✅ **Started**
- Complete MotionGenerator cleanup
- Remove unused cost calculators  
- Clean all modules of dead code

### Week 3-4: Architecture Optimization
- Refactor MotionGenerator interface
- Simplify cost calculator system
- Implement dependency injection patterns

### Week 5-6: Performance Optimization  
- Optimize STOMP algorithm hot paths
- Implement memory pooling
- Profile and eliminate bottlenecks

### Week 7-8: Dependency Reduction
- Replace Qt dependencies in core
- Modular build system implementation
- Visualization module separation

### Week 9: API Standardization & Documentation
- Consistent naming and error handling
- Comprehensive documentation
- Usage examples and integration guides

### Week 10: Final Testing & Validation
- Complete test suite execution
- Performance benchmarking
- Integration testing with all applications

---

## Success Metrics

### Quantitative Goals
- ✅ **25-40% code size reduction** in key modules
- ✅ **100% compilation success** after each change
- ✅ **Zero breaking changes** to public APIs
- ✅ **Performance neutral or improved** 

### Qualitative Goals  
- ✅ **Improved code readability** and maintainability
- ✅ **Simplified dependencies** and build process
- ✅ **Enhanced documentation** and developer experience
- ✅ **Modern C++ standards** and best practices

---

*This comprehensive treatment plan ensures TrajectoryLib becomes a modern, efficient, and maintainable trajectory planning library while preserving all critical functionality for dependent applications.*

## Precise planTrajectories Dependency Cascade Analysis ✅

### **Critical Findings from USTrajectoryPlanner::planTrajectories() Code Analysis:**

#### **MotionGenerator Methods - ABSOLUTELY PRESERVE** 🔒
Based on direct method calls in planTrajectories execution flow:

```cpp
// Core trajectory planning methods (CRITICAL):
1. setObstacleTree()               // Called 6 times across all trajectory segments
2. setWaypoints()                  // Called 6 times for waypoint-based planning  
3. performSTOMP()                  // Called 3 times for optimization
4. performHauser()                 // Called 2 times when useHauserForRepositioning=true
5. getPath()                       // Called 3 times to retrieve results
6. generateTrajectoryFromCheckpoints() // Called 1 time for scanning trajectories
```

#### **PathPlanner Methods - ABSOLUTELY PRESERVE** 🔒
Based on RRT usage in planTrajectories:

```cpp
// RRT path planning methods (CRITICAL):
1. planCheckpoints()               // Main orchestration for checkpoint planning
2. setStartPose()                  // Robot initial configuration 
3. setObstacleTree()               // Collision environment setup
4. setGoalConfiguration()          // Target robot configuration
5. runPathFinding()                // Execute RRT algorithms
6. getAnglesPath()                 // Retrieve path as joint angle matrix
```

#### **RRT Algorithms - PRESERVE ALL** 🔒 (User explicit requirement)
All RRT variants must be retained regardless of current usage:
- RRT, RRT*, InformedRRT*, RRTConnect

#### **Hauser Integration - DO NOT TOUCH** 🔒 (User explicit requirement)  
- Integration point: `motionGen->performHauser(300, "", 100)`
- Pathway: PathPlanner → geometric path → MotionGenerator → Hauser

### **Safe Removal Candidates** ❌

#### **MotionGenerator Methods NOT Called by planTrajectories:**
```cpp
// Methods confirmed as unused by planTrajectories:
- setWaypointsWithExactStart()     // NOT FOUND in planTrajectories  
- performSTOMPWithCheckpoints()    // NOT FOUND in planTrajectories
- setExplorationConstant()         // NOT FOUND in planTrajectories (used elsewhere)
- finaliseTrajectory()             // NOT FOUND in planTrajectories
- evaluateTrajectory()             // NOT FOUND in planTrajectories
- computeTorques()                 // NOT FOUND in planTrajectories
- saveTrajectoryToCSV()            // NOT FOUND in planTrajectories
- armHasCollision()                // NOT FOUND in planTrajectories (may be used internally)
```

#### **Cost Calculators - Need Deep Analysis:**
```cpp
// planTrajectories uses STOMP, which internally uses cost calculators
// Need to trace: performSTOMP() → initializeCostCalculator() → which calculators?
```

#### **STOMP Cost Calculator Usage - CRITICAL** 🔒

Based on `performSTOMP()` → `initializeCostCalculator()` analysis:

```cpp
// Current STOMP implementation uses ONLY:
void MotionGenerator::initializeCostCalculator() {
    _costCalculator = std::make_unique<CompositeCostCalculator>();
    
    // ONLY ONE CALCULATOR USED BY planTrajectories:
    _costCalculator->addCostCalculator(
        std::make_unique<ObstacleCostCalculator>(_arm, _obstacleTree, _sdf, 
                                               _sdfMinPoint, _sdfMaxPoint, _sdfResolution),
        1.0);
}
```

#### **Cost Calculator Preservation Requirements** 🔒

**ABSOLUTELY PRESERVE** (Used by planTrajectories):
- `CompositeCostCalculator` - Container for cost calculators
- `ObstacleCostCalculator` - Collision avoidance (weight: 1.0)

**SAFE TO REMOVE** (Not used by planTrajectories):
- `ConstraintCostCalculator` - Velocity/acceleration limits 
- `EndEffectorMovementCostCalculator` - End-effector motion costs
- `MagnetEndPositionCostCalculator` - Magnet position costs  
- `TaskSpacePathTrackingCostCalculator` - Path tracking costs

**NOTE**: Other calculators exist but are commented out in `initializeCostCalculatorCheckpoints()`

#### **Critical Dependency Summary** 📋

```
planTrajectories()
├── MotionGenerator (6 methods) 🔒
│   ├── setObstacleTree() 
│   ├── setWaypoints()
│   ├── performSTOMP() → initializeCostCalculator() 🔒
│   │   ├── CompositeCostCalculator 🔒
│   │   └── ObstacleCostCalculator 🔒  
│   ├── performHauser() 🔒 (DO NOT TOUCH)
│   ├── getPath()
│   └── generateTrajectoryFromCheckpoints()
├── PathPlanner (6 methods) 🔒  
│   ├── planCheckpoints() → All RRT algorithms 🔒
│   ├── setStartPose()
│   ├── setObstacleTree() 
│   ├── setGoalConfiguration()
│   ├── runPathFinding() → RRT*/InformedRRT*/RRTConnect 🔒
│   └── getAnglesPath()
└── ThreadPool integration (STOMP optimization)
```

### **IMMEDIATE ACTION PLAN** 🎯 **PHASE 1 COMPLETED ✅**

**EXECUTED**: Phase 1 Dead Code Removal - **283 lines removed successfully!**

#### **✅ COMPLETED: Safe Dead Code Removal** 
**Target Methods Removed** (Confirmed NOT used by planTrajectories or other apps):
```cpp
// MotionGenerator - Successfully removed:
✅ setWaypointsWithExactStart()     // 22 lines + declaration removed
✅ Large commented STOMP block      // 252 lines of dead commented code removed  
✅ evaluateTrajectory()             // Declaration removed (never implemented)

// PRESERVATION CONFIRMED ✅:
🔒 armHasCollision() - PRESERVED (used internally by PathPlanner)
🔒 setExplorationConstant() - PRESERVED (used by parameter tuning apps)
🔒 finaliseTrajectory() - PRESERVED (definitions exist, may be used)
🔒 computeTorques() - PRESERVED (used by evaluation apps)
🔒 saveTrajectoryToCSV() - PRESERVED (used by GUI and parameter tuning)
🔒 performSTOMPWithCheckpoints() - PRESERVED (simplified but functional)
```

#### **✅ COMPILATION VERIFIED**
- **MotionGenerator.cpp**: 2,698 → 2,425 lines (**10.1% reduction**)
- **MotionGenerator.h**: 460 → 450 lines 
- **Total cleanup**: **283 lines of dead code removed**
- **Full project compilation**: ✅ SUCCESS
- **USLib planTrajectories**: ✅ CONFIRMED WORKING

#### **Phase 2: Method-by-Method Verification** 🔍
For each MotionGenerator method not in the critical 6:
1. `grep_search` for usage patterns across entire codebase
2. Verify no indirect calls from critical methods
3. Document removal or preservation decision
4. Execute removal with systematic testing

#### **Phase 3: Critical Preservation Validation** 🔒
**Verify these are NEVER modified:**
- All 6 MotionGenerator critical methods
- All 6 PathPlanner critical methods  
- All RRT algorithm implementations
- Complete Hauser integration chain
- CompositeCostCalculator + ObstacleCostCalculator

#### **Phase 4: Documentation & Testing** 📚
- Update API documentation
- Add dependency tests for critical methods
- Performance regression testing

**READY TO PROCEED** with Phase 1 systematic removal! 🚀

---
