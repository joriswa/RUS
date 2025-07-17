# MotionGenerator Refactoring Plan

## Executive Summary

This document outlines a comprehensive refactoring plan for the MotionGenerator class and related trajectory planning components. The goal is to clean up extraneous code while preserving all functionality required by the `planTrajectories` system, RRT variants, and Hauser algorithms.

## Analysis Results

### Core Requirements (Based on USTrajectoryPlanner::planTrajectories Usage)

The `planTrajectories` function in USLib uses the following MotionGenerator methods:

1. **setObstacleTree()** - Configure collision environment
2. **setWaypoints()** - Set trajectory waypoints 
3. **setWaypointsWithExactStart()** - Set waypoints with trajectory chaining
4. **performSTOMP()** - Primary trajectory optimization algorithm
5. **performHauser()** - Hauser algorithm for geometric path planning
6. **getPath()** - Retrieve computed trajectory
7. **generateTrajectoryFromCheckpoints()** - Generate trajectory from joint checkpoints

### RRT Variants to Preserve (As Requested)

Located in PathPlanner class:
- **RRT** - Basic rapidly-exploring random tree
- **RRT*** - Asymptotically optimal variant
- **InformedRRT*** - Ellipsoidal sampling variant  
- **RRTConnect** - Bidirectional tree growth

### Cost Calculators Analysis

**ACTIVELY USED:**
- `ObstacleCostCalculator` - Used in `initializeCostCalculator()` for STOMP
- `CompositeCostCalculator` - Container for multiple cost functions

**POTENTIALLY UNUSED (Need Further Investigation):**
- `ConstraintCostCalculator` - Joint velocity/acceleration constraints
- `EndEffectorMovementCostCalculator` - End-effector movement penalties
- `MagnetEndPositionCostCalculator` - Target position attraction 
- `TaskSpacePathTrackingCostCalculator` - Task space path following
- `CheckpointTransitionCostCalculator` - Velocity continuity at checkpoints
- `ApproachDepartureCostCalculator` - Approach/departure motion costs

## Refactoring Plan

### Phase 1: Remove Dead Code

#### 1.1 Remove Unimplemented Methods
```cpp
// REMOVE: Declared but not implemented
bool performSTOMPWithEarlyTermination(const StompConfig &config,
                                      std::shared_ptr<boost::asio::thread_pool> sharedPool,
                                      std::vector<Eigen::MatrixXd> &intermediateThetas);
```

#### 1.2 Remove Unused Private Methods
- `extracted()` - Empty/placeholder method
- `evaluateTrajectory()` - Not used by planTrajectories
- Multiple overloaded `computeCost()` methods - Keep only active ones
- `computeTorques()` - Biomechanics calculation not used in planning
- `printMaximumVelocity()` - Debug utility

#### 1.3 Clean Up Redundant Trajectory Processing
- Remove duplicate trajectory cost computation methods
- Consolidate time-optimal scaling implementations
- Remove unused quintic polynomial methods

### Phase 2: Simplify Cost Calculator System

#### 2.1 Keep Essential Calculators
- `ObstacleCostCalculator` - Core collision avoidance
- `CompositeCostCalculator` - Multi-objective composition
- `ConstraintCostCalculator` - Joint limit enforcement (if used)

#### 2.2 Remove Specialized Calculators (If Unused)
After verification, remove:
- `EndEffectorMovementCostCalculator`
- `MagnetEndPositionCostCalculator` 
- `TaskSpacePathTrackingCostCalculator`
- `CheckpointTransitionCostCalculator`
- `ApproachDepartureCostCalculator`

### Phase 3: Clean Up Member Variables

#### 3.1 Remove Unused Members
- `_explorationConstant` - Not used in current STOMP implementation
- Redundant smoothing matrices if not needed
- Unused trajectory evaluation metrics

#### 3.2 Consolidate Configuration
- Merge related configuration into StompConfig
- Remove hardcoded constants scattered throughout code

### Phase 4: Streamline Public Interface

#### 4.1 Keep Core Public Methods (Used by planTrajectories)
```cpp
// Constructor
MotionGenerator(RobotArm arm);

// Configuration
void setWaypoints(const Eigen::MatrixXd &waypoints);
void setWaypointsWithExactStart(const Eigen::MatrixXd &waypoints, const Eigen::VectorXd &exactStartPosition);
void setObstacleTree(const std::shared_ptr<BVHTree> &newObstacleTree);

// Planning Algorithms  
bool performSTOMP(const StompConfig &config, std::shared_ptr<boost::asio::thread_pool> sharedPool = nullptr);
void performHauser(unsigned int maxIterations, const std::string &out = "", unsigned int outputFrequency = 100);

// Trajectory Generation
std::vector<TrajectoryPoint> generateTrajectoryFromCheckpoints(const std::vector<Eigen::VectorXd> &checkpoints);
std::vector<TrajectoryPoint> generateTaskSpaceTrajectory(const std::vector<Eigen::Affine3d> &poses, double endEffectorSpeed, const Eigen::VectorXd &initialJoints);

// Results
std::vector<TrajectoryPoint> getPath() const;

// Utilities
void saveTrajectoryToCSV(const std::string &filename);
bool armHasCollision(std::vector<double> jointPositions);
bool armHasCollision(RobotArm &arm);
```

#### 4.2 Remove Unused Public Methods
- `setExplorationConstant()` - Not used
- `finaliseTrajectory()` - Not used by planTrajectories
- `performSTOMPWithCheckpoints()` - Check if used, remove if not
- Various debug and utility methods

### Phase 5: Code Quality Improvements

#### 5.1 Remove Commented Code
- Clean up all `//` commented sections
- Remove obsolete `qDebug()` statements (already replaced with Logger)

#### 5.2 Consolidate Includes
- Remove unused include statements
- Group related includes together
- Use forward declarations where possible

#### 5.3 Improve Documentation
- Update class documentation to reflect actual usage
- Remove documentation for removed methods
- Add clear separation between public interface and internal methods

## Implementation Strategy

### Preservation Priorities
1. **HIGH PRIORITY:** All methods used by `planTrajectories`
2. **HIGH PRIORITY:** RRT variants in PathPlanner (as requested)
3. **HIGH PRIORITY:** Hauser algorithm implementation (as requested)
4. **MEDIUM PRIORITY:** STOMP-related infrastructure
5. **LOW PRIORITY:** Utility methods and debugging tools

### Safety Measures
1. **No Function Signature Changes:** As requested by user
2. **Incremental Removal:** Remove code in small, testable chunks
3. **Compilation Verification:** Ensure code compiles after each removal
4. **Usage Verification:** Check all usage sites before removing methods

### Validation Plan
1. Verify `planTrajectories` still works after each phase
2. Test RRT variants remain functional
3. Confirm Hauser algorithm unchanged
4. Run basic trajectory planning tests

## Expected Outcomes

### Code Reduction
- **Estimated 20-30% reduction** in MotionGenerator.cpp file size
- **Removal of 5-10 unused methods** from public interface
- **Elimination of 3-5 cost calculator classes** if unused

### Improved Maintainability
- Clearer separation between core planning algorithms and utilities
- Reduced cognitive load for developers
- Better focused class responsibilities

### Performance Benefits
- Reduced compilation time
- Smaller binary size
- Less memory usage from eliminated dead code

## Risk Assessment

### Low Risk
- Removing clearly unused methods (e.g., `performSTOMPWithEarlyTermination`)
- Cleaning up commented code and debug statements
- Removing unused member variables

### Medium Risk
- Removing cost calculators (need usage verification)
- Consolidating redundant trajectory processing methods
- Removing utility methods that might be used elsewhere

### High Risk (Avoid)
- Changing function signatures (explicitly forbidden)
- Modifying RRT or Hauser implementations
- Removing core STOMP infrastructure

## Next Steps

1. **Phase 1 Implementation:** Start with clearly dead code removal
2. **Usage Analysis:** Comprehensive search for cost calculator usage
3. **Incremental Testing:** Verify planTrajectories after each change
4. **Documentation Update:** Update this plan based on findings
5. **Final Validation:** Complete system test of trajectory planning

## Progress Tracking

### Completed Removals ✅

#### Phase 1.1: Unimplemented Methods
- ✅ **Removed `performSTOMPWithEarlyTermination`** declaration from MotionGenerator.h (lines 344-346)
- ✅ **Removed `extracted()` method** declaration from MotionGenerator.h (line 278)  
- ✅ **Removed `printMaximumVelocity()` method** declaration from MotionGenerator.h (line 349)

#### Phase 5.1: Commented Code Cleanup
- ✅ **Removed commented `printMaximumVelocity()` call** from MotionGenerator.cpp (line 815)
- ✅ **Identified large commented block** in `performSTOMPWithCheckpoints()` function (~250 lines of old STOMP implementation)

### Current Status
- **Compilation Status**: ✅ ALL BUILDS SUCCESSFUL
- **Dead Code Identified**: ~250 lines of commented STOMP implementation in `performSTOMPWithCheckpoints()`
- **Next Priority**: Remove large commented block safely using piece-by-piece approach

### Validation Results
- ✅ Project compiles without errors after all removals
- ✅ Only warning: unused private field in Logger.h (non-critical)
- ✅ No breaking changes to public interface
- ✅ Preserved all function signatures as required

### Files Modified
1. `/libs/TrajectoryLib/include/TrajectoryLib/Motion/MotionGenerator.h`
   - Removed 3 unused method declarations
   - Fixed Logger.h include path
2. `/libs/TrajectoryLib/src/Motion/MotionGenerator.cpp`  
   - Removed 1 commented method call
3. `/libs/TrajectoryLib/include/TrajectoryLib/Logger.h`
   - Copied from libs directory for proper include resolution

---
*This refactoring plan prioritizes safety and maintainability while achieving significant code cleanup to improve the overall project structure.*
