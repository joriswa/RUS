# TrajectoryLib Phase 2: Cost Calculator Analysis & Removal Plan

## Executive Summary

**OBJECTIVE**: Systematically analyze and remove unused cost calculators from TrajectoryLib while preserving all functionality required by planTrajectories and other applications.

**STATUS**: Phase 1 Complete ✅ (283 lines removed). Phase 2 Ready for Execution.

**SCOPE**: 8 cost calculator classes requiring verification and potential removal.

---

## Phase 1 Completion Summary ✅

- **Dead Code Removed**: 283 lines (10.1% reduction)
- **Methods Removed**: `setWaypointsWithExactStart`, `evaluateTrajectory`, large commented STOMP block
- **Compilation Status**: ✅ Full project builds successfully
- **Functionality Status**: ✅ USLib planTrajectories confirmed working

---

## Current Cost Calculator Inventory

### **CONFIRMED CRITICAL** 🔒 (Used by planTrajectories)
```cpp
1. CompositeCostCalculator     // Container for cost calculators
2. ObstacleCostCalculator      // Collision avoidance (weight: 1.0)
```

### **VERIFICATION REQUIRED** 🔍 (Status Unknown)
```cpp
3. CheckpointTransitionCostCalculator    // Velocity discontinuities at checkpoints
4. ApproachDepartureCostCalculator       // Approach/departure along z-axis  
5. ConstraintCostCalculator              // Velocity/acceleration limits
6. MagnetEndPositionCostCalculator       // Magnet position costs
7. TaskSpacePathTrackingCostCalculator   // Path tracking costs
8. EndEffectorMovementCostCalculator     // End-effector motion costs
```

---

## Phase 2 Systematic Verification Plan

### **Step 1: Implementation Verification** 🔍
For each calculator (3-8), verify:
```bash
# Check if implementation exists
grep -r "CalculatorName::" libs/TrajectoryLib/src/

# Example:
grep -r "CheckpointTransitionCostCalculator::" libs/TrajectoryLib/src/
```

**Expected Results**:
- ✅ **Has Implementation**: Constructor and computeCost method found
- ❌ **No Implementation**: Only header declaration (safe to remove)

### **Step 2: Instantiation Analysis** 🔍
Search for actual usage/instantiation:
```bash
# Search for instantiation patterns
grep -r "std::make_unique<CalculatorName>" **/*.cpp
grep -r "new CalculatorName" **/*.cpp
grep -r "CalculatorName(" **/*.cpp

# Search for addCostCalculator usage
grep -r "addCostCalculator.*CalculatorName" **/*.cpp
```

### **Step 3: Application Usage Check** 🔍
Check if used by other applications:
```bash
# Check parameter tuning applications
grep -r "CalculatorName" apps/ParameterTuning/
grep -r "CalculatorName" apps/Evaluator/
grep -r "CalculatorName" apps/PathPlanner/
```

### **Step 4: Header Dependencies** 🔍
Verify no external dependencies:
```bash
# Check if any external headers reference these calculators
grep -r "CalculatorName" libs/USLib/
grep -r "CalculatorName" apps/
```

---

## Prioritized Removal Strategy

### **PRIORITY 1: Declared-Only Classes** (Safest Removal) ✅
**Criteria**: Header declaration exists but no implementation found
- Zero risk of breaking functionality
- Immediate removal candidates

### **PRIORITY 2: Implemented-Not-Instantiated** ✅ 
**Criteria**: Implementation exists but never instantiated
- Very low risk
- Remove implementation + declaration

### **PRIORITY 3: Instantiated-Not-Used** ⚠️
**Criteria**: Instantiated but not used by any critical functionality
- Medium risk - requires careful verification
- Check all applications before removal

### **PRIORITY 4: Conditional Usage** ❌
**Criteria**: Used by non-critical applications or commented code
- High risk - preserve unless confirmed unused

---

## Detailed Execution Plan

### **Phase 2A: Quick Wins** (Estimated: 30 minutes)

#### **Target**: Priority 1 & 2 removals
1. **Verify implementations** for calculators 3-8
2. **Remove declared-only calculators** immediately
3. **Remove implemented-not-instantiated calculators**
4. **Test compilation** after each removal

#### **Expected Outcome**: 
- Additional 100-200 lines removed
- Zero functional impact
- Faster compilation

### **Phase 2B: Systematic Analysis** (Estimated: 1 hour)

#### **Target**: Priority 3 analysis
1. **Deep usage analysis** for remaining calculators
2. **Cross-reference with applications**
3. **Document usage patterns**
4. **Create removal decision matrix**

#### **Expected Outcome**:
- Clear usage documentation
- Informed removal decisions
- Risk assessment for each calculator

### **Phase 2C: Safe Removal Execution** (Estimated: 30 minutes)

#### **Target**: Execute verified safe removals
1. **Remove unused calculators** (Priority 3 confirmed safe)
2. **Update header organization**
3. **Verify compilation**
4. **Test critical functionality**

#### **Expected Outcome**:
- Significant code reduction (200-400 lines)
- Maintained functionality
- Cleaner architecture

---

## Verification & Testing Protocol

### **After Each Removal**:
```bash
# 1. Verify compilation
cd /Users/joris/Uni/MA/Code/PathPlanner_US_wip && make -j4

# 2. Check USLib specifically
grep -n "ERROR\|FAIL" build.log

# 3. Verify planTrajectories chain works
# (USLib compilation success indicates this)
```

### **Final Validation**:
```bash
# 1. Full project clean build
make clean && make -j4

# 2. Line count verification
wc -l libs/TrajectoryLib/src/Motion/MotionGenerator.*
wc -l libs/TrajectoryLib/include/TrajectoryLib/Motion/MotionGenerator.h

# 3. Critical dependency preservation check
grep -A 10 -B 5 "initializeCostCalculator" libs/TrajectoryLib/src/Motion/MotionGenerator.cpp
```

---

## Risk Management

### **Low Risk Removals** ✅
- Declared but not implemented
- Implemented but never instantiated
- Used only in commented code

### **Medium Risk Removals** ⚠️
- Used by non-critical applications
- Used in parameter tuning only
- Used in evaluation tools only

### **High Risk - DO NOT REMOVE** ❌
- Used by planTrajectories chain
- Used by core STOMP algorithm  
- Used by critical applications

---

## Expected Outcomes

### **Code Quality Improvements**
- **15-25% additional reduction** in MotionGenerator file size
- **Simplified cost calculator architecture**
- **Reduced compilation dependencies**
- **Cleaner public interface**

### **Performance Benefits**
- **Faster compilation** (fewer templates to instantiate)
- **Reduced binary size** (unused classes eliminated)
- **Simpler cost calculation logic**

### **Maintainability Gains**
- **Clearer cost calculator system**
- **Reduced cognitive overhead**
- **Focused functionality**

---

## Success Metrics

### **Quantitative Goals**
- ✅ **200-400 additional lines removed**
- ✅ **Zero compilation errors**
- ✅ **Zero functional regressions**
- ✅ **Maintained planTrajectories functionality**

### **Qualitative Goals**
- ✅ **Simplified cost calculator architecture**
- ✅ **Cleaner header organization**
- ✅ **Reduced cognitive complexity**

---

## Next Phases Preview

### **Phase 3: Architecture Simplification**
- Method signature optimization
- Interface consolidation
- Dependency injection improvements

### **Phase 4: Performance Optimization**
- STOMP algorithm hot path optimization
- Memory allocation improvements
- Threading optimization

---

**READY FOR EXECUTION** 🚀

*This systematic approach ensures safe removal of unused cost calculators while preserving all critical functionality for planTrajectories and other applications.*
