// Constraint Checking Refactoring Summary
// =====================================

/*
CHANGES MADE:

1. REMOVED HEAVY PENALTY FROM evaluateTrajectories():
   - Removed the 1000.0 penalty that was added after line 382
   - Removed excessive margin checking (1.5x multipliers)
   - Now relies on proper cost calculator implementation

2. UPDATED initializeCostCalculator() TO USE CONFIG WEIGHTS:
   - Changed signature: initializeCostCalculator(const StompConfig &config)
   - Now uses config.obstacleCostWeight instead of hardcoded 1.0
   - Now uses config.constraintCostWeight instead of hardcoded 0.5
   - Updated header file with proper documentation

3. IMPROVED ConstraintCostCalculator:
   - Changed from linear penalty (0.1 * violation) to quadratic penalty
   - Now uses: violation² / limit² for better scaling
   - Provides natural cost growth for constraint violations
   - No more artificial penalty multipliers needed

BENEFITS:
✓ Cleaner separation of concerns - constraint cost is in the cost calculator
✓ Configurable weights via StompConfig parameters  
✓ More natural cost scaling with quadratic penalties
✓ Removed excessive margins that were too restrictive
✓ STOMP can focus on initial trajectory time instead of margins
✓ Cost calculator handles constraint violations appropriately

USAGE:
- Set config.constraintCostWeight to desired value (e.g., 1.0)
- Set config.obstacleCostWeight to desired value (e.g., 1.0) 
- The cost calculator will automatically penalize constraint violations
- No manual penalty thresholds or margins needed
*/

#include "TrajectoryLib/Motion/MotionGenerator.h"

void example_usage() {
    StompConfig config;
    config.constraintCostWeight = 1.0;  // Weight for constraint violations
    config.obstacleCostWeight = 1.0;    // Weight for obstacle costs
    
    // MotionGenerator will use these weights automatically
    // in initializeCostCalculator(config)
}
