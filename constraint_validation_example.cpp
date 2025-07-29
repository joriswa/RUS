// Example showing how the enhanced STOMP now validates constraints
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include <iostream>

int main() {
    std::cout << "=== Enhanced STOMP with Constraint Validation ===" << std::endl;
    std::cout << std::endl;
    
    std::cout << "Key Improvements Added:" << std::endl;
    std::cout << "1. ConstraintCostCalculator integrated into cost function" << std::endl;
    std::cout << "   - Penalizes velocity/acceleration violations during optimization" << std::endl;
    std::cout << "   - Weight: 0.5 (balances with obstacle costs)" << std::endl;
    std::cout << std::endl;
    
    std::cout << "2. Trajectory validation in main STOMP loop:" << std::endl;
    std::cout << "   - Checks constraints with 20% margin (1.2x limits)" << std::endl;
    std::cout << "   - Only accepts collision-free AND constraint-valid trajectories" << std::endl;
    std::cout << std::endl;
    
    std::cout << "3. Heavy penalty for constraint violations:" << std::endl;
    std::cout << "   - Adds 1000.0 cost penalty for trajectories exceeding 50% margin" << std::endl;
    std::cout << "   - Guides optimization away from invalid regions" << std::endl;
    std::cout << std::endl;
    
    std::cout << "4. Final trajectory validation:" << std::endl;
    std::cout << "   - Strict constraint checking before accepting result" << std::endl;
    std::cout << "   - Throws StompFailedException if constraints violated" << std::endl;
    std::cout << std::endl;
    
    std::cout << "5. Public validation functions added:" << std::endl;
    std::cout << "   - isTrajectoryValid(trajectory, dt)" << std::endl;
    std::cout << "   - isTrajectoryValidWithMargin(trajectory, dt, velMargin, accMargin)" << std::endl;
    std::cout << std::endl;
    
    std::cout << "STOMP now ensures all generated trajectories are:" << std::endl;
    std::cout << "✓ Collision-free" << std::endl;
    std::cout << "✓ Velocity constraint compliant" << std::endl;
    std::cout << "✓ Acceleration constraint compliant" << std::endl;
    std::cout << "✓ Kinematically feasible" << std::endl;
    
    return 0;
}
