// Simple test to verify PathPlanner retry functionality
#include "TrajectoryLib/Planning/PathPlanner.h"
#include "TrajectoryLib/Robot/RobotArm.h"
#include <iostream>

int main() {
    std::cout << "Testing PathPlanner retry functionality..." << std::endl;
    
    PathPlanner planner;
    
    // Set some basic parameters
    Params params;
    params.maxIterations = 100;  // Small number to potentially force failures
    params.algo = RRT;
    planner.setParams(params);
    
    // Test without explicit retry count (should use default of 2)
    std::cout << "Testing runPathFinding() with default retries..." << std::endl;
    bool result1 = planner.runPathFinding();
    std::cout << "Result: " << (result1 ? "Success" : "Failed") << std::endl;
    
    // Test with explicit retry count
    std::cout << "Testing runPathFinding(3) with 3 retries..." << std::endl;
    bool result2 = planner.runPathFinding(3);
    std::cout << "Result: " << (result2 ? "Success" : "Failed") << std::endl;
    
    std::cout << "PathPlanner retry test completed." << std::endl;
    
    return 0;
}
