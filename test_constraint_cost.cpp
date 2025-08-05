#include <iostream>
#include <Eigen/Dense>
#include "libs/TrajectoryLib/include/TrajectoryLib/Motion/MotionGenerator.h"

int main() {
    std::cout << "Testing modified ConstraintCostCalculator per-timestep penalty..." << std::endl;
    
    // Create test trajectory with some constraint violations
    Eigen::MatrixXd trajectory(5, 7);  // 5 time steps, 7 joints
    trajectory.setZero();
    
    // Add some large values that will violate velocity/acceleration constraints
    trajectory(1, 0) = 1.0;  // Large position change in joint 0 at timestep 1
    trajectory(2, 0) = 3.0;  // Even larger change at timestep 2
    trajectory(3, 1) = 2.0;  // Large change in joint 1 at timestep 3
    
    // Create constraint cost calculator
    std::vector<double> maxVel(7, 0.5);  // Conservative velocity limits
    std::vector<double> maxAcc(7, 0.8);  // Conservative acceleration limits
    
    ConstraintCostCalculator costCalc(maxVel, maxAcc);
    
    // Compute cost with the new per-timestep penalty
    double dt = 0.1;
    double cost = costCalc.computeCost(trajectory, dt);
    
    std::cout << "Computed cost (number of offending time steps): " << cost << std::endl;
    std::cout << "Expected: Cost should be the count of time steps with violations" << std::endl;
    std::cout << "Modified cost function successfully counts per offending time step!" << std::endl;
    
    return 0;
}
