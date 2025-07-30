#include <GeometryLib/BVHTree.h>
#include <Hauser10/ParabolicRamp.h>
#include <iostream>

int main() {
    std::cout << "RUS Libraries Unified Example" << std::endl;
    std::cout << "Demonstrating geometry and trajectory planning integration" << std::endl;
    
    // Create a BVH tree for collision detection
    BVHTree tree;
    std::cout << "Environment setup: BVH tree initialized for collision detection" << std::endl;
    
    // Create a simple bounding box representing an obstacle region
    Eigen::AlignedBox3d obstacle_region(Eigen::Vector3d(4, -1, -1), Eigen::Vector3d(6, 1, 1));
    
    std::cout << "Obstacle region: " << obstacle_region.min().transpose() 
              << " to " << obstacle_region.max().transpose() << std::endl;
    
    // Plan a trajectory that needs to navigate around the obstacle
    ParabolicRamp::ParabolicRamp1D trajectory;
    trajectory.x0 = 0.0;   // start before obstacle
    trajectory.x1 = 10.0;  // end after obstacle
    trajectory.dx0 = 0.0;
    trajectory.dx1 = 0.0;
    
    if (trajectory.SolveMinTime(2.0, 3.0)) {
        std::cout << "Trajectory planning:" << std::endl;
        std::cout << "  Duration: " << trajectory.EndTime() << " seconds" << std::endl;
        
        // Check if trajectory passes through obstacle region
        double obstacle_time = trajectory.EndTime() * 0.5; // approximate
        double pos_at_obstacle = trajectory.Evaluate(obstacle_time);
        
        std::cout << "  Position when near obstacle: " << pos_at_obstacle << std::endl;
        
        if (pos_at_obstacle >= obstacle_region.min().x() && pos_at_obstacle <= obstacle_region.max().x()) {
            std::cout << "  WARNING: Trajectory passes through obstacle region!" << std::endl;
            std::cout << "  In real application, would replan using BVH tree collision checking." << std::endl;
        } else {
            std::cout << "  Trajectory successfully avoids obstacle." << std::endl;
        }
    }
    
    std::cout << "Unified example completed successfully!" << std::endl;
    std::cout << "This demonstrates integration of:" << std::endl;
    std::cout << "  - GeometryLib: Spatial data structures for collision detection" << std::endl;
    std::cout << "  - Hauser10: Optimal trajectory planning algorithms" << std::endl;
    
    return 0;
}