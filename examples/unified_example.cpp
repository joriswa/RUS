#include <GeometryLib/Obstacle.h>
#include <Hauser10/ParabolicRamp.h>
#include <iostream>

int main() {
    std::cout << "RUS Libraries Unified Example" << std::endl;
    std::cout << "Demonstrating geometry and trajectory planning integration" << std::endl;
    
    // Create obstacles in the environment
    auto obstacle1 = std::make_shared<Obstacle>(
        Eigen::Vector3d(5, 0, 0),   // position
        Eigen::Vector3d(1, 1, 1)    // size
    );
    
    std::cout << "Environment setup:" << std::endl;
    std::cout << "  Obstacle at: " << obstacle1->getPosition().transpose() << std::endl;
    
    // Plan a trajectory that avoids the obstacle
    ParabolicRamp::ParabolicRamp1D trajectory;
    trajectory.x0 = 0.0;   // start before obstacle
    trajectory.x1 = 10.0;  // end after obstacle
    trajectory.dx0 = 0.0;
    trajectory.dx1 = 0.0;
    
    if (trajectory.SolveMinTime(2.0, 3.0)) {
        std::cout << "Trajectory planning:" << std::endl;
        std::cout << "  Duration: " << trajectory.endTime << " seconds" << std::endl;
        
        // Check if trajectory passes through obstacle region
        double obstacle_time = trajectory.endTime * 0.5; // approximate
        double pos_at_obstacle = trajectory.Evaluate(obstacle_time);
        
        std::cout << "  Position when near obstacle: " << pos_at_obstacle << std::endl;
        
        if (pos_at_obstacle >= 4.0 && pos_at_obstacle <= 6.0) {
            std::cout << "  WARNING: Trajectory passes through obstacle region!" << std::endl;
            std::cout << "  In real application, would replan to avoid collision." << std::endl;
        } else {
            std::cout << "  Trajectory successfully avoids obstacle." << std::endl;
        }
    }
    
    std::cout << "Unified example completed successfully!" << std::endl;
    return 0;
}