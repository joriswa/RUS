#include "libs/USLib/include/USTrajectoryPlanner.h"
#include "TrajectoryLib/Logger.h"
#include <iostream>
#include <vector>
#include <cmath>

int main() {
    // Initialize logging
    Logger::instance().setLevel(LogLevel::DEBUG);
    
    std::cout << "Testing small movement optimization in USTrajectoryPlanner...\n";
    
    try {
        // Create a USTrajectoryPlanner instance
        UltrasoundScanTrajectoryPlanner planner("res/env.urdf");
        
        // Create test waypoints with small movements
        std::vector<Eigen::VectorXd> testWaypoints;
        
        // Starting position (7 joints for Franka)
        Eigen::VectorXd start(7);
        start << 0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0;
        testWaypoints.push_back(start);
        
        // Add several waypoints with small movements (< 0.05 rad = ~2.9°)
        for (int i = 1; i <= 5; ++i) {
            Eigen::VectorXd small_move = start;
            // Add small movement to first joint only
            small_move[0] += i * 0.01; // 0.01 rad increments (~0.57° each)
            testWaypoints.push_back(small_move);
        }
        
        // Add a large movement
        Eigen::VectorXd large_move = start;
        large_move[0] += 0.2; // 0.2 rad (~11.5°) - should be kept
        testWaypoints.push_back(large_move);
        
        // Add more small movements
        for (int i = 1; i <= 3; ++i) {
            Eigen::VectorXd small_move2 = large_move;
            small_move2[1] += i * 0.008; // Even smaller movements
            testWaypoints.push_back(small_move2);
        }
        
        // Final position with large movement
        Eigen::VectorXd end = large_move;
        end[0] += 0.15; // Another large movement
        testWaypoints.push_back(end);
        
        std::cout << "Original waypoints: " << testWaypoints.size() << std::endl;
        
        // Create a copy for testing
        std::vector<Eigen::VectorXd> optimizedWaypoints = testWaypoints;
        
        // Test the optimization
        int removedWaypoints = planner.optimizeSmallMovements(optimizedWaypoints, 0.05);
        
        std::cout << "Optimized waypoints: " << optimizedWaypoints.size() << std::endl;
        std::cout << "Waypoints removed: " << removedWaypoints << std::endl;
        
        // Expected: Should remove 4 waypoints from first sequence (keep start and waypoint 5)
        // and 2 waypoints from second sequence (keep large_move and last small_move)
        // Total expected removal: 4 + 2 = 6 waypoints
        
        if (removedWaypoints > 0) {
            std::cout << "✓ Small movement optimization successfully removed " << removedWaypoints << " waypoints\n";
            
            // Print original vs optimized waypoint counts
            std::cout << "Waypoint reduction: " << testWaypoints.size() << " → " << optimizedWaypoints.size() 
                      << " (" << (100.0 * removedWaypoints / testWaypoints.size()) << "% reduction)\n";
                      
            // Verify that start and end positions are preserved
            bool startPreserved = (optimizedWaypoints.front() - testWaypoints.front()).norm() < 1e-6;
            bool endPreserved = (optimizedWaypoints.back() - testWaypoints.back()).norm() < 1e-6;
            
            if (startPreserved && endPreserved) {
                std::cout << "✓ Start and end positions preserved correctly\n";
            } else {
                std::cout << "✗ Start or end positions not preserved!\n";
                return 1;
            }
            
        } else {
            std::cout << "✗ No waypoints were removed - optimization may not be working\n";
            return 1;
        }
        
        // Test with different threshold
        std::vector<Eigen::VectorXd> strictOptimized = testWaypoints;
        int strictRemoved = planner.optimizeSmallMovements(strictOptimized, 0.005); // Much stricter threshold
        
        std::cout << "\nWith stricter threshold (0.005 rad):\n";
        std::cout << "Waypoints removed: " << strictRemoved << std::endl;
        
        std::cout << "\n✓ Small movement optimization test completed successfully!\n";
        
    } catch (const std::exception& e) {
        std::cout << "✗ Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
