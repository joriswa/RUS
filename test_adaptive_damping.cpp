#include <iostream>
#include <vector>
#include <chrono>
#include "../apps/ComparisonIK/core/constraint_projected_newton_ik.h"
#include "../libs/uslib/include/uslib/RobotModel.h"
#include "../apps/PathPlanner/core/PathPlanner.h"
#include "../apps/ComparisonIK/utils/IkPoseReader.h"

int main() {
    std::cout << "=== Testing Constraint Projected Newton IK with Adaptive Damping ===" << std::endl;
    
    // Initialize robot model
    uslib::RobotModel robot;
    if (!robot.loadFromFile("res/franka_emika_panda_description/panda_arm.urdf", "panda_link8")) {
        std::cerr << "Failed to load robot model!" << std::endl;
        return -1;
    }
    
    // Load collision environment  
    auto path_planner = std::make_shared<PathPlanner>();
    if (!path_planner->loadObstacleEnvironment("res/obstacle_environments/panda_scene_ultrasound.scene")) {
        std::cerr << "Failed to load collision environment!" << std::endl;
        return -1;
    }
    
    // Initialize IK solver
    ConstraintProjectedNewtonIK ik_solver(robot, path_planner);
    ik_solver.setMaxIterations(100);
    ik_solver.setPositionTolerance(0.005); // 5mm
    ik_solver.setOrientationTolerance(0.1); // 5.7 degrees
    
    // Load test poses
    IkPoseReader pose_reader;
    auto poses = pose_reader.readFromCSV("two_method_comparison_results.csv");
    
    if (poses.empty()) {
        std::cerr << "Failed to load test poses!" << std::endl;
        return -1;
    }
    
    std::cout << "Loaded " << poses.size() << " test poses" << std::endl;
    
    // Test with traditional damping vs adaptive damping
    std::cout << "\n=== Comparison: Traditional vs Adaptive Damping ===" << std::endl;
    
    int traditional_successes = 0;
    int adaptive_successes = 0;
    double traditional_time = 0.0;
    double adaptive_time = 0.0;
    
    for (size_t i = 0; i < std::min(poses.size(), size_t(10)); ++i) {
        std::cout << "\nPose " << (i+1) << ":" << std::endl;
        
        // Test with traditional damping
        ik_solver.setUseAdaptiveDamping(false);
        ik_solver.setDampingFactor(0.001);
        
        auto result_traditional = ik_solver.solve(poses[i]);
        traditional_time += result_traditional.solve_time;
        
        if (result_traditional.success) {
            traditional_successes++;
            std::cout << "  Traditional: SUCCESS (" << result_traditional.solve_time << "ms, " 
                     << result_traditional.iterations << " iters)" << std::endl;
        } else {
            std::cout << "  Traditional: FAILED (" << result_traditional.solve_time << "ms, " 
                     << result_traditional.iterations << " iters)" << std::endl;
        }
        
        // Test with adaptive damping
        ik_solver.setUseAdaptiveDamping(true);
        
        auto result_adaptive = ik_solver.solve(poses[i]);
        adaptive_time += result_adaptive.solve_time;
        
        if (result_adaptive.success) {
            adaptive_successes++;
            std::cout << "  Adaptive:    SUCCESS (" << result_adaptive.solve_time << "ms, " 
                     << result_adaptive.iterations << " iters)" << std::endl;
        } else {
            std::cout << "  Adaptive:    FAILED (" << result_adaptive.solve_time << "ms, " 
                     << result_adaptive.iterations << " iters)" << std::endl;
        }
    }
    
    // Summary
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Traditional Damping:" << std::endl;
    std::cout << "  Success Rate: " << traditional_successes << "/10 (" << (traditional_successes * 10) << "%)" << std::endl;
    std::cout << "  Average Time: " << (traditional_time / 10.0) << "ms" << std::endl;
    
    std::cout << "Adaptive Damping:" << std::endl;
    std::cout << "  Success Rate: " << adaptive_successes << "/10 (" << (adaptive_successes * 10) << "%)" << std::endl;
    std::cout << "  Average Time: " << (adaptive_time / 10.0) << "ms" << std::endl;
    
    return 0;
}
