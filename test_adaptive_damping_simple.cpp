#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip>
#include "apps/ComparisonIK/core/constraint_projected_newton_ik.h"
#include "libs/TrajectoryLib/include/TrajectoryLib/Robot/RobotArm.h"

int main() {
    std::cout << "=== Testing Adaptive Damping vs Traditional Damping ===" << std::endl;
    
    // Initialize robot model
    RobotArm robot;
    if (!robot.loadFromFile("res/franka_emika_panda_description/panda_arm.urdf", "panda_link8")) {
        std::cerr << "Failed to load robot model!" << std::endl;
        return -1;
    }
    
    // Create a simple test pose
    Eigen::Affine3d test_pose = Eigen::Affine3d::Identity();
    test_pose.translation() = Eigen::Vector3d(0.5, 0.2, 0.4);
    test_pose.rotation() = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    
    // Initialize IK solver
    ConstraintProjectedNewtonIK ik_solver(robot, nullptr); // No collision checking for simplicity
    ik_solver.setMaxIterations(100);
    ik_solver.setPositionTolerance(0.001);
    ik_solver.setOrientationTolerance(0.05);
    ik_solver.setKinematicOnlyMode(true); // Use kinematic-only mode for fair comparison
    
    std::cout << "Test Configuration:" << std::endl;
    std::cout << "- Target position: [" << test_pose.translation().transpose() << "]" << std::endl;
    std::cout << "- Kinematic-only mode (no collision checking)" << std::endl;
    std::cout << "- Max iterations: 100" << std::endl;
    std::cout << "- Tolerances: 1mm position, ~2.9° orientation" << std::endl;
    
    // Test traditional damping
    std::cout << "\n=== Traditional Fixed Damping ===" << std::endl;
    ik_solver.setUseAdaptiveDamping(false);
    ik_solver.setDampingFactor(0.001);
    
    int traditional_successes = 0;
    double traditional_total_time = 0;
    double traditional_total_iterations = 0;
    
    for (int trial = 0; trial < 10; ++trial) {
        auto result = ik_solver.solve(test_pose);
        traditional_total_time += result.solve_time;
        traditional_total_iterations += result.iterations;
        
        if (result.success) {
            traditional_successes++;
            std::cout << "Trial " << (trial+1) << ": SUCCESS (" 
                     << std::fixed << std::setprecision(2) << result.solve_time << "ms, " 
                     << result.iterations << " iters, pos_err=" 
                     << std::scientific << std::setprecision(2) << result.position_error << ")" << std::endl;
        } else {
            std::cout << "Trial " << (trial+1) << ": FAILED (" 
                     << std::fixed << std::setprecision(2) << result.solve_time << "ms, " 
                     << result.iterations << " iters)" << std::endl;
        }
    }
    
    // Test adaptive damping
    std::cout << "\n=== Adaptive Levenberg-Marquardt Damping ===" << std::endl;
    ik_solver.setUseAdaptiveDamping(true);
    
    int adaptive_successes = 0;
    double adaptive_total_time = 0;
    double adaptive_total_iterations = 0;
    
    for (int trial = 0; trial < 10; ++trial) {
        auto result = ik_solver.solve(test_pose);
        adaptive_total_time += result.solve_time;
        adaptive_total_iterations += result.iterations;
        
        if (result.success) {
            adaptive_successes++;
            std::cout << "Trial " << (trial+1) << ": SUCCESS (" 
                     << std::fixed << std::setprecision(2) << result.solve_time << "ms, " 
                     << result.iterations << " iters, pos_err=" 
                     << std::scientific << std::setprecision(2) << result.position_error << ")" << std::endl;
        } else {
            std::cout << "Trial " << (trial+1) << ": FAILED (" 
                     << std::fixed << std::setprecision(2) << result.solve_time << "ms, " 
                     << result.iterations << " iters)" << std::endl;
        }
    }
    
    // Compare results
    std::cout << "\n=== Comparison Summary ===" << std::endl;
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Traditional Damping:" << std::endl;
    std::cout << "  Success Rate: " << traditional_successes << "/10 (" << (traditional_successes * 10) << "%)" << std::endl;
    std::cout << "  Average Time: " << std::setprecision(2) << (traditional_total_time / 10.0) << "ms" << std::endl;
    std::cout << "  Average Iterations: " << std::setprecision(1) << (traditional_total_iterations / 10.0) << std::endl;
    
    std::cout << "Adaptive Damping:" << std::endl;
    std::cout << "  Success Rate: " << adaptive_successes << "/10 (" << (adaptive_successes * 10) << "%)" << std::endl;
    std::cout << "  Average Time: " << std::setprecision(2) << (adaptive_total_time / 10.0) << "ms" << std::endl;
    std::cout << "  Average Iterations: " << std::setprecision(1) << (adaptive_total_iterations / 10.0) << std::endl;
    
    if (adaptive_successes > traditional_successes) {
        std::cout << "\n✅ Adaptive damping achieved better success rate!" << std::endl;
    } else if (adaptive_successes == traditional_successes && adaptive_total_time < traditional_total_time) {
        std::cout << "\n✅ Adaptive damping achieved faster convergence!" << std::endl;
    } else if (adaptive_successes == traditional_successes && adaptive_total_iterations < traditional_total_iterations) {
        std::cout << "\n✅ Adaptive damping achieved fewer iterations!" << std::endl;
    } else {
        std::cout << "\n📊 Both methods performed similarly on this simple test case." << std::endl;
    }
    
    return 0;
}
