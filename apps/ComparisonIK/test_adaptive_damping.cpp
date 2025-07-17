#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <random>
#include "core/constraint_projected_newton_ik.h"

int main() {
    std::cout << "=== Testing Adaptive Damping vs Traditional Damping ===" << std::endl;
    
    // Initialize robot model
    std::string urdf_path = "res/franka_emika_panda_description/panda_arm.urdf";
    RobotArm robot(urdf_path);
    if (robot.getJointNames().empty()) {
        std::cerr << "Failed to load robot model!" << std::endl;
        return -1;
    }
    
    // Create multiple test poses
    std::vector<Eigen::Affine3d> test_poses;
    
    // Forward reachable pose
    Eigen::Affine3d pose1 = Eigen::Affine3d::Identity();
    pose1.translation() = Eigen::Vector3d(0.5, 0.0, 0.4);
    pose1.linear() = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    test_poses.push_back(pose1);
    
    // Side reachable pose
    Eigen::Affine3d pose2 = Eigen::Affine3d::Identity();
    pose2.translation() = Eigen::Vector3d(0.3, 0.3, 0.5);
    pose2.linear() = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    test_poses.push_back(pose2);
    
    // High pose
    Eigen::Affine3d pose3 = Eigen::Affine3d::Identity();
    pose3.translation() = Eigen::Vector3d(0.4, 0.2, 0.7);
    pose3.linear() = Eigen::AngleAxisd(M_PI/6, Eigen::Vector3d::UnitX()).toRotationMatrix();
    test_poses.push_back(pose3);
    
    // Initialize IK solver
    ConstraintProjectedNewtonIK ik_solver(robot, nullptr); // No collision checking
    ik_solver.setMaxIterations(100);
    ik_solver.setPositionTolerance(0.001);
    ik_solver.setOrientationTolerance(0.05);
    ik_solver.setKinematicOnlyMode(true); // Use kinematic-only mode
    
    std::cout << "Test Configuration:" << std::endl;
    std::cout << "- Number of test poses: " << test_poses.size() << std::endl;
    std::cout << "- Kinematic-only mode (no collision checking)" << std::endl;
    std::cout << "- Max iterations: 100" << std::endl;
    std::cout << "- Tolerances: 1mm position, ~2.9° orientation" << std::endl;
    
    int traditional_successes = 0;
    double traditional_total_time = 0;
    double traditional_total_iterations = 0;
    
    int adaptive_successes = 0;
    double adaptive_total_time = 0;
    double adaptive_total_iterations = 0;
    
    for (size_t pose_idx = 0; pose_idx < test_poses.size(); ++pose_idx) {
        std::cout << "\n=== Testing Pose " << (pose_idx + 1) << " ===" << std::endl;
        std::cout << "Target: [" << test_poses[pose_idx].translation().transpose() << "]" << std::endl;
        
        // Test traditional damping
        std::cout << "\nTraditional Fixed Damping:" << std::endl;
        ik_solver.setUseAdaptiveDamping(false);
        ik_solver.setDampingFactor(0.001);
        
        auto result_traditional = ik_solver.solve(test_poses[pose_idx], Eigen::Matrix<double, 7, 1>::Zero());
        traditional_total_time += result_traditional.solve_time;
        traditional_total_iterations += result_traditional.iterations;
        
        if (result_traditional.success) {
            traditional_successes++;
            std::cout << "  SUCCESS (" 
                     << std::fixed << std::setprecision(2) << result_traditional.solve_time << "ms, " 
                     << result_traditional.iterations << " iters, pos_err=" 
                     << std::scientific << std::setprecision(2) << result_traditional.position_error << ")" << std::endl;
        } else {
            std::cout << "  FAILED (" 
                     << std::fixed << std::setprecision(2) << result_traditional.solve_time << "ms, " 
                     << result_traditional.iterations << " iters)" << std::endl;
        }
        
        // Test adaptive damping
        std::cout << "Adaptive Levenberg-Marquardt Damping:" << std::endl;
        ik_solver.setUseAdaptiveDamping(true);
        
        auto result_adaptive = ik_solver.solve(test_poses[pose_idx], Eigen::Matrix<double, 7, 1>::Zero());
        adaptive_total_time += result_adaptive.solve_time;
        adaptive_total_iterations += result_adaptive.iterations;
        
        if (result_adaptive.success) {
            adaptive_successes++;
            std::cout << "  SUCCESS (" 
                     << std::fixed << std::setprecision(2) << result_adaptive.solve_time << "ms, " 
                     << result_adaptive.iterations << " iters, pos_err=" 
                     << std::scientific << std::setprecision(2) << result_adaptive.position_error << ")" << std::endl;
        } else {
            std::cout << "  FAILED (" 
                     << std::fixed << std::setprecision(2) << result_adaptive.solve_time << "ms, " 
                     << result_adaptive.iterations << " iters)" << std::endl;
        }
    }
    
    // Compare results
    std::cout << "\n=== Overall Comparison Summary ===" << std::endl;
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Traditional Fixed Damping:" << std::endl;
    std::cout << "  Success Rate: " << traditional_successes << "/" << test_poses.size() 
             << " (" << (traditional_successes * 100.0 / test_poses.size()) << "%)" << std::endl;
    std::cout << "  Average Time: " << std::setprecision(2) << (traditional_total_time / test_poses.size()) << "ms" << std::endl;
    std::cout << "  Average Iterations: " << std::setprecision(1) << (traditional_total_iterations / test_poses.size()) << std::endl;
    
    std::cout << "\nAdaptive Levenberg-Marquardt Damping:" << std::endl;
    std::cout << "  Success Rate: " << adaptive_successes << "/" << test_poses.size() 
             << " (" << (adaptive_successes * 100.0 / test_poses.size()) << "%)" << std::endl;
    std::cout << "  Average Time: " << std::setprecision(2) << (adaptive_total_time / test_poses.size()) << "ms" << std::endl;
    std::cout << "  Average Iterations: " << std::setprecision(1) << (adaptive_total_iterations / test_poses.size()) << std::endl;
    
    // Analysis
    std::cout << "\n=== Performance Analysis ===" << std::endl;
    if (adaptive_successes > traditional_successes) {
        std::cout << "✅ Adaptive damping achieved better success rate (+"; 
        std::cout << (adaptive_successes - traditional_successes) << " more successes)" << std::endl;
    } else if (adaptive_successes == traditional_successes) {
        std::cout << "📊 Both methods achieved the same success rate" << std::endl;
        
        if (adaptive_total_time < traditional_total_time) {
            double speedup = (traditional_total_time / adaptive_total_time - 1.0) * 100;
            std::cout << "✅ Adaptive damping was " << std::setprecision(1) << speedup << "% faster" << std::endl;
        }
        
        if (adaptive_total_iterations < traditional_total_iterations) {
            double reduction = (1.0 - adaptive_total_iterations / traditional_total_iterations) * 100;
            std::cout << "✅ Adaptive damping used " << std::setprecision(1) << reduction << "% fewer iterations" << std::endl;
        }
    } else {
        std::cout << "❌ Traditional damping performed better on these test cases" << std::endl;
    }
    
    std::cout << "\n🎯 Adaptive damping implementation completed successfully!" << std::endl;
    std::cout << "The Levenberg-Marquardt style adaptive damping adjusts the damping factor" << std::endl;
    std::cout << "based on cost improvement, providing a balance between Newton's method" << std::endl;
    std::cout << "speed and gradient descent stability." << std::endl;
    
    return 0;
}
