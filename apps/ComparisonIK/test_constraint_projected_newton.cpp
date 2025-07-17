#include "core/constraint_projected_newton_ik.h"
#include "TrajectoryLib/Robot/franka_ik_He.h"
#include "TrajectoryLib/Planning/PathPlanner.h"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "=== Constraint Projected Newton's Method IK Test ===" << std::endl;
    
    try {
        // URDF path for Franka Panda robot
        std::string urdf_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf";
        
        // Create robot arm (Franka Panda) with URDF
        RobotArm robot(urdf_path);
        
        // Create constraint projected Newton IK solver (without path planner for now)
        ConstraintProjectedNewtonIK cpn_ik(robot, nullptr);
        
        // Configure solver parameters
        cpn_ik.setPositionTolerance(0.0001);        // 0.1mm (Franka repeatability)
        cpn_ik.setOrientationTolerance(0.001);      // ~0.057 degrees
        cpn_ik.setMaxIterations(100);
        cpn_ik.setLineSearchEnabled(true);
        
        // Test target pose (reachable workspace position)
        Eigen::Affine3d target_pose = Eigen::Affine3d::Identity();
        target_pose.translation() << 0.5, 0.2, 0.4; // 50cm forward, 20cm left, 40cm up
        
        // Rotation: slight pitch and yaw
        Eigen::Matrix3d target_rotation;
        target_rotation = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()) *  // 0.1 rad pitch
                         Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ());   // 0.05 rad yaw
        target_pose.linear() = target_rotation;
        
        // Initial guess (slightly away from target)
        Eigen::Matrix<double, 7, 1> initial_guess;
        initial_guess << 0.1, -0.2, 0.15, -1.8, 0.05, 1.6, 0.8;
        
        std::cout << "Target position: [" << target_pose.translation().transpose() << "]" << std::endl;
        std::cout << "Initial guess: [" << initial_guess.transpose() << "]" << std::endl;
        
        // Solve IK
        auto start_time = std::chrono::high_resolution_clock::now();
        ConstraintProjectedNewtonIKResult result = cpn_ik.solve(target_pose, initial_guess);
        auto end_time = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double solve_time_ms = duration.count() / 1000.0;
        
        // Display results
        std::cout << std::endl << "=== RESULTS ===" << std::endl;
        std::cout << "Success: " << (result.success ? "YES" : "NO") << std::endl;
        std::cout << "Solve time: " << std::fixed << std::setprecision(3) << solve_time_ms << " ms" << std::endl;
        std::cout << "Iterations: " << result.iterations << std::endl;
        std::cout << "Projection iterations: " << result.projection_iterations << std::endl;
        
        std::cout << std::endl << "Final joint angles: [";
        for (int i = 0; i < 7; ++i) {
            std::cout << std::fixed << std::setprecision(4) << result.joint_angles(i);
            if (i < 6) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        std::cout << std::endl << "Error metrics:" << std::endl;
        std::cout << "  Position error: " << std::fixed << std::setprecision(6) << result.position_error << " m" << std::endl;
        std::cout << "  Orientation error: " << std::fixed << std::setprecision(6) << result.orientation_error << " rad" << std::endl;
        std::cout << "  Total pose error: " << std::fixed << std::setprecision(6) << result.final_error << std::endl;
        std::cout << "  Final cost: " << std::fixed << std::setprecision(6) << result.final_cost << std::endl;
        
        // Verify solution by forward kinematics
        if (result.success) {
            robot.setJointAngles(result.joint_angles);
            Eigen::Affine3d achieved_pose = robot.getEndeffectorPose();
            
            Eigen::Vector3d position_diff = achieved_pose.translation() - target_pose.translation();
            
            std::cout << std::endl << "Verification:" << std::endl;
            std::cout << "  Achieved position: [" << achieved_pose.translation().transpose() << "]" << std::endl;
            std::cout << "  Position difference: [" << position_diff.transpose() << "]" << std::endl;
            std::cout << "  Position error magnitude: " << position_diff.norm() << " m" << std::endl;
            
            // Check if within Franka repeatability
            bool within_tolerance = result.position_error < 0.0001 && result.orientation_error < 0.001;
            std::cout << "  Within Franka ±0.1mm tolerance: " << (within_tolerance ? "YES" : "NO") << std::endl;
        }
        
        // Display convergence history
        if (!result.error_history.empty()) {
            std::cout << std::endl << "Convergence history (first 10 iterations):" << std::endl;
            for (size_t i = 0; i < std::min(result.error_history.size(), size_t(10)); ++i) {
                std::cout << "  Iter " << i+1 << ": error=" << std::fixed << std::setprecision(6) 
                         << result.error_history[i] << ", cost=" << result.cost_history[i];
                if (i < result.constraint_violations.size()) {
                    std::cout << ", violations=" << result.constraint_violations[i];
                }
                if (i < result.step_sizes.size()) {
                    std::cout << ", step=" << result.step_sizes[i];
                }
                std::cout << std::endl;
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << std::endl << "=== Test completed ===" << std::endl;
    return 0;
}
