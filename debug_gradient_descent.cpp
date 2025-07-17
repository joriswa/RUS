#include <iostream>
#include <iomanip>
#include "apps/ComparisonIK/core/newton_raphson_ik.h"
#include "TrajectoryLib/Robot/RobotArm.h"
#include "USLib/USTrajectoryPlanner.h"

int main() {
    std::string urdf_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf";
    std::string env_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/obstacles.xml";
    
    try {
        // Initialize robot and planner
        RobotArm robot(urdf_path);
        UltrasoundScanTrajectoryPlanner us_planner(urdf_path);
        us_planner.setEnvironment(env_path);
        
        // Create IK solver
        NewtonRaphsonIK ik_solver(robot);
        ik_solver.setPositionTolerance(1e-4);
        ik_solver.setMaxIterations(10);  // Just a few iterations for debugging
        ik_solver.setStepSize(0.01);
        ik_solver.setFiniteDifferenceEpsilon(1e-6);
        ik_solver.setPoseWeight(0.1);
        ik_solver.setUseEnvironmentCost(true);
        ik_solver.setObstacleTree(us_planner.getObstacleTree());
        
        // Test with a simple target pose
        Eigen::Affine3d target_pose = Eigen::Affine3d::Identity();
        target_pose.translation() = Eigen::Vector3d(0.5, 0.0, 0.5);
        
        // Start from home configuration
        Eigen::Matrix<double, 7, 1> initial_q = Eigen::Matrix<double, 7, 1>::Zero();
        robot.setJointAngles(initial_q);
        
        std::cout << "Initial configuration: " << initial_q.transpose() << std::endl;
        std::cout << "Target pose:" << std::endl;
        std::cout << "  Position: " << target_pose.translation().transpose() << std::endl;
        
        // Test cost function and gradient
        double initial_cost = ik_solver.computeTotalCost(initial_q, target_pose);
        Eigen::VectorXd gradient = ik_solver.computeFiniteDifferenceGradient(initial_q, target_pose);
        
        std::cout << "Initial total cost: " << initial_cost << std::endl;
        std::cout << "Gradient: " << gradient.transpose() << std::endl;
        std::cout << "Gradient norm: " << gradient.norm() << std::endl;
        
        // Test a few gradient descent steps manually
        Eigen::VectorXd q = initial_q;
        for (int i = 0; i < 5; ++i) {
            double cost = ik_solver.computeTotalCost(q, target_pose);
            Eigen::VectorXd grad = ik_solver.computeFiniteDifferenceGradient(q, target_pose);
            
            std::cout << "Step " << i << ": cost = " << cost << ", grad_norm = " << grad.norm() << std::endl;
            
            // Take gradient descent step
            q -= 0.01 * grad;
            
            // Check if any values are NaN or infinite
            if (!q.allFinite()) {
                std::cout << "ERROR: Non-finite joint angles encountered!" << std::endl;
                break;
            }
        }
        
        // Try actual solve
        std::cout << "\nTrying full solve..." << std::endl;
        auto result = ik_solver.solve(target_pose, initial_q);
        std::cout << "Success: " << result.success << std::endl;
        std::cout << "Iterations: " << result.iterations << std::endl;
        std::cout << "Final error: " << result.final_error << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
