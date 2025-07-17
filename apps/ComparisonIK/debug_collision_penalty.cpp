#include "core/constraint_projected_newton_ik.h"
#include "core/ik_cost_functions.h"
#include <TrajectoryLib/Robot/RobotArm.h>
#include <TrajectoryLib/Planning/PathPlanner.h>
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "=== Testing Gradient-Friendly Collision Penalty Debug ===" << std::endl;
    
    // Load robot configuration
    RobotArm robot;
    if (!robot.loadFromURDFFile("/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/franka_description/robots/franka_panda.urdf")) {
        std::cerr << "Failed to load robot URDF" << std::endl;
        return -1;
    }
    
    // Setup path planner with obstacles
    PathPlanner path_planner;
    bool loaded = path_planner.loadEnvironment("/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/us_probe_experiment/environment.xml");
    if (!loaded) {
        std::cerr << "Failed to load environment" << std::endl;
        return -1;
    }
    
    // Test different robot configurations
    std::vector<Eigen::Matrix<double, 7, 1>> test_configs = {
        (Eigen::Matrix<double, 7, 1>() << 0, 0, 0, -1.5, 0, 1.5, 0).finished(),      // Safe config
        (Eigen::Matrix<double, 7, 1>() << 0.5, 0.5, 0.5, -1.0, 0.5, 1.0, 0.5).finished(), // Moderate config
        (Eigen::Matrix<double, 7, 1>() << 1.0, 1.0, 1.0, -0.5, 1.0, 0.5, 1.0).finished()  // Potentially colliding
    };
    
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\nTesting collision penalty at different configurations:\n";
    std::cout << "Config | Has Collision | Min Clearance | Old Penalty | New Penalty\n";
    std::cout << "-------|---------------|---------------|-------------|------------\n";
    
    for (size_t i = 0; i < test_configs.size(); i++) {
        robot.setJointAngles(test_configs[i]);
        
        bool has_collision = path_planner.armHasCollision(robot);
        auto clearance_metrics = path_planner.computeArmClearance(robot);
        
        double old_penalty = IKCostFunctions::computeClearancePenalty(robot, &path_planner);
        double new_penalty = IKCostFunctions::computeGradientFriendlyCollisionPenalty(robot, &path_planner);
        
        std::cout << "   " << (i+1) << "   |      " << (has_collision ? "YES" : "NO") 
                  << "      |    " << std::setw(8) << clearance_metrics.min_clearance
                  << "   |   " << std::setw(8) << old_penalty
                  << "  |   " << std::setw(8) << new_penalty << "\n";
    }
    
    // Test comprehensive cost functions
    std::cout << "\nTesting comprehensive cost functions:\n";
    std::cout << "Config | Old Comprehensive | New Comprehensive\n";
    std::cout << "-------|--------------------|-----------------\n";
    
    Eigen::Affine3d dummy_target = Eigen::Affine3d::Identity();
    dummy_target.translation() << 0.5, 0.0, 0.8;
    
    for (size_t i = 0; i < test_configs.size(); i++) {
        robot.setJointAngles(test_configs[i]);
        
        double old_cost = IKCostFunctions::computeComprehensiveCost(robot, dummy_target, &path_planner);
        double new_cost = IKCostFunctions::computeGradientFriendlyComprehensiveCost(robot, dummy_target, &path_planner);
        
        std::cout << "   " << (i+1) << "   |      " << std::setw(12) << old_cost
                  << "      |     " << std::setw(12) << new_cost << "\n";
    }
    
    return 0;
}
