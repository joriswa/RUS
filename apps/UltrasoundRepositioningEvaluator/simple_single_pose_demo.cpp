/**
 * @file simple_single_pose_demo.cpp
 * @brief Simple working demonstration of SinglePoseEvaluator
 */

#include "single_pose_evaluator.h"
#include <iostream>

int main() {
    std::cout << "SinglePoseEvaluator Simple Demo" << std::endl;
    std::cout << "===============================" << std::endl;
    
    try {
        // Create configuration with actual file paths
        SinglePoseEvalConfig config;
        
        // Use actual files from the workspace
        config.robot_urdf_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf";
        config.environment_xml_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/obstacles.xml";
        config.output_directory = "./simple_pose_demo_results";
        
        // Set initial joint configuration (home position)
        config.current_joint_angles = Eigen::VectorXd::Zero(7);
        
        // Set target pose - move forward 30cm and up 20cm
        config.target_pose = SinglePoseEvaluator::createPoseFromRPY(
            Eigen::Vector3d(0.3, 0.0, 0.2),  // 30cm forward, 20cm up
            0.0, 0.0, 0.0                     // no rotation
        );
        
        // Configure for quick demo
        config.num_trials = 3;
        config.verbose = true;
        config.apply_pose_offset = false;  // Skip offset for simplicity
        
        // Use STOMP algorithm with reduced parameters for speed
        config.trajectory_algorithm = TrajectoryAlgorithm::STOMP;
        config.stomp_config.max_iterations = 50;
        config.stomp_config.num_noisy_trajectories = 6;
        config.stomp_config.learning_rate = 0.2;
        config.stomp_config.temperature = 8.0;
        
        std::cout << "Creating SinglePoseEvaluator..." << std::endl;
        SinglePoseEvaluator evaluator(config);
        
        std::cout << "Starting evaluation..." << std::endl;
        bool success = evaluator.runEvaluation();
        
        if (success) {
            std::cout << "✅ Demo completed successfully!" << std::endl;
            std::cout << "Results saved to: " << config.output_directory << std::endl;
        } else {
            std::cout << "❌ Demo failed" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cout << "❌ Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
