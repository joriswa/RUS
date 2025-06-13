/**
 * @file single_pose_examples.cpp
 * @brief Example configurations for the SinglePoseEvaluator
 * 
 * This file demonstrates how to set up different algorithm configurations
 * for evaluating trajectory planning from current joint angles to a specific
 * target pose using various planning algorithms.
 */

#include "single_pose_evaluator.h"
#include <iostream>

/**
 * @brief Create a basic STOMP configuration for single pose evaluation
 */
SinglePoseEvalConfig createBasicStompConfig() {
    SinglePoseEvalConfig config;
    
    // Basic setup
    config.robot_urdf_path = "/path/to/robot.urdf";
    config.environment_xml_path = "/path/to/environment.xml";
    config.output_directory = "./evaluation_results/single_pose_stomp";
    
    // Current joint configuration (7-DOF robot in some initial pose)
    config.current_joint_angles = Eigen::VectorXd(7);
    config.current_joint_angles << 0, -M_PI/4, 0, -3*M_PI/4, 0, M_PI/2, M_PI/4;
    
    // Target pose - 50cm forward, 30cm up, facing forward
    config.target_pose = SinglePoseEvaluator::createPoseFromRPY(
        Eigen::Vector3d(0.5, 0.0, 0.3),  // position
        0.0, 0.0, 0.0                     // roll, pitch, yaw
    );
    
    // Evaluation parameters
    config.num_trials = 15;
    config.verbose = true;
    config.apply_pose_offset = true;  // Apply 2cm offset like other evaluators
    
    // Algorithm selection
    config.trajectory_algorithm = TrajectoryAlgorithm::STOMP;
    
    // STOMP-specific configuration
    config.stomp_config.max_iterations = 150;
    config.stomp_config.num_noisy_trajectories = 12;
    config.stomp_config.learning_rate = 0.12;
    config.stomp_config.temperature = 10.0;
    
    return config;
}

/**
 * @brief Create a Hauser+RRT* configuration for high-quality planning
 */
SinglePoseEvalConfig createHauserRRTStarConfig() {
    SinglePoseEvalConfig config;
    
    // Basic setup
    config.robot_urdf_path = "/path/to/robot.urdf";
    config.environment_xml_path = "/path/to/environment.xml";
    config.output_directory = "./evaluation_results/single_pose_hauser_rrt_star";
    
    // Current joint configuration
    config.current_joint_angles = Eigen::VectorXd(7);
    config.current_joint_angles << M_PI/6, -M_PI/3, M_PI/4, -2*M_PI/3, 0, M_PI/3, 0;
    
    // Target pose - complex orientation change
    Eigen::Quaterniond target_orientation(0.707, 0.0, 0.707, 0.0);  // 90° rotation around Y
    config.target_pose = SinglePoseEvaluator::createPose(
        Eigen::Vector3d(0.4, 0.2, 0.4),   // position
        target_orientation                 // orientation
    );
    
    // Evaluation parameters
    config.num_trials = 10;
    config.verbose = true;
    config.apply_pose_offset = true;
    
    // Algorithm selection
    config.trajectory_algorithm = TrajectoryAlgorithm::HAUSER;
    
    // Path planning configuration (RRT*)
    config.path_planning_config.algorithm = PathPlanningAlgorithm::RRT_STAR;
    config.path_planning_config.max_iterations = 8000;
    config.path_planning_config.step_size = 0.08;
    config.path_planning_config.goal_bias = 0.15;
    
    // Hauser algorithm configuration
    config.hauser_config.max_iterations = 600;
    config.hauser_config.output_file = "./evaluation_results/hauser_timing.csv";
    
    return config;
}

/**
 * @brief Create a configuration for STOMP with early termination
 */
SinglePoseEvalConfig createStompEarlyTerminationConfig() {
    SinglePoseEvalConfig config = createBasicStompConfig();
    
    // Modify for early termination
    config.trajectory_algorithm = TrajectoryAlgorithm::STOMP_WITH_EARLY_TERMINATION;
    config.output_directory = "./evaluation_results/single_pose_stomp_early_term";
    
    // Allow more iterations since early termination will stop when converged
    config.stomp_config.max_iterations = 400;
    config.stomp_config.learning_rate = 0.1;  // More conservative
    
    return config;
}

/**
 * @brief Create a speed-optimized configuration
 */
SinglePoseEvalConfig createFastPlanningConfig() {
    SinglePoseEvalConfig config;
    
    // Basic setup
    config.robot_urdf_path = "/path/to/robot.urdf";
    config.environment_xml_path = "/path/to/environment.xml";
    config.output_directory = "./evaluation_results/single_pose_fast";
    
    // Simple initial pose
    config.current_joint_angles = Eigen::VectorXd::Zero(7);
    
    // Close target pose for fast planning
    config.target_pose = SinglePoseEvaluator::createPoseFromRPY(
        Eigen::Vector3d(0.3, 0.0, 0.2),  // close position
        0.0, 0.0, 0.0                     // no rotation
    );
    
    // Fast evaluation parameters
    config.num_trials = 5;
    config.verbose = false;
    config.apply_pose_offset = false;  // Skip offset for speed
    
    // Use basic STOMP with reduced parameters
    config.trajectory_algorithm = TrajectoryAlgorithm::STOMP;
    config.stomp_config.max_iterations = 30;
    config.stomp_config.num_noisy_trajectories = 6;
    config.stomp_config.learning_rate = 0.25;  // Aggressive learning
    
    return config;
}

/**
 * @brief Create a configuration for precise manipulation tasks
 */
SinglePoseEvalConfig createPrecisionConfig() {
    SinglePoseEvalConfig config;
    
    // Basic setup
    config.robot_urdf_path = "/path/to/robot.urdf";
    config.environment_xml_path = "/path/to/environment.xml";
    config.output_directory = "./evaluation_results/single_pose_precision";
    
    // Current joint configuration
    config.current_joint_angles = Eigen::VectorXd(7);
    config.current_joint_angles << 0, -M_PI/6, 0, -M_PI/2, 0, M_PI/3, M_PI/4;
    
    // Precise target pose with specific orientation
    config.target_pose = SinglePoseEvaluator::createPoseFromRPY(
        Eigen::Vector3d(0.45, 0.15, 0.25),  // precise position
        M_PI/12, -M_PI/24, M_PI/8          // small orientation changes
    );
    
    // High-quality evaluation parameters
    config.num_trials = 25;
    config.verbose = true;
    config.apply_pose_offset = true;
    
    // Use Hauser with Informed RRT* for best quality
    config.trajectory_algorithm = TrajectoryAlgorithm::HAUSER;
    
    // High-precision path planning
    config.path_planning_config.algorithm = PathPlanningAlgorithm::INFORMED_RRT_STAR;
    config.path_planning_config.max_iterations = 12000;
    config.path_planning_config.step_size = 0.05;  // Very small steps
    config.path_planning_config.goal_bias = 0.1;
    config.path_planning_config.custom_cost = true;
    
    // Precise motion generation
    config.hauser_config.max_iterations = 1000;
    
    return config;
}

/**
 * @brief Example of running different algorithm comparisons
 */
void runAlgorithmComparison() {
    std::cout << "=== Single Pose Algorithm Comparison ===" << std::endl;
    
    // Common setup for fair comparison
    Eigen::VectorXd common_start(7);
    common_start << 0, -M_PI/4, 0, -3*M_PI/4, 0, M_PI/2, M_PI/4;
    
    Eigen::Affine3d common_target = SinglePoseEvaluator::createPoseFromRPY(
        Eigen::Vector3d(0.5, 0.2, 0.35),
        0.0, M_PI/12, 0.0
    );
    
    // Create configurations for different algorithms
    std::vector<SinglePoseEvalConfig> configs = {
        createBasicStompConfig(),
        createStompEarlyTerminationConfig(),
        createHauserRRTStarConfig()
    };
    
    std::vector<std::string> config_names = {
        "STOMP_Basic",
        "STOMP_EarlyTerm",
        "Hauser_RRTStar"
    };
    
    // Set common target and start for fair comparison
    for (auto& config : configs) {
        config.current_joint_angles = common_start;
        config.target_pose = common_target;
        config.num_trials = 10;  // Consistent trial count
    }
    
    // Run evaluations
    for (size_t i = 0; i < configs.size(); ++i) {
        std::cout << "\n--- Evaluating " << config_names[i] << " ---" << std::endl;
        
        try {
            SinglePoseEvaluator evaluator(configs[i]);
            bool success = evaluator.runEvaluation();
            
            if (success) {
                std::cout << "✓ " << config_names[i] << " completed successfully" << std::endl;
            } else {
                std::cout << "✗ " << config_names[i] << " failed" << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cout << "✗ " << config_names[i] << " error: " << e.what() << std::endl;
        }
    }
}

/**
 * @brief Example of dynamic pose evaluation
 */
void runDynamicPoseEvaluation() {
    std::cout << "\n=== Dynamic Pose Evaluation ===" << std::endl;
    
    // Create base configuration
    SinglePoseEvalConfig config = createBasicStompConfig();
    config.num_trials = 5;  // Fewer trials for multiple poses
    
    try {
        SinglePoseEvaluator evaluator(config);
        
        // Define multiple target poses to evaluate
        std::vector<Eigen::Affine3d> target_poses = {
            SinglePoseEvaluator::createPoseFromRPY(Eigen::Vector3d(0.4, 0.0, 0.3), 0, 0, 0),
            SinglePoseEvaluator::createPoseFromRPY(Eigen::Vector3d(0.3, 0.3, 0.4), 0, 0, M_PI/4),
            SinglePoseEvaluator::createPoseFromRPY(Eigen::Vector3d(0.5, -0.2, 0.25), 0, M_PI/6, 0),
            SinglePoseEvaluator::createPoseFromRPY(Eigen::Vector3d(0.35, 0.25, 0.45), M_PI/12, 0, -M_PI/6)
        };
        
        // Evaluate each pose
        for (size_t i = 0; i < target_poses.size(); ++i) {
            std::cout << "\nEvaluating pose " << (i + 1) << "/" << target_poses.size() << std::endl;
            
            // Update target pose
            evaluator.setTargetPose(target_poses[i]);
            
            // Run evaluation for this pose
            bool success = evaluator.runEvaluation();
            
            if (success) {
                std::cout << "✓ Pose " << (i + 1) << " evaluation completed" << std::endl;
            } else {
                std::cout << "✗ Pose " << (i + 1) << " evaluation failed" << std::endl;
            }
        }
        
    } catch (const std::exception& e) {
        std::cout << "Error in dynamic evaluation: " << e.what() << std::endl;
    }
}

/**
 * @brief Example of custom single pose evaluation
 */
void runCustomSinglePoseEvaluation() {
    std::cout << "\n=== Custom Single Pose Evaluation ===" << std::endl;
    
    // Create custom configuration
    SinglePoseEvalConfig config;
    
    // Set paths (replace with actual paths)
    config.robot_urdf_path = "./res/robot/panda.urdf";
    config.environment_xml_path = "./res/environment.xml";
    config.output_directory = "./evaluation_results/custom_single_pose";
    
    // Set current joint angles - robot in a specific configuration
    config.current_joint_angles = Eigen::VectorXd(7);
    config.current_joint_angles << M_PI/8, -M_PI/3, M_PI/6, -2*M_PI/3, -M_PI/4, M_PI/2, M_PI/3;
    
    // Set target pose - reach to a specific point with specific orientation
    Eigen::Vector3d target_position(0.45, 0.15, 0.35);
    Eigen::Quaterniond target_orientation;
    target_orientation = Eigen::AngleAxisd(M_PI/6, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(M_PI/12, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    
    config.target_pose = SinglePoseEvaluator::createPose(target_position, target_orientation);
    
    // Configure evaluation
    config.num_trials = 8;
    config.verbose = true;
    config.apply_pose_offset = true;
    
    // Use STOMP with checkpoints for this example
    config.trajectory_algorithm = TrajectoryAlgorithm::STOMP_WITH_CHECKPOINTS;
    config.stomp_config.max_iterations = 200;
    config.stomp_config.num_noisy_trajectories = 15;
    config.stomp_config.learning_rate = 0.15;
    config.stomp_config.temperature = 12.0;
    
    try {
        std::cout << "Creating SinglePoseEvaluator..." << std::endl;
        SinglePoseEvaluator evaluator(config);
        
        std::cout << "Starting evaluation..." << std::endl;
        bool success = evaluator.runEvaluation();
        
        if (success) {
            std::cout << "✓ Custom single pose evaluation completed successfully!" << std::endl;
            std::cout << "Results saved to: " << config.output_directory << std::endl;
        } else {
            std::cout << "✗ Custom single pose evaluation failed" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

/**
 * @brief Example of using legacy parameters for backward compatibility
 */
void runLegacyParameterExample() {
    std::cout << "\n=== Legacy Parameter Compatibility Example ===" << std::endl;
    
    SinglePoseEvalConfig config;
    
    // Basic setup
    config.robot_urdf_path = "./res/robot/panda.urdf";
    config.environment_xml_path = "./res/environment.xml";
    config.output_directory = "./evaluation_results/legacy_single_pose";
    
    // Joint configuration and target pose
    config.current_joint_angles = Eigen::VectorXd::Zero(7);
    config.target_pose = SinglePoseEvaluator::createPoseFromRPY(
        Eigen::Vector3d(0.4, 0.0, 0.3), 0, 0, 0);
    
    // Use legacy parameters (automatically mapped to new structures)
    config.max_stomp_iterations = 120;
    config.stomp_learning_rate = 0.18;
    config.stomp_temperature = 15.0;
    config.num_noisy_trajectories = 14;
    
    std::cout << "Using legacy parameters:" << std::endl;
    std::cout << "  max_stomp_iterations: " << config.max_stomp_iterations << std::endl;
    std::cout << "  stomp_learning_rate: " << config.stomp_learning_rate << std::endl;
    std::cout << "  stomp_temperature: " << config.stomp_temperature << std::endl;
    
    try {
        SinglePoseEvaluator evaluator(config);
        
        // The evaluator will automatically map legacy parameters to new configuration
        std::cout << "✓ Legacy parameters successfully mapped to new configuration system" << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

int main() {
    std::cout << "SinglePoseEvaluator Examples" << std::endl;
    std::cout << "============================" << std::endl;
    
    // Uncomment one of the following to run:
    
    // Run algorithm comparison
    // runAlgorithmComparison();
    
    // Run dynamic pose evaluation
    // runDynamicPoseEvaluation();
    
    // Run custom single pose evaluation
    runCustomSinglePoseEvaluation();
    
    // Run legacy parameter example
    // runLegacyParameterExample();
    
    return 0;
}
