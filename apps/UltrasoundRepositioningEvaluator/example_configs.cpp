/**
 * @file example_configs.cpp
 * @brief Example configurations for the UltrasoundRepositioningEvaluator
 * 
 * This file demonstrates how to set up different algorithm configurations
 * for evaluating ultrasound repositioning trajectories using various
 * planning algorithms including STOMP variants and Hauser with path planning.
 */

#include "repositioning_evaluator.h"
#include <iostream>

/**
 * @brief Create a configuration for STOMP algorithm evaluation
 */
RepositioningEvalConfig createStompConfig() {
    RepositioningEvalConfig config;
    
    // Basic setup
    config.robot_urdf_path = "/path/to/robot.urdf";
    config.environment_xml_path = "/path/to/environment.xml";
    config.scan_poses_csv_path = "/path/to/scan_poses.csv";
    config.output_directory = "./evaluation_results/stomp_evaluation";
    
    // Initial joint configuration (7-DOF robot)
    config.initial_joint_config = Eigen::VectorXd(7);
    config.initial_joint_config << -M_PI/4, M_PI/8, -M_PI/8, -M_PI/3, 0, M_PI/4, 0;
    
    // Evaluation parameters
    config.num_trials = 20;
    config.verbose = true;
    
    // Algorithm selection
    config.trajectory_algorithm = TrajectoryAlgorithm::STOMP;
    
    // STOMP-specific configuration
    config.stomp_config.max_iterations = 200;
    config.stomp_config.num_noisy_trajectories = 15;
    config.stomp_config.num_best_samples = 5;
    config.stomp_config.learning_rate = 0.15;
    config.stomp_config.temperature = 12.0;
    config.stomp_config.dt = 0.05;
    config.stomp_config.joint_std_devs = Eigen::VectorXd::Constant(7, 0.08);
    
    return config;
}

/**
 * @brief Create a configuration for STOMP with early termination
 */
RepositioningEvalConfig createStompEarlyTerminationConfig() {
    RepositioningEvalConfig config = createStompConfig(); // Start with base STOMP config
    
    // Change algorithm variant
    config.trajectory_algorithm = TrajectoryAlgorithm::STOMP_WITH_EARLY_TERMINATION;
    config.output_directory = "./evaluation_results/stomp_early_termination_evaluation";
    
    // Adjust parameters for early termination variant
    config.stomp_config.max_iterations = 500; // Allow more iterations since early termination will stop when converged
    config.stomp_config.learning_rate = 0.1;  // More conservative learning rate
    
    return config;
}

/**
 * @brief Create a configuration for STOMP with checkpoints
 */
RepositioningEvalConfig createStompCheckpointsConfig() {
    RepositioningEvalConfig config = createStompConfig(); // Start with base STOMP config
    
    // Change algorithm variant
    config.trajectory_algorithm = TrajectoryAlgorithm::STOMP_WITH_CHECKPOINTS;
    config.output_directory = "./evaluation_results/stomp_checkpoints_evaluation";
    
    // Adjust parameters for checkpoint variant
    config.stomp_config.max_iterations = 150;
    config.stomp_config.num_noisy_trajectories = 12;
    config.stomp_config.temperature = 8.0; // Lower temperature for more focused search around checkpoints
    
    return config;
}

/**
 * @brief Create a configuration for Hauser algorithm with RRT path planning
 */
RepositioningEvalConfig createHauserRRTConfig() {
    RepositioningEvalConfig config;
    
    // Basic setup
    config.robot_urdf_path = "/path/to/robot.urdf";
    config.environment_xml_path = "/path/to/environment.xml";
    config.scan_poses_csv_path = "/path/to/scan_poses.csv";
    config.output_directory = "./evaluation_results/hauser_rrt_evaluation";
    
    // Initial joint configuration
    config.initial_joint_config = Eigen::VectorXd(7);
    config.initial_joint_config << -M_PI/4, M_PI/8, -M_PI/8, -M_PI/3, 0, M_PI/4, 0;
    
    // Evaluation parameters
    config.num_trials = 20;
    config.verbose = true;
    
    // Algorithm selection
    config.trajectory_algorithm = TrajectoryAlgorithm::HAUSER;
    
    // Path planning configuration (Phase 1)
    config.path_planning_config.algorithm = PathPlanningAlgorithm::RRT;
    config.path_planning_config.max_iterations = 8000;
    config.path_planning_config.step_size = 0.15;
    config.path_planning_config.goal_bias = 0.2;
    config.path_planning_config.custom_cost = false;
    
    // Hauser algorithm configuration (Phase 2)
    config.hauser_config.max_iterations = 800;
    config.hauser_config.output_file = "./evaluation_results/hauser_timing.csv";
    
    return config;
}

/**
 * @brief Create a configuration for Hauser algorithm with RRT* path planning
 */
RepositioningEvalConfig createHauserRRTStarConfig() {
    RepositioningEvalConfig config = createHauserRRTConfig(); // Start with base Hauser config
    
    // Change path planning algorithm
    config.path_planning_config.algorithm = PathPlanningAlgorithm::RRT_STAR;
    config.output_directory = "./evaluation_results/hauser_rrt_star_evaluation";
    
    // Adjust parameters for RRT*
    config.path_planning_config.max_iterations = 10000; // RRT* typically needs more iterations
    config.path_planning_config.step_size = 0.1; // Smaller step size for better optimization
    config.path_planning_config.goal_bias = 0.15; // Lower goal bias for better exploration
    
    return config;
}

/**
 * @brief Create a configuration for Hauser algorithm with Informed RRT* path planning
 */
RepositioningEvalConfig createHauserInformedRRTStarConfig() {
    RepositioningEvalConfig config = createHauserRRTStarConfig(); // Start with RRT* config
    
    // Change to Informed RRT*
    config.path_planning_config.algorithm = PathPlanningAlgorithm::INFORMED_RRT_STAR;
    config.output_directory = "./evaluation_results/hauser_informed_rrt_star_evaluation";
    
    // Informed RRT* specific adjustments
    config.path_planning_config.max_iterations = 12000; // Allow more iterations for convergence
    config.path_planning_config.step_size = 0.08; // Even smaller step size for precision
    
    return config;
}

/**
 * @brief Create a high-performance configuration optimized for speed
 */
RepositioningEvalConfig createHighSpeedConfig() {
    RepositioningEvalConfig config;
    
    // Basic setup
    config.robot_urdf_path = "/path/to/robot.urdf";
    config.environment_xml_path = "/path/to/environment.xml";
    config.scan_poses_csv_path = "/path/to/scan_poses.csv";
    config.output_directory = "./evaluation_results/high_speed_evaluation";
    
    // Initial joint configuration
    config.initial_joint_config = Eigen::VectorXd(7);
    config.initial_joint_config << -M_PI/4, M_PI/8, -M_PI/8, -M_PI/3, 0, M_PI/4, 0;
    
    // Evaluation parameters
    config.num_trials = 10; // Fewer trials for faster evaluation
    config.verbose = false; // Reduce logging
    
    // Use STOMP for speed
    config.trajectory_algorithm = TrajectoryAlgorithm::STOMP;
    
    // Speed-optimized STOMP configuration
    config.stomp_config.max_iterations = 50; // Fewer iterations
    config.stomp_config.num_noisy_trajectories = 8; // Fewer samples
    config.stomp_config.num_best_samples = 3;
    config.stomp_config.learning_rate = 0.2; // Higher learning rate
    config.stomp_config.dt = 0.1; // Larger time step
    
    return config;
}

/**
 * @brief Create a high-quality configuration optimized for trajectory quality
 */
RepositioningEvalConfig createHighQualityConfig() {
    RepositioningEvalConfig config;
    
    // Basic setup
    config.robot_urdf_path = "/path/to/robot.urdf";
    config.environment_xml_path = "/path/to/environment.xml";
    config.scan_poses_csv_path = "/path/to/scan_poses.csv";
    config.output_directory = "./evaluation_results/high_quality_evaluation";
    
    // Initial joint configuration
    config.initial_joint_config = Eigen::VectorXd(7);
    config.initial_joint_config << -M_PI/4, M_PI/8, -M_PI/8, -M_PI/3, 0, M_PI/4, 0;
    
    // Evaluation parameters
    config.num_trials = 50; // More trials for statistical significance
    config.verbose = true;
    
    // Use Hauser with Informed RRT* for highest quality
    config.trajectory_algorithm = TrajectoryAlgorithm::HAUSER;
    
    // High-quality path planning
    config.path_planning_config.algorithm = PathPlanningAlgorithm::INFORMED_RRT_STAR;
    config.path_planning_config.max_iterations = 15000;
    config.path_planning_config.step_size = 0.05; // Very small step size
    config.path_planning_config.goal_bias = 0.1;
    config.path_planning_config.custom_cost = true; // Enable custom cost function
    
    // High-quality motion generation
    config.hauser_config.max_iterations = 1500;
    
    return config;
}

/**
 * @brief Example of running a comparative evaluation
 */
void runComparativeEvaluation() {
    std::cout << "=== Comparative Algorithm Evaluation ===" << std::endl;
    
    // Create different configurations
    std::vector<RepositioningEvalConfig> configs = {
        createStompConfig(),
        createStompEarlyTerminationConfig(),
        createHauserRRTConfig(),
        createHauserRRTStarConfig()
    };
    
    std::vector<std::string> config_names = {
        "STOMP",
        "STOMP_Early_Termination", 
        "Hauser_RRT",
        "Hauser_RRT_Star"
    };
    
    // Run evaluation for each configuration
    for (size_t i = 0; i < configs.size(); ++i) {
        std::cout << "\n--- Evaluating " << config_names[i] << " ---" << std::endl;
        
        try {
            RepositioningEvaluator evaluator(configs[i]);
            bool success = evaluator.runEvaluation();
            
            if (success) {
                std::cout << "✓ " << config_names[i] << " evaluation completed successfully" << std::endl;
            } else {
                std::cout << "✗ " << config_names[i] << " evaluation failed" << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cout << "✗ " << config_names[i] << " evaluation error: " << e.what() << std::endl;
        }
    }
    
    std::cout << "\n=== Comparative evaluation completed ===" << std::endl;
}

/**
 * @brief Example of running a custom configuration
 */
void runCustomConfiguration() {
    std::cout << "=== Custom Configuration Example ===" << std::endl;
    
    // Create a custom configuration
    RepositioningEvalConfig config;
    
    // Set paths (replace with actual paths)
    config.robot_urdf_path = "./res/robot/panda.urdf";
    config.environment_xml_path = "./res/environment.xml";
    config.scan_poses_csv_path = "./sample_scan_poses.csv";
    config.output_directory = "./evaluation_results/custom_evaluation";
    
    // Set initial joint configuration
    config.initial_joint_config = Eigen::VectorXd(7);
    config.initial_joint_config << 0, -M_PI/4, 0, -3*M_PI/4, 0, M_PI/2, M_PI/4;
    
    // Customize evaluation parameters
    config.num_trials = 15;
    config.verbose = true;
    
    // Choose algorithm
    config.trajectory_algorithm = TrajectoryAlgorithm::STOMP_WITH_EARLY_TERMINATION;
    
    // Fine-tune STOMP parameters
    config.stomp_config.max_iterations = 300;
    config.stomp_config.num_noisy_trajectories = 20;
    config.stomp_config.learning_rate = 0.12;
    config.stomp_config.temperature = 15.0;
    config.stomp_config.joint_std_devs = Eigen::VectorXd::Constant(7, 0.06);
    
    try {
        RepositioningEvaluator evaluator(config);
        bool success = evaluator.runEvaluation();
        
        if (success) {
            std::cout << "✓ Custom evaluation completed successfully" << std::endl;
            std::cout << "Results saved to: " << config.output_directory << std::endl;
        } else {
            std::cout << "✗ Custom evaluation failed" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cout << "✗ Custom evaluation error: " << e.what() << std::endl;
    }
}

int main() {
    std::cout << "UltrasoundRepositioningEvaluator Configuration Examples" << std::endl;
    std::cout << "======================================================" << std::endl;
    
    // Uncomment one of the following to run:
    
    // Run comparative evaluation of different algorithms
    // runComparativeEvaluation();
    
    // Run a single custom configuration
    runCustomConfiguration();
    
    return 0;
}
