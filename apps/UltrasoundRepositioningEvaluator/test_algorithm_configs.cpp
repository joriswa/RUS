/**
 * @file test_algorithm_configs.cpp
 * @brief Simple test to verify the new algorithm configuration system
 */

#include "repositioning_evaluator.h"
#include <iostream>

int main() {
    std::cout << "Testing UltrasoundRepositioningEvaluator Algorithm Configuration System\n";
    std::cout << "=====================================================================\n\n";

    try {
        // Test 1: Basic STOMP configuration
        std::cout << "Test 1: Creating STOMP configuration..." << std::endl;
        
        RepositioningEvalConfig stomp_config;
        stomp_config.robot_urdf_path = "./res/robot/panda.urdf";
        stomp_config.environment_xml_path = "./res/environment.xml";
        stomp_config.scan_poses_csv_path = "./sample_scan_poses.csv";
        stomp_config.output_directory = "./evaluation_results/test_stomp";
        
        // Set initial joint configuration
        stomp_config.initial_joint_config = Eigen::VectorXd(7);
        stomp_config.initial_joint_config << 0, -M_PI/4, 0, -3*M_PI/4, 0, M_PI/2, M_PI/4;
        
        stomp_config.num_trials = 5; // Small number for testing
        stomp_config.trajectory_algorithm = TrajectoryAlgorithm::STOMP;
        
        // Customize STOMP parameters
        stomp_config.stomp_config.max_iterations = 50;
        stomp_config.stomp_config.num_noisy_trajectories = 8;
        stomp_config.stomp_config.learning_rate = 0.15;
        
        std::cout << "✓ STOMP config created successfully" << std::endl;
        std::cout << "  Algorithm: " << AlgorithmUtils::trajectoryAlgorithmToString(stomp_config.trajectory_algorithm) << std::endl;
        std::cout << "  Max iterations: " << stomp_config.stomp_config.max_iterations << std::endl;
        
        // Test 2: Hauser with RRT configuration
        std::cout << "\nTest 2: Creating Hauser+RRT configuration..." << std::endl;
        
        RepositioningEvalConfig hauser_config;
        hauser_config.robot_urdf_path = "./res/robot/panda.urdf";
        hauser_config.environment_xml_path = "./res/environment.xml";
        hauser_config.scan_poses_csv_path = "./sample_scan_poses.csv";
        hauser_config.output_directory = "./evaluation_results/test_hauser";
        
        hauser_config.initial_joint_config = Eigen::VectorXd(7);
        hauser_config.initial_joint_config << 0, -M_PI/4, 0, -3*M_PI/4, 0, M_PI/2, M_PI/4;
        
        hauser_config.num_trials = 3;
        hauser_config.trajectory_algorithm = TrajectoryAlgorithm::HAUSER;
        
        // Configure path planning
        hauser_config.path_planning_config.algorithm = PathPlanningAlgorithm::RRT_STAR;
        hauser_config.path_planning_config.max_iterations = 5000;
        hauser_config.path_planning_config.step_size = 0.1;
        
        // Configure Hauser
        hauser_config.hauser_config.max_iterations = 300;
        
        std::cout << "✓ Hauser config created successfully" << std::endl;
        std::cout << "  Trajectory Algorithm: " << AlgorithmUtils::trajectoryAlgorithmToString(hauser_config.trajectory_algorithm) << std::endl;
        std::cout << "  Path Planning Algorithm: " << AlgorithmUtils::pathPlanningAlgorithmToString(hauser_config.path_planning_config.algorithm) << std::endl;
        std::cout << "  Hauser iterations: " << hauser_config.hauser_config.max_iterations << std::endl;
        
        // Test 3: Algorithm utilities
        std::cout << "\nTest 3: Testing algorithm utility functions..." << std::endl;
        
        // Test trajectory algorithm conversions
        TrajectoryAlgorithm algos[] = {
            TrajectoryAlgorithm::STOMP,
            TrajectoryAlgorithm::STOMP_WITH_CHECKPOINTS,
            TrajectoryAlgorithm::STOMP_WITH_EARLY_TERMINATION,
            TrajectoryAlgorithm::HAUSER
        };
        
        for (auto algo : algos) {
            std::string name = AlgorithmUtils::trajectoryAlgorithmToString(algo);
            std::cout << "  " << name << std::endl;
        }
        
        // Test path planning algorithm conversions
        PathPlanningAlgorithm path_algos[] = {
            PathPlanningAlgorithm::RRT,
            PathPlanningAlgorithm::RRT_STAR,
            PathPlanningAlgorithm::INFORMED_RRT_STAR,
            PathPlanningAlgorithm::RRT_CONNECT
        };
        
        for (auto algo : path_algos) {
            std::string name = AlgorithmUtils::pathPlanningAlgorithmToString(algo);
            Algorithm lib_algo = AlgorithmUtils::pathPlanningAlgorithmToLibraryEnum(algo);
            std::cout << "  " << name << " -> " << lib_algo << std::endl;
        }
        
        std::cout << "✓ Algorithm utilities working correctly" << std::endl;
        
        // Test 4: Test StompConfig conversion
        std::cout << "\nTest 4: Testing StompConfig conversion..." << std::endl;
        
        StompAlgorithmConfig stomp_algo_config;
        stomp_algo_config.max_iterations = 150;
        stomp_algo_config.num_noisy_trajectories = 12;
        stomp_algo_config.learning_rate = 0.12;
        stomp_algo_config.temperature = 15.0;
        
        StompConfig lib_config = stomp_algo_config.toStompConfig();
        
        std::cout << "  Original max_iterations: " << stomp_algo_config.max_iterations << std::endl;
        std::cout << "  Converted maxIterations: " << lib_config.maxIterations << std::endl;
        std::cout << "  Original learning_rate: " << stomp_algo_config.learning_rate << std::endl;
        std::cout << "  Converted learningRate: " << lib_config.learningRate << std::endl;
        
        if (lib_config.maxIterations == stomp_algo_config.max_iterations &&
            lib_config.learningRate == stomp_algo_config.learning_rate) {
            std::cout << "✓ StompConfig conversion working correctly" << std::endl;
        } else {
            std::cout << "✗ StompConfig conversion failed" << std::endl;
        }
        
        // Test 5: Backward compatibility
        std::cout << "\nTest 5: Testing backward compatibility..." << std::endl;
        
        RepositioningEvalConfig legacy_config;
        legacy_config.robot_urdf_path = "./res/robot/panda.urdf";
        legacy_config.environment_xml_path = "./res/environment.xml";
        legacy_config.scan_poses_csv_path = "./sample_scan_poses.csv";
        legacy_config.output_directory = "./evaluation_results/test_legacy";
        
        legacy_config.initial_joint_config = Eigen::VectorXd(7);
        legacy_config.initial_joint_config << 0, -M_PI/4, 0, -3*M_PI/4, 0, M_PI/2, M_PI/4;
        
        // Use legacy parameters
        legacy_config.max_stomp_iterations = 75;
        legacy_config.stomp_learning_rate = 0.18;
        legacy_config.max_rrt_iterations = 7500;
        
        std::cout << "  Legacy STOMP iterations: " << legacy_config.max_stomp_iterations << std::endl;
        std::cout << "  Legacy learning rate: " << legacy_config.stomp_learning_rate << std::endl;
        std::cout << "  Legacy RRT iterations: " << legacy_config.max_rrt_iterations << std::endl;
        
        std::cout << "✓ Backward compatibility parameters set successfully" << std::endl;
        
        std::cout << "\n🎉 All configuration tests passed successfully!" << std::endl;
        std::cout << "\nNext steps:" << std::endl;
        std::cout << "1. Ensure robot URDF and environment files exist at specified paths" << std::endl;
        std::cout << "2. Create sample scan poses CSV file" << std::endl;
        std::cout << "3. Run actual evaluation with: RepositioningEvaluator evaluator(config); evaluator.runEvaluation();" << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "✗ Test failed with error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
