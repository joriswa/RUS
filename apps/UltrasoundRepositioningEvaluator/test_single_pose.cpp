/**
 * @file test_single_pose.cpp
 * @brief Simple test for SinglePoseEvaluator functionality
 */

#include "single_pose_evaluator.h"
#include <iostream>
#include <filesystem>

/**
 * @brief Create a simple test configuration for verification
 */
SinglePoseEvalConfig createTestConfig() {
    SinglePoseEvalConfig config;
    
    // Use actual files from the workspace
    config.robot_urdf_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf";
    config.environment_xml_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/obstacles.xml";
    config.output_directory = "./test_results/single_pose_test";
    
    // Simple initial joint configuration (all joints at 0 degrees)
    config.current_joint_angles = Eigen::VectorXd::Zero(7);
    
    // Target pose - move forward 30cm and up 20cm
    config.target_pose = SinglePoseEvaluator::createPoseFromRPY(
        Eigen::Vector3d(0.3, 0.0, 0.2),  // position: 30cm forward, 20cm up
        0.0, 0.0, 0.0                     // no rotation
    );
    
    // Minimal evaluation parameters for quick test
    config.num_trials = 3;
    config.verbose = true;
    config.apply_pose_offset = false;  // Skip offset for simplicity
    
    // Enable clearance analysis with custom thresholds
    config.enable_clearance_analysis = true;
    config.critical_clearance_threshold = 0.03;  // 3cm critical threshold
    config.safe_clearance_threshold = 0.10;      // 10cm safe threshold
    config.compute_self_clearance = true;
    config.clearance_sample_rate = 2;            // Sample every 2nd point for speed
    
    // Use basic STOMP with reduced parameters for speed
    config.trajectory_algorithm = TrajectoryAlgorithm::STOMP;
    config.stomp_config.max_iterations = 50;
    config.stomp_config.num_noisy_trajectories = 6;
    config.stomp_config.learning_rate = 0.2;
    config.stomp_config.temperature = 8.0;
    
    return config;
}

bool testBasicFunctionality() {
    std::cout << "=== Testing Basic SinglePoseEvaluator Functionality ===" << std::endl;
    
    try {
        SinglePoseEvalConfig config = createTestConfig();
        
        // Verify that the files exist
        if (!std::filesystem::exists(config.robot_urdf_path)) {
            std::cout << "❌ Robot URDF file not found: " << config.robot_urdf_path << std::endl;
            return false;
        }
        
        if (!std::filesystem::exists(config.environment_xml_path)) {
            std::cout << "❌ Environment XML file not found: " << config.environment_xml_path << std::endl;
            return false;
        }
        
        std::cout << "✅ Required files found" << std::endl;
        
        // Create evaluator
        std::cout << "📝 Creating SinglePoseEvaluator..." << std::endl;
        SinglePoseEvaluator evaluator(config);
        std::cout << "✅ SinglePoseEvaluator created successfully" << std::endl;
        
        // Test pose creation utilities
        std::cout << "📝 Testing pose creation utilities..." << std::endl;
        
        auto pose1 = SinglePoseEvaluator::createPoseFromRPY(
            Eigen::Vector3d(0.5, 0.1, 0.3), 
            M_PI/6, 0.0, M_PI/4
        );
        std::cout << "✅ createPoseFromRPY works" << std::endl;
        
        Eigen::Quaterniond quat(0.707, 0.0, 0.707, 0.0);
        auto pose2 = SinglePoseEvaluator::createPose(
            Eigen::Vector3d(0.4, 0.2, 0.35), 
            quat
        );
        std::cout << "✅ createPose with quaternion works" << std::endl;
        
        // Test dynamic pose setting
        std::cout << "📝 Testing dynamic pose updates..." << std::endl;
        evaluator.setTargetPose(pose1);
        std::cout << "✅ setTargetPose works" << std::endl;
        
        // Test joint angle updates
        Eigen::VectorXd new_joints(7);
        new_joints << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7;
        evaluator.setCurrentJointAngles(new_joints);
        std::cout << "✅ setCurrentJointAngles works" << std::endl;
        
        std::cout << "🎉 All basic functionality tests passed!" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cout << "❌ Error during basic functionality test: " << e.what() << std::endl;
        return false;
    }
}

bool testConfigurationValidation() {
    std::cout << "\n=== Testing Configuration Validation ===" << std::endl;
    
    try {
        // Test AlgorithmUtils functions
        std::cout << "📝 Testing AlgorithmUtils..." << std::endl;
        
        // Test string conversion
        std::string stomp_str = AlgorithmUtils::trajectoryAlgorithmToString(TrajectoryAlgorithm::STOMP);
        std::string hauser_str = AlgorithmUtils::trajectoryAlgorithmToString(TrajectoryAlgorithm::HAUSER);
        
        if (stomp_str == "STOMP" && hauser_str == "HAUSER") {
            std::cout << "✅ trajectoryAlgorithmToString works" << std::endl;
        } else {
            std::cout << "❌ trajectoryAlgorithmToString failed" << std::endl;
            return false;
        }
        
        // Test path planning algorithm conversion
        std::string rrt_str = AlgorithmUtils::pathPlanningAlgorithmToString(PathPlanningAlgorithm::RRT);
        if (rrt_str == "RRT") {
            std::cout << "✅ pathPlanningAlgorithmToString works" << std::endl;
        } else {
            std::cout << "❌ pathPlanningAlgorithmToString failed" << std::endl;
            return false;
        }
        
        // Test enum conversion
        Algorithm lib_algo = AlgorithmUtils::pathPlanningAlgorithmToLibraryEnum(PathPlanningAlgorithm::RRT_STAR);
        std::cout << "✅ pathPlanningAlgorithmToLibraryEnum works" << std::endl;
        
        // Test Params creation
        PathPlanningConfig path_config;
        path_config.algorithm = PathPlanningAlgorithm::RRT;
        path_config.max_iterations = 1000;
        path_config.step_size = 0.1;
        
        Params params = AlgorithmUtils::createParamsFromConfig(path_config);
        if (params.maxIterations == 1000 && params.stepSize == 0.1) {
            std::cout << "✅ createParamsFromConfig works" << std::endl;
        } else {
            std::cout << "❌ createParamsFromConfig failed" << std::endl;
            return false;
        }
        
        // Test StompConfig conversion
        StompAlgorithmConfig stomp_config;
        stomp_config.max_iterations = 100;
        stomp_config.learning_rate = 0.15;
        
        StompConfig converted = stomp_config.toStompConfig();
        if (converted.maxIterations == 100 && converted.learningRate == 0.15) {
            std::cout << "✅ StompAlgorithmConfig.toStompConfig works" << std::endl;
        } else {
            std::cout << "❌ StompAlgorithmConfig.toStompConfig failed" << std::endl;
            return false;
        }
        
        std::cout << "🎉 All configuration validation tests passed!" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cout << "❌ Error during configuration validation test: " << e.what() << std::endl;
        return false;
    }
}

int main() {
    std::cout << "SinglePoseEvaluator Test Suite" << std::endl;
    std::cout << "==============================" << std::endl;
    
    bool all_passed = true;
    
    // Run tests
    all_passed &= testBasicFunctionality();
    all_passed &= testConfigurationValidation();
    
    // Final result
    std::cout << "\n==============================" << std::endl;
    if (all_passed) {
        std::cout << "🎉 ALL TESTS PASSED! 🎉" << std::endl;
        std::cout << "SinglePoseEvaluator is ready to use!" << std::endl;
    } else {
        std::cout << "❌ SOME TESTS FAILED ❌" << std::endl;
        std::cout << "Please check the errors above." << std::endl;
    }
    
    return all_passed ? 0 : 1;
}
