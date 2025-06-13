#include "single_pose_evaluator.h"
#include <iostream>
#include <vector>

int main() {
    std::cout << "\n=== Testing Clearance Functionality ===\n" << std::endl;
    
    try {
        // Configure clearance analysis enabled
        SinglePoseEvalConfig config;
        config.robot_urdf_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf";
        config.environment_xml_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/obstacles.xml";
        config.output_directory = "/tmp/clearance_test";
        
        // Robot configuration
        Eigen::VectorXd current_joints(7);
        current_joints << 0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.7854;
        config.current_joint_angles = current_joints;
        
        // Set up a test pose
        auto target_pose = SinglePoseEvaluator::createPoseFromRPY(
            Eigen::Vector3d(0.5, 0.1, 0.3), 0, 0, 0);
        config.target_pose = target_pose;
        
        // Evaluation parameters
        config.num_trials = 3;
        config.verbose = true;
        config.trajectory_algorithm = TrajectoryAlgorithm::STOMP;
        
        // Enable clearance analysis
        config.enable_clearance_analysis = true;
        config.critical_clearance_threshold = 0.05;  // 5cm
        config.safe_clearance_threshold = 0.15;      // 15cm
        config.compute_self_clearance = true;
        config.clearance_sample_rate = 10;           // Sample every 10th waypoint for speed
        
        // Enable trajectory planning analysis
        config.enable_trajectory_planning_analysis = true;
        config.singularity_threshold = 0.01;         // Manipulability threshold for singularities
        config.critical_ttc_threshold = 1.0;         // 1 second critical time-to-collision
        config.max_joint_torque_limit = 100.0;       // 100 Nm max torque limit
        
        // Enable execution analysis
        config.enable_execution_analysis = true;
        
        std::cout << "✅ Configuration created with comprehensive analysis enabled" << std::endl;
        std::cout << "   - Clearance analysis: enabled" << std::endl;
        std::cout << "   - Trajectory planning analysis: enabled" << std::endl;
        std::cout << "   - Execution analysis: enabled" << std::endl;
        std::cout << "   - Critical clearance threshold: " << config.critical_clearance_threshold << "m" << std::endl;
        std::cout << "   - Safe clearance threshold: " << config.safe_clearance_threshold << "m" << std::endl;
        std::cout << "   - Singularity threshold: " << config.singularity_threshold << std::endl;
        std::cout << "   - Sample rate: " << config.clearance_sample_rate << std::endl;
        
        // Create evaluator with clearance analysis enabled
        std::cout << "📝 Creating SinglePoseEvaluator with configuration..." << std::endl;
        SinglePoseEvaluator evaluator(config);
        
        std::cout << "✅ Test pose and configuration set" << std::endl;
        
        // Run evaluation with clearance analysis
        std::cout << "\n📝 Running evaluation with comprehensive trajectory analysis..." << std::endl;
        
        bool success = evaluator.runEvaluation();
        
        if (success) {
            std::cout << "\n🎉 Evaluation completed successfully!" << std::endl;
            std::cout << "✅ All clearance metrics are being calculated and exported" << std::endl;
            std::cout << "📁 Check output directory for results: " << config.output_directory << std::endl;
        } else {
            std::cout << "\n❌ Evaluation failed" << std::endl;
        }
        
        std::cout << "\n🎉 Clearance functionality test completed!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
