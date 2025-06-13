#include "repositioning_evaluator.h"
#include <iostream>
#include <filesystem>
#include <fstream>

/**
 * @brief Create a simple two-pose CSV file for testing
 * @param filename Output filename
 */
void createSamplePoseFile(const std::string& filename) {
    std::ofstream file(filename);
    
    // First pose: position (0.5, 0.2, 0.6), identity rotation
    file << "0.5,0.2,0.6,1.0,0.0,0.0,0.0\n";
    
    // Second pose: position (0.4, -0.1, 0.5), slight rotation around Z
    file << "0.4,-0.1,0.5,0.9659,0.0,0.0,0.2588\n";
    
    file.close();
    std::cout << "Created sample pose file: " << filename << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "=== Ultrasound Repositioning Evaluator ===" << std::endl;
    std::cout << "Evaluating trajectory planning for ultrasound probe repositioning" << std::endl;
    std::cout << "====================================================" << std::endl;
    
    try {
        // Configuration for the evaluation
        RepositioningEvalConfig config;
        
        // Default paths - can be overridden by command line arguments
        config.robot_urdf_path = "../../../res/robot/panda.urdf";
        config.environment_xml_path = "../../../res/scenario_1/obstacles.xml";
        config.scan_poses_csv_path = "./sample_scan_poses.csv";
        config.output_directory = "./evaluation_results";
        
        // Parse command line arguments
        if (argc >= 2) {
            config.robot_urdf_path = argv[1];
        }
        if (argc >= 3) {
            config.environment_xml_path = argv[2];
        }
        if (argc >= 4) {
            config.scan_poses_csv_path = argv[3];
        }
        if (argc >= 5) {
            config.output_directory = argv[4];
        }
        
        // Set initial robot configuration (Panda home position)
        config.initial_joint_config = Eigen::VectorXd(7);
        config.initial_joint_config << 0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.7854;
        
        // Evaluation parameters
        config.num_trials = 5;  // Start with fewer trials for testing
        config.verbose = true;
        
        // STOMP parameters
        config.max_stomp_iterations = 50;
        config.num_noisy_trajectories = 5;
        config.stomp_learning_rate = 0.1;
        config.stomp_temperature = 10.0;
        
        std::cout << "Configuration:" << std::endl;
        std::cout << "  Robot URDF: " << config.robot_urdf_path << std::endl;
        std::cout << "  Environment: " << config.environment_xml_path << std::endl;
        std::cout << "  Scan Poses: " << config.scan_poses_csv_path << std::endl;
        std::cout << "  Output Dir: " << config.output_directory << std::endl;
        std::cout << "  Trials: " << config.num_trials << std::endl;
        std::cout << std::endl;
        
        // Check if files exist
        if (!std::filesystem::exists(config.robot_urdf_path)) {
            std::cerr << "Error: Robot URDF file not found: " << config.robot_urdf_path << std::endl;
            std::cout << "Trying alternative path..." << std::endl;
            config.robot_urdf_path = "../../res/robot/panda.urdf";
            if (!std::filesystem::exists(config.robot_urdf_path)) {
                std::cerr << "Error: Robot URDF file not found at alternative path: " << config.robot_urdf_path << std::endl;
                return 1;
            }
        }
        
        if (!std::filesystem::exists(config.environment_xml_path)) {
            std::cerr << "Warning: Environment file not found: " << config.environment_xml_path << std::endl;
            std::cout << "Trying alternative path..." << std::endl;
            config.environment_xml_path = "../../res/scenario_1/obstacles.xml";
            if (!std::filesystem::exists(config.environment_xml_path)) {
                std::cerr << "Warning: Environment file not found at alternative path either." << std::endl;
                std::cerr << "Continuing without environment obstacles..." << std::endl;
                config.environment_xml_path = "";
            }
        }
        
        if (!std::filesystem::exists(config.scan_poses_csv_path)) {
            std::cout << "Scan poses file not found. Creating sample file..." << std::endl;
            createSamplePoseFile(config.scan_poses_csv_path);
        }
        
        // Create and run the evaluator
        std::cout << "Initializing evaluator..." << std::endl;
        RepositioningEvaluator evaluator(config);
        
        std::cout << "Starting evaluation..." << std::endl;
        bool success = evaluator.runEvaluation();
        
        if (success) {
            std::cout << std::endl;
            std::cout << "===== EVALUATION COMPLETED SUCCESSFULLY =====" << std::endl;
            std::cout << "Results have been saved to: " << config.output_directory << std::endl;
            std::cout << "Check the generated CSV and statistics files for detailed analysis." << std::endl;
            return 0;
        } else {
            std::cerr << "Evaluation failed!" << std::endl;
            return 1;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [robot_urdf] [environment_xml] [scan_poses_csv] [output_dir]" << std::endl;
    std::cout << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  robot_urdf      Path to robot URDF file (default: ../../../res/robot/panda.urdf)" << std::endl;
    std::cout << "  environment_xml Path to environment XML file (default: ../../../res/scenario_1/obstacles.xml)" << std::endl;
    std::cout << "  scan_poses_csv  Path to scan poses CSV file (default: ./sample_scan_poses.csv)" << std::endl;
    std::cout << "  output_dir      Output directory for results (default: ./evaluation_results)" << std::endl;
    std::cout << std::endl;
    std::cout << "CSV Format:" << std::endl;
    std::cout << "  Each line: x,y,z,qw,qx,qy,qz (position and quaternion orientation)" << std::endl;
    std::cout << "  Example: 0.5,0.2,0.6,1.0,0.0,0.0,0.0" << std::endl;
}
