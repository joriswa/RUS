#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <sstream>
#include <array>
#include <cmath>
#include <Eigen/Dense>
#include "USLib/USTrajectoryPlanner.h"

/**
 * @brief Cost data generator using the underlying selectGoalPose cost function
 * 
 * This generator uses the evaluateSelectGoalPoseCost function to directly access
 * the same cost function that selectGoalPose uses internally. This shows the
 * raw cost function values across different q7 values and poses.
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <sstream>
#include <array>
#include <cmath>
#include <Eigen/Dense>
#include "USLib/USTrajectoryPlanner.h"

/**
 * @brief Evaluate cost for a specific pose and q7 value using PathPlanner's evaluateSelectGoalPoseCost method
 * 
 * This function uses the newly implemented evaluateSelectGoalPoseCost method to access the EXACT same cost
 * function that selectGoalPoseSimulatedAnnealing uses internally. This gives us direct access to the real
 * optimization landscape used by the PathPlanner.
 */
std::pair<double, bool> evaluatePoseCost(const Eigen::Affine3d &pose, double q7, PathPlanner* pathPlanner) {
    return pathPlanner->evaluateSelectGoalPoseCost(pose, q7);
}

std::vector<Eigen::Affine3d> readPosesFromCSV(const std::string& filename) {
    std::vector<Eigen::Affine3d> poses;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        // Skip empty lines and comment lines
        if (line.empty() || line[0] == '#' || line[0] == '/' || line.find("//") != std::string::npos) {
            continue;
        }

        std::vector<double> values;
        std::stringstream ss(line);
        std::string token;

        while (std::getline(ss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing line: " << line << std::endl;
                break;
            }
        }

        if (values.size() >= 7) {  // px, py, pz, qw, qx, qy, qz
            Eigen::Affine3d pose = Eigen::Affine3d::Identity();
            
            // Position
            pose.translation() = Eigen::Vector3d(values[0], values[1], values[2]);
            
            // Orientation (quaternion: w, x, y, z)
            Eigen::Quaterniond q(values[3], values[4], values[5], values[6]);
            q.normalize();
            pose.linear() = q.toRotationMatrix();
            
            poses.push_back(pose);
        }
    }

    return poses;
}

int main() {
    std::cout << "=== SelectGoalPose Underlying Cost Function Generator ===" << std::endl;
    std::cout << "Using evaluateSelectGoalPoseCost to access the raw cost function" << std::endl;
    std::cout << "Same setup as three_method_comparison.cpp" << std::endl;
    
    // Setup - same paths as three_method_comparison and real_poses_cost_generator
    std::string urdf_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf";
    std::string env_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/obstacles.xml";
    std::string csv_file = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/scan_poses.csv";
    
    std::cout << "URDF: " << urdf_path << std::endl;
    std::cout << "Environment: " << env_path << std::endl;
    std::cout << "Poses: " << csv_file << std::endl;
    
    // Initialize planner
    UltrasoundScanTrajectoryPlanner planner(urdf_path);
    planner.setEnvironment(env_path);
    
    // Set initial joint configuration
    Eigen::VectorXd current_joints(7);
    current_joints << 0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.7854;
    planner.setCurrentJoints(current_joints);
    
    // Get PathPlanner instance for direct cost evaluation
    auto pathPlanner = planner.getPathPlanner();
    
    // Load poses from CSV
    std::vector<Eigen::Affine3d> poses = readPosesFromCSV(csv_file);
    
    if (poses.empty()) {
        std::cerr << "No poses loaded from CSV file!" << std::endl;
        return 1;
    }
    
    std::cout << "Loaded " << poses.size() << " real scan poses" << std::endl;
    
    // Apply the same -2cm local z-axis transformation as three_method_comparison.cpp
    for (auto& pose : poses) {
        Eigen::Vector3d local_translation(0, 0, -0.02);  // -2cm in local z-axis
        Eigen::Vector3d global_translation = pose.linear() * local_translation;
        
        Eigen::Affine3d adjusted_pose = pose;
        adjusted_pose.translation() = pose.translation() + global_translation;
        pose = adjusted_pose;
    }
    
    // Generate q7 values with reasonable resolution for PathPlanner-based evaluation
    // Using fewer samples since each evaluation calls the full selectGoalPose method
    const int num_q7_samples = 100;  // Reduced for PathPlanner-based approach
    const double q7_min = -2.8973;  // Joint 7 limit
    const double q7_max = 2.8973;   // Joint 7 limit
    std::vector<double> q7_values;
    
    // Use denser sampling around common q7 values (-π to π range)
    for (int i = 0; i < num_q7_samples; i++) {
        double q7 = q7_min + (q7_max - q7_min) * i / (num_q7_samples - 1);
        q7_values.push_back(q7);
    }
    
    std::cout << "Generating underlying cost function data for " << poses.size() 
              << " poses with " << num_q7_samples << " q7 samples each..." << std::endl;
    
    // Open output file
    std::ofstream outFile("underlying_cost_function_data.csv");
    outFile << "pose_id,pose_name,px,py,pz,qw,qx,qy,qz,q7,cost,success" << std::endl;
    
    int total_evaluations = 0;
    int successful_evaluations = 0;
    
    for (size_t poseIdx = 0; poseIdx < poses.size(); ++poseIdx) {
        std::cout << "Processing ScanPose_" << (poseIdx + 1) << " (" << (poseIdx + 1) << "/" << poses.size() << ")" << std::endl;
        
        const auto& pose = poses[poseIdx];
        
        // Extract pose components for CSV
        Eigen::Vector3d position = pose.translation();
        Eigen::Quaterniond quaternion(pose.linear());
        
        for (double q7 : q7_values) {
            total_evaluations++;
            
            // Use our custom cost evaluation function that mimics selectGoalPose
            auto [cost, success] = evaluatePoseCost(pose, q7, pathPlanner);
            
            if (success) {
                successful_evaluations++;
            }
            
            // Write to CSV
            outFile << poseIdx << ",";
            outFile << "ScanPose_" << (poseIdx + 1) << ",";
            outFile << std::fixed << std::setprecision(6);
            outFile << position.x() << "," << position.y() << "," << position.z() << ",";
            outFile << quaternion.w() << "," << quaternion.x() << "," << quaternion.y() << "," << quaternion.z() << ",";
            outFile << q7 << ",";
            
            if (success) {
                outFile << cost;
            } else {
                outFile << "inf";
            }
            
            outFile << "," << (success ? 1 : 0) << std::endl;
        }
    }
    
    outFile.close();
    
    std::cout << std::endl;
    std::cout << "=== Generation Complete ===" << std::endl;
    std::cout << "Total evaluations: " << total_evaluations << std::endl;
    std::cout << "Successful evaluations: " << successful_evaluations 
              << " (" << std::fixed << std::setprecision(1) 
              << (100.0 * successful_evaluations / total_evaluations) << "%)" << std::endl;
    std::cout << "Results saved to: underlying_cost_function_data.csv" << std::endl;
    std::cout << std::endl;
    std::cout << "This data shows the RAW cost function values that selectGoalPose" << std::endl;
    std::cout << "uses internally for optimization. Lower values = better solutions." << std::endl;
    
    return 0;
}
