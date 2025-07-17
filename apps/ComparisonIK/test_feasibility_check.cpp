#include "core/constraint_projected_newton_ik.h"
#include "USLib/USTrajectoryPlanner.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <random>

// Same pose reading function
std::vector<Eigen::Affine3d> readPosesFromCSV(const std::string& filename) {
    std::vector<Eigen::Affine3d> poses;
    std::ifstream file(filename);
    std::string line;
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open poses file: " << filename << std::endl;
        return poses;
    }
    
    // Skip header line
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;
        
        while (std::getline(ss, token, ',')) {
            values.push_back(std::stod(token));
        }

        if (values.size() >= 7) {
            Eigen::Vector3d position(values[0], values[1], values[2]);
            Eigen::Quaterniond quaternion(values[3], values[4], values[5], values[6]);
            quaternion.normalize();

            Eigen::Affine3d pose = Eigen::Affine3d::Identity();
            pose.linear() = quaternion.toRotationMatrix();
            pose.translation() = position;
            
            const Eigen::Vector3d local_move(0.0, 0.0, 0.0);
            pose.translation() += pose.rotation() * local_move;
            
            poses.push_back(pose);
        }
    }

    return poses;
}

int main() {
    std::cout << "=== Collision-Free Feasibility Check ===" << std::endl;
    
    try {
        // Initialize robot and planners
        std::string urdf_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf";
        std::string env_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/obstacles.xml";
        std::string poses_csv = "/Users/joris/Downloads/07_09_2/ext_post_rotated.csv";
        
        RobotArm robot(urdf_path);
        UltrasoundScanTrajectoryPlanner us_planner(urdf_path);
        us_planner.setEnvironment(env_path);
        
        // Load poses
        std::vector<Eigen::Affine3d> target_poses = readPosesFromCSV(poses_csv);
        std::cout << "Loaded " << target_poses.size() << " poses" << std::endl;
        
        // Random number generator for joint configurations
        std::random_device rd;
        std::mt19937 gen(rd());
        
        // Franka joint limits
        const double q_min[7] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
        const double q_max[7] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
        
        std::vector<std::uniform_real_distribution<double>> joint_distributions;
        for (int i = 0; i < 7; i++) {
            joint_distributions.emplace_back(q_min[i], q_max[i]);
        }
        
        // Test first 10 poses to see if ANY collision-free solutions exist
        const int num_poses_to_test = std::min(10, static_cast<int>(target_poses.size()));
        const int samples_per_pose = 10000; // Test many random configurations
        
        std::cout << "Testing " << num_poses_to_test << " poses with " << samples_per_pose << " random samples each..." << std::endl;
        
        for (int pose_idx = 0; pose_idx < num_poses_to_test; pose_idx++) {
            std::cout << "\\nPose " << (pose_idx + 1) << ": ";
            
            int collision_free_count = 0;
            int valid_ik_count = 0;
            double best_position_error = std::numeric_limits<double>::max();
            double best_orientation_error = std::numeric_limits<double>::max();
            
            for (int sample = 0; sample < samples_per_pose; sample++) {
                // Generate random joint configuration
                Eigen::Matrix<double, 7, 1> random_q;
                for (int j = 0; j < 7; j++) {
                    random_q(j) = joint_distributions[j](gen);
                }
                
                // Set robot configuration and check kinematics
                robot.setJointAngles(random_q);
                Eigen::Affine3d current_pose = robot.getEndeffectorPose();
                
                // Compute kinematic error
                Eigen::Vector3d pos_error = target_poses[pose_idx].translation() - current_pose.translation();
                Eigen::Matrix3d rot_error = target_poses[pose_idx].rotation() * current_pose.rotation().transpose();
                Eigen::AngleAxisd angle_axis(rot_error);
                double orientation_error = angle_axis.angle();
                double position_error = pos_error.norm();
                
                // Check if this is a valid IK solution (loose tolerance)
                if (position_error < 0.01 && orientation_error < 0.1) { // 1cm, ~5.7 degrees
                    valid_ik_count++;
                    best_position_error = std::min(best_position_error, position_error);
                    best_orientation_error = std::min(best_orientation_error, orientation_error);
                    
                    // Check if collision-free
                    if (!us_planner.getPathPlanner()->armHasCollision(robot)) {
                        collision_free_count++;
                    }
                }
                
                // Progress indicator
                if ((sample + 1) % 2000 == 0) {
                    std::cout << (sample + 1) << " ";
                    std::cout.flush();
                }
            }
            
            std::cout << std::endl;
            std::cout << "  Valid IK solutions: " << valid_ik_count << "/" << samples_per_pose;
            if (valid_ik_count > 0) {
                std::cout << " (best errors: " << best_position_error*1000 << "mm, " 
                          << best_orientation_error*180/M_PI << "°)";
            }
            std::cout << std::endl;
            std::cout << "  Collision-free solutions: " << collision_free_count << "/" << samples_per_pose;
            if (collision_free_count > 0) {
                std::cout << " ✅ FEASIBLE";
            } else if (valid_ik_count > 0) {
                std::cout << " ⚠️  CONSTRAINED (IK possible but collides)";
            } else {
                std::cout << " ❌ IMPOSSIBLE (no valid IK found)";
            }
            std::cout << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
