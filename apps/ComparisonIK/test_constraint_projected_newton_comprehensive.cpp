#include "core/constraint_projected_newton_ik.h"
#include "core/newton_raphson_ik.h"
#include "core/grid_search_ik.h"
#include "TrajectoryLib/Robot/franka_ik_He.h"
#include "USLib/USTrajectoryPlanner.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>
#include <cstdlib>   // For rand()
#include <ctime>     // For srand()

// Pose reading function (same as three_method_comparison.cpp)
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
            
            // Apply -2cm local z-axis transformation (same as mainwindow.cpp)
            const Eigen::Vector3d local_move(0.0, 0.0, 0.0);
            pose.translation() += pose.rotation() * local_move;
            
            poses.push_back(pose);
        }
    }

    return poses;
}

struct ComprehensiveIKResult {
    std::string method_name;
    bool success;
    double solve_time_ms;
    double position_error;
    double orientation_error;
    double final_cost;
    int iterations;
    Eigen::Matrix<double, 7, 1> joint_angles;
    bool collision_free;
    double min_clearance;
    double avg_clearance;
    double min_joint_limit_distance;
    double avg_joint_limit_distance;
};

ComprehensiveIKResult testConstraintProjectedNewtonMultiStart(ConstraintProjectedNewtonIK& solver, 
                                                           const Eigen::Affine3d& target_pose, 
                                                           UltrasoundScanTrajectoryPlanner& planner, 
                                                           RobotArm& robot, int pose_id) {
    const int num_attempts = 50; // Try MANY more random initializations for thorough testing
    std::vector<ComprehensiveIKResult> attempts;
    
    // Get joint limits for smart initialization
    auto joint_limits = robot.jointLimits();
    
    for (int attempt = 0; attempt < num_attempts; attempt++) {
        // Generate smart random initialization (middle 80% of joint range)
        Eigen::Matrix<double, 7, 1> initial_config;
        for (int j = 0; j < 7; j++) {
            double range = joint_limits[j].second - joint_limits[j].first;
            double center = (joint_limits[j].first + joint_limits[j].second) / 2.0;
            double random_offset = (static_cast<double>(rand()) / RAND_MAX - 0.5) * 0.8 * range;
            initial_config(j) = center + random_offset;
        }
        
        // Try this initialization
        ComprehensiveIKResult result;
        result.method_name = "ConstraintProjectedNewton";
        
        auto cpn_result = solver.solve(target_pose, initial_config);
        result.success = cpn_result.success;
        result.solve_time_ms = cpn_result.solve_time;
        result.iterations = cpn_result.iterations;
        result.joint_angles = cpn_result.joint_angles;
        
        // For collision avoidance mode, check the actual kinematic errors manually
        robot.setJointAngles(result.joint_angles);
        Eigen::Affine3d achieved_pose = robot.getEndeffectorPose();
        
        // Compute actual pose errors
        Eigen::Vector3d pos_error = target_pose.translation() - achieved_pose.translation();
        result.position_error = pos_error.norm();
        
        // Orientation error using axis-angle
        Eigen::Matrix3d rot_error = target_pose.rotation() * achieved_pose.rotation().transpose();
        Eigen::AngleAxisd angle_axis(rot_error);
        result.orientation_error = angle_axis.angle();
        
        // Set final cost
        result.final_cost = cpn_result.final_cost;
        
        // Check for collisions and compute clearance
        result.collision_free = !planner.getPathPlanner()->armHasCollision(robot);
        
        if (result.collision_free) {
            // Compute clearance metrics
            auto clearance_metrics = planner.getPathPlanner()->computeArmClearance(robot);
            result.min_clearance = clearance_metrics.min_clearance;
            result.avg_clearance = clearance_metrics.avg_clearance;
        } else {
            result.min_clearance = std::numeric_limits<double>::infinity();
            result.avg_clearance = std::numeric_limits<double>::infinity();
        }
        
        // Calculate joint limit distances
        const double q_min[7] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
        const double q_max[7] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
        
        double min_distance = std::numeric_limits<double>::max();
        double total_distance = 0.0;
        
        for (int j = 0; j < 7; j++) {
            double dist_to_lower = result.joint_angles(j) - q_min[j];
            double dist_to_upper = q_max[j] - result.joint_angles(j);
            double joint_min_distance = std::min(dist_to_lower, dist_to_upper);
            min_distance = std::min(min_distance, joint_min_distance);
            total_distance += joint_min_distance;
        }
        
        result.min_joint_limit_distance = min_distance;
        result.avg_joint_limit_distance = total_distance / 7.0;
        
        attempts.push_back(result);
        
        // If we found a success, return it immediately
        if (result.success) {
            return result;
        }
    }
    
    // No successes, return the best attempt (lowest combined error)
    ComprehensiveIKResult best_result = attempts[0];
    double best_error = best_result.position_error + 0.1 * best_result.orientation_error;
    
    for (const auto& attempt : attempts) {
        double error = attempt.position_error + 0.1 * attempt.orientation_error;
        if (error < best_error) {
            best_error = error;
            best_result = attempt;
        }
    }
    
    return best_result;
}

void printComprehensiveResult(const ComprehensiveIKResult& result, int pose_id) {
    std::cout << std::setw(5) << pose_id << " | ";
    std::cout << std::setw(7) << (result.success ? "SUCCESS" : "FAIL") << " | ";
    std::cout << std::setw(10) << std::fixed << std::setprecision(2) << result.solve_time_ms << " | ";
    std::cout << std::setw(12) << std::scientific << std::setprecision(2) << result.position_error << " | ";
    std::cout << std::setw(12) << std::scientific << std::setprecision(2) << result.orientation_error << " | ";
    std::cout << std::setw(10) << std::scientific << std::setprecision(2) << result.final_cost << " | ";
    std::cout << std::setw(8) << result.iterations << " | ";
    std::cout << std::setw(12) << (result.collision_free ? "FREE" : "COLLISION") << " | ";
    
    if (result.success && result.collision_free) {
        std::cout << std::setw(8) << std::fixed << std::setprecision(4) << result.min_clearance << " | ";
        std::cout << std::setw(8) << std::fixed << std::setprecision(4) << result.avg_clearance << " | ";
        std::cout << std::setw(8) << std::fixed << std::setprecision(4) << result.min_joint_limit_distance << " | ";
        std::cout << std::setw(8) << std::fixed << std::setprecision(4) << result.avg_joint_limit_distance;
    } else {
        std::cout << std::setw(8) << "N/A" << " | ";
        std::cout << std::setw(8) << "N/A" << " | ";
        std::cout << std::setw(8) << "N/A" << " | ";
        std::cout << std::setw(8) << "N/A";
    }
    std::cout << std::endl;
}

void printComprehensiveHeader() {
    std::cout << "=========================================================================================================================================================" << std::endl;
    std::cout << "                                     CONSTRAINT PROJECTED NEWTON'S METHOD - COMPREHENSIVE EVALUATION" << std::endl;
    std::cout << "                                    Testing with Same Poses as Three Methods Comparison" << std::endl;
    std::cout << "=========================================================================================================================================================" << std::endl;
    std::cout << std::setw(5) << "Pose" << " | ";
    std::cout << std::setw(7) << "Result" << " | ";
    std::cout << std::setw(10) << "Time(ms)" << " | ";
    std::cout << std::setw(12) << "Pos Error" << " | ";
    std::cout << std::setw(12) << "Ori Error" << " | ";
    std::cout << std::setw(10) << "Cost" << " | ";
    std::cout << std::setw(8) << "Iters" << " | ";
    std::cout << std::setw(12) << "Collision" << " | ";
    std::cout << std::setw(8) << "MinClr" << " | ";
    std::cout << std::setw(8) << "AvgClr" << " | ";
    std::cout << std::setw(8) << "MinJLim" << " | ";
    std::cout << "AvgJLim" << std::endl;
    std::cout << "---------------------------------------------------------------------------------------------------------------------------------------------------------" << std::endl;
}

int main() {
    // Seed random number generator for multiple initialization attempts
    srand(static_cast<unsigned>(time(nullptr)));
    
    std::cout << "=== Constraint Projected Newton's Method - Comprehensive Test ===" << std::endl;
    
    try {
        // URDF and environment paths (same as three_method_comparison.cpp)
        std::string urdf_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf";
        std::string env_path = "/Users/joris/Downloads/07_09_2/obstacles.xml";
        std::string poses_csv = "/Users/joris/Downloads/07_09_2/ext_post_rotated.csv";
        
        // Initialize robot and planners
        RobotArm robot(urdf_path);
        UltrasoundScanTrajectoryPlanner us_planner(urdf_path);
        us_planner.setEnvironment(env_path);
        
        // Create constraint projected Newton IK solver
        ConstraintProjectedNewtonIK cpn_ik(robot, us_planner.getPathPlanner());
        
        // Configure solver with reasonable tolerances + multiple random initialization
        cpn_ik.setPositionTolerance(5e-3);         // 5mm - reasonable for challenging poses
        cpn_ik.setOrientationTolerance(0.1);       // ~5.7 degrees - reasonable precision
        cpn_ik.setMaxIterations(5000);             // WAY MORE iterations for thorough testing
        cpn_ik.setLineSearchEnabled(true);         // Enable line search for step size control
        cpn_ik.setJointLimitMargin(0.0001);        // Very minimal margin
        cpn_ik.setDampingFactor(0.005);            // Lower damping for less regularization
        cpn_ik.setKinematicOnlyMode(true);          // Focus on pose accuracy first
        cpn_ik.setCollisionAvoidanceOnlyMode(false); // Test kinematic performance with better step control
        cpn_ik.setUseAdaptiveDamping(true);         // ENABLE ADAPTIVE DAMPING for testing
        
        // Load poses from CSV (same as three methods comparison)
        std::vector<Eigen::Affine3d> target_poses = readPosesFromCSV(poses_csv);
        
        if (target_poses.empty()) {
            std::cerr << "Error: No poses loaded from " << poses_csv << std::endl;
            return 1;
        }
        
        std::cout << "Loaded " << target_poses.size() << " poses from CSV" << std::endl;
        std::cout << "Testing Constraint Projected Newton's method with BVH collision detection..." << std::endl << std::endl;
        
        // Test parameters (same as three_method_comparison.cpp)
        const int max_poses_to_test = std::min(50, static_cast<int>(target_poses.size())); // Test first 50 poses
        Eigen::Matrix<double, 7, 1> initial_config = Eigen::Matrix<double, 7, 1>::Zero(); // Use home config like comparison
        
        // Results storage
        std::vector<ComprehensiveIKResult> cpn_results;
        
        printComprehensiveHeader();
        
        // Test each pose
        for (int i = 0; i < max_poses_to_test; ++i) {
            const auto& target_pose = target_poses[i];
            
            // Test constraint projected Newton's method with multiple random initializations
            auto cpn_result = testConstraintProjectedNewtonMultiStart(cpn_ik, target_pose, us_planner, robot, i+1);
            cpn_results.push_back(cpn_result);
            
            printComprehensiveResult(cpn_result, i+1);
            
            // Early termination if too many failures
            if (i >= 10) {
                int failures = 0;
                for (const auto& result : cpn_results) {
                    if (!result.success) failures++;
                }
                if (failures > i * 0.8) { // More than 80% failure rate
                    std::cout << "Early termination due to high failure rate." << std::endl;
                    break;
                }
            }
        }
        
        // Summary statistics
        int successes = 0, collision_free = 0;
        double total_time = 0, total_cost = 0;
        double min_clearance_sum = 0, avg_clearance_sum = 0;
        double min_joint_limit_sum = 0, avg_joint_limit_sum = 0;
        int clearance_count = 0, joint_limit_count = 0;
        double pos_error_sum = 0, ori_error_sum = 0;
        int error_count = 0;
        
        for (const auto& result : cpn_results) {
            if (result.success) {
                successes++;
                total_cost += result.final_cost;
                pos_error_sum += result.position_error;
                ori_error_sum += result.orientation_error;
                error_count++;
                
                if (result.collision_free) {
                    collision_free++;
                    if (result.min_clearance != std::numeric_limits<double>::infinity()) {
                        min_clearance_sum += result.min_clearance;
                        avg_clearance_sum += result.avg_clearance;
                        clearance_count++;
                    }
                    if (result.min_joint_limit_distance != std::numeric_limits<double>::infinity()) {
                        min_joint_limit_sum += result.min_joint_limit_distance;
                        avg_joint_limit_sum += result.avg_joint_limit_distance;
                        joint_limit_count++;
                    }
                }
            }
            total_time += result.solve_time_ms;
        }
        
        int num_tests = cpn_results.size();
        
        std::cout << std::endl << "=========================================================================================================================================================" << std::endl;
        std::cout << "                                               CONSTRAINT PROJECTED NEWTON - FINAL RESULTS" << std::endl;
        std::cout << "=========================================================================================================================================================" << std::endl;
        
        std::cout << "Test Summary:" << std::endl;
        std::cout << "- Total poses tested: " << num_tests << std::endl;
        std::cout << "- Success rate: " << std::fixed << std::setprecision(1) << (100.0 * successes / num_tests) << "% (" << successes << "/" << num_tests << ")" << std::endl;
        std::cout << "- Collision-free rate: " << std::fixed << std::setprecision(1) << (100.0 * collision_free / num_tests) << "% (" << collision_free << "/" << num_tests << ")" << std::endl;
        std::cout << "- Average solve time: " << std::fixed << std::setprecision(2) << (total_time / num_tests) << " ms" << std::endl;
        
        if (error_count > 0) {
            std::cout << std::endl << "Accuracy Analysis (Franka ±0.1mm spec):" << std::endl;
            double avg_pos_error = pos_error_sum / error_count;
            double avg_ori_error = ori_error_sum / error_count;
            std::cout << "- Average position error: " << std::scientific << std::setprecision(3) << avg_pos_error << " m (" << std::fixed << std::setprecision(2) << (avg_pos_error * 1000) << " mm)" << std::endl;
            std::cout << "- Average orientation error: " << std::scientific << std::setprecision(3) << avg_ori_error << " rad (" << std::fixed << std::setprecision(2) << (avg_ori_error * 180 / M_PI) << "°)" << std::endl;
            std::cout << "- Within Franka tolerance: " << (avg_pos_error < 0.0001 ? "✅ YES" : "❌ NO") << " (position), " << (avg_ori_error < 0.001 ? "✅ YES" : "❌ NO") << " (orientation)" << std::endl;
            std::cout << "- Average cost: " << std::scientific << std::setprecision(3) << (total_cost / successes) << std::endl;
        }
        
        if (clearance_count > 0) {
            std::cout << std::endl << "Collision Analysis:" << std::endl;
            std::cout << "- Average minimum clearance: " << std::fixed << std::setprecision(4) << (min_clearance_sum / clearance_count) << " m" << std::endl;
            std::cout << "- Average overall clearance: " << std::fixed << std::setprecision(4) << (avg_clearance_sum / clearance_count) << " m" << std::endl;
        }
        
        if (joint_limit_count > 0) {
            std::cout << std::endl << "Joint Limit Analysis:" << std::endl;
            double avg_min_joint_limit = min_joint_limit_sum / joint_limit_count;
            double avg_avg_joint_limit = avg_joint_limit_sum / joint_limit_count;
            std::cout << "- Average minimum joint limit distance: " << std::fixed << std::setprecision(4) << avg_min_joint_limit << " rad (" << std::setprecision(1) << (avg_min_joint_limit * 180.0 / M_PI) << "°)" << std::endl;
            std::cout << "- Average overall joint limit distance: " << std::fixed << std::setprecision(4) << avg_avg_joint_limit << " rad (" << std::setprecision(1) << (avg_avg_joint_limit * 180.0 / M_PI) << "°)" << std::endl;
            
            // Safety assessment
            double critical_threshold = 0.1;  // 0.1 rad ≈ 5.7°
            double warning_threshold = 0.2;   // 0.2 rad ≈ 11.5°
            
            if (avg_min_joint_limit < critical_threshold) {
                std::cout << "  ⚠️  WARNING: Solutions are in CRITICAL zone (<0.1 rad from joint limits)" << std::endl;
            } else if (avg_min_joint_limit < warning_threshold) {
                std::cout << "  ⚠️  CAUTION: Solutions are in WARNING zone (<0.2 rad from joint limits)" << std::endl;
            } else {
                std::cout << "  ✅ Solutions maintain safe distance from joint limits (≥0.2 rad)" << std::endl;
            }
        }
        
        std::cout << std::endl << "Method Assessment:" << std::endl;
        std::cout << "- Convergence: " << (successes > num_tests * 0.8 ? "✅ Excellent" : successes > num_tests * 0.6 ? "⚠️  Good" : "❌ Poor") << " success rate" << std::endl;
        std::cout << "- Speed: " << (total_time / num_tests < 10.0 ? "✅ Fast" : total_time / num_tests < 50.0 ? "⚠️  Moderate" : "❌ Slow") << " average solve time" << std::endl;
        std::cout << "- Safety: " << (collision_free > num_tests * 0.9 ? "✅ Excellent" : collision_free > num_tests * 0.7 ? "⚠️  Good" : "❌ Poor") << " collision avoidance" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << std::endl << "=== Comprehensive test completed ===" << std::endl;
    return 0;
}
