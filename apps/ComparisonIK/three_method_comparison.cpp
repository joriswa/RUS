#include <iostream>
#include <iomanip>
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>

#include "TrajectoryLib/Robot/RobotArm.h"
#include "USLib/USTrajectoryPlanner.h"
#include "core/newton_raphson_ik.h"

// =============================================================================
// SIMULATED ANNEALING PARAMETERS - OPTIMIZED FOR REAL POSES
// =============================================================================
// These parameters were optimized using real ultrasound scan poses from 
// scenario_1/scan_poses.csv and achieved 77.27% success rate with 97.55ms 
// average execution time across 22 real poses.
// 
// Parameter optimization results:
// - Tested 243 parameter combinations
// - Found 9 Pareto optimal solutions
// - Best overall: T_max=10.0, T_min=1.0, alpha=0.9, iter=1000, no_imp=10
// =============================================================================

struct IKResult {
    std::string method_name;
    bool success;
    double solve_time_ms;
    double position_error;
    double orientation_error;
    int iterations;
    Eigen::Matrix<double, 7, 1> joint_angles;
    bool collision_free;
    double min_clearance;
    double avg_clearance;
    int num_links_checked;
};

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
            const Eigen::Vector3d local_move(0.0, 0.0, -0.02);
            pose.translation() += pose.rotation() * local_move;
            
            poses.push_back(pose);
        }
    }

    return poses;
}

IKResult testNewtonRaphson(NewtonRaphsonIK& solver, const Eigen::Affine3d& target_pose, 
                          const Eigen::Matrix<double, 7, 1>& initial_config, UltrasoundScanTrajectoryPlanner& planner, 
                          RobotArm& robot) {
    IKResult result;
    result.method_name = "Newton-Raphson";
    
    auto nr_result = solver.solve(target_pose, initial_config);
    result.success = nr_result.success;
    result.solve_time_ms = nr_result.solve_time;
    result.position_error = nr_result.position_error;
    result.orientation_error = nr_result.orientation_error;
    result.iterations = nr_result.iterations;
    result.joint_angles = nr_result.joint_angles;
    
    // Check for collisions and compute clearance if solution was successful
    if (result.success) {
        robot.setJointAngles(result.joint_angles);
        result.collision_free = !planner.getPathPlanner()->armHasCollision(robot);
        
        // Compute clearance metrics (excluding first two base links and end effector)
        auto clearance_metrics = planner.getPathPlanner()->computeArmClearance(robot);
        result.min_clearance = clearance_metrics.min_clearance;
        result.avg_clearance = clearance_metrics.avg_clearance;
        result.num_links_checked = clearance_metrics.num_links_checked;
    } else {
        result.collision_free = false;
        result.min_clearance = std::numeric_limits<double>::infinity();
        result.avg_clearance = std::numeric_limits<double>::infinity();
        result.num_links_checked = 0;
    }
    
    return result;
}

IKResult testSimulatedAnnealingGoalPose(UltrasoundScanTrajectoryPlanner& planner, 
                                       RobotArm& robot, 
                                       const Eigen::Affine3d& target_pose) {
    IKResult result;
    result.method_name = "SA-Optimized";
    result.iterations = -1; // Not applicable for this implementation
    
    // Use optimized SA parameters from real pose analysis
    // Best overall performance: 77.27% success rate, 97.55ms execution time
    const double T_max = 10.0;
    const double T_min = 1.0;
    const double alpha = 0.9;
    const int max_iterations = 10000;
    const int max_no_improvement = 500;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    auto sa_result = planner.getPathPlanner()->selectGoalPoseSimulatedAnnealing(
        target_pose, T_max, T_min, alpha, max_iterations, max_no_improvement);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.solve_time_ms = duration.count() / 1000.0;
    
    result.success = sa_result.second;
    
    if (result.success) {
        result.joint_angles = sa_result.first.getJointAngles();
        result.collision_free = !planner.getPathPlanner()->armHasCollision(sa_result.first);
        
        // Compute clearance metrics (excluding first two base links and end effector)
        auto clearance_metrics = planner.getPathPlanner()->computeArmClearance(sa_result.first);
        result.min_clearance = clearance_metrics.min_clearance;
        result.avg_clearance = clearance_metrics.avg_clearance;
        result.num_links_checked = clearance_metrics.num_links_checked;
        
        // Calculate errors
        robot.setJointAngles(result.joint_angles);
        Eigen::Affine3d achieved_pose = robot.getEndeffectorPose();
        
        Eigen::Vector3d pos_diff = target_pose.translation() - achieved_pose.translation();
        result.position_error = pos_diff.norm();
        
        Eigen::Matrix3d rot_diff = target_pose.linear() * achieved_pose.linear().transpose();
        Eigen::AngleAxisd axis_angle(rot_diff);
        result.orientation_error = std::abs(axis_angle.angle());
    } else {
        result.collision_free = false;
        result.position_error = std::numeric_limits<double>::infinity();
        result.orientation_error = std::numeric_limits<double>::infinity();
        result.joint_angles.setZero();
        result.min_clearance = std::numeric_limits<double>::infinity();
        result.avg_clearance = std::numeric_limits<double>::infinity();
        result.num_links_checked = 0;
    }
    
    return result;
}

void printResult(const IKResult& result) {
    std::cout << std::setw(15) << result.method_name << " | ";
    std::cout << std::setw(7) << (result.success ? "SUCCESS" : "FAIL") << " | ";
    std::cout << std::setw(10) << std::fixed << std::setprecision(2) << result.solve_time_ms << " ms | ";
    std::cout << std::setw(12) << std::scientific << std::setprecision(2) << result.position_error << " | ";
    std::cout << std::setw(12) << std::scientific << std::setprecision(2) << result.orientation_error << " | ";
    std::cout << std::setw(12) << (result.collision_free ? "COLLISION-FREE" : "COLLISION") << " | ";
    
    // Display clearance information for successful solutions
    if (result.success && result.collision_free) {
        std::cout << std::setw(8) << std::fixed << std::setprecision(4) << result.min_clearance << " | ";
        std::cout << std::setw(8) << std::fixed << std::setprecision(4) << result.avg_clearance << " | ";
    } else {
        std::cout << std::setw(8) << "N/A" << " | ";
        std::cout << std::setw(8) << "N/A" << " | ";
    }
    
    if (result.iterations >= 0) {
        std::cout << std::setw(4) << result.iterations << " iter";
    } else {
        std::cout << std::setw(7) << "N/A";
    }
    std::cout << std::endl;
}

void printHeader() {
    std::cout << "=================================================================================================================================" << std::endl;
    std::cout << "                                     INVERSE KINEMATICS METHODS COMPARISON" << std::endl;
    std::cout << "                           Two Core Methods: Newton-Raphson vs SA-Optimized (with Clearance Analysis)" << std::endl;
    std::cout << "=================================================================================================================================" << std::endl;
    std::cout << std::setw(15) << "Method" << " | ";
    std::cout << std::setw(7) << "Result" << " | ";
    std::cout << std::setw(13) << "Time" << " | ";
    std::cout << std::setw(12) << "Pos Error" << " | ";
    std::cout << std::setw(12) << "Ori Error" << " | ";
    std::cout << std::setw(12) << "Collision" << " | ";
    std::cout << std::setw(8) << "Min Clr" << " | ";
    std::cout << std::setw(8) << "Avg Clr" << " | ";
    std::cout << "Details" << std::endl;
    std::cout << "---------------------------------------------------------------------------------------------------------------------------------" << std::endl;
}

void printSummary(const std::vector<IKResult>& nr_results, 
                  const std::vector<IKResult>& sa_results) {
    int nr_successes = 0, sa_successes = 0;
    int nr_collision_free = 0, sa_collision_free = 0;
    double nr_total_time = 0, sa_total_time = 0;
    double nr_min_clearance_sum = 0, sa_min_clearance_sum = 0;
    double nr_avg_clearance_sum = 0, sa_avg_clearance_sum = 0;
    int nr_clearance_count = 0, sa_clearance_count = 0;
    
    for (const auto& result : nr_results) {
        if (result.success) nr_successes++;
        if (result.collision_free) {
            nr_collision_free++;
            if (result.min_clearance != std::numeric_limits<double>::infinity()) {
                nr_min_clearance_sum += result.min_clearance;
                nr_avg_clearance_sum += result.avg_clearance;
                nr_clearance_count++;
            }
        }
        nr_total_time += result.solve_time_ms;
    }
    
    for (const auto& result : sa_results) {
        if (result.success) sa_successes++;
        if (result.collision_free) {
            sa_collision_free++;
            if (result.min_clearance != std::numeric_limits<double>::infinity()) {
                sa_min_clearance_sum += result.min_clearance;
                sa_avg_clearance_sum += result.avg_clearance;
                sa_clearance_count++;
            }
        }
        sa_total_time += result.solve_time_ms;
    }
    
    int num_tests = nr_results.size();
    
    std::cout << "\n=================================================================================================================================" << std::endl;
    std::cout << "                                               SUMMARY STATISTICS" << std::endl;
    std::cout << "=================================================================================================================================" << std::endl;
    
    std::cout << std::setw(15) << "Method" << " | ";
    std::cout << std::setw(12) << "Success Rate" << " | ";
    std::cout << std::setw(15) << "Collision-Free" << " | ";
    std::cout << std::setw(15) << "Avg Time (ms)" << " | ";
    std::cout << std::setw(12) << "Avg Min Clr" << " | ";
    std::cout << std::setw(12) << "Avg Avg Clr" << std::endl;
    std::cout << "---------------------------------------------------------------------------------------------------------------------------------" << std::endl;
    
    std::cout << std::setw(15) << "Newton-Raphson" << " | ";
    std::cout << std::setw(10) << std::fixed << std::setprecision(1) << (100.0 * nr_successes / num_tests) << "%" << " | ";
    std::cout << std::setw(13) << std::fixed << std::setprecision(1) << (100.0 * nr_collision_free / num_tests) << "%" << " | ";
    std::cout << std::setw(13) << std::fixed << std::setprecision(2) << (nr_total_time / num_tests) << " | ";
    if (nr_clearance_count > 0) {
        std::cout << std::setw(10) << std::fixed << std::setprecision(4) << (nr_min_clearance_sum / nr_clearance_count) << " | ";
        std::cout << std::setw(10) << std::fixed << std::setprecision(4) << (nr_avg_clearance_sum / nr_clearance_count) << std::endl;
    } else {
        std::cout << std::setw(10) << "N/A" << " | ";
        std::cout << std::setw(10) << "N/A" << std::endl;
    }
    
    std::cout << std::setw(15) << "SA-Optimized" << " | ";
    std::cout << std::setw(10) << std::fixed << std::setprecision(1) << (100.0 * sa_successes / num_tests) << "%" << " | ";
    std::cout << std::setw(13) << std::fixed << std::setprecision(1) << (100.0 * sa_collision_free / num_tests) << "%" << " | ";
    std::cout << std::setw(13) << std::fixed << std::setprecision(2) << (sa_total_time / num_tests) << " | ";
    if (sa_clearance_count > 0) {
        std::cout << std::setw(10) << std::fixed << std::setprecision(4) << (sa_min_clearance_sum / sa_clearance_count) << " | ";
        std::cout << std::setw(10) << std::fixed << std::setprecision(4) << (sa_avg_clearance_sum / sa_clearance_count) << std::endl;
    } else {
        std::cout << std::setw(10) << "N/A" << " | ";
        std::cout << std::setw(10) << "N/A" << std::endl;
    }
    
    std::cout << "\nKey Findings:" << std::endl;
    std::cout << "- Newton-Raphson: " << (100.0 * nr_successes / num_tests) << "% success rate, " 
              << (100.0 * nr_collision_free / num_tests) << "% collision-free" << std::endl;
    std::cout << "- SA-Optimized: " << (100.0 * sa_successes / num_tests) << "% success rate, " 
              << (100.0 * sa_collision_free / num_tests) << "% collision-free" << std::endl;
    
    // Clearance comparisons
    if (nr_clearance_count > 0 && sa_clearance_count > 0) {
        double nr_avg_min_clearance = nr_min_clearance_sum / nr_clearance_count;
        double sa_avg_min_clearance = sa_min_clearance_sum / sa_clearance_count;
        double nr_avg_avg_clearance = nr_avg_clearance_sum / nr_clearance_count;
        double sa_avg_avg_clearance = sa_avg_clearance_sum / sa_clearance_count;
        
        std::cout << "- Newton-Raphson clearance: min=" << std::fixed << std::setprecision(4) << nr_avg_min_clearance 
                  << " m, avg=" << nr_avg_avg_clearance << " m" << std::endl;
        std::cout << "- SA-Optimized clearance: min=" << std::fixed << std::setprecision(4) << sa_avg_min_clearance 
                  << " m, avg=" << sa_avg_avg_clearance << " m" << std::endl;
    }
    
    // Performance comparisons
    if (nr_successes > 0 && sa_successes > 0) {
        double speedup_nr_vs_sa = (sa_total_time / sa_successes) / (nr_total_time / nr_successes);
        std::cout << "- Newton-Raphson is " << std::fixed << std::setprecision(1) << speedup_nr_vs_sa 
                  << "x faster than SA-Optimized (on successful cases)" << std::endl;
    }
}

int main() {
    std::string urdf_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf";
    std::string env_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/obstacles.xml";
    std::string poses_csv = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/scan_poses.csv";
    
    try {
        // Initialize robot and planners
        RobotArm robot(urdf_path);
        UltrasoundScanTrajectoryPlanner us_planner(urdf_path);
        us_planner.setEnvironment(env_path);
        
        // Load poses from CSV
        std::vector<Eigen::Affine3d> target_poses = readPosesFromCSV(poses_csv);
        
        if (target_poses.empty()) {
            std::cerr << "Error: No poses loaded from " << poses_csv << std::endl;
            return 1;
        }
        
        std::cout << "Loaded " << target_poses.size() << " poses from CSV file" << std::endl;
        
        printHeader();
        
        // Initialize unified IK solver
        NewtonRaphsonIK ik_solver(robot);
        
        // Storage for results
        std::vector<IKResult> nr_results, sa_results;
        
        // Test each pose
        Eigen::Matrix<double, 7, 1> home_config = Eigen::Matrix<double, 7, 1>::Zero();
        
        for (size_t i = 0; i < target_poses.size(); ++i) {
            std::cout << "\n--- Pose " << (i + 1) << "/" << target_poses.size() << " ---" << std::endl;
            
            const auto& target_pose = target_poses[i];
            
            // Test Newton-Raphson method
            ik_solver.setUseDampedLeastSquares(false);  // Use Newton-Raphson
            ik_solver.setPositionTolerance(1e-4);
            ik_solver.setOrientationTolerance(1e-3);
            ik_solver.setDampingFactor(0.001);  // Very small damping for numerical stability
            ik_solver.setAdaptiveDamping(true);  // Use adaptive damping near singularities
            
            robot.setJointAngles(home_config);
            auto nr_result = testNewtonRaphson(ik_solver, target_pose, home_config, us_planner, robot);
            nr_results.push_back(nr_result);
            printResult(nr_result);
            
            // Test simulated annealing selectGoalPose
            robot.setJointAngles(home_config);
            auto sa_result = testSimulatedAnnealingGoalPose(us_planner, robot, target_pose);
            sa_results.push_back(sa_result);
            printResult(sa_result);
        }
        
        // Print summary statistics
        printSummary(nr_results, sa_results);
        
        // Save results to CSV for further analysis
        std::ofstream results_file("two_method_comparison_results.csv");
        results_file << "pose_id,method,success,time_ms,pos_error,ori_error,iterations,collision_free,min_clearance,avg_clearance,num_links_checked\n";
        
        for (size_t i = 0; i < target_poses.size(); ++i) {
            results_file << i << ",Newton-Raphson," << nr_results[i].success << "," 
                        << nr_results[i].solve_time_ms << "," << nr_results[i].position_error << "," 
                        << nr_results[i].orientation_error << "," << nr_results[i].iterations << ","
                        << nr_results[i].collision_free << "," << nr_results[i].min_clearance << ","
                        << nr_results[i].avg_clearance << "," << nr_results[i].num_links_checked << "\n";
            
            results_file << i << ",SA-Optimized," << sa_results[i].success << "," 
                        << sa_results[i].solve_time_ms << "," << sa_results[i].position_error << "," 
                        << sa_results[i].orientation_error << "," << sa_results[i].iterations << ","
                        << sa_results[i].collision_free << "," << sa_results[i].min_clearance << ","
                        << sa_results[i].avg_clearance << "," << sa_results[i].num_links_checked << "\n";
        }
        
        std::cout << "\nResults saved to: two_method_comparison_results.csv" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}