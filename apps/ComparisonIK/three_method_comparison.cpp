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
#include "core/grid_search_ik.h"
#include "core/ik_cost_functions.h"

// =============================================================================
// GRID SEARCH PARAMETERS - USING SA FUNCTION SIGNATURE
// =============================================================================
// These parameters control the grid-based search that replaces the original
// simulated annealing implementation while maintaining the same function
// signature for compatibility.
// 
// Parameter usage in grid search:
// - T_max: Controls grid resolution (higher = finer grid, more thorough)
// - max_iterations: Maximum evaluation budget for the search
// - Other parameters (T_min, alpha, max_no_improvement): Maintained for compatibility
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
    double min_joint_limit_distance;  // Minimum distance to any joint limit
    double avg_joint_limit_distance;  // Average distance to joint limits
    int pose_id = -1;                 // Pose identifier for analysis
    int run_id = -1;                  // Run identifier for analysis
};

// Function to calculate distance to joint limits (now using shared utilities)
std::pair<double, double> calculateJointLimitDistances(const Eigen::Matrix<double, 7, 1>& joint_angles) {
    return IKCostFunctions::computeJointLimitDistances(joint_angles);
}

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

IKResult testNewtonRaphson(NewtonRaphsonIK& solver, const Eigen::Affine3d& target_pose, 
                          const Eigen::Matrix<double, 7, 1>& initial_config, UltrasoundScanTrajectoryPlanner& planner, 
                          RobotArm& robot) {
    IKResult result;
    result.method_name = "Method1";
    
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
        
        // Calculate joint limit distances
        auto joint_limit_distances = calculateJointLimitDistances(result.joint_angles);
        result.min_joint_limit_distance = joint_limit_distances.first;
        result.avg_joint_limit_distance = joint_limit_distances.second;
    } else {
        result.collision_free = false;
        result.min_clearance = std::numeric_limits<double>::infinity();
        result.avg_clearance = std::numeric_limits<double>::infinity();
        result.num_links_checked = 0;
        result.min_joint_limit_distance = std::numeric_limits<double>::infinity();
        result.avg_joint_limit_distance = std::numeric_limits<double>::infinity();
    }
    
    return result;
}

IKResult testSimulatedAnnealingGoalPose(UltrasoundScanTrajectoryPlanner& planner, 
                                       RobotArm& robot, 
                                       const Eigen::Affine3d& target_pose) {
    IKResult result;
    result.method_name = "Method2";
    result.iterations = -1; // Not applicable for this implementation
    
    // Use parameters to control grid search behavior
    // T_max controls grid resolution, max_iterations controls evaluation budget
    const double T_max = 10.0;           // Grid resolution parameter (higher = finer grid)
    const double T_min = 1.0;            // Unused in grid approach (kept for compatibility)
    const double alpha = 0.9;            // Unused in grid approach (kept for compatibility)
    const int max_iterations = 10000;    // Maximum evaluations budget
    const int max_no_improvement = 500;  // Unused in grid approach (kept for compatibility)
    
    auto start_time = std::chrono::high_resolution_clock::now();
    // Note: Function name is selectGoalPoseSimulatedAnnealing but now implements grid search
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
        
        // Calculate joint limit distances
        auto joint_limit_distances = calculateJointLimitDistances(result.joint_angles);
        result.min_joint_limit_distance = joint_limit_distances.first;
        result.avg_joint_limit_distance = joint_limit_distances.second;
        
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
        result.min_joint_limit_distance = std::numeric_limits<double>::infinity();
        result.avg_joint_limit_distance = std::numeric_limits<double>::infinity();
    }
    
    return result;
}

IKResult testGridSearch(GridSearchIK& solver, const Eigen::Affine3d& target_pose, 
                      const Eigen::Matrix<double, 7, 1>& initial_config, UltrasoundScanTrajectoryPlanner& planner, 
                      RobotArm& robot) {
    IKResult result;
    result.method_name = "Method3";
    
    auto gs_result = solver.solve(target_pose, initial_config);
    result.success = gs_result.success;
    result.solve_time_ms = gs_result.solve_time;
    result.position_error = gs_result.position_error;
    result.orientation_error = gs_result.orientation_error;
    result.iterations = gs_result.iterations; // Number of q7 values tested
    result.joint_angles = gs_result.joint_angles;
    
    // Check for collisions and compute clearance if solution was successful
    if (result.success) {
        robot.setJointAngles(result.joint_angles);
        result.collision_free = !planner.getPathPlanner()->armHasCollision(robot);
        
        // Compute clearance metrics (excluding first two base links and end effector)
        auto clearance_metrics = planner.getPathPlanner()->computeArmClearance(robot);
        result.min_clearance = clearance_metrics.min_clearance;
        result.avg_clearance = clearance_metrics.avg_clearance;
        result.num_links_checked = clearance_metrics.num_links_checked;
        
        // Calculate joint limit distances
        auto joint_limit_distances = calculateJointLimitDistances(result.joint_angles);
        result.min_joint_limit_distance = joint_limit_distances.first;
        result.avg_joint_limit_distance = joint_limit_distances.second;
    } else {
        result.collision_free = false;
        result.min_clearance = std::numeric_limits<double>::infinity();
        result.avg_clearance = std::numeric_limits<double>::infinity();
        result.num_links_checked = 0;
        result.min_joint_limit_distance = std::numeric_limits<double>::infinity();
        result.avg_joint_limit_distance = std::numeric_limits<double>::infinity();
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
                  const std::vector<IKResult>& sa_results,
                  const std::vector<IKResult>& gs_results) {
    int nr_successes = 0, sa_successes = 0, gs_successes = 0;
    int nr_collision_free = 0, sa_collision_free = 0, gs_collision_free = 0;
    double nr_total_time = 0, sa_total_time = 0, gs_total_time = 0;
    double nr_min_clearance_sum = 0, sa_min_clearance_sum = 0, gs_min_clearance_sum = 0;
    double nr_avg_clearance_sum = 0, sa_avg_clearance_sum = 0, gs_avg_clearance_sum = 0;
    int nr_clearance_count = 0, sa_clearance_count = 0, gs_clearance_count = 0;
    
    // Joint limit distance tracking
    double nr_min_joint_limit_sum = 0, sa_min_joint_limit_sum = 0, gs_min_joint_limit_sum = 0;
    double nr_avg_joint_limit_sum = 0, sa_avg_joint_limit_sum = 0, gs_avg_joint_limit_sum = 0;
    int nr_joint_limit_count = 0, sa_joint_limit_count = 0, gs_joint_limit_count = 0;
    
    for (const auto& result : nr_results) {
        if (result.success) nr_successes++;
        if (result.collision_free) {
            nr_collision_free++;
            if (result.min_clearance != std::numeric_limits<double>::infinity()) {
                nr_min_clearance_sum += result.min_clearance;
                nr_avg_clearance_sum += result.avg_clearance;
                nr_clearance_count++;
            }
            // Add joint limit distance calculations for collision-free solutions
            if (result.min_joint_limit_distance != std::numeric_limits<double>::infinity()) {
                nr_min_joint_limit_sum += result.min_joint_limit_distance;
                nr_avg_joint_limit_sum += result.avg_joint_limit_distance;
                nr_joint_limit_count++;
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
            // Add joint limit distance calculations for collision-free solutions
            if (result.min_joint_limit_distance != std::numeric_limits<double>::infinity()) {
                sa_min_joint_limit_sum += result.min_joint_limit_distance;
                sa_avg_joint_limit_sum += result.avg_joint_limit_distance;
                sa_joint_limit_count++;
            }
        }
        sa_total_time += result.solve_time_ms;
    }
    
    for (const auto& result : gs_results) {
        if (result.success) gs_successes++;
        if (result.collision_free) {
            gs_collision_free++;
            if (result.min_clearance != std::numeric_limits<double>::infinity()) {
                gs_min_clearance_sum += result.min_clearance;
                gs_avg_clearance_sum += result.avg_clearance;
                gs_clearance_count++;
            }
            // Add joint limit distance calculations for collision-free solutions
            if (result.min_joint_limit_distance != std::numeric_limits<double>::infinity()) {
                gs_min_joint_limit_sum += result.min_joint_limit_distance;
                gs_avg_joint_limit_sum += result.avg_joint_limit_distance;
                gs_joint_limit_count++;
            }
        }
        gs_total_time += result.solve_time_ms;
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
    
    std::cout << std::setw(15) << "Grid-Search" << " | ";
    std::cout << std::setw(10) << std::fixed << std::setprecision(1) << (100.0 * gs_successes / num_tests) << "%" << " | ";
    std::cout << std::setw(13) << std::fixed << std::setprecision(1) << (100.0 * gs_collision_free / num_tests) << "%" << " | ";
    std::cout << std::setw(13) << std::fixed << std::setprecision(2) << (gs_total_time / num_tests) << " | ";
    if (gs_clearance_count > 0) {
        std::cout << std::setw(10) << std::fixed << std::setprecision(4) << (gs_min_clearance_sum / gs_clearance_count) << " | ";
        std::cout << std::setw(10) << std::fixed << std::setprecision(4) << (gs_avg_clearance_sum / gs_clearance_count) << std::endl;
    } else {
        std::cout << std::setw(10) << "N/A" << " | ";
        std::cout << std::setw(10) << "N/A" << std::endl;
    }
    
    std::cout << "\nKey Findings:" << std::endl;
    std::cout << "- Newton-Raphson: " << (100.0 * nr_successes / num_tests) << "% success rate, " 
              << (100.0 * nr_collision_free / num_tests) << "% collision-free" << std::endl;
    std::cout << "- SA-Optimized: " << (100.0 * sa_successes / num_tests) << "% success rate, " 
              << (100.0 * sa_collision_free / num_tests) << "% collision-free" << std::endl;
    std::cout << "- Grid-Search: " << (100.0 * gs_successes / num_tests) << "% success rate, " 
              << (100.0 * gs_collision_free / num_tests) << "% collision-free" << std::endl;
    
    // Clearance comparisons
    if (nr_clearance_count > 0 && sa_clearance_count > 0 && gs_clearance_count > 0) {
        double nr_avg_min_clearance = nr_min_clearance_sum / nr_clearance_count;
        double sa_avg_min_clearance = sa_min_clearance_sum / sa_clearance_count;
        double gs_avg_min_clearance = gs_min_clearance_sum / gs_clearance_count;
        double nr_avg_avg_clearance = nr_avg_clearance_sum / nr_clearance_count;
        double sa_avg_avg_clearance = sa_avg_clearance_sum / sa_clearance_count;
        double gs_avg_avg_clearance = gs_avg_clearance_sum / gs_clearance_count;
        
        std::cout << "- Newton-Raphson clearance: min=" << std::fixed << std::setprecision(4) << nr_avg_min_clearance 
                  << " m, avg=" << nr_avg_avg_clearance << " m" << std::endl;
        std::cout << "- SA-Optimized clearance: min=" << std::fixed << std::setprecision(4) << sa_avg_min_clearance 
                  << " m, avg=" << sa_avg_avg_clearance << " m" << std::endl;
        std::cout << "- Grid-Search clearance: min=" << std::fixed << std::setprecision(4) << gs_avg_min_clearance 
                  << " m, avg=" << gs_avg_avg_clearance << " m" << std::endl;
    }
    
    // Joint limit distance comparisons
    if (nr_joint_limit_count > 0 && sa_joint_limit_count > 0 && gs_joint_limit_count > 0) {
        double nr_avg_min_joint_limit = nr_min_joint_limit_sum / nr_joint_limit_count;
        double sa_avg_min_joint_limit = sa_min_joint_limit_sum / sa_joint_limit_count;
        double gs_avg_min_joint_limit = gs_min_joint_limit_sum / gs_joint_limit_count;
        double nr_avg_avg_joint_limit = nr_avg_joint_limit_sum / nr_joint_limit_count;
        double sa_avg_avg_joint_limit = sa_avg_joint_limit_sum / sa_joint_limit_count;
        double gs_avg_avg_joint_limit = gs_avg_joint_limit_sum / gs_joint_limit_count;
        
        std::cout << "- Newton-Raphson joint limits: min=" << std::fixed << std::setprecision(4) << nr_avg_min_joint_limit 
                  << " rad (" << std::setprecision(1) << (nr_avg_min_joint_limit * 180.0 / M_PI) << "°), avg=" 
                  << std::setprecision(4) << nr_avg_avg_joint_limit << " rad (" 
                  << std::setprecision(1) << (nr_avg_avg_joint_limit * 180.0 / M_PI) << "°)" << std::endl;
        std::cout << "- SA-Optimized joint limits: min=" << std::fixed << std::setprecision(4) << sa_avg_min_joint_limit 
                  << " rad (" << std::setprecision(1) << (sa_avg_min_joint_limit * 180.0 / M_PI) << "°), avg=" 
                  << std::setprecision(4) << sa_avg_avg_joint_limit << " rad (" 
                  << std::setprecision(1) << (sa_avg_avg_joint_limit * 180.0 / M_PI) << "°)" << std::endl;
        std::cout << "- Grid-Search joint limits: min=" << std::fixed << std::setprecision(4) << gs_avg_min_joint_limit 
                  << " rad (" << std::setprecision(1) << (gs_avg_min_joint_limit * 180.0 / M_PI) << "°), avg=" 
                  << std::setprecision(4) << gs_avg_avg_joint_limit << " rad (" 
                  << std::setprecision(1) << (gs_avg_avg_joint_limit * 180.0 / M_PI) << "°)" << std::endl;
        
        // Safety analysis
        double critical_threshold = 0.1;  // 0.1 rad ≈ 5.7°
        double warning_threshold = 0.2;   // 0.2 rad ≈ 11.5°
        
        if (nr_avg_min_joint_limit < critical_threshold || sa_avg_min_joint_limit < critical_threshold) {
            std::cout << "  ⚠️  WARNING: Some solutions are in CRITICAL zone (<0.1 rad from joint limits)" << std::endl;
        } else if (nr_avg_min_joint_limit < warning_threshold || sa_avg_min_joint_limit < warning_threshold) {
            std::cout << "  ⚠️  CAUTION: Some solutions are in WARNING zone (<0.2 rad from joint limits)" << std::endl;
        } else {
            std::cout << "  ✅ All solutions maintain safe distance from joint limits (≥0.2 rad)" << std::endl;
        }
    }
    
    // Performance comparisons
    std::cout << "\nTiming Analysis:" << std::endl;
    if (nr_successes > 0) {
        double nr_avg_time = nr_total_time / nr_successes;
        std::cout << "- Newton-Raphson avg time: " << std::fixed << std::setprecision(3) << nr_avg_time << " ms" << std::endl;
    }
    if (sa_successes > 0) {
        double sa_avg_time = sa_total_time / sa_successes;
        std::cout << "- SA-Optimized avg time: " << std::fixed << std::setprecision(3) << sa_avg_time << " ms" << std::endl;
    }
    if (gs_successes > 0) {
        double gs_avg_time = gs_total_time / gs_successes;
        std::cout << "- Grid-Search avg time: " << std::fixed << std::setprecision(3) << gs_avg_time << " ms" << std::endl;
    }
    
    if (nr_successes > 0 && sa_successes > 0) {
        double speedup_nr_vs_sa = (sa_total_time / sa_successes) / (nr_total_time / nr_successes);
        std::cout << "- Newton-Raphson is " << std::fixed << std::setprecision(1) << speedup_nr_vs_sa 
                  << "x faster than SA-Optimized" << std::endl;
    }
    if (nr_successes > 0 && gs_successes > 0) {
        double speedup_nr_vs_gs = (gs_total_time / gs_successes) / (nr_total_time / nr_successes);
        std::cout << "- Newton-Raphson is " << std::fixed << std::setprecision(1) << speedup_nr_vs_gs 
                  << "x faster than Grid-Search" << std::endl;
    }
    if (sa_successes > 0 && gs_successes > 0) {
        double speedup_sa_vs_gs = (gs_total_time / gs_successes) / (sa_total_time / sa_successes);
        std::cout << "- SA-Optimized is " << std::fixed << std::setprecision(1) << speedup_sa_vs_gs 
                  << "x faster than Grid-Search" << std::endl;
    }
}

int main() {
    std::string urdf_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf";
    std::string env_path = "/Users/joris/Downloads/07_09_2/obstacles.xml";
    std::string poses_csv = "/Users/joris/Downloads/07_09_2/ext_post_rotated.csv";
    
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
        
        // Initialize unified IK solvers
        NewtonRaphsonIK ik_solver(robot);
        GridSearchIK grid_solver(robot, us_planner.getPathPlanner());
        
        // Configure GridSearchIK
        grid_solver.setPositionTolerance(1e-4);
        grid_solver.setOrientationTolerance(1e-3);
        grid_solver.setUseEncoderPrecision(false); // Use faster mode for comparison
        
        // Storage for results - now with multiple runs per pose
        std::vector<IKResult> nr_results, sa_results, gs_results;
        
        // Configuration: 100 runs per method per pose
        const int RUNS_PER_POSE = 10;
        
        // Test each pose
        Eigen::Matrix<double, 7, 1> home_config = Eigen::Matrix<double, 7, 1>::Zero();
        
        for (size_t i = 0; i < target_poses.size(); ++i) {
            std::cout << "\n--- Pose " << (i + 1) << "/" << target_poses.size() << " (" << RUNS_PER_POSE << " runs per method) ---" << std::endl;
            
            const auto& target_pose = target_poses[i];
            
            // Initialize solver settings once per pose
            ik_solver.setUseDampedLeastSquares(false);  // Use Newton-Raphson
            ik_solver.setPositionTolerance(1e-4);
            ik_solver.setOrientationTolerance(1e-3);
            ik_solver.setDampingFactor(0.001);  // Very small damping for numerical stability
            ik_solver.setAdaptiveDamping(true);  // Use adaptive damping near singularities
            
            // Track statistics for this pose
            int nr_successes = 0, sa_successes = 0, gs_successes = 0;
            int nr_collision_free = 0, sa_collision_free = 0, gs_collision_free = 0;
            
            // Test Newton-Raphson method - 100 runs
            std::cout << "  Newton-Raphson: ";
            for (int run = 0; run < RUNS_PER_POSE; ++run) {
                robot.setJointAngles(home_config);
                auto nr_result = testNewtonRaphson(ik_solver, target_pose, home_config, us_planner, robot);
                nr_result.pose_id = i;  // Add pose ID for analysis
                nr_result.run_id = run; // Add run ID for analysis
                nr_results.push_back(nr_result);
                
                if (nr_result.success) nr_successes++;
                if (nr_result.collision_free) nr_collision_free++;
                
                // Progress indicator
                if ((run + 1) % 20 == 0) std::cout << (run + 1) << " ";
            }
            std::cout << "| " << nr_successes << "/" << RUNS_PER_POSE << " success, " 
                      << nr_collision_free << "/" << RUNS_PER_POSE << " collision-free" << std::endl;
            
            // Reset counters
            sa_successes = 0;
            sa_collision_free = 0;
            
            // Test simulated annealing selectGoalPose - 100 runs
            std::cout << "  SA-Optimized:   ";
            for (int run = 0; run < RUNS_PER_POSE; ++run) {
                robot.setJointAngles(home_config);
                auto sa_result = testSimulatedAnnealingGoalPose(us_planner, robot, target_pose);
                sa_result.pose_id = i;  // Add pose ID for analysis
                sa_result.run_id = run; // Add run ID for analysis
                sa_results.push_back(sa_result);
                
                if (sa_result.success) sa_successes++;
                if (sa_result.collision_free) sa_collision_free++;
                
                // Progress indicator
                if ((run + 1) % 20 == 0) std::cout << (run + 1) << " ";
            }
            std::cout << "| " << sa_successes << "/" << RUNS_PER_POSE << " success, " 
                      << sa_collision_free << "/" << RUNS_PER_POSE << " collision-free" << std::endl;
            
            // Reset counters
            gs_successes = 0;
            gs_collision_free = 0;
            
            // Test Grid Search - Only 1 run per pose (deterministic)
            std::cout << "  Grid-Search:    ";
            robot.setJointAngles(home_config);
            auto gs_result = testGridSearch(grid_solver, target_pose, home_config, us_planner, robot);
            gs_result.pose_id = i;  // Add pose ID for analysis
            gs_result.run_id = 0;   // Only one run needed (deterministic)
            
            // Replicate the result for all runs to maintain consistency with other methods
            for (int run = 0; run < RUNS_PER_POSE; ++run) {
                IKResult gs_result_copy = gs_result;
                gs_result_copy.run_id = run;
                gs_results.push_back(gs_result_copy);
                
                if (gs_result.success) gs_successes++;
                if (gs_result.collision_free) gs_collision_free++;
                
                // Progress indicator (quick since it's the same result)
                if ((run + 1) % 20 == 0) std::cout << (run + 1) << " ";
            }
            std::cout << "| " << gs_successes << "/" << RUNS_PER_POSE << " success, " 
                      << gs_collision_free << "/" << RUNS_PER_POSE << " collision-free" << std::endl;
        }
        
        // Print summary statistics
        printSummary(nr_results, sa_results, gs_results);
        
        // Save results to CSV for further analysis
        std::ofstream results_file("three_method_comparison_results.csv");
        results_file << "pose_id,run_id,method,success,time_ms,pos_error,ori_error,iterations,collision_free,min_clearance,avg_clearance,num_links_checked,q1,q2,q3,q4,q5,q6,q7\n";
        
        // Output all Newton-Raphson results
        for (const auto& result : nr_results) {
            results_file << result.pose_id << "," << result.run_id << "," << result.method_name << "," 
                        << result.success << "," << result.solve_time_ms << "," 
                        << result.position_error << "," << result.orientation_error << "," 
                        << result.iterations << "," << result.collision_free << "," 
                        << result.min_clearance << "," << result.avg_clearance << "," 
                        << result.num_links_checked;
            
            // Add joint angles (or NaN if failed)
            if (result.success) {
                for (int j = 0; j < 7; j++) {
                    results_file << "," << result.joint_angles(j);
                }
            } else {
                for (int j = 0; j < 7; j++) {
                    results_file << ",nan";
                }
            }
            results_file << "\n";
        }
        
        // Output all SA-Optimized results
        for (const auto& result : sa_results) {
            results_file << result.pose_id << "," << result.run_id << "," << result.method_name << "," 
                        << result.success << "," << result.solve_time_ms << "," 
                        << result.position_error << "," << result.orientation_error << "," 
                        << result.iterations << "," << result.collision_free << "," 
                        << result.min_clearance << "," << result.avg_clearance << "," 
                        << result.num_links_checked;
            
            // Add joint angles (or NaN if failed)
            if (result.success) {
                for (int j = 0; j < 7; j++) {
                    results_file << "," << result.joint_angles(j);
                }
            } else {
                for (int j = 0; j < 7; j++) {
                    results_file << ",nan";
                }
            }
            results_file << "\n";
        }
        
        // Output all Grid-Search results
        for (const auto& result : gs_results) {
            results_file << result.pose_id << "," << result.run_id << "," << result.method_name << "," 
                        << result.success << "," << result.solve_time_ms << "," 
                        << result.position_error << "," << result.orientation_error << "," 
                        << result.iterations << "," << result.collision_free << "," 
                        << result.min_clearance << "," << result.avg_clearance << "," 
                        << result.num_links_checked;
            
            // Add joint angles (or NaN if failed)
            if (result.success) {
                for (int j = 0; j < 7; j++) {
                    results_file << "," << result.joint_angles(j);
                }
            } else {
                for (int j = 0; j < 7; j++) {
                    results_file << ",nan";
                }
            }
            results_file << "\n";
        }
        
        std::cout << "\nResults saved to: three_method_comparison_results.csv" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}