#include "grid_search_ik.h"
#include "TrajectoryLib/Robot/franka_ik_He.h"
#include "TrajectoryLib/Planning/PathPlanner.h"
#include <chrono>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <thread>
#include <future>

GridSearchIKResult GridSearchIK::solve(const Eigen::Affine3d& target_pose, 
                                       const Eigen::Matrix<double, 7, 1>& initial_guess) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    GridSearchIKResult result;
    result.success = false;
    result.iterations = 0;
    result.joint_angles = initial_guess;
    result.final_cost = std::numeric_limits<double>::infinity();
    
    // Get starting configuration for IK solver
    std::array<double, 7> jointAnglesArray;
    Eigen::Map<Eigen::Matrix<double, 7, 1>>(jointAnglesArray.data()) = initial_guess;
    
    // Transform target pose (same as PathPlanner approach)
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d axes = Rx * Ry * Rz;
    
    Eigen::Affine3d conv;
    conv.linear() = axes;
    
    Eigen::Affine3d total = target_pose * robot.getEndeffectorTransform().inverse();
    
    // Pose accuracy thresholds - aligned with Franka Panda ±0.1mm repeatability
    const double max_position_error = 0.0001;     // 0.1mm tolerance (matches Franka repeatability)
    const double max_orientation_error = 0.001;   // ~0.057 degree tolerance (matches newton_raphson_ik.h)
    
    // Generate q7 grid values
    std::vector<double> q7_values = generateQ7Grid();
    
    // Track best solution
    Eigen::Matrix<double, 7, 1> best_solution;
    double best_cost = std::numeric_limits<double>::infinity();
    bool has_valid_solution = false;
    double best_q7 = 0.0;
    
    // Decide whether to use parallel processing
    size_t effective_threads = (max_threads == 0) ? std::thread::hardware_concurrency() : max_threads;
    const size_t min_chunk_size = 50; // Minimum work per thread to justify parallelization
    bool should_parallelize = use_parallel_processing && 
                              q7_values.size() >= min_chunk_size && 
                              effective_threads > 1;
    
    if (should_parallelize) {
        // PARALLEL PROCESSING with Boost.Asio
        
        // Create thread pool with optimal thread count
        boost::asio::thread_pool pool(effective_threads);
        
        // Calculate optimal chunk size (aim for 2-4 chunks per thread)
        size_t target_chunks = effective_threads * 3;
        size_t chunk_size = std::max(min_chunk_size, q7_values.size() / target_chunks);
        
        // Vector to hold futures from each task
        std::vector<std::future<LocalSearchResult>> futures;
        futures.reserve((q7_values.size() + chunk_size - 1) / chunk_size);
        
        // Launch parallel tasks using Boost.Asio thread pool
        for (size_t start = 0; start < q7_values.size(); start += chunk_size) {
            size_t end = std::min(start + chunk_size, q7_values.size());
            
            // Create chunk of q7 values
            std::vector<double> q7_chunk(q7_values.begin() + start, q7_values.begin() + end);
            
            // Create packaged task for this chunk
            auto task = std::make_shared<std::packaged_task<LocalSearchResult()>>(
                [this, q7_chunk, total, target_pose, jointAnglesArray, max_position_error, max_orientation_error]() {
                    return processQ7Chunk(q7_chunk, total, target_pose, jointAnglesArray, 
                                         max_position_error, max_orientation_error);
                }
            );
            
            futures.push_back(task->get_future());
            
            // Post task to thread pool
            boost::asio::post(pool, [task]() { (*task)(); });
        }
        
        // Wait for thread pool to complete all tasks
        pool.join();
        
        // Collect results from all threads
        for (auto& fut : futures) {
            LocalSearchResult local_result = fut.get();
            result.iterations += local_result.iterations;
            
            // Update global best solution if this thread found a better one
            if (local_result.has_valid_solution && local_result.best_cost < best_cost) {
                best_cost = local_result.best_cost;
                best_solution = local_result.best_solution;
                has_valid_solution = true;
                best_q7 = local_result.best_q7;
            }
        }
        
    } else {
        // SEQUENTIAL PROCESSING (fallback for small problems or single-threaded)
        
        for (double q7 : q7_values) {
            result.iterations++;
            
            // Solve IK for this q7 value
            std::array<std::array<double, 7>, 4> ikSolutions = franka_IK_EE(total, q7, jointAnglesArray);
            
            // Evaluate all IK solutions for this q7
            for (const auto &sol : ikSolutions) {
                if (std::any_of(sol.begin(), sol.end(), [](double val) { return std::isnan(val); })) {
                    continue;
                }
                
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> angles(sol.data());
                RobotArm temp = robot;
                temp.setJointAngles(angles);
                
                // Check pose accuracy first
                double pose_error = IKCostFunctions::computePoseErrorMagnitude(temp, target_pose);
                if (pose_error > max_position_error + max_orientation_error) {
                    continue;
                }
                
                // Check for collisions
                if (path_planner && armHasCollision(temp)) {
                    continue;
                }
                
                // Compute full cost function using shared utilities
                double cost = IKCostFunctions::computeComprehensiveCost(temp, target_pose, path_planner);
                
                // Track best solution
                if (!has_valid_solution || cost < best_cost) {
                    best_cost = cost;
                    best_solution = angles;
                    has_valid_solution = true;
                    best_q7 = q7;
                }
            }
        }
    }
    
    // Prepare result
    if (has_valid_solution) {
        result.success = true;
        result.joint_angles = best_solution;
        result.best_q7 = best_q7;
        result.final_cost = best_cost;
        
        // Compute final errors
        robot.setJointAngles(best_solution);
        Eigen::Matrix<double, 6, 1> pose_error = computePoseError(robot.getEndeffectorPose(), target_pose);
        result.position_error = pose_error.head<3>().norm();
        result.orientation_error = pose_error.tail<3>().norm();
        result.final_error = pose_error.norm();
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.solve_time = duration.count() / 1000.0; // Convert to milliseconds
    
    return result;
}

std::pair<RobotArm, bool> GridSearchIK::solveIK(const Eigen::Affine3d& target_pose) {
    // Use current robot configuration as initial guess
    Eigen::Matrix<double, 7, 1> initial_guess = robot.getJointAngles();
    
    // Solve IK
    GridSearchIKResult result = solve(target_pose, initial_guess);
    
    // Create result robot arm
    RobotArm result_robot = robot;
    if (result.success) {
        result_robot.setJointAngles(result.joint_angles);
    }
    
    return std::make_pair(result_robot, result.success);
}

std::vector<double> GridSearchIK::generateQ7Grid() {
    std::vector<double> q7_values;
    
    if (use_encoder_precision) {
        // Generate grid based on 14-bit encoder precision
        double q7_range = q7_max - q7_min;
        double step_size = q7_range / (num_encoder_steps - 1);
        
        for (int i = 0; i < num_encoder_steps; ++i) {
            double q7 = q7_min + i * step_size;
            q7_values.push_back(q7);
        }
    } else {
        // Coarser grid for faster testing (1000 steps)
        int num_steps = 1000;
        double q7_range = q7_max - q7_min;
        double step_size = q7_range / (num_steps - 1);
        
        for (int i = 0; i < num_steps; ++i) {
            double q7 = q7_min + i * step_size;
            q7_values.push_back(q7);
        }
    }
    
    return q7_values;
}

Eigen::Vector3d GridSearchIK::rotationMatrixToAxisAngle(const Eigen::Matrix3d& R) {
    // Convert rotation matrix to axis-angle representation
    Eigen::AngleAxisd axis_angle(R);
    return axis_angle.angle() * axis_angle.axis();
}

Eigen::Matrix<double, 6, 1> GridSearchIK::computePoseError(const Eigen::Affine3d& current_pose, 
                                                          const Eigen::Affine3d& target_pose) {
    Eigen::Matrix<double, 6, 1> error;
    
    // Position error
    error.head<3>() = target_pose.translation() - current_pose.translation();
    
    // Orientation error using axis-angle representation
    Eigen::Matrix3d rotation_error = target_pose.rotation() * current_pose.rotation().transpose();
    error.tail<3>() = rotationMatrixToAxisAngle(rotation_error);
    
    return error;
}

Eigen::Matrix<double, 7, 1> GridSearchIK::enforceJointLimits(const Eigen::Matrix<double, 7, 1>& joint_angles) {
    auto joint_limits = robot.jointLimits();
    Eigen::Matrix<double, 7, 1> clamped_angles = joint_angles;
    
    for (int i = 0; i < 7; ++i) {
        double lower_limit = joint_limits[i].first + joint_limit_margin;
        double upper_limit = joint_limits[i].second - joint_limit_margin;
        clamped_angles(i) = std::clamp(clamped_angles(i), lower_limit, upper_limit);
    }
    
    return clamped_angles;
}

// Note: computePoseErrorMagnitude and computeCostFunction methods 
// have been moved to IKCostFunctions for reusability across all IK solvers

bool GridSearchIK::armHasCollision(RobotArm& arm) {
    if (path_planner) {
        return path_planner->armHasCollision(arm);
    }
    return false; // No collision checking if no PathPlanner available
}

GridSearchIK::LocalSearchResult GridSearchIK::processQ7Chunk(
    const std::vector<double>& q7_chunk,
    const Eigen::Affine3d& total_transform,
    const Eigen::Affine3d& target_pose,
    const std::array<double, 7>& joint_angles_array,
    double max_position_error,
    double max_orientation_error) const {
    
    LocalSearchResult local_result;
    
    // Create a local copy of the robot for this thread
    RobotArm local_robot = robot;
    
    for (double q7 : q7_chunk) {
        local_result.iterations++;
        
        // Solve IK for this q7 value
        std::array<std::array<double, 7>, 4> ikSolutions = franka_IK_EE(total_transform, q7, joint_angles_array);
        
        // Evaluate all IK solutions for this q7
        for (const auto &sol : ikSolutions) {
            if (std::any_of(sol.begin(), sol.end(), [](double val) { return std::isnan(val); })) {
                continue;
            }
            
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> angles(sol.data());
            local_robot.setJointAngles(angles);
            
            // Check pose accuracy first
            double pose_error = IKCostFunctions::computePoseErrorMagnitude(local_robot, target_pose);
            if (pose_error > max_position_error + max_orientation_error) {
                continue;
            }
            
            // Check for collisions (each thread has its own robot copy)
            if (path_planner && path_planner->armHasCollision(local_robot)) {
                continue;
            }
            
            // Compute full cost function using shared utilities
            double cost = IKCostFunctions::computeComprehensiveCost(local_robot, target_pose, path_planner);
            
            // Track best solution in this chunk
            if (!local_result.has_valid_solution || cost < local_result.best_cost) {
                local_result.best_cost = cost;
                local_result.best_solution = angles;
                local_result.best_q7 = q7;
                local_result.has_valid_solution = true;
            }
        }
    }
    
    return local_result;
}
