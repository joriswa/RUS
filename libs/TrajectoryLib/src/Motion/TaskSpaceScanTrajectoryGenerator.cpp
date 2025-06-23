#include "TrajectoryLib/Motion/TaskSpaceScanTrajectoryGenerator.h"
#include "TrajectoryLib/Core/franka_IK_EE.h"
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include <iostream>
#include <cmath>
#include <algorithm>

TaskSpaceScanTrajectoryGenerator::TaskSpaceScanTrajectoryGenerator(const RobotArm& robot_arm)
    : robot_arm_(robot_arm)
{
    // Default collision checker that always returns false (no collision)
    collision_checker_ = [](const Eigen::VectorXd&) { return false; };
}

void TaskSpaceScanTrajectoryGenerator::setCollisionChecker(
    std::function<bool(const Eigen::VectorXd&)> collision_checker)
{
    collision_checker_ = collision_checker;
}

TaskSpaceScanTrajectoryGenerator::GenerationResult 
TaskSpaceScanTrajectoryGenerator::generateTrajectory(
    const std::vector<Eigen::Affine3d>& scan_poses,
    const Eigen::VectorXd& start_joints,
    const ScanConfig& config)
{
    GenerationResult result;
    
    if (scan_poses.empty()) {
        result.error_message = "No scan poses provided";
        return result;
    }
    
    if (start_joints.size() != 7) {
        result.error_message = "Start joints must have 7 elements";
        return result;
    }

    try {
        // Initialize with starting joint configuration
        Eigen::VectorXd current_joints = start_joints;
        std::vector<MotionGenerator::TrajectoryPoint> full_trajectory;
        double current_time = 0.0;
        int total_ik_failures = 0;
        
        // Generate trajectory for each segment between consecutive poses
        for (size_t i = 0; i < scan_poses.size() - 1; ++i) {
            const auto& start_pose = scan_poses[i];
            const auto& end_pose = scan_poses[i + 1];
            
            // Generate segment trajectory
            auto segment_result = generateSegmentTrajectory(start_pose, end_pose, current_joints, config);
            
            if (!segment_result.success) {
                result.error_message = "Failed to generate segment " + std::to_string(i) + 
                                     ": " + segment_result.error_message;
                return result;
            }
            
            // Adjust timing for continuous trajectory
            for (auto& point : segment_result.trajectory) {
                point.time += current_time;
                full_trajectory.push_back(point);
            }
            
            // Update for next segment
            if (!segment_result.trajectory.empty()) {
                const auto& last_point = segment_result.trajectory.back();
                current_joints = Eigen::Map<const Eigen::VectorXd>(
                    last_point.position.data(), last_point.position.size());
                current_time = last_point.time;
            }
            
            total_ik_failures += segment_result.num_ik_failures;
        }
        
        // Calculate metrics
        result.trajectory = full_trajectory;
        result.success = true;
        result.num_waypoints_generated = static_cast<int>(full_trajectory.size());
        result.num_ik_failures = total_ik_failures;
        result.total_duration = current_time;
        
        // Calculate actual average speed
        if (!scan_poses.empty() && result.total_duration > 0) {
            double total_distance = 0.0;
            for (size_t i = 0; i < scan_poses.size() - 1; ++i) {
                total_distance += computeCartesianDistance(scan_poses[i], scan_poses[i + 1]);
            }
            result.actual_average_speed = total_distance / result.total_duration;
        }
        
    } catch (const std::exception& e) {
        result.error_message = "Exception during trajectory generation: " + std::string(e.what());
    }
    
    return result;
}

TaskSpaceScanTrajectoryGenerator::GenerationResult 
TaskSpaceScanTrajectoryGenerator::generateSegmentTrajectory(
    const Eigen::Affine3d& start_pose,
    const Eigen::Affine3d& end_pose,
    const Eigen::VectorXd& start_joints,
    const ScanConfig& config)
{
    GenerationResult result;
    
    try {
        // Step 1: Interpolate Cartesian path
        auto cartesian_waypoints = interpolateCartesianPath(
            start_pose, end_pose, config.scan_speed_ms, config.control_frequency, 
            config.interpolate_orientation);
            
        if (cartesian_waypoints.empty()) {
            result.error_message = "Failed to generate Cartesian waypoints";
            return result;
        }
        
        // Step 2: Convert to joint trajectory using analytical IK
        auto joint_waypoints = convertToJointTrajectory(cartesian_waypoints, start_joints, config);
        
        if (joint_waypoints.empty()) {
            result.error_message = "Failed to solve IK for any waypoints";
            return result;
        }
        
        // Step 3: Apply quintic polynomial smoothing
        auto smoothed_trajectory = smoothJointTrajectory(joint_waypoints, config);
        
        // Set results
        result.trajectory = smoothed_trajectory;
        result.success = true;
        result.num_waypoints_generated = static_cast<int>(joint_waypoints.size());
        result.num_ik_failures = static_cast<int>(cartesian_waypoints.size() - joint_waypoints.size());
        
        if (!smoothed_trajectory.empty()) {
            result.total_duration = smoothed_trajectory.back().time;
        }
        
    } catch (const std::exception& e) {
        result.error_message = "Exception in segment generation: " + std::string(e.what());
    }
    
    return result;
}

std::vector<std::pair<Eigen::Affine3d, double>> 
TaskSpaceScanTrajectoryGenerator::interpolateCartesianPath(
    const Eigen::Affine3d& start_pose,
    const Eigen::Affine3d& end_pose,
    double scan_speed,
    double control_frequency,
    bool interpolate_orientation)
{
    std::vector<std::pair<Eigen::Affine3d, double>> waypoints;
    
    // Compute distance and duration
    double distance = computeCartesianDistance(start_pose, end_pose);
    double duration = distance / scan_speed;
    double dt = 1.0 / control_frequency;
    int num_waypoints = static_cast<int>(std::ceil(duration / dt)) + 1;
    
    // Ensure minimum number of waypoints
    num_waypoints = std::max(num_waypoints, 2);
    
    for (int i = 0; i < num_waypoints; ++i) {
        double t = static_cast<double>(i) / (num_waypoints - 1);
        double time = t * duration;
        
        // Interpolate position linearly
        Eigen::Vector3d pos = start_pose.translation() + 
                             t * (end_pose.translation() - start_pose.translation());
        
        // Interpolate orientation
        Eigen::Matrix3d rot;
        if (interpolate_orientation) {
            rot = interpolateOrientation(start_pose.rotation(), end_pose.rotation(), t);
        } else {
            rot = end_pose.rotation(); // Use target orientation throughout
        }
        
        // Create interpolated pose
        Eigen::Affine3d interpolated_pose = Eigen::Affine3d::Identity();
        interpolated_pose.translation() = pos;
        interpolated_pose.rotation() = rot;
        
        waypoints.emplace_back(interpolated_pose, time);
    }
    
    return waypoints;
}

std::vector<std::pair<Eigen::VectorXd, double>> 
TaskSpaceScanTrajectoryGenerator::convertToJointTrajectory(
    const std::vector<std::pair<Eigen::Affine3d, double>>& cartesian_waypoints,
    const Eigen::VectorXd& start_joints,
    const ScanConfig& config)
{
    std::vector<std::pair<Eigen::VectorXd, double>> joint_waypoints;
    
    if (cartesian_waypoints.empty()) {
        return joint_waypoints;
    }
    
    Eigen::VectorXd current_joints = start_joints;
    
    for (const auto& [pose, time] : cartesian_waypoints) {
        bool ik_success = false;
        Eigen::VectorXd best_joints;
        
        // Try IK with multiple attempts if needed
        for (int attempt = 0; attempt < config.max_ik_attempts; ++attempt) {
            Eigen::Affine3d target_pose = pose;
            
            // Apply small perturbation for retry attempts
            if (attempt > 0) {
                double perturbation = config.ik_perturbation_radius * attempt;
                Eigen::Vector3d random_offset = Eigen::Vector3d::Random() * perturbation;
                target_pose.translation() += random_offset;
            }
            
            // Compute transformation matrix for IK solver
            Eigen::Matrix<double, 4, 4> target_matrix = 
                target_pose.matrix() * robot_arm_.getEndeffectorTransform().inverse().matrix();
            
            // Try different q7 values around current joint 6
            std::vector<double> q7_candidates;
            if (current_joints.size() >= 7) {
                q7_candidates = {
                    current_joints[6],
                    current_joints[6] + 0.1,
                    current_joints[6] - 0.1,
                    current_joints[6] + 0.2,
                    current_joints[6] - 0.2
                };
            } else {
                q7_candidates = {0.0, 0.1, -0.1, 0.2, -0.2};
            }
            
            for (double q7 : q7_candidates) {
                try {
                    // Convert current joints to array format
                    std::array<double, 7> current_joints_array;
                    for (int i = 0; i < 7; ++i) {
                        current_joints_array[i] = (i < current_joints.size()) ? current_joints[i] : 0.0;
                    }
                    
                    // Solve IK
                    std::array<std::array<double, 7>, 4> ik_solutions = 
                        franka_IK_EE(target_matrix, q7, current_joints_array);
                    
                    // Select best solution
                    Eigen::VectorXd candidate_joints = selectBestIKSolution(ik_solutions, current_joints);
                    
                    if (candidate_joints.size() == 7 && validateJointConfiguration(candidate_joints)) {
                        best_joints = candidate_joints;
                        ik_success = true;
                        break;
                    }
                } catch (const std::exception&) {
                    // Continue to next q7 candidate
                    continue;
                }
            }
            
            if (ik_success) break;
        }
        
        if (ik_success) {
            joint_waypoints.emplace_back(best_joints, time);
            current_joints = best_joints;
        } else {
            // Skip this waypoint but continue with the rest
            std::cout << "Warning: IK failed for waypoint at time " << time << std::endl;
        }
    }
    
    return joint_waypoints;
}

std::vector<MotionGenerator::TrajectoryPoint> 
TaskSpaceScanTrajectoryGenerator::smoothJointTrajectory(
    const std::vector<std::pair<Eigen::VectorXd, double>>& joint_waypoints,
    const ScanConfig& config)
{
    std::vector<MotionGenerator::TrajectoryPoint> trajectory;
    
    if (joint_waypoints.empty()) {
        return trajectory;
    }
    
    // Convert to MotionGenerator format
    std::vector<MotionGenerator::TrajectoryPoint> raw_trajectory;
    
    for (const auto& [joints, time] : joint_waypoints) {
        MotionGenerator::TrajectoryPoint point;
        point.time = time;
        point.position.resize(7);
        point.velocity.resize(7);
        point.acceleration.resize(7);
        
        for (int i = 0; i < 7; ++i) {
            point.position[i] = (i < joints.size()) ? joints[i] : 0.0;
            point.velocity[i] = 0.0;
            point.acceleration[i] = 0.0;
        }
        
        raw_trajectory.push_back(point);
    }
    
    // Apply quintic polynomial smoothing using existing MotionGenerator functionality
    // For now, return the raw trajectory. This could be enhanced by calling
    // MotionGenerator::applyQuinticPolynomialFit if we can access it
    trajectory = raw_trajectory;
    
    // Calculate velocities and accelerations using finite differences
    for (size_t i = 0; i < trajectory.size(); ++i) {
        if (i > 0 && i < trajectory.size() - 1) {
            double dt_prev = trajectory[i].time - trajectory[i-1].time;
            double dt_next = trajectory[i+1].time - trajectory[i].time;
            
            for (int j = 0; j < 7; ++j) {
                // Central difference for velocity
                if (dt_prev > 0 && dt_next > 0) {
                    trajectory[i].velocity[j] = 
                        (trajectory[i+1].position[j] - trajectory[i-1].position[j]) / 
                        (dt_prev + dt_next);
                }
                
                // Central difference for acceleration
                if (dt_prev > 0 && dt_next > 0) {
                    double v_next = (trajectory[i+1].position[j] - trajectory[i].position[j]) / dt_next;
                    double v_prev = (trajectory[i].position[j] - trajectory[i-1].position[j]) / dt_prev;
                    trajectory[i].acceleration[j] = (v_next - v_prev) / ((dt_prev + dt_next) / 2.0);
                }
            }
        }
    }
    
    return trajectory;
}

Eigen::VectorXd TaskSpaceScanTrajectoryGenerator::selectBestIKSolution(
    const std::array<std::array<double, 7>, 4>& ik_solutions,
    const Eigen::VectorXd& previous_joints)
{
    double min_distance = std::numeric_limits<double>::max();
    int best_idx = -1;
    
    for (int i = 0; i < 4; ++i) {
        if (!hasNaN(ik_solutions[i])) {
            // Calculate joint distance
            double distance = 0.0;
            for (int j = 0; j < 7; ++j) {
                double diff = ik_solutions[i][j] - 
                             ((j < previous_joints.size()) ? previous_joints[j] : 0.0);
                distance += diff * diff;
            }
            distance = std::sqrt(distance);
            
            if (distance < min_distance) {
                min_distance = distance;
                best_idx = i;
            }
        }
    }
    
    if (best_idx >= 0) {
        Eigen::VectorXd result(7);
        for (int i = 0; i < 7; ++i) {
            result[i] = ik_solutions[best_idx][i];
        }
        return result;
    }
    
    return Eigen::VectorXd(); // Return empty vector if no valid solution
}

Eigen::Matrix3d TaskSpaceScanTrajectoryGenerator::interpolateOrientation(
    const Eigen::Matrix3d& start_rot,
    const Eigen::Matrix3d& end_rot,
    double t)
{
    Eigen::Quaterniond q1(start_rot);
    Eigen::Quaterniond q2(end_rot);
    
    // Normalize quaternions
    q1.normalize();
    q2.normalize();
    
    // SLERP interpolation
    Eigen::Quaterniond q_interp = q1.slerp(t, q2);
    
    return q_interp.toRotationMatrix();
}

double TaskSpaceScanTrajectoryGenerator::computeCartesianDistance(
    const Eigen::Affine3d& pose1,
    const Eigen::Affine3d& pose2,
    bool include_rotation)
{
    // Positional distance
    double pos_distance = (pose2.translation() - pose1.translation()).norm();
    
    if (!include_rotation) {
        return pos_distance;
    }
    
    // Add rotational distance (angle between orientations)
    Eigen::Quaterniond q1(pose1.rotation());
    Eigen::Quaterniond q2(pose2.rotation());
    q1.normalize();
    q2.normalize();
    
    double dot_product = std::abs(q1.dot(q2));
    dot_product = std::min(dot_product, 1.0); // Clamp for numerical stability
    double angle_distance = 2.0 * std::acos(dot_product);
    
    // Weight rotational distance (convert radians to equivalent meters)
    // This is somewhat arbitrary - adjust based on application needs
    double rotation_weight = 0.1; // 1 radian ≈ 0.1 meters
    
    return pos_distance + rotation_weight * angle_distance;
}

bool TaskSpaceScanTrajectoryGenerator::validateJointConfiguration(const Eigen::VectorXd& joints)
{
    if (joints.size() != 7) {
        return false;
    }
    
    // Check joint limits (Franka Panda limits)
    const std::vector<std::pair<double, double>> joint_limits = {
        {-2.8973, 2.8973},  // Joint 1
        {-1.7628, 1.7628},  // Joint 2
        {-2.8973, 2.8973},  // Joint 3
        {-3.0718, -0.0698}, // Joint 4
        {-2.8973, 2.8973},  // Joint 5
        {-0.0175, 3.7525},  // Joint 6
        {-2.8973, 2.8973}   // Joint 7
    };
    
    for (int i = 0; i < 7; ++i) {
        if (joints[i] < joint_limits[i].first || joints[i] > joint_limits[i].second) {
            return false;
        }
    }
    
    // Check for collisions if collision checker is available
    if (collision_checker_) {
        return !collision_checker_(joints);
    }
    
    return true;
}

bool TaskSpaceScanTrajectoryGenerator::hasNaN(const std::array<double, 7>& arr)
{
    return std::any_of(arr.begin(), arr.end(), [](double val) { return std::isnan(val); });
}
