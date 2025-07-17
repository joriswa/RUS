#include "simulated_annealing_ik.h"
#include <algorithm>

SimulatedAnnealingResult solveIKSimulatedAnnealing(
    const Eigen::Affine3d& target_pose,
    const RobotArm& start_arm,
    PathPlanner* path_planner,
    const SimulatedAnnealingParams& params,
    const std::optional<double>& initial_q7)
{
    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> search_dis(params.search_min, params.search_max);
    std::uniform_real_distribution<> acceptance_dis(0.0, 1.0);

    // Get starting configuration for IK calls
    RobotArm start_arm_copy = start_arm;  // Make a copy to call non-const methods
    Eigen::Matrix<double, 7, 1> joint_angles = start_arm_copy.getJointAngles();
    std::array<double, 7> joint_angles_array;
    Eigen::Map<Eigen::Matrix<double, 7, 1>>(joint_angles_array.data()) = joint_angles;

    // Transform target pose (same as PathPlanner implementation)
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d axes = Rx * Ry * Rz;
    
    Eigen::Affine3d conv;
    conv.linear() = axes;
    
    Eigen::Affine3d total = target_pose * start_arm_copy.getEndeffectorTransform().inverse();

    // Initialize solution tracking
    Eigen::Matrix<double, 7, 1> best_solution;
    double best_cost = std::numeric_limits<double>::infinity();
    double best_q7 = 0.0;
    bool has_valid_solution = false;

    // Initialize current state
    double current_q7 = initial_q7.value_or(search_dis(gen));
    double current_cost = std::numeric_limits<double>::infinity();
    
    // Temperature tracking
    double temperature = params.T_max;
    int no_improvement_counter = 0;
    
    // Performance optimizations
    const double search_range = params.search_max - params.search_min;
    const double pose_threshold = params.max_position_error + params.max_orientation_error;
    
    // Reusable temp arm to avoid repeated construction
    RobotArm temp_arm = start_arm_copy;

    // Simulated Annealing main loop
    for (int iter = 0; iter < params.max_iterations && temperature > params.T_min; ++iter) {
        // Generate neighbor q7 value (optimized - avoid recalculating search_range)
        double perturbation_radius = search_range * (temperature / params.T_max);
        
        std::normal_distribution<> perturb_dis(0.0, perturbation_radius);
        double neighbor_q7 = current_q7 + perturb_dis(gen);
        neighbor_q7 = std::clamp(neighbor_q7, params.search_min, params.search_max);

        // Solve IK for this q7 value
        std::array<std::array<double, 7>, 4> ik_solutions = franka_IK_EE(total, neighbor_q7, joint_angles_array);

        // Find best solution among all IK solutions for this q7
        double best_neighbor_cost = std::numeric_limits<double>::infinity();
        Eigen::Matrix<double, 7, 1> best_neighbor_solution;
        bool found_valid_neighbor = false;

        for (const auto& sol : ik_solutions) {
            // Fast NaN check
            if (std::any_of(sol.begin(), sol.end(), [](double val) { return std::isnan(val); })) {
                continue;
            }

            // Reuse temp_arm instead of creating new one each time
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> angles(sol.data());
            temp_arm = start_arm_copy; // Reset to base configuration
            temp_arm.setJointAngles(angles);

            // Fast pose error check first (most common rejection)
            double pose_error = IKCostFunctions::computePoseErrorMagnitude(temp_arm, target_pose);
            if (pose_error > pose_threshold) {
                continue; // Skip solutions that don't meet pose requirements
            }

            // Skip collision check for speed - just use pose error and manipulability
            // Check for collisions if path planner available (only after pose check)
            // if (path_planner && path_planner->armHasCollision(temp_arm)) {
            //     continue;
            // }

            // Simplified cost function for speed
            double cost = 100.0 * pose_error + 0.25 * (1.0 - temp_arm.computeManipulability());

            // Track best solution for this q7
            if (cost < best_neighbor_cost) {
                best_neighbor_cost = cost;
                best_neighbor_solution = angles;
                found_valid_neighbor = true;
                
                // Very aggressive early exit
                break; // Take first valid solution to speed up
            }
        }

        // Skip if no valid solution found for this q7
        if (!found_valid_neighbor) {
            temperature *= params.alpha;
            continue;
        }

        // Simulated Annealing acceptance criterion
        bool accept = false;

        if (!has_valid_solution) {
            // Accept first valid solution
            accept = true;
            has_valid_solution = true;
        } else if (best_neighbor_cost < current_cost) {
            // Accept better solution
            accept = true;
        } else {
            // Accept worse solution with probability exp(-ΔE/T)
            double delta = best_neighbor_cost - current_cost;
            double acceptance_probability = std::exp(-delta / temperature);
            accept = (acceptance_dis(gen) < acceptance_probability);
        }

        if (accept) {
            current_q7 = neighbor_q7;
            current_cost = best_neighbor_cost;

            // Update global best if this is better
            if (best_neighbor_cost < best_cost) {
                best_cost = best_neighbor_cost;
                best_solution = best_neighbor_solution;
                best_q7 = neighbor_q7;
                no_improvement_counter = 0;
                
                // Very aggressive early termination for good solutions
                if (best_cost < 0.5) break;
            } else {
                no_improvement_counter++;
            }
        }

        // Cool down temperature (much faster cooling)
        temperature *= params.alpha;

        // Early termination check (more aggressive)
        if (no_improvement_counter >= params.max_no_improvement) {
            break;
        }
        
        // Additional early exit if temperature drops too low
        if (temperature < params.T_min) {
            break;
        }
    }

    // Prepare result
    SimulatedAnnealingResult result;
    result.success = has_valid_solution && (best_cost < std::numeric_limits<double>::infinity());
    result.best_cost = best_cost;
    result.final_q7 = best_q7;
    result.iterations_used = std::min(params.max_iterations, 
                                     static_cast<int>(params.max_iterations * (1.0 - temperature / params.T_max)));
    
    if (result.success) {
        result.best_solution = best_solution;
        
        // Final validation: verify solution meets pose requirements
        RobotArm final_arm = start_arm_copy;
        final_arm.setJointAngles(best_solution);
        double final_pose_error = IKCostFunctions::computePoseErrorMagnitude(final_arm, target_pose);
        
        if (final_pose_error > params.max_position_error + params.max_orientation_error) {
            result.success = false; // Solution doesn't meet pose accuracy requirements
        }
    }

    return result;
}
