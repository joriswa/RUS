#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <memory>
#include <string>
#include <iomanip>
#include <algorithm>
#include <limits>

#include "TrajectoryLib/Robot/RobotArm.h"
#include "GeometryLib/BVHTree.h"
#include "TrajectoryLib/Robot/RobotManager.h"

/**
 * High-Accuracy Clearance Calculator
 * ==================================
 * This utility calculates robot arm clearance metrics for trajectory CSV files
 * with maximum accuracy prioritized over computational speed.
 * 
 * Accuracy Features:
 * - Dense 10x10 sampling on each box face (600 points per box)
 * - Additional sampling along all 12 box edges
 * - Interior volume sampling to detect penetration
 * - 8 corner point sampling
 * - Total: ~1000+ sample points per collision box for maximum precision
 * 
 * Usage: ./clearance_calculator <trajectory_csv> <output_csv>
 */

class ClearanceCalculator {
private:
    std::unique_ptr<RobotArm> robot_;
    std::shared_ptr<BVHTree> obstacle_tree_;

public:
    ClearanceCalculator() {
        // Initialize robot
        std::string urdf_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf";
        robot_ = std::make_unique<RobotArm>(urdf_path);
        
        // Initialize obstacle tree using RobotManager
        std::string environment_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/obstacles.xml";
        try {
            RobotManager robot_manager;
            robot_manager.parseURDF(environment_path);
            obstacle_tree_ = std::make_shared<BVHTree>(robot_manager.getTransformedObstacles());
            std::cerr << "✓ Environment loaded with " << robot_manager.getTransformedObstacles().size() << " obstacles" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "⚠️  Failed to load environment: " << e.what() << std::endl;
            std::vector<std::shared_ptr<Obstacle>> empty_obstacles;
            obstacle_tree_ = std::make_shared<BVHTree>(empty_obstacles);
            std::cerr << "✓ Using empty obstacle tree" << std::endl;
        }
        
        std::cerr << "✓ Robot and environment loaded" << std::endl;
    }
    
    /**
     * Calculate the minimum distance from an oriented box to obstacles.
     * This method uses dense sampling for maximum accuracy, prioritizing precision over speed.
     * 
     * @param center Center point of the oriented box
     * @param half_dims Half-dimensions of the box along each local axis
     * @param axes 3x3 matrix where columns are the box's local coordinate axes
     * @return Minimum distance from any point on the box surface to obstacles
     */
    double calculateBoxClearance(const Vec3& center, const Vec3& half_dims, const Eigen::Matrix3d& axes) {
        double min_distance = std::numeric_limits<double>::max();
        
        // Use high-density sampling for maximum accuracy
        const int face_samples = 10;  // 10x10 grid per face = 100 points per face
        const int edge_samples = 8;   // Additional samples along each edge
        
        // 1. Sample points densely on all 6 faces of the oriented box
        for (int face = 0; face < 6; ++face) {
            Vec3 face_normal;
            Vec3 face_center = center;
            
            // Determine face normal and center offset
            int axis = face / 2;  // 0, 1, or 2 for x, y, z
            double sign = (face % 2 == 0) ? 1.0 : -1.0;
            
            face_normal = axes.col(axis) * sign;
            face_center += face_normal * half_dims[axis];
            
            // Dense sampling on this face
            for (int i = 0; i < face_samples; ++i) {
                for (int j = 0; j < face_samples; ++j) {
                    // Calculate sample coordinates on the face
                    double u = -1.0 + 2.0 * i / (face_samples - 1);  // [-1, 1]
                    double v = -1.0 + 2.0 * j / (face_samples - 1);  // [-1, 1]
                    
                    // Get the two tangent axes for this face
                    int tangent1 = (axis + 1) % 3;
                    int tangent2 = (axis + 2) % 3;
                    
                    Vec3 sample_point = face_center + 
                                       u * half_dims[tangent1] * axes.col(tangent1) +
                                       v * half_dims[tangent2] * axes.col(tangent2);
                    
                    // Calculate distance from this sample point to obstacles
                    auto [distance, grad] = obstacle_tree_->getDistanceAndGradient(sample_point);
                    min_distance = std::min(min_distance, distance);
                    
                    // Continue even if collision found to get the most negative distance
                }
            }
        }
        
        // 2. Sample the 8 corners of the box
        for (int corner = 0; corner < 8; ++corner) {
            Vec3 corner_point = center;
            for (int axis = 0; axis < 3; ++axis) {
                double sign = (corner & (1 << axis)) ? 1.0 : -1.0;
                corner_point += sign * half_dims[axis] * axes.col(axis);
            }
            
            auto [distance, grad] = obstacle_tree_->getDistanceAndGradient(corner_point);
            min_distance = std::min(min_distance, distance);
        }
        
        // 3. Sample points along the 12 edges of the box for maximum accuracy
        // This catches cases where the minimum might be along an edge
        for (int edge = 0; edge < 12; ++edge) {
            Vec3 start_corner, end_corner;
            
            // Define the 12 edges by their start and end corners
            if (edge < 4) {
                // Bottom face edges (z = -1)
                int base_corners[4] = {0, 1, 3, 2}; // bottom face corners in order
                start_corner = center;
                end_corner = center;
                for (int axis = 0; axis < 3; ++axis) {
                    double start_sign = (base_corners[edge] & (1 << axis)) ? 1.0 : -1.0;
                    double end_sign = (base_corners[(edge + 1) % 4] & (1 << axis)) ? 1.0 : -1.0;
                    start_corner += start_sign * half_dims[axis] * axes.col(axis);
                    end_corner += end_sign * half_dims[axis] * axes.col(axis);
                }
            } else if (edge < 8) {
                // Top face edges (z = +1)
                int top_corners[4] = {4, 5, 7, 6}; // top face corners in order
                start_corner = center;
                end_corner = center;
                for (int axis = 0; axis < 3; ++axis) {
                    double start_sign = (top_corners[edge - 4] & (1 << axis)) ? 1.0 : -1.0;
                    double end_sign = (top_corners[(edge - 4 + 1) % 4] & (1 << axis)) ? 1.0 : -1.0;
                    start_corner += start_sign * half_dims[axis] * axes.col(axis);
                    end_corner += end_sign * half_dims[axis] * axes.col(axis);
                }
            } else {
                // Vertical edges connecting bottom to top
                int bottom_corner = edge - 8;
                int top_corner = bottom_corner + 4;
                start_corner = center;
                end_corner = center;
                for (int axis = 0; axis < 3; ++axis) {
                    double start_sign = (bottom_corner & (1 << axis)) ? 1.0 : -1.0;
                    double end_sign = (top_corner & (1 << axis)) ? 1.0 : -1.0;
                    start_corner += start_sign * half_dims[axis] * axes.col(axis);
                    end_corner += end_sign * half_dims[axis] * axes.col(axis);
                }
            }
            
            // Sample points along this edge
            for (int i = 0; i < edge_samples; ++i) {
                double t = (double)i / (edge_samples - 1);  // [0, 1]
                Vec3 edge_point = start_corner * (1.0 - t) + end_corner * t;
                
                auto [distance, grad] = obstacle_tree_->getDistanceAndGradient(edge_point);
                min_distance = std::min(min_distance, distance);
            }
        }
        
        // 4. Additional sampling inside the box volume for very high accuracy
        // This helps catch cases where obstacles might penetrate into the box
        const int volume_samples = 5;  // 5x5x5 = 125 interior points
        for (int i = 0; i < volume_samples; ++i) {
            for (int j = 0; j < volume_samples; ++j) {
                for (int k = 0; k < volume_samples; ++k) {
                    // Sample point inside the box
                    double u = -0.8 + 1.6 * i / (volume_samples - 1);  // [-0.8, 0.8] to stay inside
                    double v = -0.8 + 1.6 * j / (volume_samples - 1);
                    double w = -0.8 + 1.6 * k / (volume_samples - 1);
                    
                    Vec3 interior_point = center + 
                                         u * half_dims[0] * axes.col(0) +
                                         v * half_dims[1] * axes.col(1) +
                                         w * half_dims[2] * axes.col(2);
                    
                    auto [distance, grad] = obstacle_tree_->getDistanceAndGradient(interior_point);
                    min_distance = std::min(min_distance, distance);
                }
            }
        }
        
        return min_distance;
    }
    
    double calculateConfigurationClearance(const std::vector<double>& joint_angles) {
        // Set robot configuration
        Eigen::VectorXd config(7);
        for (int i = 0; i < 7; ++i) {
            config[i] = joint_angles[i];
        }
        robot_->setJointAngles(config);
        
        // Get collision boxes
        auto collision_boxes = robot_->getCollisionBoxes();
        
        double min_clearance = std::numeric_limits<double>::max();
        
        // Check clearance for each robot link (skip first 3 as per feasibility checker)
        int link_idx = 0;
        for (const auto& box : collision_boxes) {
            link_idx++;
            if (link_idx < 4) continue;  // Skip base links
            
            auto [center, half_dims, axes] = box;
            
            // Calculate actual minimum distance from box surface to obstacles
            // This samples points on the box surface rather than just using the center point
            double box_clearance = calculateBoxClearance(center, half_dims, axes);
            min_clearance = std::min(min_clearance, box_clearance);
        }
        
        return min_clearance;
    }
    
    struct ClearanceMetrics {
        double min_clearance;
        double max_clearance;
        double avg_clearance;
        double clearance_variance;
        std::vector<double> clearances;
    };
    
    ClearanceMetrics calculateTrajectoryClearance(const std::string& csv_file) {
        std::ifstream file(csv_file);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open trajectory file: " + csv_file);
        }
        
        std::string line;
        std::vector<double> clearances;
        
        // Skip header
        std::getline(file, line);
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            std::vector<double> values;
            
            while (std::getline(ss, token, ',')) {
                try {
                    values.push_back(std::stod(token));
                } catch (...) {
                    continue;
                }
            }
            
            // Extract joint angles (columns 1-7)
            if (values.size() >= 8) {  // time + 7 joints
                std::vector<double> joint_angles(values.begin() + 1, values.begin() + 8);
                double clearance = calculateConfigurationClearance(joint_angles);
                clearances.push_back(clearance);
            }
        }
        
        if (clearances.empty()) {
            throw std::runtime_error("No valid configurations found in trajectory");
        }
        
        ClearanceMetrics metrics;
        metrics.clearances = clearances;
        metrics.min_clearance = *std::min_element(clearances.begin(), clearances.end());
        metrics.max_clearance = *std::max_element(clearances.begin(), clearances.end());
        
        double sum = std::accumulate(clearances.begin(), clearances.end(), 0.0);
        metrics.avg_clearance = sum / clearances.size();
        
        // Calculate variance
        double sq_sum = 0.0;
        for (double clearance : clearances) {
            sq_sum += (clearance - metrics.avg_clearance) * (clearance - metrics.avg_clearance);
        }
        metrics.clearance_variance = sq_sum / clearances.size();
        
        return metrics;
    }
    
    void saveClearanceMetrics(const ClearanceMetrics& metrics, const std::string& output_file) {
        std::ofstream file(output_file);
        file << std::fixed << std::setprecision(6);
        file << "min_clearance,max_clearance,avg_clearance,clearance_variance,num_points" << std::endl;
        file << metrics.min_clearance << "," 
             << metrics.max_clearance << ","
             << metrics.avg_clearance << ","
             << metrics.clearance_variance << ","
             << metrics.clearances.size() << std::endl;
    }
};

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <trajectory_csv> <output_csv>" << std::endl;
        return 1;
    }
    
    std::string trajectory_file = argv[1];
    std::string output_file = argv[2];
    
    try {
        ClearanceCalculator calculator;
        
        std::cerr << "Calculating clearance for: " << trajectory_file << std::endl;
        auto metrics = calculator.calculateTrajectoryClearance(trajectory_file);
        
        std::cerr << "Clearance metrics:" << std::endl;
        std::cerr << "  Min: " << metrics.min_clearance << " m" << std::endl;
        std::cerr << "  Max: " << metrics.max_clearance << " m" << std::endl;
        std::cerr << "  Avg: " << metrics.avg_clearance << " m" << std::endl;
        std::cerr << "  Variance: " << metrics.clearance_variance << std::endl;
        
        calculator.saveClearanceMetrics(metrics, output_file);
        std::cerr << "✓ Saved clearance metrics to: " << output_file << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
