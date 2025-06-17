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
 * Clearance Calculator
 * ===================
 * This utility calculates robot arm clearance metrics for trajectory CSV files.
 * It can be called from Python to compute clearance statistics.
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
            
            // Calculate minimum distance to obstacles using center point
            auto [distance, _] = obstacle_tree_->getDistanceAndGradient(center);
            min_clearance = std::min(min_clearance, distance);
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
