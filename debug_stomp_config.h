#pragma once

#include <eigen3/Eigen/Core>
#include "TrajectoryLib/Motion/MotionGenerator.h"

/**
 * @brief Configuration scenarios for STOMP debugging
 * 
 * Modify these scenarios to test different cases
 */

namespace StompDebugScenarios {

    /**
     * @brief Test Case 1: Simple reach movement
     */
    struct SimpleReach {
        static Eigen::VectorXd getStart() {
            Eigen::VectorXd joints(7);
            joints << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785; // Franka home
            return joints;
        }
        
        static Eigen::VectorXd getGoal() {
            Eigen::VectorXd joints(7);
            joints << 0.3, -0.5, 0.1, -2.0, 0.05, 1.8, 0.9; // Slight movement
            return joints;
        }
        
        static std::string getDescription() {
            return "Simple reach movement - should succeed easily";
        }
    };

    /**
     * @brief Test Case 2: Moderate challenge with obstacles
     */
    struct ModerateChallenge {
        static Eigen::VectorXd getStart() {
            Eigen::VectorXd joints(7);
            joints << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
            return joints;
        }
        
        static Eigen::VectorXd getGoal() {
            Eigen::VectorXd joints(7);
            joints << 0.8, -0.2, 0.4, -1.8, -0.3, 2.2, 1.2; // More movement
            return joints;
        }
        
        static std::string getDescription() {
            return "Moderate challenge - tests obstacle avoidance";
        }
    };

    /**
     * @brief Test Case 3: High constraint challenge
     */
    struct HighConstraintChallenge {
        static Eigen::VectorXd getStart() {
            Eigen::VectorXd joints(7);
            joints << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
            return joints;
        }
        
        static Eigen::VectorXd getGoal() {
            Eigen::VectorXd joints(7);
            joints << 1.2, 0.5, -0.8, -1.0, 1.0, 2.8, -0.5; // Large movement
            return joints;
        }
        
        static std::string getDescription() {
            return "High constraint challenge - tests velocity/acceleration limits";
        }
    };

    /**
     * @brief Test Case 4: Collision test (goal in collision)
     */
    struct CollisionTest {
        static Eigen::VectorXd getStart() {
            Eigen::VectorXd joints(7);
            joints << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
            return joints;
        }
        
        static Eigen::VectorXd getGoal() {
            Eigen::VectorXd joints(7);
            // This pose should place the end-effector near the table obstacle
            joints << 0.5, 0.2, 0.0, -1.0, 0.0, 1.2, 0.785;
            return joints;
        }
        
        static std::string getDescription() {
            return "Collision test - goal may be in collision, tests recovery";
        }
    };

    /**
     * @brief Test Case 5: Custom scenario (modify as needed)
     */
    struct Custom {
        static Eigen::VectorXd getStart() {
            Eigen::VectorXd joints(7);
            // MODIFY THESE VALUES FOR YOUR TEST
            joints << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
            return joints;
        }
        
        static Eigen::VectorXd getGoal() {
            Eigen::VectorXd joints(7);
            // MODIFY THESE VALUES FOR YOUR TEST  
            joints << 0.5, -0.5, 0.2, -2.0, 0.1, 1.8, 1.0;
            return joints;
        }
        
        static std::string getDescription() {
            return "Custom scenario - modify the joint angles above";
        }
    };
}

/**
 * @brief STOMP configuration presets for different testing scenarios
 */
namespace StompDebugConfigs {
    
    /**
     * @brief Fast debug config - for quick testing
     */
    StompConfig getFastConfig() {
        StompConfig config;
        config.numNoisyTrajectories = 6;
        config.maxIterations = 30;
        config.N = 20;
        config.learningRate = 0.15;
        config.temperature = 3.0;
        config.enableEarlyStopping = true;
        config.earlyStoppingPatience = 8;
        config.maxComputeTimeMs = 15000.0; // 15 seconds
        config.obstacleCostWeight = 2.0;
        config.constraintCostWeight = 1.0;
        config.jointStdDevs = (Eigen::VectorXd(7) << 0.03, 0.05, 0.03, 0.02, 0.015, 0.04, 0.015).finished();
        return config;
    }
    
    /**
     * @brief Thorough config - for difficult problems
     */
    StompConfig getThoroughConfig() {
        StompConfig config;
        config.numNoisyTrajectories = 16;
        config.maxIterations = 100;
        config.N = 50;
        config.learningRate = 0.08;
        config.temperature = 8.0;
        config.enableEarlyStopping = true;
        config.earlyStoppingPatience = 20;
        config.maxComputeTimeMs = 60000.0; // 60 seconds
        config.obstacleCostWeight = 2.0;
        config.constraintCostWeight = 1.0;
        config.jointStdDevs = (Eigen::VectorXd(7) << 0.08, 0.12, 0.08, 0.05, 0.04, 0.10, 0.04).finished();
        return config;
    }
    
    /**
     * @brief High exploration config - for very difficult problems
     */
    StompConfig getHighExplorationConfig() {
        StompConfig config;
        config.numNoisyTrajectories = 20;
        config.maxIterations = 150;
        config.N = 40;
        config.learningRate = 0.05;
        config.temperature = 15.0;
        config.enableEarlyStopping = true;
        config.earlyStoppingPatience = 30;
        config.maxComputeTimeMs = 120000.0; // 2 minutes
        config.obstacleCostWeight = 1.5;
        config.constraintCostWeight = 0.8;
        config.jointStdDevs = (Eigen::VectorXd(7) << 0.12, 0.18, 0.12, 0.08, 0.06, 0.15, 0.06).finished();
        return config;
    }
}

/**
 * @brief Obstacle scenarios for testing
 */
namespace ObstacleScenarios {
    
    /**
     * @brief Simple table obstacle
     */
    void addTableObstacle() {
        std::cout << "Adding table obstacle (80cm x 120cm x 4cm) at (0.5, 0.0, 0.4)" << std::endl;
        // Implementation depends on your BVHTree interface
    }
    
    /**
     * @brief Wall obstacle
     */
    void addWallObstacle() {
        std::cout << "Adding wall obstacle (4cm x 60cm x 100cm) at (0.8, 0.5, 0.8)" << std::endl;
        // Implementation depends on your BVHTree interface
    }
    
    /**
     * @brief Multiple obstacles - challenging scenario
     */
    void addMultipleObstacles() {
        std::cout << "Adding multiple obstacles for challenging scenario" << std::endl;
        addTableObstacle();
        addWallObstacle();
        // Add more obstacles as needed
    }
    
    /**
     * @brief No obstacles - test pure constraint optimization
     */
    void addNoObstacles() {
        std::cout << "No obstacles - testing pure constraint optimization" << std::endl;
    }
}
