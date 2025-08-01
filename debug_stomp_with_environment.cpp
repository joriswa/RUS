/**
 * @brief Enhanced STOMP debug script using USTrajectoryPlanner infrastructure
 * 
 * This script uses the same environment parsing and obstacle handling as the main system.
 * You can pass XML environment files and test STOMP with realistic obstacle scenarios.
 * 
 * Usage: ./debug_stomp_with_environment [environment_file.xml] [scenario]
 * 
 * Examples:
 *   ./debug_stomp_with_environment res/environments/simple_table.xml fast
 *   ./debug_stomp_with_environment res/environments/complex_scene.xml thorough
 */

#include "USLib/USTrajectoryPlanner.h"
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Robot/RobotArm.h"
#include "TrajectoryLib/Logger.h"
#include <iostream>
#include <fstream>
#include <filesystem>

struct DebugScenario {
    std::string name;
    std::string description;
    Eigen::VectorXd startJoints;
    Eigen::VectorXd goalJoints;
    StompConfig config;
};

void printUsage() {
    std::cout << "\n=== STOMP Environment Debug Script ===" << std::endl;
    std::cout << "Usage: ./debug_stomp_with_environment [environment_file.xml] [scenario]" << std::endl;
    std::cout << "\nScenarios:" << std::endl;
    std::cout << "  fast       - Quick testing (8 samples, 30 iterations, 15s timeout)" << std::endl;
    std::cout << "  thorough   - Balanced testing (16 samples, 100 iterations, 60s timeout)" << std::endl;
    std::cout << "  exploration- Difficult problems (24 samples, 150 iterations, 120s timeout)" << std::endl;
    std::cout << "  minimal    - Minimal testing (4 samples, 20 iterations, 10s timeout)" << std::endl;
    std::cout << "\nEnvironment files should be in URDF/XML format with obstacle definitions." << std::endl;
    std::cout << "\nExamples:" << std::endl;
    std::cout << "  ./debug_stomp_with_environment res/environments/simple_table.xml fast" << std::endl;
    std::cout << "  ./debug_stomp_with_environment res/environments/complex_scene.xml thorough" << std::endl;
}

StompConfig createScenarioConfig(const std::string& scenario) {
    StompConfig config;
    
    // Default base configuration
    config.numJoints = 7;
    config.N = 30;
    config.dt = 0.1;
    config.learningRate = 0.08;
    config.temperature = 10.0;
    config.numBestSamples = 4;
    config.enableEarlyStopping = true;
    config.obstacleCostWeight = 2.0;
    config.constraintCostWeight = 1.0;
    config.outputFrequency = 1000.0;
    config.disableInternalParallelization = false;
    
    // Scenario-specific adjustments
    if (scenario == "fast") {
        config.numNoisyTrajectories = 8;
        config.maxIterations = 30;
        config.earlyStoppingPatience = 8;
        config.maxComputeTimeMs = 15000.0;
        config.learningRate = 0.12;
        config.temperature = 8.0;
        config.jointStdDevs = (Eigen::VectorXd(7) << 0.06, 0.10, 0.06, 0.04, 0.03, 0.08, 0.03).finished();
        
    } else if (scenario == "thorough") {
        config.numNoisyTrajectories = 16;
        config.maxIterations = 100;
        config.earlyStoppingPatience = 15;
        config.maxComputeTimeMs = 60000.0;
        config.learningRate = 0.08;
        config.temperature = 10.0;
        config.jointStdDevs = (Eigen::VectorXd(7) << 0.08, 0.12, 0.08, 0.05, 0.04, 0.10, 0.04).finished();
        
    } else if (scenario == "exploration") {
        config.numNoisyTrajectories = 24;
        config.maxIterations = 150;
        config.earlyStoppingPatience = 20;
        config.maxComputeTimeMs = 120000.0;
        config.learningRate = 0.06;
        config.temperature = 12.0;
        config.jointStdDevs = (Eigen::VectorXd(7) << 0.10, 0.15, 0.10, 0.07, 0.05, 0.12, 0.05).finished();
        
    } else if (scenario == "minimal") {
        config.numNoisyTrajectories = 4;
        config.maxIterations = 20;
        config.earlyStoppingPatience = 5;
        config.maxComputeTimeMs = 10000.0;
        config.learningRate = 0.15;
        config.temperature = 6.0;
        config.jointStdDevs = (Eigen::VectorXd(7) << 0.04, 0.06, 0.04, 0.03, 0.02, 0.05, 0.02).finished();
        
    } else {
        std::cout << "Unknown scenario '" << scenario << "', using 'thorough' defaults" << std::endl;
        return createScenarioConfig("thorough");
    }
    
    std::cout << "\nSTOMP Configuration (" << scenario << "):" << std::endl;
    std::cout << "- Noisy trajectories: " << config.numNoisyTrajectories << std::endl;
    std::cout << "- Max iterations: " << config.maxIterations << std::endl;
    std::cout << "- Early stopping patience: " << config.earlyStoppingPatience << std::endl;
    std::cout << "- Timeout: " << config.maxComputeTimeMs/1000.0 << "s" << std::endl;
    std::cout << "- Learning rate: " << config.learningRate << std::endl;
    std::cout << "- Temperature: " << config.temperature << std::endl;
    std::cout << "- Obstacle cost weight: " << config.obstacleCostWeight << std::endl;
    std::cout << "- Constraint cost weight: " << config.constraintCostWeight << std::endl;
    
    return config;
}

std::vector<DebugScenario> createDebugScenarios() {
    std::vector<DebugScenario> scenarios;
    
    // Scenario 1: Simple reach
    DebugScenario simple;
    simple.name = "Simple Reach";
    simple.description = "Basic reach movement to test STOMP functionality";
    simple.startJoints = (Eigen::VectorXd(7) << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785).finished(); // Home
    simple.goalJoints = (Eigen::VectorXd(7) << 0.3, -0.5, 0.1, -2.0, 0.1, 1.8, 1.0).finished(); // Side reach
    scenarios.push_back(simple);
    
    // Scenario 2: Moderate challenge
    DebugScenario moderate;
    moderate.name = "Moderate Challenge";
    moderate.description = "More complex movement requiring obstacle avoidance";
    moderate.startJoints = (Eigen::VectorXd(7) << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785).finished();
    moderate.goalJoints = (Eigen::VectorXd(7) << 0.8, 0.2, 0.4, -1.5, -0.3, 2.2, 0.6).finished(); // Around obstacles
    scenarios.push_back(moderate);
    
    // Scenario 3: High constraint challenge
    DebugScenario challenge;
    challenge.name = "High Constraint Challenge";
    challenge.description = "Difficult pose requiring significant joint motion";
    challenge.startJoints = (Eigen::VectorXd(7) << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785).finished();
    challenge.goalJoints = (Eigen::VectorXd(7) << -0.6, 0.6, -0.4, -1.2, 1.0, 2.4, -0.2).finished(); // Extreme pose
    scenarios.push_back(challenge);
    
    // Scenario 4: Collision test
    DebugScenario collision;
    collision.name = "Collision Test";
    collision.description = "Tests STOMP behavior with difficult collision scenarios";
    collision.startJoints = (Eigen::VectorXd(7) << 0.2, -0.5, 0.1, -2.0, 0.0, 1.5, 0.8).finished();
    collision.goalJoints = (Eigen::VectorXd(7) << 1.2, 0.5, 0.6, -1.0, -0.8, 2.8, 0.3).finished(); // Through obstacles
    scenarios.push_back(collision);
    
    return scenarios;
}

void analyzeEnvironment(UltrasoundScanTrajectoryPlanner& planner) {
    auto obstacleTree = planner.getObstacleTree();
    if (!obstacleTree) {
        std::cout << "No obstacle tree loaded" << std::endl;
        return;
    }
    
    std::cout << "\n=== ENVIRONMENT ANALYSIS ===" << std::endl;
    // You can add more detailed analysis here based on your BVHTree interface
    std::cout << "Obstacle tree loaded successfully" << std::endl;
}

void testStartGoalPositions(UltrasoundScanTrajectoryPlanner& planner, 
                          const Eigen::VectorXd& startJoints, 
                          const Eigen::VectorXd& goalJoints) {
    std::cout << "\n=== START/GOAL VALIDATION ===" << std::endl;
    
    // Test start position
    auto* motionGen = planner.getMotionGenerator();
    if (motionGen) {
        bool startCollisionFree = !motionGen->armHasCollision(std::vector<double>(
            startJoints.data(), startJoints.data() + startJoints.size()));
        bool goalCollisionFree = !motionGen->armHasCollision(std::vector<double>(
            goalJoints.data(), goalJoints.data() + goalJoints.size()));
        
        std::cout << "Start position: [";
        for (int i = 0; i < 7; ++i) {
            std::cout << std::fixed << std::setprecision(3) << startJoints[i];
            if (i < 6) std::cout << ", ";
        }
        std::cout << "] - " << (startCollisionFree ? "✓ collision-free" : "✗ COLLISION") << std::endl;
        
        std::cout << "Goal position:  [";
        for (int i = 0; i < 7; ++i) {
            std::cout << std::fixed << std::setprecision(3) << goalJoints[i];
            if (i < 6) std::cout << ", ";
        }
        std::cout << "] - " << (goalCollisionFree ? "✓ collision-free" : "✗ COLLISION") << std::endl;
        
        if (!startCollisionFree) {
            std::cout << "⚠️  WARNING: Start position is in collision!" << std::endl;
        }
        if (!goalCollisionFree) {
            std::cout << "⚠️  WARNING: Goal position is in collision!" << std::endl;
        }
    }
}

void runStompDirectly(UltrasoundScanTrajectoryPlanner& planner, 
                     const Eigen::VectorXd& startJoints,
                     const Eigen::VectorXd& goalJoints,
                     const StompConfig& config) {
    
    std::cout << "\n=== RUNNING DIRECT STOMP TEST ===" << std::endl;
    
    auto* motionGen = planner.getMotionGenerator();
    if (!motionGen) {
        std::cout << "❌ No motion generator available" << std::endl;
        return;
    }
    
    // Set up waypoints
    Eigen::MatrixXd waypoints(2, 7);
    waypoints.row(0) = startJoints;
    waypoints.row(1) = goalJoints;
    motionGen->setWaypoints(waypoints);
    
    std::cout << "Running STOMP optimization..." << std::endl;
    std::cout << "This may take up to " << config.maxComputeTimeMs/1000.0 << " seconds..." << std::endl;
    
    bool success = false;
    try {
        success = motionGen->performSTOMP(config);
        
        if (success) {
            std::cout << "\n🎉 STOMP SUCCEEDED! 🎉" << std::endl;
            
            auto trajectory = motionGen->getPath();
            std::cout << "Generated trajectory: " << trajectory.size() << " points" << std::endl;
            
            if (!trajectory.empty()) {
                std::cout << "Total time: " << std::fixed << std::setprecision(3) 
                          << trajectory.back().time << "s" << std::endl;
                
                // Save trajectory
                std::string filename = "debug_stomp_environment_trajectory.csv";
                motionGen->saveTrajectoryToCSV(filename);
                std::cout << "Trajectory saved to: " << filename << std::endl;
            }
            
        } else {
            std::cout << "\n❌ STOMP FAILED ❌" << std::endl;
        }
        
    } catch (const StompTimeoutException& e) {
        std::cout << "\n⏰ STOMP TIMED OUT ⏰" << std::endl;
        std::cout << "Reason: " << e.what() << std::endl;
        
    } catch (const StompFailedException& e) {
        std::cout << "\n💥 STOMP FAILED WITH EXCEPTION 💥" << std::endl;
        std::cout << "Reason: " << e.what() << std::endl;
    }
}

void runUSTrajectoryPlannerTest(UltrasoundScanTrajectoryPlanner& planner,
                               const Eigen::VectorXd& startJoints,
                               const Eigen::VectorXd& goalJoints) {
    
    std::cout << "\n=== RUNNING US TRAJECTORY PLANNER TEST ===" << std::endl;
    
    // Set current joints
    planner.setCurrentJoints(startJoints);
    
    // Create a single target pose from goal joints
    RobotArm goalArm(*planner.getMotionGenerator()->_arm); // Access arm from motion generator
    goalArm.setJointAngles(goalJoints);
    
    // Set single pose for planning
    std::vector<Eigen::Affine3d> poses;
    poses.push_back(goalArm.getEndEffectorPose());
    planner.setPoses(poses);
    
    std::cout << "Planning single trajectory using USTrajectoryPlanner..." << std::endl;
    
    try {
        bool success = planner.planTrajectories(false, true); // Use STOMP, enable shortcutting
        
        if (success) {
            std::cout << "\n🎉 US TRAJECTORY PLANNER SUCCEEDED! 🎉" << std::endl;
            
            auto trajectories = planner.getTrajectories();
            std::cout << "Generated " << trajectories.size() << " trajectory segments" << std::endl;
            
            for (size_t i = 0; i < trajectories.size(); ++i) {
                const auto& segment = trajectories[i].first;
                bool isContactForce = trajectories[i].second;
                
                std::cout << "Segment " << i << ": " << segment.size() << " points, "
                          << (isContactForce ? "Contact Force" : "Free Movement");
                if (!segment.empty()) {
                    std::cout << ", duration: " << std::fixed << std::setprecision(3) 
                              << segment.back().time << "s";
                }
                std::cout << std::endl;
            }
            
        } else {
            std::cout << "\n❌ US TRAJECTORY PLANNER FAILED ❌" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cout << "\n💥 US TRAJECTORY PLANNER EXCEPTION 💥" << std::endl;
        std::cout << "Reason: " << e.what() << std::endl;
    }
}

int main(int argc, char* argv[]) {
    try {
        // Parse command line arguments
        std::string environmentFile = "res/environments/default.xml";
        std::string scenario = "thorough";
        
        if (argc >= 2) {
            environmentFile = argv[1];
        }
        if (argc >= 3) {
            scenario = argv[2];
        }
        
        printUsage();
        
        // Check if environment file exists
        if (!std::filesystem::exists(environmentFile)) {
            std::cout << "\n❌ Environment file not found: " << environmentFile << std::endl;
            std::cout << "Creating a simple default environment..." << std::endl;
            // You could create a simple default URDF here or use existing files
            std::cout << "Please provide a valid URDF environment file." << std::endl;
            return 1;
        }
        
        // Enable debug logging
        LOGGING_ENABLE_ALL();
        std::cout << "\n=== DEBUG LOGGING ENABLED ===" << std::endl;
        
        std::cout << "\nLoading environment: " << environmentFile << std::endl;
        std::cout << "Using scenario: " << scenario << std::endl;
        
        // Create US trajectory planner with environment
        UltrasoundScanTrajectoryPlanner planner(environmentFile);
        std::cout << "✓ US Trajectory Planner created" << std::endl;
        
        // Analyze the loaded environment
        analyzeEnvironment(planner);
        
        // Get test scenarios
        auto scenarios = createDebugScenarios();
        std::cout << "\nAvailable test scenarios:" << std::endl;
        for (size_t i = 0; i < scenarios.size(); ++i) {
            std::cout << "  " << i+1 << ". " << scenarios[i].name << " - " << scenarios[i].description << std::endl;
        }
        
        // Use the first scenario (or allow user to select)
        auto testScenario = scenarios[0]; // Simple reach by default
        std::cout << "\nUsing scenario: " << testScenario.name << std::endl;
        
        // Create configuration
        StompConfig config = createScenarioConfig(scenario);
        
        // Test start/goal positions
        testStartGoalPositions(planner, testScenario.startJoints, testScenario.goalJoints);
        
        // Run direct STOMP test
        runStompDirectly(planner, testScenario.startJoints, testScenario.goalJoints, config);
        
        // Run full US trajectory planner test
        runUSTrajectoryPlannerTest(planner, testScenario.startJoints, testScenario.goalJoints);
        
        std::cout << "\n=== DEBUG SESSION COMPLETE ===" << std::endl;
        std::cout << "\nDebugging tips:" << std::endl;
        std::cout << "1. Check debug output for detailed STOMP analysis" << std::endl;
        std::cout << "2. Try different scenarios with: " << argv[0] << " " << environmentFile << " [fast|thorough|exploration|minimal]" << std::endl;
        std::cout << "3. Modify start/goal positions in the code for custom tests" << std::endl;
        std::cout << "4. Analyze the generated CSV trajectory file" << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "\n💥 FATAL ERROR 💥" << std::endl;
        std::cout << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
