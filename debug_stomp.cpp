#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Robot/RobotArm.h"
#include "GeometryLib/BVHTree.h"
#include "TrajectoryLib/Logger.h"
#include "debug_stomp_config.h"
#include <iostream>
#include <vector>
#include <memory>

/**
 * @brief Debug script for testing STOMP with a single target pose
 * 
 * Usage: ./debug_stomp [scenario] [config]
 * 
 * Scenarios: simple, moderate, challenge, collision, custom
 * Configs: fast, thorough, exploration
 * 
 * Example: ./debug_stomp moderate fast
 */

void printUsage() {
    std::cout << "\n=== STOMP Debug Script ===" << std::endl;
    std::cout << "Usage: ./debug_stomp [scenario] [config]" << std::endl;
    std::cout << "\nScenarios:" << std::endl;
    std::cout << "  simple     - " << StompDebugScenarios::SimpleReach::getDescription() << std::endl;
    std::cout << "  moderate   - " << StompDebugScenarios::ModerateChallenge::getDescription() << std::endl;
    std::cout << "  challenge  - " << StompDebugScenarios::HighConstraintChallenge::getDescription() << std::endl;
    std::cout << "  collision  - " << StompDebugScenarios::CollisionTest::getDescription() << std::endl;
    std::cout << "  custom     - " << StompDebugScenarios::Custom::getDescription() << std::endl;
    std::cout << "\nConfigs:" << std::endl;
    std::cout << "  fast       - Quick testing (6 samples, 30 iterations, 15s timeout)" << std::endl;
    std::cout << "  thorough   - Balanced testing (16 samples, 100 iterations, 60s timeout)" << std::endl;
    std::cout << "  exploration- Difficult problems (20 samples, 150 iterations, 120s timeout)" << std::endl;
    std::cout << "\nExample: ./debug_stomp moderate thorough" << std::endl;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> getScenarioJoints(const std::string& scenario) {
    if (scenario == "simple") {
        return {StompDebugScenarios::SimpleReach::getStart(), StompDebugScenarios::SimpleReach::getGoal()};
    } else if (scenario == "moderate") {
        return {StompDebugScenarios::ModerateChallenge::getStart(), StompDebugScenarios::ModerateChallenge::getGoal()};
    } else if (scenario == "challenge") {
        return {StompDebugScenarios::HighConstraintChallenge::getStart(), StompDebugScenarios::HighConstraintChallenge::getGoal()};
    } else if (scenario == "collision") {
        return {StompDebugScenarios::CollisionTest::getStart(), StompDebugScenarios::CollisionTest::getGoal()};
    } else if (scenario == "custom") {
        return {StompDebugScenarios::Custom::getStart(), StompDebugScenarios::Custom::getGoal()};
    } else {
        std::cout << "Unknown scenario '" << scenario << "', using simple scenario." << std::endl;
        return {StompDebugScenarios::SimpleReach::getStart(), StompDebugScenarios::SimpleReach::getGoal()};
    }
}

StompConfig getConfigPreset(const std::string& configName) {
    if (configName == "fast") {
        return StompDebugConfigs::getFastConfig();
    } else if (configName == "thorough") {
        return StompDebugConfigs::getThoroughConfig();
    } else if (configName == "exploration") {
        return StompDebugConfigs::getHighExplorationConfig();
    } else {
        std::cout << "Unknown config '" << configName << "', using fast config." << std::endl;
        return StompDebugConfigs::getFastConfig();
    }
}

void printConfiguration(const StompConfig& config, const std::string& configName) {
    std::cout << "\nSTOMP Configuration (" << configName << "):" << std::endl;
    std::cout << "- Noisy trajectories: " << config.numNoisyTrajectories << std::endl;
    std::cout << "- Max iterations: " << config.maxIterations << std::endl;
    std::cout << "- Trajectory points: " << config.N << std::endl;
    std::cout << "- Learning rate: " << config.learningRate << std::endl;
    std::cout << "- Temperature: " << config.temperature << std::endl;
    std::cout << "- Timeout: " << config.maxComputeTimeMs/1000.0 << "s" << std::endl;
    std::cout << "- Obstacle cost weight: " << config.obstacleCostWeight << std::endl;
    std::cout << "- Constraint cost weight: " << config.constraintCostWeight << std::endl;
}

void printJointAngles(const Eigen::VectorXd& joints, const std::string& label) {
    std::cout << label << " joint angles (rad): [";
    for (int i = 0; i < joints.size(); ++i) {
        std::cout << std::fixed << std::setprecision(3) << joints[i];
        if (i < joints.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

void analyzeTrajectory(MotionGenerator& motionGen, const StompConfig& config) {
    auto trajectory = motionGen.getPath();
    
    std::cout << "\n=== TRAJECTORY ANALYSIS ===" << std::endl;
    std::cout << "Generated trajectory points: " << trajectory.size() << std::endl;
    
    if (!trajectory.empty()) {
        std::cout << "Total trajectory time: " << trajectory.back().time << "s" << std::endl;
        
        // Check for constraint violations
        double maxVel = 0.0, maxAcc = 0.0;
        for (const auto& point : trajectory) {
            for (size_t i = 0; i < point.velocity.size(); ++i) {
                maxVel = std::max(maxVel, std::abs(point.velocity[i]));
            }
            for (size_t i = 0; i < point.acceleration.size(); ++i) {
                maxAcc = std::max(maxAcc, std::abs(point.acceleration[i]));
            }
        }
        
        std::cout << "Max velocity: " << maxVel << " rad/s" << std::endl;
        std::cout << "Max acceleration: " << maxAcc << " rad/s²" << std::endl;
        
        // Check for collisions at key points
        std::cout << "\nCollision check at key points:" << std::endl;
        std::vector<int> checkPoints = {0, trajectory.size()/4, trajectory.size()/2, 3*trajectory.size()/4, trajectory.size()-1};
        
        for (int idx : checkPoints) {
            if (idx < trajectory.size()) {
                bool hasCollision = motionGen.armHasCollision(trajectory[idx].position);
                std::cout << "Point " << idx << " (t=" << trajectory[idx].time << "s): " 
                          << (hasCollision ? "COLLISION" : "collision-free") << std::endl;
            }
        }
    }
}

void saveDebugTrajectory(MotionGenerator& motionGen) {
    std::string filename = "debug_stomp_trajectory.csv";
    motionGen.saveTrajectoryToCSV(filename);
    std::cout << "\nTrajectory saved to: " << filename << std::endl;
    std::cout << "You can analyze this file with plotting tools." << std::endl;
}
    auto trajectory = motionGen.getPath();
    
    std::cout << "\n=== TRAJECTORY ANALYSIS ===" << std::endl;
    std::cout << "Generated trajectory points: " << trajectory.size() << std::endl;
    
    if (!trajectory.empty()) {
        std::cout << "Total trajectory time: " << trajectory.back().time << "s" << std::endl;
        
        // Check for constraint violations
        double maxVel = 0.0, maxAcc = 0.0;
        for (const auto& point : trajectory) {
            for (size_t i = 0; i < point.velocity.size(); ++i) {
                maxVel = std::max(maxVel, std::abs(point.velocity[i]));
            }
            for (size_t i = 0; i < point.acceleration.size(); ++i) {
                maxAcc = std::max(maxAcc, std::abs(point.acceleration[i]));
            }
        }
        
        std::cout << "Max velocity: " << maxVel << " rad/s" << std::endl;
        std::cout << "Max acceleration: " << maxAcc << " rad/s²" << std::endl;
        
        // Check for collisions at key points
        std::cout << "\nCollision check at key points:" << std::endl;
        std::vector<int> checkPoints = {0, trajectory.size()/4, trajectory.size()/2, 3*trajectory.size()/4, trajectory.size()-1};
        
        for (int idx : checkPoints) {
            if (idx < trajectory.size()) {
                bool hasCollision = motionGen.armHasCollision(trajectory[idx].position);
                std::cout << "Point " << idx << " (t=" << trajectory[idx].time << "s): " 
                          << (hasCollision ? "COLLISION" : "collision-free") << std::endl;
            }
        }
    }
}

void saveDebugTrajectory(MotionGenerator& motionGen) {
    std::string filename = "debug_stomp_trajectory.csv";
    motionGen.saveTrajectoryToCSV(filename);
    std::cout << "\nTrajectory saved to: " << filename << std::endl;
    std::cout << "You can analyze this file with plotting tools." << std::endl;
}

int main(int argc, char* argv[]) {
    try {
        // Parse command line arguments
        std::string scenario = (argc > 1) ? argv[1] : "simple";
        std::string configName = (argc > 2) ? argv[2] : "fast";
        
        printUsage();
        
        // Enable all debug logging
        LOGGING_ENABLE_ALL();
        std::cout << "\n=== DEBUG LOGGING ENABLED ===" << std::endl;
        
        // Get scenario configuration
        auto [startJoints, goalJoints] = getScenarioJoints(scenario);
        StompConfig config = getConfigPreset(configName);
        
        std::cout << "\n=== SCENARIO: " << scenario << " ===" << std::endl;
        printJointAngles(startJoints, "Start");
        printJointAngles(goalJoints, "Goal");
        printConfiguration(config, configName);
        
        // Create robot arm (you may need to adjust this based on your RobotArm constructor)
        RobotArm arm; // Adjust constructor parameters as needed
        std::cout << "\nRobot arm created successfully." << std::endl;
        
        // Create obstacle tree
        auto obstacleTree = std::make_shared<BVHTree>();
        
        // Add obstacles based on scenario
        if (scenario == "simple") {
            ObstacleScenarios::addNoObstacles();
        } else {
            ObstacleScenarios::addTableObstacle();
            if (scenario == "challenge" || scenario == "collision") {
                ObstacleScenarios::addWallObstacle();
            }
        }
        
        // Create motion generator
        MotionGenerator motionGen(arm);
        motionGen.setObstacleTree(obstacleTree);
        std::cout << "\nMotion generator initialized." << std::endl;
        
        // Check if start and goal are collision-free
        std::cout << "\n=== INITIAL COLLISION CHECK ===" << std::endl;
        std::vector<double> startVec(startJoints.data(), startJoints.data() + startJoints.size());
        std::vector<double> goalVec(goalJoints.data(), goalJoints.data() + goalJoints.size());
        
        bool startCollisionFree = !motionGen.armHasCollision(startVec);
        bool goalCollisionFree = !motionGen.armHasCollision(goalVec);
        
        std::cout << "Start pose collision-free: " << (startCollisionFree ? "YES" : "NO") << std::endl;
        std::cout << "Goal pose collision-free: " << (goalCollisionFree ? "YES" : "NO") << std::endl;
        
        if (!startCollisionFree) {
            std::cout << "⚠️  WARNING: Start pose is in collision! STOMP will likely fail." << std::endl;
        }
        if (!goalCollisionFree) {
            std::cout << "⚠️  WARNING: Goal pose is in collision! STOMP will likely fail." << std::endl;
        }
        
        // Create waypoints matrix
        Eigen::MatrixXd waypoints(2, 7);
        waypoints.row(0) = startJoints;
        waypoints.row(1) = goalJoints;
        motionGen.setWaypoints(waypoints);
        
        std::cout << "\n=== RUNNING STOMP OPTIMIZATION ===" << std::endl;
        std::cout << "Scenario: " << scenario << ", Config: " << configName << std::endl;
        std::cout << "This may take up to " << config.maxComputeTimeMs/1000.0 << " seconds..." << std::endl;
        
        // Run STOMP with detailed debug output
        bool success = false;
        try {
            success = motionGen.performSTOMP(config);
            
            if (success) {
                std::cout << "\n🎉 STOMP SUCCEEDED! 🎉" << std::endl;
                analyzeTrajectory(motionGen, config);
                saveDebugTrajectory(motionGen);
            } else {
                std::cout << "\n❌ STOMP FAILED ❌" << std::endl;
                std::cout << "Check the debug output above for failure reasons." << std::endl;
            }
            
        } catch (const StompTimeoutException& e) {
            std::cout << "\n⏰ STOMP TIMED OUT ⏰" << std::endl;
            std::cout << "Reason: " << e.what() << std::endl;
            std::cout << "💡 Try: ./debug_stomp " << scenario << " thorough" << std::endl;
            
        } catch (const StompFailedException& e) {
            std::cout << "\n💥 STOMP FAILED WITH EXCEPTION 💥" << std::endl;
            std::cout << "Reason: " << e.what() << std::endl;
            std::cout << "Check debug output for detailed failure analysis." << std::endl;
        }
        
        std::cout << "\n=== DEBUG SESSION COMPLETE ===" << std::endl;
        std::cout << "\n💡 Debugging Tips:" << std::endl;
        std::cout << "1. ✅ Check if start/goal poses are collision-free" << std::endl;
        std::cout << "2. 🔍 Look for high obstacle or constraint costs in debug output" << std::endl;
        std::cout << "3. 📊 Check if collision-free samples are being generated" << std::endl;
        std::cout << "4. 📈 Verify trajectory change magnitude > 0" << std::endl;
        std::cout << "5. ⚙️  Consider adjusting cost weights or STOMP parameters" << std::endl;
        
        std::cout << "\n🚀 Try different scenarios:" << std::endl;
        std::cout << "  ./debug_stomp simple fast      (easiest)" << std::endl;
        std::cout << "  ./debug_stomp moderate thorough (balanced)" << std::endl;
        std::cout << "  ./debug_stomp challenge exploration (hardest)" << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "\n💥 FATAL ERROR 💥" << std::endl;
        std::cout << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
