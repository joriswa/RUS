#include <iostream>
#include <string>
#include <vector>
#include <memory>

// TrajectoryLib includes
#include "TrajectoryLib/Robot/RobotArm.h"
#include "TrajectoryLib/Robot/RobotManager.h"
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Logger.h"

// GeometryLib includes
#include "GeometryLib/BVHTree.h"

int main(int argc, char* argv[])
{
    // Use fixed obstacles file from ParameterTuning data
    std::string obstacleFile = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning/Data/Ext_Ant_Scan/obstacles.xml";
    
    // Disable debug logging for cleaner output
    Logger::instance().setDebugEnabled(false);
    
    LOG_INFO << "STOMP Debugger Starting";
    LOG_INFO << "Obstacle file: " << obstacleFile;
    
    try {
        // 1. Create robot arm from URDF
        std::string robotUrdf = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf";
        LOG_INFO << "Loading robot: " << robotUrdf;
        RobotArm arm(robotUrdf);
        
        // 2. Parse obstacles from XML environment file
        LOG_INFO << "Parsing obstacles from: " << obstacleFile;
        RobotManager robotManager;
        robotManager.parseURDF(obstacleFile);
        qDebug() << "Parsed " << robotManager.getTransformedObstacles().size() << " obstacles";
        // 3. Create BVH tree from obstacles  
        LOG_INFO << "Creating BVH tree from obstacles";
        auto obstacleTree = std::make_shared<BVHTree>(robotManager.getTransformedObstacles());

        // 4. Create motion generator and set obstacle tree
        LOG_INFO << "Setting up motion generator";
        MotionGenerator motionGenerator(arm);
        motionGenerator.setObstacleTree(obstacleTree);
        
        // 5. Define test scenario - simple start/end configuration
        Eigen::VectorXd startJoints(7);
        Eigen::VectorXd endJoints(7);
        
        // Start configuration (from PathPlanner mainwindow)
        startJoints << 0.374894, -0.043533, 0.087470, -1.533429, 0.02237, 1.050135, 0.075773;
        
        // Goal configuration (user provided)
        endJoints << 0.596436, 1.26305, -1.17123, -2.0521, -2.42531, 2.00703, 2.35084;
        
        LOG_INFO << "Start joints: [" << startJoints.transpose() << "]";
        LOG_INFO << "End joints: [" << endJoints.transpose() << "]";
        
        // 6. Set waypoints (2x7 matrix: start and end)
        Eigen::MatrixXd waypoints(2, 7);
        waypoints.row(0) = startJoints.transpose();
        waypoints.row(1) = endJoints.transpose();
        
        LOG_INFO << "Setting waypoints for STOMP";
        motionGenerator.setWaypoints(waypoints);
        
        // 7. Run STOMP with safe configuration for debugging
        LOG_INFO << "Running STOMP optimization...";
        StompConfig config; // Use default configuration
        
        // SAFETY: Disable parallelization to prevent hanging in debug mode
        config.disableInternalParallelization = true;
        
        // Reduce complexity for debugging
        config.numNoisyTrajectories = 6;  // Reduce from default 12
        config.maxIterations = 50;        // Reduce from default 200
        config.maxComputeTimeMs = 10000;  // 10 second timeout for safety
        
        LOG_INFO << "STOMP Config: " << config.numNoisyTrajectories << " samples, " 
                 << config.maxIterations << " iterations, " 
                 << config.maxComputeTimeMs << "ms timeout";
        
        // Enable basic logging to see iteration progress
        Logger::instance().setDebugEnabled(true);
        
        bool success = motionGenerator.performSTOMP(config);
        
        // Disable debug logging after STOMP
        Logger::instance().setDebugEnabled(false);
        
        if (success) {
            LOG_INFO << "STOMP SUCCESS!";
            
            auto trajectory = motionGenerator.getPath();
            LOG_INFO << "Generated trajectory with " << trajectory.size() << " points";
            
            if (!trajectory.empty()) {
                LOG_INFO << "First point time: " << trajectory.front().time;
                LOG_INFO << "Last point time: " << trajectory.back().time;
                LOG_INFO << "Trajectory duration: " << (trajectory.back().time - trajectory.front().time) << "s";
            }
            
        } else {
            LOG_ERROR << "STOMP FAILED - trajectory optimization unsuccessful";
            return 1;
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR << "Exception: " << e.what();
        return 1;
    }
    
    LOG_INFO << "STOMP Debugger Complete";
    return 0;
}
