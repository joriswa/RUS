#include <iostream>
#include <QCoreApplication>
#include <QDebug>
#include "libs/TrajectoryLib/include/TrajectoryLib/Motion/MotionGenerator.h"
#include "libs/TrajectoryLib/include/TrajectoryLib/Robot/RobotArm.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    
    qDebug() << "Testing outputFrequency feature...";
    
    // Create robot arm
    RobotArm robot;
    robot.setFrame(0, 0.0, 0.0, 0.0, M_PI_2, 0.0, 0.3330);
    robot.setFrame(1, 0.0, 0.0, 0.0, -M_PI_2, 0.0, 0.0);
    robot.setFrame(2, 0.0, 0.0, 0.0, M_PI_2, 0.0, 0.3160);
    robot.setFrame(3, 0.0, 0.0, 0.0, -M_PI_2, 0.0, 0.0);
    robot.setFrame(4, 0.0, 0.0, 0.0, M_PI_2, 0.0, 0.3840);
    robot.setFrame(5, 0.0, 0.0, 0.0, -M_PI_2, 0.0, 0.0);
    robot.setFrame(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2070);
    
    // Create motion generator
    MotionGenerator motionGen(robot);
    
    // Set simple waypoints
    Eigen::MatrixXd waypoints(2, 7);
    waypoints << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
    
    motionGen.setWaypoints(waypoints);
    
    // Test STOMP with different output frequencies
    qDebug() << "\n=== Testing STOMP with outputFrequency ===";
    
    StompConfig config;
    config.maxIterations = 50;  // Quick test
    config.outputFrequency = 100.0;  // 100 Hz
    
    qDebug() << "Running STOMP with outputFrequency:" << config.outputFrequency << "Hz";
    bool stompSuccess = motionGen.performSTOMP(config);
    
    if (stompSuccess) {
        auto path = motionGen.getPath();
        qDebug() << "STOMP successful! Generated" << path.size() << "trajectory points";
        
        if (path.size() > 1) {
            double actualDt = path[1].time - path[0].time;
            double actualFreq = 1.0 / actualDt;
            qDebug() << "Actual time step:" << actualDt << "s";
            qDebug() << "Actual frequency:" << actualFreq << "Hz";
            qDebug() << "Total duration:" << path.back().time << "s";
        }
    } else {
        qDebug() << "STOMP failed";
    }
    
    // Test STOMP without output frequency
    qDebug() << "\n=== Testing STOMP without outputFrequency ===";
    config.outputFrequency = 0.0;  // Disable resampling
    
    qDebug() << "Running STOMP without outputFrequency (original trajectory)";
    bool stompSuccess2 = motionGen.performSTOMP(config);
    
    if (stompSuccess2) {
        auto path2 = motionGen.getPath();
        qDebug() << "STOMP successful! Generated" << path2.size() << "trajectory points";
        
        if (path2.size() > 1) {
            double actualDt = path2[1].time - path2[0].time;
            double actualFreq = 1.0 / actualDt;
            qDebug() << "Actual time step:" << actualDt << "s";
            qDebug() << "Actual frequency:" << actualFreq << "Hz";
            qDebug() << "Total duration:" << path2.back().time << "s";
        }
    } else {
        qDebug() << "STOMP failed";
    }
    
    qDebug() << "\n=== Test completed ===";
    
    return 0;
}
