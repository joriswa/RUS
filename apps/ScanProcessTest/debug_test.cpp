#include <iostream>
#include <Eigen/Dense>
#include "USLib/USTrajectoryPlanner.h"

int main() {
    try {
        std::cout << "Creating planner..." << std::endl;
        UltrasoundScanTrajectoryPlanner planner("../../../res/robot/panda.urdf");
        std::cout << "Planner created successfully." << std::endl;
        
        std::cout << "Setting environment..." << std::endl;
        planner.setEnvironment("/Users/joris/Downloads/3/obstacles.xml");
        std::cout << "Environment set successfully." << std::endl;
        
        std::cout << "Setting joints..." << std::endl;
        Eigen::VectorXd joints(7);
        joints << 0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.7854;
        planner.setCurrentJoints(joints);
        std::cout << "Joints set successfully." << std::endl;
        
        std::cout << "Creating simple test pose..." << std::endl;
        std::vector<Eigen::Affine3d> poses;
        Eigen::Affine3d test_pose = Eigen::Affine3d::Identity();
        test_pose.translation() << 0.5, 0.0, 0.5; // Simple forward position
        poses.push_back(test_pose);
        
        std::cout << "Setting poses..." << std::endl;
        planner.setPoses(poses);
        std::cout << "Poses set successfully." << std::endl;
        
        std::cout << "All initialization completed successfully!" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
