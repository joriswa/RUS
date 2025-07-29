#include <iostream>
#include "TrajectoryLib/Robot/RobotArm.h"

int main() {
    try {
        std::string urdf_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf";
        std::cout << "Creating RobotArm with: " << urdf_path << std::endl;
        RobotArm* robot = new RobotArm(urdf_path);
        std::cout << "SUCCESS: Robot created with " << robot->getNumberOfJoints() << " joints" << std::endl;
        std::cout << "Trying to get joint angles..." << std::endl;
        auto angles = robot->getJointAngles();
        std::cout << "SUCCESS: Got joint angles" << std::endl;
        delete robot;
        return 0;
    } catch (const std::exception& e) {
        std::cout << "ERROR: " << e.what() << std::endl;
        return 1;
    }
}
