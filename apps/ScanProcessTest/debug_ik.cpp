// Debug IK solver to understand why it's failing
#include <iostream>
#include <TrajectoryLib/franka_ik_He.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

int main() {
    std::cout << "=== IK Solver Debug Test ===" << std::endl;
    
    // Test with a simple, known reachable pose
    Eigen::Affine3d test_pose = Eigen::Affine3d::Identity();
    test_pose.translation() = Eigen::Vector3d(0.3, 0.0, 0.5);  // Simple front position
    test_pose.linear() = Eigen::Quaterniond::Identity().toRotationMatrix();
    
    std::cout << "Test pose matrix:" << std::endl;
    std::cout << test_pose.matrix() << std::endl;
    
    // Test current joint configuration (reasonable starting position)
    std::array<double, 7> current_joints = {{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}};
    
    std::cout << "\nCurrent joints: ";
    for (int i = 0; i < 7; i++) {
        std::cout << current_joints[i] << " ";
    }
    std::cout << std::endl;
    
    // Test IK with different q7 values
    std::cout << "\n=== Testing IK with different q7 values ===" << std::endl;
    
    for (double q7 = -2.5; q7 <= 2.5; q7 += 0.5) {
        std::cout << "\nTesting q7 = " << q7 << std::endl;
        
        auto solutions = franka_IK_EE(test_pose, q7, current_joints);
        
        bool found_valid = false;
        for (int i = 0; i < 4; i++) {
            bool is_valid = true;
            for (int j = 0; j < 7; j++) {
                if (std::isnan(solutions[i][j])) {
                    is_valid = false;
                    break;
                }
            }
            
            if (is_valid) {
                std::cout << "  Solution " << i << ": ";
                for (int j = 0; j < 7; j++) {
                    std::cout << solutions[i][j] << " ";
                }
                std::cout << std::endl;
                found_valid = true;
            }
        }
        
        if (!found_valid) {
            std::cout << "  No valid solutions found" << std::endl;
        }
    }
    
    // Now test with the actual first scan pose
    std::cout << "\n=== Testing with first scan pose ===" << std::endl;
    
    // From the CSV: 0.550252728462219,-0.319555840206281,0.454720935133134,0.0514442378052986,0.3167581059381,0.891220960680185,0.320535476918591
    double x = 0.550252728462219;
    double y = -0.319555840206281;
    double z = 0.454720935133134;
    Eigen::Quaterniond q(0.0514442378052986, 0.3167581059381, 0.891220960680185, 0.320535476918591);
    q.normalize();
    
    Eigen::Affine3d scan_pose = Eigen::Affine3d::Identity();
    scan_pose.translation() = Eigen::Vector3d(x, y, z);
    scan_pose.linear() = q.toRotationMatrix();
    
    // Apply the local offset like in the scan_process_test.cpp
    const Eigen::Vector3d local_move(0.0, 0.0, -0.02);
    scan_pose.translation() += scan_pose.linear() * local_move;
    
    std::cout << "Scan pose matrix:" << std::endl;
    std::cout << scan_pose.matrix() << std::endl;
    
    std::cout << "\nTesting scan pose with different q7 values:" << std::endl;
    
    for (double q7 = -2.5; q7 <= 2.5; q7 += 0.5) {
        auto solutions = franka_IK_EE(scan_pose, q7, current_joints);
        
        bool found_valid = false;
        for (int i = 0; i < 4; i++) {
            bool is_valid = true;
            for (int j = 0; j < 7; j++) {
                if (std::isnan(solutions[i][j])) {
                    is_valid = false;
                    break;
                }
            }
            
            if (is_valid) {
                std::cout << "  q7=" << q7 << " Solution " << i << ": ";
                for (int j = 0; j < 7; j++) {
                    std::cout << solutions[i][j] << " ";
                }
                std::cout << std::endl;
                found_valid = true;
            }
        }
        
        if (!found_valid) {
            std::cout << "  q7=" << q7 << " No valid solutions" << std::endl;
        }
    }
    
    return 0;
}
