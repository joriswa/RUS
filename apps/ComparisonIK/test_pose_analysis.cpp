#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>

std::vector<Eigen::Affine3d> readPosesFromCSV(const std::string& filename) {
    std::vector<Eigen::Affine3d> poses;
    std::ifstream file(filename);
    std::string line;
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open poses file: " << filename << std::endl;
        return poses;
    }
    
    // Skip header line
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;
        
        while (std::getline(ss, token, ',')) {
            values.push_back(std::stod(token));
        }

        if (values.size() >= 7) {
            Eigen::Vector3d position(values[0], values[1], values[2]);
            Eigen::Quaterniond quaternion(values[3], values[4], values[5], values[6]);
            quaternion.normalize();

            Eigen::Affine3d pose = Eigen::Affine3d::Identity();
            pose.linear() = quaternion.toRotationMatrix();
            pose.translation() = position;
            
            poses.push_back(pose);
        }
    }

    return poses;
}

int main() {
    std::string poses_csv = "/Users/joris/Downloads/07_09_2/ext_post_rotated.csv";
    
    std::vector<Eigen::Affine3d> poses = readPosesFromCSV(poses_csv);
    
    std::cout << "=== CSV Pose Analysis ===" << std::endl;
    std::cout << "Loaded " << poses.size() << " poses" << std::endl;
    
    for (int i = 0; i < std::min(5, static_cast<int>(poses.size())); i++) {
        std::cout << "\\nPose " << (i+1) << ":" << std::endl;
        std::cout << "Position: [" << poses[i].translation().transpose() << "]" << std::endl;
        
        Eigen::Quaterniond q(poses[i].rotation());
        std::cout << "Quaternion: [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]" << std::endl;
        
        // Check if position is reasonable for Franka robot
        double distance_from_origin = poses[i].translation().norm();
        std::cout << "Distance from origin: " << distance_from_origin << " m" << std::endl;
        
        if (distance_from_origin > 1.5) {
            std::cout << "⚠️  WARNING: Pose may be outside robot reach (>1.5m from base)" << std::endl;
        }
        if (distance_from_origin < 0.1) {
            std::cout << "⚠️  WARNING: Pose may be too close to robot base (<0.1m)" << std::endl;
        }
    }
    
    return 0;
}
