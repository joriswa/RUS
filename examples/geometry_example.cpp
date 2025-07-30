#include <GeometryLib/BVHTree.h>
#include <iostream>

int main() {
    std::cout << "RUS GeometryLib Example" << std::endl;
    
    // Create a BVH tree for collision detection
    BVHTree tree;
    
    std::cout << "Created empty BVH tree" << std::endl;
    
    // Create a simple bounding box to demonstrate the library
    Eigen::AlignedBox3d bbox(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
    
    std::cout << "Created bounding box from " 
              << bbox.min().transpose() << " to " 
              << bbox.max().transpose() << std::endl;
    
    std::cout << "GeometryLib example completed successfully!" << std::endl;
    std::cout << "Note: This library provides collision detection infrastructure." << std::endl;
    
    return 0;
}