#include <GeometryLib/Obstacle.h>
#include <GeometryLib/BVHTree.h>
#include <iostream>

int main() {
    std::cout << "RUS GeometryLib Example" << std::endl;
    
    // Create a simple obstacle
    auto obstacle = std::make_shared<Obstacle>(
        Eigen::Vector3d(0, 0, 0),  // position
        Eigen::Vector3d(1, 1, 1)   // size
    );
    
    std::cout << "Created obstacle at position: " 
              << obstacle->getPosition().transpose() << std::endl;
    
    // Create a BVH tree for collision detection
    BVHTree tree;
    tree.addObstacle(obstacle);
    
    std::cout << "Added obstacle to BVH tree" << std::endl;
    std::cout << "GeometryLib example completed successfully!" << std::endl;
    
    return 0;
}