#include <iostream>
#include "libs/GeometryLib/include/GeometryLib/BVHTree.h"
#include "libs/GeometryLib/include/GeometryLib/Obstacle.h"
#include <Eigen/Dense>

int main() {
    std::cout << "Testing BVH Tree with signed distance function..." << std::endl;
    
    // Create a simple box obstacle
    Vec3 scale(1.0, 1.0, 1.0);
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() = Vec3(0.0, 0.0, 1.0); // Move box up
    
    auto box = std::make_shared<BoxObstacle>(scale, transform);
    
    // Create BVH tree
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    obstacles.push_back(box);
    BVHTree tree(obstacles);
    
    std::cout << "\n=== Testing BVH Tree distance queries ===" << std::endl;
    
    // Test point clearly inside the obstacle
    Vec3 insidePoint(0.0, 0.0, 1.0); // Center of box
    auto [distance1, gradient1] = tree.getDistanceAndGradient(insidePoint);
    std::cout << "Point inside obstacle (0,0,1): distance = " << distance1 << std::endl;
    
    // Test point outside the obstacle  
    Vec3 outsidePoint(2.0, 0.0, 1.0); // Outside box
    auto [distance2, gradient2] = tree.getDistanceAndGradient(outsidePoint);
    std::cout << "Point outside obstacle (2,0,1): distance = " << distance2 << std::endl;
    
    // Test point on surface
    Vec3 surfacePoint(0.5, 0.0, 1.0); // On face of box
    auto [distance3, gradient3] = tree.getDistanceAndGradient(surfacePoint);
    std::cout << "Point on surface (0.5,0,1): distance = " << distance3 << std::endl;
    
    std::cout << "\nExpected behavior:" << std::endl;
    std::cout << "- Inside point should have NEGATIVE distance" << std::endl;
    std::cout << "- Outside point should have POSITIVE distance" << std::endl;
    std::cout << "- Surface point should have distance near ZERO" << std::endl;
    
    return 0;
}
