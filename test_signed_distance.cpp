#include <iostream>
#include "libs/GeometryLib/include/GeometryLib/Obstacle.h"
#include <Eigen/Dense>

int main() {
    std::cout << "Testing BoxObstacle signed distance function..." << std::endl;
    
    // Create a simple box obstacle at origin with size (2, 2, 2)
    Vec3 scale(2.0, 2.0, 2.0);
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    BoxObstacle box(scale, transform);
    
    // Test points inside the obstacle
    std::cout << "\n=== Testing points INSIDE obstacle ===" << std::endl;
    Vec3 insidePoint1(0.0, 0.0, 0.0);  // Center of box
    Vec3 insidePoint2(0.5, 0.5, 0.5);  // Inside but near corner
    Vec3 insidePoint3(0.9, 0.0, 0.0);  // Close to face
    
    double dist1 = box.getDistance(insidePoint1);
    double dist2 = box.getDistance(insidePoint2);
    double dist3 = box.getDistance(insidePoint3);
    
    std::cout << "Point at center (0,0,0): distance = " << dist1 << std::endl;
    std::cout << "Point at (0.5,0.5,0.5): distance = " << dist2 << std::endl;
    std::cout << "Point at (0.9,0,0): distance = " << dist3 << std::endl;
    
    // Test points outside the obstacle
    std::cout << "\n=== Testing points OUTSIDE obstacle ===" << std::endl;
    Vec3 outsidePoint1(2.0, 0.0, 0.0);  // Just outside on x face
    Vec3 outsidePoint2(3.0, 3.0, 3.0);  // Far outside
    
    double dist4 = box.getDistance(outsidePoint1);
    double dist5 = box.getDistance(outsidePoint2);
    
    std::cout << "Point at (2,0,0): distance = " << dist4 << std::endl;
    std::cout << "Point at (3,3,3): distance = " << dist5 << std::endl;
    
    // Test points on the surface  
    std::cout << "\n=== Testing points ON surface ===" << std::endl;
    Vec3 surfacePoint(1.0, 0.0, 0.0);  // On face
    double dist6 = box.getDistance(surfacePoint);
    std::cout << "Point at (1,0,0): distance = " << dist6 << std::endl;
    
    std::cout << "\nExpected results:" << std::endl;
    std::cout << "- Inside points should have NEGATIVE distances" << std::endl;
    std::cout << "- Outside points should have POSITIVE distances" << std::endl;
    std::cout << "- Surface points should have distances near ZERO" << std::endl;
    
    return 0;
}
