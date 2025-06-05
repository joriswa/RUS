#include "GeometryLib/Obstacle.h"
#include <cassert>
#include <cmath>
#include <iostream>

void testCenterPoint(const BoxObstacle& box) {
    Vec3 point(0.0, 0.0, 0.0);
    assert(std::abs(box.getDistance(point) - (-0.5)) < 1e-6);
    assert((box.getClosestPoint(point) - Vec3(0.5, 0.0, 0.0)).norm() <= 0.5 + 1e-6);
    assert(std::abs(box.getDistanceGradient(point).norm() - 1.0) < 1e-6);
    std::cout << "Center point test passed" << std::endl;
}

void testInsidePoint(const BoxObstacle& box) {
    Vec3 point(0.25, 0.25, 0.25);
    assert(std::abs(box.getDistance(point) - (-0.25)) < 1e-6);
    assert((box.getClosestPoint(point) - Vec3(0.5, 0.25, 0.25)).norm() < 1e-6);
    assert((box.getDistanceGradient(point) - Vec3(1, 0, 0)).norm() < 1e-6);
    std::cout << "Inside point test passed" << std::endl;
}

void testSurfacePoint(const BoxObstacle& box) {
    Vec3 point(0.5, 0.0, 0.0);
    assert(std::abs(box.getDistance(point)) < 1e-6);
    assert((box.getClosestPoint(point) - point).norm() < 1e-6);
    assert((box.getDistanceGradient(point) - Vec3(1, 0, 0)).norm() < 1e-6);
    std::cout << "Surface point test passed" << std::endl;
}

void testOutsidePoint(const BoxObstacle& box) {
    Vec3 point(1.0, 0.0, 0.0);
    assert(std::abs(box.getDistance(point) - 0.5) < 1e-6);
    assert((box.getClosestPoint(point) - Vec3(0.5, 0.0, 0.0)).norm() < 1e-6);
    assert((box.getDistanceGradient(point) - Vec3(1, 0, 0)).norm() < 1e-6);
    std::cout << "Outside point test passed" << std::endl;
}

void testOutsideDiagonalPoint(const BoxObstacle& box) {
    Vec3 point(0.75, 0.75, 0.75);
    assert(std::abs(box.getDistance(point) - std::sqrt(3 * 0.25 * 0.25)) < 1e-6);
    assert((box.getClosestPoint(point) - Vec3(0.5, 0.5, 0.5)).norm() < 1e-6);
    assert((box.getDistanceGradient(point) - Vec3(1, 1, 1).normalized()).norm() < 1e-6);
    std::cout << "Outside diagonal point test passed" << std::endl;
}

int main() {
    BoxObstacle box;

    testCenterPoint(box);
    testInsidePoint(box);
    testSurfacePoint(box);
    testOutsidePoint(box);
    testOutsideDiagonalPoint(box);

    std::cout << "All Box Obstacle tests passed!" << std::endl;
    return 0;
}
