#include "GeometryLib/BVHTree.h"
#include <cassert>
#include <iostream>
#include <random>

Eigen::Affine3d getRandomTransform() {
    static std::mt19937 gen(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<> dis(-5.0, 5.0);
    std::uniform_real_distribution<> rot(-M_PI, M_PI);

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translate(Eigen::Vector3d(dis(gen), dis(gen), dis(gen)));
    transform.rotate(Eigen::AngleAxisd(rot(gen), Eigen::Vector3d::UnitX()));
    transform.rotate(Eigen::AngleAxisd(rot(gen), Eigen::Vector3d::UnitY()));
    transform.rotate(Eigen::AngleAxisd(rot(gen), Eigen::Vector3d::UnitZ()));

    return transform;
}

void testEmptyTree() {
    BVHTree tree;
    Vec3 start(0, 0, 0);
    Vec3 end(1, 1, 1);
    assert(!tree.isSegmentIntersecting(start, end));
    std::cout << "Empty tree test passed" << std::endl;
}

void testSingleObstacle() {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    obstacles.push_back(std::make_shared<BoxObstacle>());
    BVHTree tree(obstacles);

    Vec3 start(0, 0, 0);
    Vec3 end(1, 1, 1);
    assert(tree.isSegmentIntersecting(start, end));

    Vec3 outside_start(2, 2, 2);
    Vec3 outside_end(3, 3, 3);
    assert(!tree.isSegmentIntersecting(outside_start, outside_end));

    std::cout << "Single obstacle test passed" << std::endl;
}

void testMultipleObstacles() {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    obstacles.push_back(std::make_shared<BoxObstacle>());
    obstacles.push_back(std::make_shared<BoxObstacle>(Vec3(1, 1, 1), Eigen::Translation3d(2, 2, 2)));
    BVHTree tree(obstacles);

    Vec3 start(0, 0, 0);
    Vec3 end(3, 3, 3);
    assert(tree.isSegmentIntersecting(start, end));

    Vec3 between_start(1.5, 1.5, 1.5);
    Vec3 between_end(1.6, 1.6, 1.6);
    assert(!tree.isSegmentIntersecting(between_start, between_end));

    std::cout << "Multiple obstacles test passed" << std::endl;
}

void testBoxIntersection() {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    obstacles.push_back(std::make_shared<BoxObstacle>());
    BVHTree tree(obstacles);

    Vec3 center(0, 0, 0);
    Vec3 halfDims(0.1, 0.1, 0.1);
    Eigen::Matrix3d axes = Eigen::Matrix3d::Identity();
    assert(tree.isBoxIntersecting(center, halfDims, axes));

    Vec3 outside_center(2, 2, 2);
    assert(!tree.isBoxIntersecting(outside_center, halfDims, axes));

    std::cout << "Box intersection test passed" << std::endl;
}


void testPerformance() {
    std::vector<std::shared_ptr<Obstacle>> obstacles;

    for (int i = 0; i < 1000; ++i) {
        auto transform = getRandomTransform();
        obstacles.push_back(std::make_shared<BoxObstacle>(Vec3(1, 1, 1), transform));
    }

    auto start = std::chrono::high_resolution_clock::now();
    BVHTree tree(obstacles);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;
    std::cout << "Time to build BVH tree with 1000 obstacles: " << diff.count() << " s" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    int intersections = 0;
    for (int i = 0; i < 1000; ++i) {
        Vec3 start_point(getRandomTransform().translation());
        Vec3 end_point(getRandomTransform().translation());

        if (tree.isSegmentIntersecting(start_point, end_point)) {
            intersections++;
        }
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    std::cout << "Time to check 1000 random segment intersections: " << diff.count() << " s" << std::endl;
    std::cout << "Number of intersections: " << intersections << std::endl;
}

void testGetDistanceAndGradient() {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    obstacles.push_back(std::make_shared<BoxObstacle>(Vec3(1, 1, 1), Eigen::Translation3d(0, 0, 0)));
    BVHTree tree(obstacles);

    {
        Vec3 point(0, 0, 0);
        auto [distance, gradient] = tree.getDistanceAndGradient(point);
        assert(distance < 0);
        assert(gradient.norm() > 0);
    }

    {
        Vec3 point(2, 0, 0);
        auto [distance, gradient] = tree.getDistanceAndGradient(point);
        assert(std::abs(distance - 1.5) < 1e-6);
        assert((gradient - Vec3(1, 0, 0)).norm() < 1e-6);
    }

    std::cout << "GetDistanceAndGradient basic test passed" << std::endl;
}

void testGetDistanceAndGradientRandom() {
    std::vector<std::shared_ptr<Obstacle>> obstacles;

    Vec3 obstacleScale(2, 2, 2);
    Eigen::Affine3d obstacleTransform = Eigen::Affine3d::Identity();
    obstacles.push_back(std::make_shared<BoxObstacle>(obstacleScale, obstacleTransform));

    BVHTree tree(obstacles);

    static std::mt19937 gen(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<> dis(0, 2*M_PI);
    std::uniform_real_distribution<> dis_radius(2, 10);
    double theta = dis(gen);
    double phi = dis(gen);
    double radius = dis_radius(gen);

    double x = radius * sin(phi) * cos(theta);
    double y = radius * sin(phi) * sin(theta);
    double z = radius * cos(phi);

    Vec3 randomPoint(x, y, z);

    double expectedDistance = radius - 1.0;
    Vec3 expectedGradient = randomPoint.normalized();

    auto [actualDistance, actualGradient] = tree.getDistanceAndGradient(randomPoint);

    assert(std::abs(actualDistance - expectedDistance) < 1e-6);

    assert((actualGradient - expectedGradient).norm() < 1e-6);

    double epsilon = 1e-4;
    auto [newDistance, _] = tree.getDistanceAndGradient(randomPoint + epsilon * actualGradient);
    assert(newDistance > actualDistance);

    std::cout << "GetDistanceAndGradient random test passed" << std::endl;
}

void testGetDistanceAndGradientMultipleObstacles() {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    obstacles.push_back(std::make_shared<BoxObstacle>(Vec3(1, 1, 1), Eigen::Translation3d(0, 0, 0)));
    obstacles.push_back(std::make_shared<BoxObstacle>(Vec3(1, 1, 1), Eigen::Translation3d(3, 0, 0)));
    BVHTree tree(obstacles);

    Vec3 point(1.5, 0, 0);
    auto [distance, gradient] = tree.getDistanceAndGradient(point);

    assert(std::abs(distance - 0.5) < 1e-6);
    assert((gradient - Vec3(-1, 0, 0)).norm() < 1e-6);

    std::cout << "GetDistanceAndGradient multiple obstacles test passed" << std::endl;
}

void testGetDistanceAndGradientPerformance() {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    for (int i = 0; i < 1000; ++i) {
        auto transform = getRandomTransform();
        obstacles.push_back(std::make_shared<BoxObstacle>(Vec3(1, 1, 1), transform));
    }
    BVHTree tree(obstacles);

    auto start = std::chrono::high_resolution_clock::now();
    int closeObstacles = 0;
    for (int i = 0; i < 1000; ++i) {
        Vec3 point(getRandomTransform().translation());
        auto [distance, gradient] = tree.getDistanceAndGradient(point);
        if (distance < 1.0) {
            closeObstacles++;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;
    std::cout << "Time to get distance and gradient for 1000 random points: " << diff.count() << " s" << std::endl;
    std::cout << "Number of points close to obstacles: " << closeObstacles << std::endl;
}

void testSDFConversion() {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    obstacles.push_back(std::make_shared<BoxObstacle>(Vec3(1, 1, 1), Eigen::Translation3d(0, 0, 0)));
    obstacles.push_back(std::make_shared<BoxObstacle>(Vec3(1, 1, 1), Eigen::Translation3d(3, 0, 0)));
    BVHTree tree(obstacles);

    Eigen::Vector3d min_point(-5, -5, -5);
    Eigen::Vector3d max_point(5, 5, 5);
    double resolution = 0.5;

    auto sdf = tree.toSDF(min_point, max_point, resolution);

    // Check dimensions
    int expected_size = static_cast<int>((max_point.x() - min_point.x()) / resolution);
    assert(sdf.size() == expected_size);
    assert(sdf[0].size() == expected_size);
    assert(sdf[0][0].size() == expected_size);

    // Check specific points
    std::vector<std::tuple<Eigen::Vector3d, double>> test_points = {
        {Eigen::Vector3d(0, 0, 0), -0.5},    // Inside first box
        {Eigen::Vector3d(3, 0, 0), -0.5},    // Inside second box
        {Eigen::Vector3d(1.5, 0, 0), 0.5},   // Between boxes
        {Eigen::Vector3d(-2, 0, 0), 1.5},    // Outside boxes
        {Eigen::Vector3d(5, 0, 0), 1.5}      // Outside boxes
    };

    for (const auto& [point, expected_distance] : test_points) {
        Eigen::Vector3d index = (point - min_point) / resolution;
        int i = static_cast<int>(index.x());
        int j = static_cast<int>(index.y());
        int k = static_cast<int>(index.z());

        double actual_distance = sdf[i][j][k];
        assert(std::abs(actual_distance - expected_distance) < resolution);
    }

    // Check gradient
    for (int i = 1; i < sdf.size() - 1; ++i) {
        for (int j = 1; j < sdf[i].size() - 1; ++j) {
            for (int k = 1; k < sdf[i][j].size() - 1; ++k) {
                Eigen::Vector3d gradient(
                    (sdf[i+1][j][k] - sdf[i-1][j][k]) / (2 * resolution),
                    (sdf[i][j+1][k] - sdf[i][j-1][k]) / (2 * resolution),
                    (sdf[i][j][k+1] - sdf[i][j][k-1]) / (2 * resolution)
                    );

                // The gradient should be a unit vector or zero
                assert(std::abs(gradient.norm() - 1.0) < 1e-6 || gradient.norm() < 1e-6);
            }
        }
    }

    std::cout << "SDF conversion test passed" << std::endl;
}

int main() {
    testEmptyTree();
    testSingleObstacle();
    testMultipleObstacles();
    testBoxIntersection();
    testPerformance();
    testGetDistanceAndGradient();
    testGetDistanceAndGradientRandom();
    testGetDistanceAndGradientMultipleObstacles();
    testGetDistanceAndGradientPerformance();
    testSDFConversion();
    std::cout << "All BVHTree tests passed!" << std::endl;
    return 0;
}

