#ifndef BVHTREE_H
#define BVHTREE_H

#include "GeometryLib/Obstacle.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <future>
#include <memory>
#include <queue>
#include <thread>
#include <vector>

/**
 * @class BVHNode
 * @brief Node of a Bounding Volume Hierarchy (BVH) tree for spatial partitioning.
 *
 * Each node contains a bounding box, optional left/right children, and a list of obstacles
 * (only for leaf nodes).
 */
class BVHNode
{
public:
    Eigen::AlignedBox3d boundingBox;                  ///< Axis-aligned bounding box of this node
    std::unique_ptr<BVHNode> left;                    ///< Pointer to left child node
    std::unique_ptr<BVHNode> right;                   ///< Pointer to right child node
    std::vector<std::shared_ptr<Obstacle>> obstacles; ///< Obstacles in this node (leaf only)
};

/**
 * @class BVHTree
 * @brief Bounding Volume Hierarchy (BVH) tree for efficient spatial queries on obstacles.
 *
 * Supports fast intersection tests, distance queries, and signed distance field (SDF) generation
 * for a set of 3D obstacles.
 */
class BVHTree
{
public:
    /**
     * @brief Default constructor (creates an empty BVH).
     */
    BVHTree() = default;

    /**
     * @brief Construct a BVH tree from a list of obstacles.
     * @param obstacles Vector of shared pointers to obstacles.
     */
    explicit BVHTree(const std::vector<std::shared_ptr<Obstacle>> &obstacles);

    /**
     * @brief Check if a segment intersects any obstacle in the BVH.
     * @param start Start point of the segment.
     * @param end End point of the segment.
     * @return True if an intersection is found, false otherwise.
     */
    bool isSegmentIntersecting(const Vec3 &start, const Vec3 &end) const;

    /**
     * @brief Check if an oriented box intersects any obstacle in the BVH.
     * @param center Center of the box.
     * @param halfDims Half-dimensions of the box along each axis.
     * @param axes 3x3 matrix whose columns are the box axes.
     * @return True if an intersection is found, false otherwise.
     */
    bool isBoxIntersecting(const Vec3 &center,
                           const Vec3 &halfDims,
                           const Eigen::Matrix3d &axes) const;

    /**
     * @brief Compute the minimum distance and gradient to the closest obstacle.
     * @param point Query point in 3D space.
     * @return Tuple (distance, gradient vector).
     */
    std::tuple<double, Vec3> getDistanceAndGradient(const Vec3 &point) const;

    /**
     * @brief Compute the minimum distance, gradient, and closest point on any obstacle.
     * @param point Query point in 3D space.
     * @return Tuple (distance, gradient, closest point).
     */
    std::tuple<double, Vec3, Vec3> getDistanceAndClosestPoint(const Vec3 &point) const;

    /**
     * @brief Generate a signed distance field (SDF) grid over a 3D region.
     * @param min_point Minimum (corner) point of the grid.
     * @param max_point Maximum (corner) point of the grid.
     * @param resolution Grid spacing (cell size).
     * @return 3D vector of SDF values.
     */
    std::vector<std::vector<std::vector<double>>> toSDF(const Eigen::Vector3d &min_point,
                                                        const Eigen::Vector3d &max_point,
                                                        double resolution) const;

    /**
     * @brief Get access to the root node for visualization purposes.
     * @return Pointer to the root BVHNode, or nullptr if tree is empty.
     */
    const BVHNode* getRoot() const { return _root.get(); }

private:
    std::unique_ptr<BVHNode> _root; ///< Root node of the BVH tree.

    /**
     * @brief Recursively build the BVH tree.
     * @param obstacles Obstacles to partition.
     * @param start Start index in the obstacles vector.
     * @param end End index in the obstacles vector.
     * @param depth Current recursion depth.
     * @return Unique pointer to the constructed BVHNode.
     */
    std::unique_ptr<BVHNode> buildTree(std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                       int start,
                                       int end,
                                       int depth);

    /**
     * @brief Recursively check for segment intersection in the BVH.
     */
    bool isSegmentIntersectingNode(const BVHNode *node, const Vec3 &start, const Vec3 &end) const;

    /**
     * @brief Recursively check for oriented box intersection in the BVH.
     */
    bool isBoxIntersectingNode(const BVHNode *node,
                               const Vec3 &center,
                               const Vec3 &halfDims,
                               const Eigen::Matrix3d &axes) const;

    /**
     * @brief Compute minimum distance and gradient to obstacles using best-first search.
     */
    std::tuple<double, Vec3, const Obstacle *> getDistanceAndGradientRecursive(
        const BVHNode *node, const Vec3 &point) const;

    /**
     * @brief Compute a chunk of the SDF grid in parallel.
     * @param start_i Start index along the x-axis.
     * @param end_i End index along the x-axis.
     * @param min_point Minimum grid point.
     * @param resolution Grid spacing.
     * @param sdf Reference to the SDF grid to fill.
     */
    void computeSDFChunk(int start_i,
                         int end_i,
                         const Eigen::Vector3d &min_point,
                         double resolution,
                         std::vector<std::vector<std::vector<double>>> &sdf) const;
};

#endif // BVHTREE_H
