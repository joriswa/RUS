#include "GeometryLib/BVHTree.h"

/**
 * @brief Construct a BVH tree from a list of obstacles.
 */
BVHTree::BVHTree(const std::vector<std::shared_ptr<Obstacle>> &obstacles)
{
    if (!obstacles.empty()) {
        auto mutableObstacles = obstacles;
        _root = buildTree(mutableObstacles, 0, static_cast<int>(mutableObstacles.size()), 0);
    }
}

/**
 * @brief Recursively build the BVH tree using in-place nth_element partitioning.
 */
std::unique_ptr<BVHNode> BVHTree::buildTree(std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                            int start,
                                            int end,
                                            int depth)
{
    if (start >= end)
        return nullptr;

    Eigen::AlignedBox3d bbox = obstacles[start]->getBoundingBox();
    for (int i = start + 1; i < end; ++i) {
        bbox.extend(obstacles[i]->getBoundingBox());
    }

    auto node = std::make_unique<BVHNode>();
    node->boundingBox = bbox;

    if (end - start == 1) {
        node->obstacles.push_back(obstacles[start]);
        return node;
    }

    int axis = depth % 3;
    int mid = start + (end - start) / 2;

    std::nth_element(obstacles.begin() + start,
                     obstacles.begin() + mid,
                     obstacles.begin() + end,
                     [axis](const std::shared_ptr<Obstacle> &a, const std::shared_ptr<Obstacle> &b) {
                         return a->getBoundingBox().center()[axis]
                                < b->getBoundingBox().center()[axis];
                     });

    node->left = buildTree(obstacles, start, mid, depth + 1);
    node->right = buildTree(obstacles, mid, end, depth + 1);
    return node;
}

/**
 * @brief Check if a segment intersects any obstacle in the BVH.
 */
bool BVHTree::isSegmentIntersecting(const Vec3 &start, const Vec3 &end) const
{
    return isSegmentIntersectingNode(_root.get(), start, end);
}

/**
 * @brief Recursively check for segment intersection in the BVH.
 */
bool BVHTree::isSegmentIntersectingNode(const BVHNode *node,
                                        const Vec3 &start,
                                        const Vec3 &end) const
{
    if (!node)
        return false;

    if (!node->boundingBox.intersects(Eigen::AlignedBox3d(start, end)))
        return false;

    for (const auto &obstacle : node->obstacles) {
        if (obstacle->isSegmentInObstacle(start, end))
            return true;
    }

    return isSegmentIntersectingNode(node->left.get(), start, end)
           || isSegmentIntersectingNode(node->right.get(), start, end);
}

/**
 * @brief Check if an oriented box intersects any obstacle in the BVH.
 */
bool BVHTree::isBoxIntersecting(const Vec3 &center,
                                const Vec3 &halfDims,
                                const Eigen::Matrix3d &axes) const
{
    return isBoxIntersectingNode(_root.get(), center, halfDims, axes);
}

/**
 * @brief Recursively check for oriented box intersection in the BVH.
 */
bool BVHTree::isBoxIntersectingNode(const BVHNode *node,
                                    const Vec3 &center,
                                    const Vec3 &halfDims,
                                    const Eigen::Matrix3d &axes) const
{
    if (!node)
        return false;

    auto getOBBCorners = [](const Vec3 &center, const Vec3 &halfDims, const Eigen::Matrix3d &axes) {
        std::vector<Vec3> corners(8);
        corners[0] = center + axes * Vec3(halfDims.x(), halfDims.y(), halfDims.z());
        corners[1] = center + axes * Vec3(-halfDims.x(), halfDims.y(), halfDims.z());
        corners[2] = center + axes * Vec3(halfDims.x(), -halfDims.y(), halfDims.z());
        corners[3] = center + axes * Vec3(halfDims.x(), halfDims.y(), -halfDims.z());
        corners[4] = center + axes * Vec3(-halfDims.x(), -halfDims.y(), halfDims.z());
        corners[5] = center + axes * Vec3(-halfDims.x(), halfDims.y(), -halfDims.z());
        corners[6] = center + axes * Vec3(halfDims.x(), -halfDims.y(), -halfDims.z());
        corners[7] = center + axes * Vec3(-halfDims.x(), -halfDims.y(), -halfDims.z());
        return corners;
    };

    auto computeBoundingBox = [](const std::vector<Vec3> &corners) {
        Eigen::AlignedBox3d box;
        for (const auto &corner : corners) {
            box.extend(corner);
        }
        return box;
    };

    std::vector<Vec3> corners = getOBBCorners(center, halfDims, axes);
    Eigen::AlignedBox3d orientedBox = computeBoundingBox(corners);

    if (!node->boundingBox.intersects(orientedBox))
        return false;

    for (const auto &obstacle : node->obstacles) {
        if (obstacle->intersectsOBB(center, halfDims, axes))
            return true;
    }

    return isBoxIntersectingNode(node->left.get(), center, halfDims, axes)
           || isBoxIntersectingNode(node->right.get(), center, halfDims, axes);
}

/**
 * @brief Compute the minimum distance and gradient to the closest obstacle.
 */
std::tuple<double, Vec3> BVHTree::getDistanceAndGradient(const Vec3 &point) const
{
    if (!_root) {
        return std::make_tuple(std::numeric_limits<double>::infinity(), Vec3::Zero());
    }
    auto [distance, gradient, _] = getDistanceAndGradientRecursive(_root.get(), point);
    return std::make_tuple(distance, gradient);
}

/**
 * @brief Compute minimum distance and gradient to obstacles using best-first search.
 */
std::tuple<double, Vec3, const Obstacle *> BVHTree::getDistanceAndGradientRecursive(
    const BVHNode *node, const Vec3 &point) const
{
    using QueueElem = std::pair<double, const BVHNode *>;
    auto cmp = [](const QueueElem &a, const QueueElem &b) { return a.first > b.first; };
    std::priority_queue<QueueElem, std::vector<QueueElem>, decltype(cmp)> queue(cmp);

    double min_dist = std::numeric_limits<double>::infinity();
    Vec3 closest_grad = Vec3::Zero();
    const Obstacle *closest_obstacle = nullptr;

    if (node)
        queue.emplace(node->boundingBox.squaredExteriorDistance(point), node);

    while (!queue.empty()) {
        auto [dist_sq, current] = queue.top();
        queue.pop();

        if (std::sqrt(dist_sq) >= min_dist)
            break;

        if (current->obstacles.empty()) {
            if (current->left) {
                double left_dist = current->left->boundingBox.squaredExteriorDistance(point);
                if (left_dist < min_dist * min_dist)
                    queue.emplace(left_dist, current->left.get());
            }
            if (current->right) {
                double right_dist = current->right->boundingBox.squaredExteriorDistance(point);
                if (right_dist < min_dist * min_dist)
                    queue.emplace(right_dist, current->right.get());
            }
        } else {
            for (const auto &obstacle : current->obstacles) {
                double dist = obstacle->getDistance(point);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_grad = obstacle->getDistanceGradient(point);
                    closest_obstacle = obstacle.get();
                }
            }
        }
    }

    return {min_dist, closest_grad, closest_obstacle};
}

/**
 * @brief Compute the minimum distance, gradient, and closest point on any obstacle.
 */
std::tuple<double, Vec3, Vec3> BVHTree::getDistanceAndClosestPoint(const Vec3 &point) const
{
    if (!_root) {
        return std::make_tuple(std::numeric_limits<double>::infinity(), Vec3::Zero(), Vec3::Zero());
    }

    // Use the same priority queue approach for finding closest point
    using QueueElem = std::pair<double, const BVHNode *>;
    auto cmp = [](const QueueElem &a, const QueueElem &b) { return a.first > b.first; };
    std::priority_queue<QueueElem, std::vector<QueueElem>, decltype(cmp)> queue(cmp);

    double min_dist = std::numeric_limits<double>::infinity();
    Vec3 closest_grad = Vec3::Zero();
    Vec3 closest_point = Vec3::Zero();

    if (_root)
        queue.emplace(_root->boundingBox.squaredExteriorDistance(point), _root.get());

    while (!queue.empty()) {
        auto [dist_sq, current] = queue.top();
        queue.pop();

        if (std::sqrt(dist_sq) >= min_dist)
            break;

        if (current->obstacles.empty()) {
            if (current->left) {
                double left_dist = current->left->boundingBox.squaredExteriorDistance(point);
                if (left_dist < min_dist * min_dist)
                    queue.emplace(left_dist, current->left.get());
            }
            if (current->right) {
                double right_dist = current->right->boundingBox.squaredExteriorDistance(point);
                if (right_dist < min_dist * min_dist)
                    queue.emplace(right_dist, current->right.get());
            }
        } else {
            for (const auto &obstacle : current->obstacles) {
                double dist = obstacle->getDistance(point);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_grad = obstacle->getDistanceGradient(point);
                    closest_point = obstacle->getClosestPoint(point);
                }
            }
        }
    }

    return std::make_tuple(min_dist, closest_point, closest_grad);
}

/**
 * @brief Generate a signed distance field (SDF) grid over a 3D region.
 */
std::vector<std::vector<std::vector<double>>> BVHTree::toSDF(const Eigen::Vector3d &min_point,
                                                             const Eigen::Vector3d &max_point,
                                                             double resolution) const
{
    Eigen::Vector3d dimensions = (max_point - min_point) / resolution;
    int nx = static_cast<int>(std::ceil(dimensions.x()));
    int ny = static_cast<int>(std::ceil(dimensions.y()));
    int nz = static_cast<int>(std::ceil(dimensions.z()));

    std::vector<std::vector<std::vector<double>>>
        sdf(nx,
            std::vector<std::vector<double>>(
                ny, std::vector<double>(nz, std::numeric_limits<double>::infinity())));

    unsigned int num_threads = std::thread::hardware_concurrency();
    int chunk_size = std::max(1, nx / static_cast<int>(num_threads));
    std::vector<std::future<void>> futures;

    for (int i = 0; i < nx; i += chunk_size) {
        int end_i = std::min(i + chunk_size, nx);
        futures.emplace_back(std::async(std::launch::async,
                                        &BVHTree::computeSDFChunk,
                                        this,
                                        i,
                                        end_i,
                                        std::cref(min_point),
                                        resolution,
                                        std::ref(sdf)));
    }

    for (auto &f : futures)
        f.wait();
    return sdf;
}

/**
 * @brief Compute a chunk of the SDF grid in parallel.
 */
void BVHTree::computeSDFChunk(int start_i,
                              int end_i,
                              const Eigen::Vector3d &min_point,
                              double resolution,
                              std::vector<std::vector<std::vector<double>>> &sdf) const
{
    int ny = static_cast<int>(sdf[0].size());
    int nz = static_cast<int>(sdf[0][0].size());

    for (int i = start_i; i < end_i; ++i) {
        for (int j = 0; j < ny; ++j) {
            for (int k = 0; k < nz; ++k) {
                Vec3 point = min_point + Eigen::Vector3d(i, j, k) * resolution;
                auto [distance, _] = getDistanceAndGradient(point);
                sdf[i][j][k] = distance;
            }
        }
    }
}
