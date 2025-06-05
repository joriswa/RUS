#include "GeometryLib/Obstacle.h"

double BoxObstacle::implicitFunction(Vec3 point) const
{
    Vec3 localPoint = _transform.inverse() * point;
    localPoint = localPoint.cwiseQuotient(_scale);
    localPoint = localPoint.cwiseAbs() - Vec3(0.5, 0.5, 0.5);
    localPoint = localPoint.cwiseMax(Vec3::Zero());
    return localPoint.norm();
}

bool BoxObstacle::isSegmentInObstacle(Vec3 start, Vec3 end) const
{
    Vec3 localStart = _transform.inverse() * start;
    Vec3 localEnd = _transform.inverse() * end;
    localStart = localStart.cwiseQuotient(_scale);
    localEnd = localEnd.cwiseQuotient(_scale);
    Vec3 d = localEnd - localStart;

    for (int i = 0; i < 3; ++i) {
        if (std::abs(d[i]) < 1e-10) {
            if (localStart[i] < -0.5 || localStart[i] > 0.5)
                return false;
        } else {
            double t1 = (-0.5 - localStart[i]) / d[i];
            double t2 = (0.5 - localStart[i]) / d[i];
            double tmin = std::min(t1, t2);
            double tmax = std::max(t1, t2);
            if (tmax < 0 || tmin > 1)
                return false;
        }
    }

    return true;
}

bool BoxObstacle::intersectsOBB(Vec3 centerBox, Vec3 halfDims, Eigen::Matrix3d axes) const
{
    const double EPSILON = 1e-6;
    Vec3 translationVector = centerBox - _transform.translation();
    Eigen::Matrix3d thisAxes = _transform.linear();

    // Test the 6 face normal axes first (early exit optimization)
    for (int i = 0; i < 3; ++i) {
        Vec3 axis = thisAxes.col(i);
        double r1 = 0, r2 = 0;

        for (int j = 0; j < 3; ++j) {
            r1 += std::abs(thisAxes.col(j).dot(axis)) * (_scale[j] / 2);
            r2 += std::abs(axes.col(j).dot(axis)) * halfDims[j];
        }

        if (std::abs(translationVector.dot(axis)) > r1 + r2 + EPSILON) {
            return false;
        }
    }

    for (int i = 0; i < 3; ++i) {
        Vec3 axis = axes.col(i);
        double r1 = 0, r2 = 0;

        for (int j = 0; j < 3; ++j) {
            r1 += std::abs(thisAxes.col(j).dot(axis)) * (_scale[j] / 2);
            r2 += std::abs(axes.col(j).dot(axis)) * halfDims[j];
        }

        if (std::abs(translationVector.dot(axis)) > r1 + r2 + EPSILON) {
            return false;
        }
    }

    // Only test cross product axes if needed
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vec3 axis = thisAxes.col(i).cross(axes.col(j));
            double axisLength = axis.norm();

            if (axisLength < EPSILON) {
                continue; // Skip if axes are parallel
            }

            axis /= axisLength; // Normalize

            double r1 = 0, r2 = 0;
            for (int k = 0; k < 3; ++k) {
                r1 += std::abs(thisAxes.col(k).dot(axis)) * (_scale[k] / 2);
                r2 += std::abs(axes.col(k).dot(axis)) * halfDims[k];
            }

            if (std::abs(translationVector.dot(axis)) > r1 + r2 + EPSILON) {
                return false;
            }
        }
    }

    return true;
}
Eigen::AlignedBox3d BoxObstacle::getBoundingBox() const
{
    Vec3 halfExtents = 0.5 * _scale;

    Vec3 vertices[8];
    vertices[0] = Vec3(-halfExtents.x(), -halfExtents.y(), -halfExtents.z());
    vertices[1] = Vec3(halfExtents.x(), -halfExtents.y(), -halfExtents.z());
    vertices[2] = Vec3(-halfExtents.x(), halfExtents.y(), -halfExtents.z());
    vertices[3] = Vec3(halfExtents.x(), halfExtents.y(), -halfExtents.z());
    vertices[4] = Vec3(-halfExtents.x(), -halfExtents.y(), halfExtents.z());
    vertices[5] = Vec3(halfExtents.x(), -halfExtents.y(), halfExtents.z());
    vertices[6] = Vec3(-halfExtents.x(), halfExtents.y(), halfExtents.z());
    vertices[7] = Vec3(halfExtents.x(), halfExtents.y(), halfExtents.z());

    Eigen::Vector3d minCorner = _transform * vertices[0];
    Eigen::Vector3d maxCorner = _transform * vertices[0];

    for (int i = 1; i < 8; ++i) {
        Eigen::Vector3d transformedVertex = _transform * vertices[i];
        minCorner = minCorner.cwiseMin(transformedVertex);
        maxCorner = maxCorner.cwiseMax(transformedVertex);
    }

    return Eigen::AlignedBox3d(minCorner, maxCorner);
}

std::shared_ptr<Obstacle> BoxObstacle::transform(const Eigen::Affine3d &transformMatrix) const
{
    Eigen::Affine3d newTransform = transformMatrix * _transform;
    return std::make_shared<BoxObstacle>(_scale, newTransform);
}

Vec3 BoxObstacle::getClosestPoint(const Vec3 &point) const
{
    Vec3 localPoint = _transform.inverse() * point;
    localPoint = localPoint.cwiseQuotient(_scale);
    localPoint = localPoint.cwiseMax(Vec3(-0.5, -0.5, -0.5)).cwiseMin(Vec3(0.5, 0.5, 0.5));
    return _transform * (localPoint.cwiseProduct(_scale));
}

double BoxObstacle::getDistance(const Vec3 &point) const
{
    Vec3 localPoint = _transform.inverse() * point;
    localPoint = localPoint.cwiseQuotient(_scale);
    Vec3 closestPoint = localPoint.cwiseMax(Vec3(-0.5, -0.5, -0.5)).cwiseMin(Vec3(0.5, 0.5, 0.5));
    return (_transform * (closestPoint.cwiseProduct(_scale)) - point).norm();
}

Vec3 BoxObstacle::getDistanceGradient(const Vec3 &point) const
{
    Vec3 localPoint = _transform.inverse() * point;
    localPoint = localPoint.cwiseQuotient(_scale);
    Vec3 d = localPoint.cwiseAbs() - Vec3(0.5, 0.5, 0.5);

    if (d.maxCoeff() < 0) {
        Eigen::Vector3i::Index maxIndex;
        d.cwiseAbs().maxCoeff(&maxIndex);
        Vec3 localGrad = Vec3::Zero();
        localGrad[maxIndex] = localPoint[maxIndex] > 0 ? 1 : -1;
        return _transform.linear() * localGrad.cwiseQuotient(_scale);
    } else {
        Vec3 localGrad = localPoint.cwiseSign().cwiseProduct(d.cwiseMax(Vec3::Zero())).normalized();
        return _transform.linear() * localGrad.cwiseQuotient(_scale);
    }
}
