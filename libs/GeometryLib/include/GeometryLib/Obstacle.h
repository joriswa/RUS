#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <Eigen/Dense>
#include <memory>

using Vec3 = Eigen::Vector3d;

enum class ObstacleType { SPHERE, CYLINDER, BOX };

class Obstacle
{
protected:
    Eigen::Affine3d _transform;
    Vec3 _scale;

public:
    Obstacle(const Vec3 &scale = Vec3(1.0, 1.0, 1.0),
             const Eigen::Affine3d &transform = Eigen::Affine3d::Identity())
        : _transform(transform)
        , _scale(scale)
    {}

    virtual ~Obstacle() = default;

    virtual double implicitFunction(Vec3 point) const = 0;
    virtual bool isSegmentInObstacle(Vec3 start, Vec3 end) const = 0;
    virtual bool intersectsOBB(Vec3 centerBox, Vec3 halfDims, Eigen::Matrix3d axes) const = 0;
    virtual ObstacleType getType() const = 0;
    virtual Eigen::AlignedBox3d getBoundingBox() const = 0;
    virtual std::unique_ptr<Obstacle> clone() const = 0;
    virtual std::shared_ptr<Obstacle> transform(const Eigen::Affine3d &transformMatrix) const = 0;

    virtual Vec3 getClosestPoint(const Vec3 &point) const = 0;
    virtual double getDistance(const Vec3 &point) const = 0;
    virtual Vec3 getDistanceGradient(const Vec3 &point) const = 0;

    const Vec3 &getScale() const { return _scale; }
    const Eigen::Affine3d &getTransform() const { return _transform; };
    void setScale(const Vec3 &scale) { _scale = scale; }
};

class BoxObstacle : public Obstacle
{
public:
    BoxObstacle(const Vec3 &scale = Vec3(1.0, 1.0, 1.0),
                const Eigen::Affine3d &transform = Eigen::Affine3d::Identity())
        : Obstacle(scale, transform)
    {}

    BoxObstacle(const Vec3 &scale, const Eigen::Translation3d &translation)
        : Obstacle(scale, Eigen::Affine3d(translation))
    {}

    double implicitFunction(Vec3 point) const override;
    bool isSegmentInObstacle(Vec3 start, Vec3 end) const override;
    bool intersectsOBB(Vec3 centerBox, Vec3 halfDims, Eigen::Matrix3d axes) const override;
    ObstacleType getType() const override { return ObstacleType::BOX; }
    Eigen::AlignedBox3d getBoundingBox() const override;
    std::unique_ptr<Obstacle> clone() const override
    {
        return std::make_unique<BoxObstacle>(_scale, _transform);
    }
    std::shared_ptr<Obstacle> transform(const Eigen::Affine3d &transformMatrix) const override;

    Vec3 getClosestPoint(const Vec3 &point) const override;
    double getDistance(const Vec3 &point) const override;
    Vec3 getDistanceGradient(const Vec3 &point) const override;
    bool checkCollision(const Obstacle &other) const;
};

#endif // OBSTACLE_H
