#ifndef UTIL_H
#define UTIL_H

#include <QFile>
#include <QIODevice>
#include <QMatrix4x4>
#include <QString>
#include <QVector3D>
#include <Qt3DCore/QGeometry>
#include <Eigen/Dense>
#include <vector>

namespace Util {

    bool loadSTLFile(const QString &fileName, QVector<QVector3D> &vertices);
    void readSTLFile(const QString& filename, std::vector<QVector3D>& vertices, std::vector<QVector3D>& normals);
    Qt3DCore::QGeometry* loadSTL(const QString& filename);
    void findMinMaxVertices(const QVector<QVector3D> &vertices, Eigen::Vector3d &minVertex, Eigen::Vector3d &maxVertex);
    QMatrix4x4 convertEigenAffine3dToQMatrix4x4(const Eigen::Affine3d& transform);
}

#endif // UTIL_H
