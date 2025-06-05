#include "TrajectoryLib/Util.h"

namespace Util {

void readSTLFile(const QString& filename, std::vector<QVector3D>& vertices, std::vector<QVector3D>& normals)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning("Failed to open STL file.");
        return;
    }

    char header[80];
    file.read(header, 80);

    unsigned int numTriangles;
    file.read(reinterpret_cast<char*>(&numTriangles), sizeof(numTriangles));

    vertices.reserve(numTriangles * 3);
    normals.reserve(numTriangles);

    for (unsigned int i = 0; i < numTriangles; ++i) {
        QVector3D normal, vertex1, vertex2, vertex3;

        file.read(reinterpret_cast<char*>(&normal[0]), sizeof(float));
        file.read(reinterpret_cast<char*>(&normal[1]), sizeof(float));
        file.read(reinterpret_cast<char*>(&normal[2]), sizeof(float));

        file.read(reinterpret_cast<char*>(&vertex1[0]), sizeof(float));
        file.read(reinterpret_cast<char*>(&vertex1[1]), sizeof(float));
        file.read(reinterpret_cast<char*>(&vertex1[2]), sizeof(float));

        file.read(reinterpret_cast<char*>(&vertex2[0]), sizeof(float));
        file.read(reinterpret_cast<char*>(&vertex2[1]), sizeof(float));
        file.read(reinterpret_cast<char*>(&vertex2[2]), sizeof(float));

        file.read(reinterpret_cast<char*>(&vertex3[0]), sizeof(float));
        file.read(reinterpret_cast<char*>(&vertex3[1]), sizeof(float));
        file.read(reinterpret_cast<char*>(&vertex3[2]), sizeof(float));

        vertices.push_back(vertex1);
        vertices.push_back(vertex2);
        vertices.push_back(vertex3);

        normals.push_back(normal);
        normals.push_back(normal);
        normals.push_back(normal);

        quint16 attributeByteCount;
        file.read(reinterpret_cast<char*>(&attributeByteCount), sizeof(attributeByteCount));
    }

    file.close();
}

Qt3DCore::QGeometry* loadSTL(const QString& filename)
{
    std::vector<QVector3D> vertices;
    std::vector<QVector3D> normals;

    readSTLFile(filename, vertices, normals);

    auto geometry = new Qt3DCore::QGeometry();
    Qt3DCore::QBuffer *vertexDataBuffer = new Qt3DCore::QBuffer(geometry);
    Qt3DCore::QBuffer *indexDataBuffer = new Qt3DCore::QBuffer(geometry);
    Qt3DCore::QBuffer *normalDataBuffer = new Qt3DCore::QBuffer(geometry);

    QByteArray vertexData;
    QByteArray indexData;
    QByteArray normalData;

    for (const auto& vertex : vertices) {
        vertexData.append(reinterpret_cast<const char*>(&vertex), sizeof(QVector3D));
    }

    for (unsigned int i = 0; i < vertices.size(); ++i) {
        indexData.append(reinterpret_cast<const char*>(&i), sizeof(unsigned int));
    }

    for (const auto& normal : normals) {
        normalData.append(reinterpret_cast<const char*>(&normal), sizeof(QVector3D));
    }

    vertexDataBuffer->setData(vertexData);
    indexDataBuffer->setData(indexData);
    normalDataBuffer->setData(normalData);

    auto positionAttribute = new Qt3DCore::QAttribute();
    positionAttribute->setName(Qt3DCore::QAttribute::defaultPositionAttributeName());
    positionAttribute->setVertexBaseType(Qt3DCore::QAttribute::Float);
    positionAttribute->setVertexSize(3);
    positionAttribute->setAttributeType(Qt3DCore::QAttribute::VertexAttribute);
    positionAttribute->setBuffer(vertexDataBuffer);
    positionAttribute->setByteStride(3 * sizeof(float));
    positionAttribute->setCount(vertices.size());

    auto normalAttribute = new Qt3DCore::QAttribute();
    normalAttribute->setName(Qt3DCore::QAttribute::defaultNormalAttributeName());
    normalAttribute->setVertexBaseType(Qt3DCore::QAttribute::Float);
    normalAttribute->setVertexSize(3);
    normalAttribute->setAttributeType(Qt3DCore::QAttribute::VertexAttribute);
    normalAttribute->setBuffer(normalDataBuffer);
    normalAttribute->setByteStride(3 * sizeof(float));
    normalAttribute->setCount(normals.size());

    auto indexAttribute = new Qt3DCore::QAttribute();
    indexAttribute->setVertexBaseType(Qt3DCore::QAttribute::UnsignedInt);
    indexAttribute->setAttributeType(Qt3DCore::QAttribute::IndexAttribute);
    indexAttribute->setBuffer(indexDataBuffer);
    indexAttribute->setCount(vertices.size());

    geometry->addAttribute(positionAttribute);
    geometry->addAttribute(normalAttribute);
    geometry->addAttribute(indexAttribute);

    return geometry;
}

bool loadSTLFile(const QString &fileName, QVector<QVector3D> &vertices) {
    QFile file(fileName);

    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Cannot open file for reading:" << file.errorString();
        return false;
    }

    QByteArray header = file.read(80);
    Q_UNUSED(header);

    quint32 numTriangles;
    file.read(reinterpret_cast<char*>(&numTriangles), sizeof(numTriangles));

    for (quint32 i = 0; i < numTriangles; ++i) {
        QVector3D normal;
        QVector3D vertex1, vertex2, vertex3;

        file.read(reinterpret_cast<char*>(&normal), sizeof(QVector3D));
        file.read(reinterpret_cast<char*>(&vertex1), sizeof(QVector3D));
        file.read(reinterpret_cast<char*>(&vertex2), sizeof(QVector3D));
        file.read(reinterpret_cast<char*>(&vertex3), sizeof(QVector3D));

        vertices.append(vertex1);
        vertices.append(vertex2);
        vertices.append(vertex3);

        quint16 attributeByteCount;
        file.read(reinterpret_cast<char*>(&attributeByteCount), sizeof(attributeByteCount));
    }

    file.close();
    return true;
}

void findMinMaxVertices(const QVector<QVector3D> &vertices, Eigen::Vector3d &minVertex, Eigen::Vector3d &maxVertex) {
    minVertex = Eigen::Vector3d(std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max());
    maxVertex = Eigen::Vector3d(std::numeric_limits<float>::lowest(),
                                std::numeric_limits<float>::lowest(),
                                std::numeric_limits<float>::lowest());

    for (const QVector3D &vertex : vertices) {
        minVertex.x() = std::min(minVertex.x(), static_cast<double>(vertex.x()));
        minVertex.y() = std::min(minVertex.y(), static_cast<double>(vertex.y()));
        minVertex.z() = std::min(minVertex.z(), static_cast<double>(vertex.z()));

        maxVertex.x() = std::max(maxVertex.x(), static_cast<double>(vertex.x()));
        maxVertex.y() = std::max(maxVertex.y(), static_cast<double>(vertex.y()));
        maxVertex.z() = std::max(maxVertex.z(), static_cast<double>(vertex.z()));
    }
}

QMatrix4x4 convertEigenAffine3dToQMatrix4x4(const Eigen::Affine3d& transform) {
    QMatrix4x4 matrix;
    matrix.setToIdentity();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            matrix(i, j) = transform.rotation()(i, j);
        }
        matrix(i, 3) = transform.translation()(i);
    }

    matrix(3, 3) = 1.0;

    return matrix;
}
}

