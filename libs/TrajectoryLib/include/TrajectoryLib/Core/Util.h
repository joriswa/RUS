/**
 * @file Util.h
 * @brief Utility functions for STL file handling and geometry processing
 * 
 * This file contains utility functions for loading STL files, processing 3D geometry,
 * and converting between different coordinate system representations.
 */

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
    /**
     * @brief Load an STL file and extract vertices
     * @param fileName Path to the STL file
     * @param vertices Vector to store the loaded vertices
     * @return true if file was loaded successfully, false otherwise
     */
    bool loadSTLFile(const QString &fileName, QVector<QVector3D> &vertices);
    
    /**
     * @brief Read STL file and extract vertices and normals
     * @param filename Path to the STL file
     * @param vertices Vector to store the loaded vertices
     * @param normals Vector to store the loaded normals
     */
    void readSTLFile(const QString& filename, std::vector<QVector3D>& vertices, std::vector<QVector3D>& normals);
    
    /**
     * @brief Load STL file and create Qt3D geometry
     * @param filename Path to the STL file
     * @return Pointer to Qt3D geometry object, nullptr if failed
     */
    Qt3DCore::QGeometry* loadSTL(const QString& filename);
    
    /**
     * @brief Find minimum and maximum vertices in a collection
     * @param vertices Vector of vertices to analyze
     * @param minVertex Reference to store minimum vertex coordinates
     * @param maxVertex Reference to store maximum vertex coordinates
     */
    void findMinMaxVertices(const QVector<QVector3D> &vertices, Eigen::Vector3d &minVertex, Eigen::Vector3d &maxVertex);
    
    /**
     * @brief Convert Eigen Affine3d transformation to Qt QMatrix4x4
     * @param transform Eigen transformation matrix
     * @return Equivalent Qt transformation matrix
     */
    QMatrix4x4 convertEigenAffine3dToQMatrix4x4(const Eigen::Affine3d& transform);
}

#endif // UTIL_H
