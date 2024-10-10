//
// Created by cameronh on 26/04/24.
//

#ifndef AUTOCARVER_GEOMETRYBUILDER_H
#define AUTOCARVER_GEOMETRYBUILDER_H

#include <vector>
#include <unordered_map>
#include <cstdint>

#include <iostream>

namespace Qt3DCore {
    class QGeometry;
    class QAttribute;
    class QBuffer;
}

class QVector3D;
class QString;

class Triangle;
class Tesselation;

class GeometryBuilder {
public:
    static void add(Tesselation *sink, Qt3DCore::QGeometry *geometry);

    static std::vector<QVector3D> reduce(const std::vector<QVector3D> &vertices, std::vector<Triangle> &triangles);

    static Qt3DCore::QGeometry* convert(const Tesselation &tessel);
    static Qt3DCore::QGeometry* convert(const Tesselation &tessel, const std::vector<QVector3D> &color);

private:
    static uint32_t addVertex(std::vector<QVector3D> &vertices, std::unordered_map<size_t, uint32_t> &vertexMap, float x, float y, float z);
    static uint32_t addVertex(std::vector<QVector3D> &vertices, std::unordered_map<size_t, uint32_t> &vertexMap, QVector3D vertex, float tolerance = 0.001f);
    static size_t hash(size_t a, size_t b, size_t c);
    static size_t cantor(size_t a, size_t b);

    static void expand(const Tesselation &tessel, std::vector<QVector3D> &vertices, std::vector<QVector3D> &normals, std::vector<Triangle> &triangles);
    static Qt3DCore::QGeometry* convert(const std::vector<Triangle> &triangles
                                        , const std::initializer_list<std::vector<QVector3D>> &attributes
                                        , const std::initializer_list<QString> &names);
};


#endif //AUTOCARVER_GEOMETRYBUILDER_H
