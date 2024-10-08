//
// Created by cameronh on 26/04/24.
//

#include "GeometryBuilder.h"

#include <QGeometry>
#include <Qt3DCore/QAttribute>
#include <QBuffer>
#include <QByteArray>

#include "Triangle.h"
#include "Tesselation.h"

#include "../core/Timer.h"

void GeometryBuilder::add(Tesselation *sink, Qt3DCore::QGeometry *geometry)
{
    ScopedTimer timer("Tesselation Reconstruction");

    std::unordered_map<size_t, uint32_t> vertexMap;
    std::vector<uint32_t> indexMap;

    std::vector<QVector3D> vertices;
    std::vector<Triangle> triangles;

    for (auto *attrib : geometry->attributes()){
        if (attrib->attributeType() == Qt3DCore::QAttribute::VertexAttribute) {
            if (attrib->name() == "vertexPosition") {

                auto buffer = reinterpret_cast<const float*>(attrib->buffer()->data().constData());

                uint32_t stride = attrib->byteStride() / sizeof(float);
                for (uint32_t i = 0; i < attrib->count(); i++) {
                    uint32_t idx = GeometryBuilder::addVertex(vertices, vertexMap, *buffer, *(buffer + 1), *(buffer + 2));
                    indexMap.push_back(idx);
                    buffer += stride;
                }
            }
//            else if (attrib->name() == "vertexNormal") {
//
//            }
        } else if (attrib->attributeType() == Qt3DCore::QAttribute::IndexAttribute) {

            switch (attrib->vertexBaseType()){
//                case Qt3DCore::QAttribute::UnsignedByte:
//                    break;
                case Qt3DCore::QAttribute::UnsignedShort:
                {
                    auto buffer = reinterpret_cast<const uint16_t*>(attrib->buffer()->data().constData());
                    for (uint32_t i = 0; i < attrib->count(); i += 3) {
                        triangles.emplace_back(indexMap[*buffer], indexMap[*(buffer + 1)], indexMap[*(buffer + 2)]);
                        buffer += 3;
                    }
                }
                    break;
                case Qt3DCore::QAttribute::UnsignedInt:
                    {
                        auto buffer = reinterpret_cast<const uint32_t*>(attrib->buffer()->data().constData());
                        for (uint32_t i = 0; i < attrib->count(); i += 3) {
                            triangles.emplace_back(indexMap[*buffer], indexMap[*(buffer + 1)], indexMap[*(buffer + 2)]);
                            buffer += 3;
                        }
                    }
                    break;
                default:
                    std::cout << "Failure! Unhandled condition when parsing geometry\n";
            }
        }
    }

//    for (const QVector3D& vertex : vertices){
//        std::cout << "v " << vertex.x() << " " << vertex.y() << " " << vertex.z() << "\n";
//    }
//
//    for (Triangle& tri : triangles){
//        std::cout << "f " << tri.m_I0 << " " << tri.m_I1 << " " << tri.m_I2 << "\n";
//    }

    sink->append(vertices, triangles);
}

std::vector<QVector3D> GeometryBuilder::reduce(const std::vector<QVector3D> &vertices, std::vector<Triangle> &triangles)
{
    ScopedTimer timer("Tesselation Reduction");

    std::unordered_map<size_t, uint32_t> vertexMap;
    std::vector<uint32_t> indexMap;

    std::vector<QVector3D> set;
    set.reserve(vertices.size());

    // Identify & remove duplicate vertices
    for (const auto &vertex : vertices) {
        indexMap.emplace_back(GeometryBuilder::addVertex(set, vertexMap, vertex));
    }

    // Adjust triangle indexing to account for vertex removals
    for (Triangle &tri : triangles) {
        tri = {indexMap[tri.m_I0], indexMap[tri.m_I1], indexMap[tri.m_I2]};
    }

    return set;
}

uint32_t GeometryBuilder::addVertex(std::vector<QVector3D>& vertices, std::unordered_map<size_t, uint32_t>& vertexMap, float x, float y, float z)
{
    return addVertex(vertices, vertexMap, QVector3D(x, y, z));
}

uint32_t GeometryBuilder::addVertex(std::vector<QVector3D>& vertices, std::unordered_map<size_t, uint32_t>& vertexMap, QVector3D vertex, float tolerance)
{
    float factor = 1 / tolerance;
    size_t h = hash((uint32_t)(vertex.x() * factor), (uint32_t)(vertex.y() * factor), (uint32_t)(vertex.z() * factor));

    auto it = vertexMap.find(h);
    if(it == vertexMap.end()){
        vertices.push_back(vertex);
        vertexMap[h] = vertices.size() - 1;
        return vertices.size() - 1;
    }

    return it->second;
}

size_t GeometryBuilder::hash(size_t a, size_t b, size_t c){
    return cantor(a, cantor(b, c));
}

size_t GeometryBuilder::cantor(size_t a, size_t b){
    return (a + b + 1) * (a + b) / 2 + b;
}

Qt3DCore::QGeometry* GeometryBuilder::convert(const Tesselation& tessel)
{
    const auto attributes = {
            tessel.getVertices(),
            tessel.getNormals()
    };

    const auto attributeNames = {
            Qt3DCore::QAttribute::defaultPositionAttributeName(),
            Qt3DCore::QAttribute::defaultNormalAttributeName()
    };

    return GeometryBuilder::convert(tessel.getTriangles(), attributes, attributeNames);
}

Qt3DCore::QGeometry* GeometryBuilder::convert(const Tesselation &tessel, const std::vector<QVector3D> &color)
{
    if(tessel.getTriangleCount() == color.size())
    {
        std::vector<QVector3D> vertices;
        std::vector<QVector3D> normals;
        std::vector<Triangle> triangles;
        expand(tessel, vertices, normals, triangles);

        std::vector<QVector3D> colors(color.size() * 3);
        for (uint32_t i = 0; i < vertices.size(); i += 3){
            colors[i] = colors[i + 1] = colors[i + 2] = color[i / 3];
        }

        const auto attributes = {
                vertices,
                normals,
                colors
        };

        const auto attributeNames = {
                Qt3DCore::QAttribute::defaultPositionAttributeName(),
                Qt3DCore::QAttribute::defaultNormalAttributeName(),
                Qt3DCore::QAttribute::defaultColorAttributeName()
        };

        return GeometryBuilder::convert(triangles, attributes, attributeNames);
    }

    return convert(tessel);
}

void GeometryBuilder::expand(const Tesselation &tessel, std::vector<QVector3D> &vertices, std::vector<QVector3D> &normals, std::vector<Triangle> &triangles)
{
    uint32_t idx = 0;
    for (const Triangle& tri : tessel.getTriangles()) {
        vertices.push_back(tessel.getVertices()[tri.m_I0]);
        vertices.push_back(tessel.getVertices()[tri.m_I1]);
        vertices.push_back(tessel.getVertices()[tri.m_I2]);

        normals.push_back(tessel.getVertexNormals()[tri.m_I0]);
        normals.push_back(tessel.getVertexNormals()[tri.m_I1]);
        normals.push_back(tessel.getVertexNormals()[tri.m_I2]);

        triangles.emplace_back(idx, idx + 1, idx + 2);
        idx += 3;
    }
}

Qt3DCore::QGeometry* GeometryBuilder::convert(const std::vector<Triangle> &triangles
                                              , const std::initializer_list<std::vector<QVector3D>>& attributes
                                              , const std::initializer_list<QString>& names)
{
    auto geometry = new Qt3DCore::QGeometry();

    uint32_t vertexCount = (*(attributes.begin())).size();
    uint32_t stride = 3 * attributes.size();// TODO - Low priority to support texture coordinate conversions
    size_t size = stride * vertexCount * sizeof(float);

    // Generate buffer for vertices
    QByteArray vertexData;
    vertexData.resize(size);

    auto vertexPtr = reinterpret_cast<float*>(vertexData.begin());
    for (uint32_t i = 0; i < vertexCount; i++) {
        for (const auto& attribute : attributes) {
            *vertexPtr++ = attribute[i].x();
            *vertexPtr++ = attribute[i].y();
            *vertexPtr++ = attribute[i].z();
        }
    }
    auto vertexBuffer = new Qt3DCore::QBuffer(geometry);
    vertexBuffer->setData(vertexData);

    // Prepare vertex attributes
    for (uint32_t i = 0; i < attributes.size(); i++) {
        auto vertexAttribute = new Qt3DCore::QAttribute(geometry);
        vertexAttribute->setName(*(names.begin() + i));
        vertexAttribute->setAttributeType(Qt3DCore::QAttribute::VertexAttribute);
        vertexAttribute->setVertexBaseType(Qt3DCore::QAttribute::VertexBaseType::Float);
        vertexAttribute->setBuffer(vertexBuffer);
        vertexAttribute->setVertexSize(3);
        vertexAttribute->setByteStride(stride * sizeof(float));
        vertexAttribute->setByteOffset(3 * i * sizeof(float));
        vertexAttribute->setCount(vertexCount);

        geometry->addAttribute(vertexAttribute);
    }


    // Generate buffer for indices
    // TODO support uint16_t simultaneously
    size = 3 * triangles.size() * sizeof(uint32_t);
    QByteArray indexData;
    indexData.resize(size);

    auto indexPtr = reinterpret_cast<uint32_t*>(indexData.data());
    for (const Triangle &tri : triangles) {
        *indexPtr++ = tri.m_I0;//(uint16_t)
        *indexPtr++ = tri.m_I1;//(uint16_t)
        *indexPtr++ = tri.m_I2;//(uint16_t)
    }
    auto indexBuffer = new Qt3DCore::QBuffer(geometry);
    indexBuffer->setData(indexData);

    // Prepare index attributes
    auto indexAttribute = new Qt3DCore::QAttribute(geometry);
    indexAttribute->setAttributeType(Qt3DCore::QAttribute::IndexAttribute);
    indexAttribute->setVertexBaseType(Qt3DCore::QAttribute::VertexBaseType::UnsignedInt);
    indexAttribute->setBuffer(indexBuffer);
    indexAttribute->setCount(3 * triangles.size());

    geometry->addAttribute(indexAttribute);

    return geometry;
}