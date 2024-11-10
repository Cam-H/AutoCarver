//
// Created by Cam on 2024-10-20.
//

#include "Mesh.h"

#include <QVector3D>

// Mesh manipulation

#include <iostream>

#include "core/Timer.h"

Mesh::Mesh(float vertices[], uint32_t vertexCount, uint32_t indices[], uint32_t indexCount)
    : m_vertices(vertices)
    , m_vertexCount(vertexCount)
    , m_indices(indices)
    , m_indexCount(indexCount)
    , m_triNormals(nullptr)
    , m_normals(nullptr)
    , m_faces(nullptr)
    , m_faceSizes(nullptr)
    , m_faceCount(indexCount)
    , m_colors(nullptr)
{

    ScopedTimer timer("Make mesh");

    calculateFaceNormals();
    calculateVertexNormals();
}

Mesh::Mesh(const ConvexHull& hull)
        : m_vertices(new float[hull.vertexCount() * STRIDE])
        , m_vertexCount(hull.vertexCount())
        , m_indices(nullptr)
        , m_indexCount(0)
        , m_triNormals(nullptr)
        , m_normals(nullptr)
        , m_faces(nullptr)
        , m_faceSizes(new uint32_t[hull.facetCount()])
        , m_faceCount(hull.facetCount())
        , m_colors(nullptr)
{

    ScopedTimer timer("Convert convex hull to mesh");

    std::copy(hull.vertices(), hull.vertices() + hull.vertexCount() * STRIDE, m_vertices);
    m_indices = hull.triangulate(m_indexCount);

    for (uint32_t i = 0; i < m_faceCount; i++) {
        m_faceSizes[i] = hull.facetSizes()[i] - 2;
    }

    m_faces = new uint32_t[triangleCount()];
    for (uint32_t i = 0; i < triangleCount(); i++) m_faces[i] = i;


    m_colors = new float[3 * triangleCount() * STRIDE];
    setBaseColor({200, 200, 20});
    for (uint32_t i = 0; i < faceCount(); i+= 2) {
        setFaceColor(i, {200, 150, 10});
    }

    calculateFaceNormals();
    calculateVertexNormals();
}

Mesh::~Mesh()
{
    delete[] m_vertices;

    delete[] m_indices;

    delete[] m_triNormals;
    delete[] m_normals;

    delete[] m_faces;
    delete[] m_faceSizes;

    delete[] m_colors;
}

void Mesh::calculateFaceNormals()
{
    delete[] m_triNormals;

    m_triNormals = new float[3 * m_indexCount];

    for (uint32_t i = 0; i < m_indexCount; i++) {
        QVector3D normal = QVector3D::crossProduct(
                QVector3D(
                        m_vertices[3 * m_indices[3 * i + 1]]     - m_vertices[3 * m_indices[3 * i]],
                        m_vertices[3 * m_indices[3 * i + 1] + 1] - m_vertices[3 * m_indices[3 * i] + 1],
                        m_vertices[3 * m_indices[3 * i + 1] + 2] - m_vertices[3 * m_indices[3 * i] + 2]
                        ),
                QVector3D(
                        m_vertices[3 * m_indices[3 * i + 2]]     - m_vertices[3 * m_indices[3 * i]],
                        m_vertices[3 * m_indices[3 * i + 2] + 1] - m_vertices[3 * m_indices[3 * i] + 1],
                        m_vertices[3 * m_indices[3 * i + 2] + 2] - m_vertices[3 * m_indices[3 * i] + 2]
                        )
                ).normalized();

        m_triNormals[3 * i] = normal.x();
        m_triNormals[3 * i + 1] = normal.y();
        m_triNormals[3 * i + 2] = normal.z();
    }
}

void Mesh::calculateVertexNormals()
{
    delete[] m_normals;

    m_normals = new float[3 * m_vertexCount];
    for (uint32_t i = 0; i < 3 * m_vertexCount; i++) m_normals[i] = 0;

    for (uint32_t i = 0; i < 3 * m_indexCount; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            m_normals[3 * m_indices[i] + j] += m_triNormals[(i /3) * 3 + j];
        }
    }

    for (uint32_t i = 0; i < m_vertexCount; i++) {
        auto norm = 1 / (float)sqrt(pow(m_vertices[3 * i], 2) + pow(m_vertices[3 * i + 1], 2) + pow(m_vertices[3 * i + 2], 2));
        m_normals[3 * i] *= norm;
        m_normals[3 * i + 1] *= norm;
        m_normals[3 * i + 2] *= norm;
    }
}

void Mesh::directRepresentation(float *vertices, float *normals)
{
    uint32_t idx = 0;
    for (uint32_t i = 0; i < m_indexCount; i++) { // For each triangle
        for (uint32_t j = 0; j < 3; j++) { // For each vertex in the triangle
            uint32_t vIdx = 3 * m_indices[3 * i + j];

            for (uint8_t k = 0; k < 3; k++) { // Copy components of each vector
                vertices[idx + k] = m_vertices[vIdx + k];
                normals[idx + k] = m_normals[vIdx + k];
            }

            idx += 3;
        }
    }
}

//void Mesh::setTranslation(QVector3D translation)
//{
//    m_transform->setTranslation(translation);
//}
//
//void Mesh::setRotation(QQuaternion rotation)
//{
//    m_transform->setRotation(rotation);
//}
//
//Qt3DCore::QTransform *Mesh::transformation()
//{
//    return m_transform;
//}

void Mesh::setBaseColor(QColor color)
{
    ScopedTimer timer("Base color assignment");
    float *cPtr = m_colors;
    for (uint32_t i = 0; i < 3 * triangleCount(); i++) {
        *cPtr++ = color.redF();
        *cPtr++ = color.greenF();
        *cPtr++ = color.blueF();
    }
}

void Mesh::setFaceColor(uint32_t faceIdx, QColor color)
{
    if (faceIdx >= faceCount()) return;

    // Calculate offset index for the face contents
    uint32_t idx = 0;
    for (uint32_t i = 0; i < faceIdx; i++) {
        idx += m_faceSizes[i];
    }

    for (uint32_t i = 0; i < m_faceSizes[faceIdx]; i++) { // For each triangle in the face
        uint32_t triIdx = m_faces[idx + i];

        for (uint8_t j = 0; j < 3; j++) { // For each vertex in the triangle
            uint32_t vertexIdx = (3 * triIdx + j) * STRIDE;

            m_colors[vertexIdx] = color.redF();
            m_colors[vertexIdx + 1] = color.greenF();
            m_colors[vertexIdx + 2] = color.blueF();
        }
    }
}

uint32_t Mesh::vertexCount() const
{
    return m_vertexCount;
}

float* Mesh::vertices() const
{
    return m_vertices;
}

float* Mesh::normals() const
{
    return m_normals;
}

float* Mesh::colors() const
{
    return m_colors;
}

uint32_t Mesh::triangleCount() const
{
    return m_indexCount;
}

uint32_t* Mesh::indices()
{
    return m_indices;
}

uint32_t Mesh::faceCount() const
{
    return m_faceCount;
}
