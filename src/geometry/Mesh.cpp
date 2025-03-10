//
// Created by Cam on 2024-10-20.
//

#include "Mesh.h"

#include <QVector3D>

// Mesh manipulation

#include <iostream>

#include "core/Timer.h"

Mesh::Mesh(float vertices[], uint32_t vertexCount, uint32_t indices[], uint32_t indexCount)
    : m_vertices(vertices, vertexCount)
    , m_faces(indices, indexCount)
    , m_faceNormals(nullptr, 0)
    , m_vertexNormals(nullptr, 0)
    , m_indices(indices)
    , m_indexCount(indexCount)
    , m_colors(nullptr, 0)
{

//    ScopedTimer timer("Make mesh");

    calculateFaceNormals();
    calculateVertexNormals();
}

Mesh::Mesh(const ConvexHull& hull)
        : m_vertices(hull.vertices())
        , m_faces(hull.faces())
        , m_faceNormals(nullptr, 0)
        , m_vertexNormals(nullptr, 0)
        , m_indexCount(0)
        , m_colors(new float[hull.facetCount() * STRIDE], hull.facetCount())
{

//    ScopedTimer timer("Convert convex hull to mesh");

    m_indexCount = m_faces.triangleCount();
    m_indices = new uint32_t[m_indexCount * STRIDE];
    m_faces.triangulation(m_indices);

    // Convex hull renders default to alternating yellow-orange color pattern
    setBaseColor({0.8f, 0.8f, 0.1f});
    for (uint32_t i = 0; i < faceCount(); i+= 2) {
        setFaceColor(i, {0.8f, 0.6f, 0.1f});
    }

    calculateFaceNormals();
    calculateVertexNormals();
}

Mesh::Mesh(const float *vertices, uint32_t vertexCount, const uint32_t *faceIndices, const uint32_t *faceSizes, uint32_t faceCount)
    : m_vertices(vertices, vertexCount)
    , m_faces(faceIndices, faceSizes, faceCount)
    , m_faceNormals(nullptr, 0)
    , m_vertexNormals(nullptr, 0)
    , m_indexCount(0)
    , m_colors(nullptr, 0)
{
//    ScopedTimer timer("Triangulation of mesh");

    m_indexCount = m_faces.triangleCount();
    m_indices = new uint32_t[m_indexCount * STRIDE];
    m_faces.triangulation(m_indices);


//    m_colors = new float[3 * m_indexCount * STRIDE];
//    setBaseColor({100, 100, 100});
//    for (uint32_t i = 0; i < faceCount(); i+= 2) {
//        setFaceColor(i, {200, 150, 10});
//    }

    calculateFaceNormals();
    calculateVertexNormals();
}

Mesh::Mesh(const VertexArray& vertices, const FaceArray& faces)
    : m_vertices(vertices)
    , m_faces(faces)
    , m_faceNormals(nullptr, 0)
    , m_vertexNormals(nullptr, 0)
    , m_indexCount(0)
    , m_colors(nullptr, 0)
{
    m_indexCount = m_faces.triangleCount();
    m_indices = new uint32_t[m_indexCount * STRIDE];
    m_faces.triangulation(m_indices);


    calculateFaceNormals();
    calculateVertexNormals();
}

Mesh::~Mesh()
{
    delete[] m_indices;
}


void Mesh::calculateFaceNormals()
{
    auto normals = new float[3 * m_faces.faceCount()], ptr = normals;
    uint32_t idx = 0;

    for (uint32_t i = 0; i < m_faces.faceCount(); i++) {
        auto face = &m_faces.faces()[idx];
        vec3f normal = m_vertices[face[0]];

        normal = (m_vertices[face[1]] - normal).cross(m_vertices[face[2]] - normal).normalized();

        *ptr++ = normal.x;
        *ptr++ = normal.y;
        *ptr++ = normal.z;

        idx += m_faces.faceSizes()[i];
    }

    m_faceNormals = {normals, m_faces.faceCount()};

    delete[] normals;
}

void Mesh::calculateVertexNormals()
{
    auto normals = new float[m_vertices.size() * STRIDE];
    for (uint32_t i = 0; i < m_vertices.size() * STRIDE; i++) normals[i] = 0;

    auto idx = m_indices;
    for (uint32_t i = 0; i < m_faces.faceCount(); i++) {
        for (uint32_t j = 0; j < 3 * (m_faces.faceSizes()[i] - 2); j++) {
            normals[STRIDE * *idx    ] += m_faceNormals[i].x;
            normals[STRIDE * *idx + 1] += m_faceNormals[i].y;
            normals[STRIDE * *idx + 2] += m_faceNormals[i].z;
            idx++;
        }
    }

    for (uint32_t i = 0; i < m_vertices.vertexCount(); i++) {
        auto norm = 1 / vec3f::length({normals[3 * i], normals[3 * i + 1], normals[3 * i + 2]});
        normals[3 * i    ] *= norm;
        normals[3 * i + 1] *= norm;
        normals[3 * i + 2] *= norm;
    }

    m_vertexNormals = {normals, m_vertices.size()};
    delete[] normals;
}

void Mesh::directRepresentation(float *vertices, float *normals, float* colors)
{
    auto idx = m_indices;
    for (uint32_t i = 0; i < m_faces.faceCount(); i++) { // For every face
        for (uint32_t j = 0; j < 3 * (m_faces.faceSizes()[i] - 2); j++) { // For every triangle in the face

            const vec3f& vertex = m_vertices[*idx];
            *vertices++ = vertex.x;
            *vertices++ = vertex.y;
            *vertices++ = vertex.z;

//            const vec3f& normal = m_vertexNormals[*idx];
            const vec3f& normal = m_faceNormals[i];
            *normals++ = normal.x;
            *normals++ = normal.y;
            *normals++ = normal.z;

            const vec3f& color = m_colors[i];
            *colors++ = color.x;
            *colors++ = color.y;
            *colors++ = color.z;

            idx++;
        }
    }
}

void Mesh::print() const
{
    std::cout << "==========[ MESH ]==========\n";

    m_vertices.print();

    m_faces.print();

    std::cout << "============================\n";
}

void Mesh::scale(float scalar)
{
    m_vertices.scale(scalar);
}

void Mesh::scale(float x, float y, float z)
{
    m_vertices.scale(x, y, z);
}

void Mesh::translate(float x, float y, float z)
{
    auto translation = new float[3] {x, y, z};
    m_vertices.translate(translation);
    delete[] translation;
}

void Mesh::rotate(float x, float y, float z, float theta)
{
    auto axis = new float[3] {x, y, z};
    m_vertices.rotate(axis, theta);
    delete[] axis;
}

void Mesh::xExtent(float &near, float &far)
{
    auto axis = new float[3] {1, 0, 0};
    m_vertices.extents(axis, near, far);
    delete[] axis;
}
void Mesh::yExtent(float &near, float &far)
{
    auto axis = new float[3] {0, 1, 0};
    m_vertices.extents(axis, near, far);
    delete[] axis;
}
void Mesh::zExtent(float &near, float &far)
{
    auto axis = new float[3] {0, 0, 1};
    m_vertices.extents(axis, near, far);
    delete[] axis;
}

void Mesh::setBaseColor(const vec3f& color)
{
    if (m_colors.empty()) m_colors = {new float[m_faces.faceCount() * STRIDE], m_faces.faceCount()};

    for (uint32_t i = 0; i < m_colors.vertexCount(); i++) m_colors.replace(i, color);
}

void Mesh::setFaceColor(uint32_t faceIdx, const vec3f& color)
{
    if (m_colors.empty()) {
        m_colors = {new float[m_faces.faceCount() * STRIDE], m_faces.faceCount()};
        setBaseColor({1, 1, 1});
    }

    if (faceIdx < m_colors.vertexCount()) m_colors.replace(faceIdx, color);
//    if (faceIdx >= faceCount()) return;

    // Calculate offset index for the face contents
//    uint32_t idx = 0;
//    for (uint32_t i = 0; i < faceIdx; i++) {
//        idx += m_faces.faceSizes()[i];
//    }
//
//    for (uint32_t i = 0; i < m_faces.faceSizes()[faceIdx]; i++) { // For each triangle in the face
//        uint32_t triIdx = m_faces.faces()[idx + i];
//
//        for (uint8_t j = 0; j < 3; j++) { // For each vertex in the triangle
//            uint32_t vertexIdx = (3 * triIdx + j) * STRIDE;
//
//            m_colors[vertexIdx] = color.redF();
//            m_colors[vertexIdx + 1] = color.greenF();
//            m_colors[vertexIdx + 2] = color.blueF();
//        }
//    }
}

uint32_t Mesh::vertexCount() const
{
    return m_vertices.vertexCount();
}

const VertexArray& Mesh::vertices() const
{
    return m_vertices;
}

const VertexArray& Mesh::faceNormals() const
{
    return m_faceNormals;
}
const VertexArray& Mesh::vertexNormals() const
{
    return m_vertexNormals;
}

const VertexArray& Mesh::colors() const
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
    return m_faces.faceCount();
}

const FaceArray& Mesh::faces() const
{
    return m_faces;
}

float Mesh::volume() const
{
    float sum = 0;
    for (uint32_t i = 0; i < triangleCount(); i++) {
        sum += m_vertices[m_indices[i * STRIDE]].dot(vec3f::cross(m_vertices[m_indices[i * STRIDE + 1]], m_vertices[m_indices[i * STRIDE + 2]]));
    }

    return sum / 6.0f;
}

float Mesh::surfaceArea() const
{
    float sum = 0;
    for (uint32_t i = 0; i < m_faces.size(); i++) sum += faceArea(i);

    return sum;
}

std::vector<uint32_t> Mesh::sharedFaces(const std::shared_ptr<Mesh>& reference) const
{
    std::vector<uint32_t> selection;

    std::vector<float> srcArea(m_faces.faceCount(), -1);
    std::vector<float> refArea(reference->m_faces.faceCount(), -1);

    for (uint32_t i = 0; i < m_faces.faceCount(); i++) {
        for (uint32_t j = 0; j < reference->m_faces.faceCount(); j++) {
            if (m_faces.faceSizes()[i] == reference->m_faces.faceSizes()[j]) { // Check equal number of vertices

                // Check equal face area
                if (srcArea[i] == -1) srcArea[i] = faceArea(i);
                if (refArea[j] == -1) refArea[j] = reference->faceArea(j);

                if (std::abs(srcArea[i] - refArea[j]) > 1e-6) continue;

                // TODO Could be good to check face normals, perimeter too to reduce risk of incorrect matches

                selection.emplace_back(i);
                break;
            }
        }
    }

    return selection;
}

float Mesh::faceArea(uint32_t faceIdx) const
{
    uint32_t *indices = m_faces[faceIdx], count = m_faces.faceSizes()[faceIdx];
    if (count < 3) return -1;

    std::vector<vec3f> vertices(count);
    for (uint32_t i = 0; i < count; i++) vertices[i] = m_vertices[indices[i]];

    return faceArea(vertices);
}

float Mesh::faceArea(const std::vector<vec3f>& vertices)
{
    vec3f total = {0, 0, 0};
    for (uint32_t i = 0; i < vertices.size(); i++) {
        vec3f vi1 = vertices[i];
        vec3f vi2 = (i == vertices.size() - 1) ? vertices[0] : vertices[i + 1];

        total += vec3f::cross(vi1, vi2);
    }

    return std::abs(total.dot(vec3f::unitNormal(vertices[0], vertices[1], vertices[2])) / 2);
}