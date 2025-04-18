//
// Created by Cam on 2024-10-20.
//

#include "Mesh.h"

#include <QVector3D>

// Mesh manipulation

#include <fstream>
#include <iostream>
#include <glm/glm.hpp>
//#include <glm/gtx/norm.hpp>

#include "core/Timer.h"

Mesh::Mesh(float vertices[], uint32_t vertexCount, uint32_t indices[], uint32_t indexCount)
    : m_vertices(vertices, vertexCount)
    , m_faces(indices, indexCount)
    , m_faceNormals(nullptr, 0)
    , m_vertexNormals(nullptr, 0)
    , m_indices(indices)
    , m_indexCount(indexCount)
    , m_colors(nullptr, 0)
    , m_baseColor(1.0f, 1.0f, 1.0f)
    , m_adjacencyOK(false)
{
    initialize(false);
}

Mesh::Mesh(const ConvexHull& hull, bool applyColorPattern)
        : m_vertices(hull.vertices())
        , m_faces(hull.faces())
        , m_faceNormals(nullptr, 0)
        , m_vertexNormals(nullptr, 0)
        , m_indexCount(0)
        , m_colors(applyColorPattern ? new float[hull.facetCount() * STRIDE] : nullptr, applyColorPattern * hull.facetCount())
        , m_baseColor(0.8f, 0.8f, 0.1f)
        , m_adjacencyOK(false)
{

    // Convex hull renders default to alternating yellow-orange color pattern
    if (applyColorPattern) {
        for (uint32_t i = 0; i < faceCount(); i+= 2) {
            setFaceColor(i, {0.8f, 0.6f, 0.1f});
        }
    }

    initialize();
}

Mesh::Mesh(const float *vertices, uint32_t vertexCount, const uint32_t *faceIndices, const uint32_t *faceSizes, uint32_t faceCount)
    : m_vertices(vertices, vertexCount)
    , m_faces(faceIndices, faceSizes, faceCount)
    , m_faceNormals(nullptr, 0)
    , m_vertexNormals(nullptr, 0)
    , m_indexCount(0)
    , m_colors(nullptr, 0)
    , m_baseColor(1.0f, 1.0f, 1.0f)
{
    initialize();
}

Mesh::Mesh(const std::vector<glm::vec3>& vertices, const std::vector<Triangle>& faces)
    : m_vertices(vertices)
    , m_faces(faces)
    , m_faceNormals(nullptr, 0)
    , m_vertexNormals(nullptr, 0)
    , m_indexCount(0)
    , m_colors(nullptr, 0)
    , m_baseColor(1.0f, 1.0f, 1.0f)
{

    m_indexCount = m_faces.triangleCount();
    m_indices = new uint32_t[m_indexCount * STRIDE];
    memcpy(m_indices, m_faces.faces(), m_indexCount * STRIDE * sizeof(uint32_t));

    initialize(false);
}

Mesh::Mesh(const VertexArray& vertices, const FaceArray& faces)
    : m_vertices(vertices)
    , m_faces(faces)
    , m_faceNormals(nullptr, 0)
    , m_vertexNormals(nullptr, 0)
    , m_indexCount(0)
    , m_colors(nullptr, 0)
    , m_baseColor(1.0f, 1.0f, 1.0f)
{
    initialize();
}


Mesh::Mesh(const std::string& filename)
    : m_vertices(nullptr, 0)
    , m_faces(nullptr, nullptr, 0)
    , m_faceNormals(nullptr, 0)
    , m_vertexNormals(nullptr, 0)
    , m_indexCount(0)
    , m_colors(nullptr, 0)
    , m_baseColor(1.0f, 1.0f, 1.0f)
{
    Serializable::deserialize(filename);
    initialize();
}

Mesh::Mesh(std::ifstream& file)
    : m_vertices(nullptr, 0)
    , m_faces(nullptr, nullptr, 0)
    , m_faceNormals(nullptr, 0)
    , m_vertexNormals(nullptr, 0)
    , m_indexCount(0)
    , m_colors(nullptr, 0)
    , m_baseColor(1.0f, 1.0f, 1.0f)
{

    if (Mesh::deserialize(file)) initialize();
    else std::cerr << "Failed to deserialize mesh properly!\n";
}



void Mesh::initialize(bool prepareIndexing)
{
    if (prepareIndexing) {
        m_indexCount = m_faces.triangleCount();
        m_indices = new uint32_t[m_indexCount * STRIDE];
        m_faces.triangulation(m_indices);
    }

    calculateFaceNormals();
    calculateVertexNormals();
}

Mesh::~Mesh()
{
    delete[] m_indices;
}

bool Mesh::serialize(const std::string& filename)
{
    return Serializable::serialize(filename);
}
bool Mesh::serialize(std::ofstream& file)
{
    if (m_vertices.serialize(file) && m_faces.serialize(file)) {

        Serializer::writeVec3(file, m_baseColor);
        Serializer::writeBool(file, faceColorsAssigned());

        if (faceColorsAssigned()) return m_colors.serialize(file);

        return true;
    }

    return false;
}

bool Mesh::deserialize(const std::string& filename)
{
    return Serializable::deserialize(filename);
}
bool Mesh::deserialize(std::ifstream& file)
{
    m_vertices = VertexArray::deserialize(file);
    m_faces = FaceArray::deserialize(file);

    if (m_vertices.vertexCount() > 0 && m_faces.faceCount() > 0) {

        m_baseColor = Serializer::readVec3(file);
        bool faceColors = Serializer::readBool(file);

        if (faceColors) {
            m_colors = VertexArray::deserialize(file);
        }

        return true;
    }

    return false;
}

void Mesh::calculateFaceNormals()
{
    auto normals = new float[3 * m_faces.faceCount()], ptr = normals;
    uint32_t idx = 0;

    for (uint32_t i = 0; i < m_faces.faceCount(); i++) {
        auto face = &m_faces.faces()[idx];
        glm::vec3 normal = m_vertices[face[0]];

        normal = glm::normalize(glm::cross(m_vertices[face[1]] - normal, m_vertices[face[2]] - normal));

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
        auto norm = 1 / glm::length(glm::vec3{normals[3 * i], normals[3 * i + 1], normals[3 * i + 2]});
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

            const glm::vec3& vertex = m_vertices[*idx];
            *vertices++ = vertex.x;
            *vertices++ = vertex.y;
            *vertices++ = vertex.z;

//            const vec3f& normal = m_vertexNormals[*idx];
            const glm::vec3& normal = m_faceNormals[i];
            *normals++ = normal.x;
            *normals++ = normal.y;
            *normals++ = normal.z;

            const glm::vec3& color = m_colors[i];
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

// Assign a base color to the mesh. If vertex colors are in use, overwrites colors of the original base to the new base
void Mesh::setBaseColor(const glm::vec3& color)
{
    if (!m_colors.empty()) {
        for (uint32_t i = 0; i < m_colors.vertexCount(); i++) {
            if (m_colors[i] == m_baseColor) m_colors.replace(i, color);
        }
    }

    m_baseColor =  color;
}

// Assign color to the specified face. If vertex colors are not currently in use, prepares an appropriate array
void Mesh::setFaceColor(uint32_t faceIdx, const glm::vec3& color)
{
    if (m_colors.empty()) {
        m_colors = {new float[m_faces.faceCount() * STRIDE], m_faces.faceCount()};
        for (uint32_t i = 0; i < m_colors.vertexCount(); i++) m_colors.replace(i, m_baseColor);
    }

    if (faceIdx < m_colors.vertexCount()) m_colors.replace(faceIdx, color);
}

void Mesh::calculateAdjacencies()
{
    if (m_adjacencyOK) return; // TODO prepare method to invalidate adjacencies if mesh is updated
    m_adjacencies = m_faces.adjacencies();
    m_adjacencyOK = true;
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

const glm::vec3& Mesh::baseColor() const
{
    return m_baseColor;
}

glm::vec3 Mesh::faceColor(uint32_t faceIdx) const
{
    if (m_colors.empty() || faceIdx >= m_faces.faceCount()) {
        return NULL_COLOR;
    }

    return m_colors[faceIdx];
}

bool Mesh::faceColorsAssigned() const
{
    return !m_colors.empty();
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

std::vector<uint32_t> Mesh::outline(const glm::vec3& axis) // const TODO revert back to const
{
    ScopedTimer timer("Mesh outline calculation");

//    const std::vector<std::vector<uint32_t>>& adjacencies = m_adjacencyOK ? m_adjacencies : m_faces.adjacencies();
    const std::vector<std::vector<uint32_t>>& adjacencies = m_faces.adjacencies();

//    std::cout << "Neighbors\n";
//    auto zz = 0;
//    for (const auto& n : adjacencies) {
//        std::cout << zz++ << ": ";
//        for (uint32_t ni : n) std::cout << ni << " ";
//        std::cout << "\n";
//    }

    std::vector<uint16_t> status = identifyHorizonFaces(axis, adjacencies);
    std::vector<std::vector<uint32_t>> loops;

    std::cout << "--> |" << m_adjacencyOK << "| " << adjacencies.size() << " " << status.size() << "\n";



//    std::cout << "MID Status: ";
//    for (unsigned short s : status) {
//        std::cout << s << " ";
//    } std::cout << "\n";

    // Subdivide horizon faces into continuous sets
    for (uint32_t i = 0; i < m_faces.faceCount(); i++) {
        if (status[i] != 1) continue; // Proceed with only unhandled horizon faces

        std::vector<uint32_t>& loop = loops.emplace_back();
        status[i] = loops.size() + 1;
        loop.push_back(i);

        for (uint32_t j = 0; j < loop.size(); j++) {
//            std::cout << "~ " << i << " " << j << " " << loop[j] << "\n";
            for (uint32_t k = 0; k < adjacencies[loop[j]].size(); k++) {
                uint32_t testIdx = adjacencies[loop[j]][k];
//                std::cout << "=== " << k << " " << testIdx << " ";

                // Select adjacent, unhandled faces that straddle the horizon
                if (testIdx != std::numeric_limits<uint32_t>::max() && status[testIdx] == 1) {
                    status[testIdx] = loops.size() + 1; // Mark face & prevent reevaluation
                    loop.push_back(testIdx);
                }

//                std::cout << "\n";
            }
        }
    }

    std::cout << "Found " << loops.size() << " loops\n";
//    for (const std::vector<uint32_t>& loop : loops) {
//        for (uint32_t idx : loop) std::cout << idx << " ";
//        std::cout << "\n";
//    }
//    std::cout << "===================================\n";

    if (loops.empty()) { // Serious issue - Should not be possible for a normal mesh
        std::cout << "SERIOUS ERROR WITH MESH!\n";
        return {};
    }


//    std::cout << "Status: ";
    for (uint32_t i = 0; i < status.size(); i++) {
        if (status[i] > 0) status[i]--;
//        std::cout << status[i] << " ";
        if (status[i] > 0) {
            setFaceColor(i, {status[i] / (float)loops.size(), 0, 1 - status[i] / (float)loops.size()});
        }

    }
//    std::cout << "\n";

    if (loops.size() > 1) {
        // TODO composite loops to develop the outline
        std::cout << "Multiple loops unhandled! Results are not to be trusted\n";
        return loops[0];
    } else {
        return loops[0];
    }
}

std::vector<uint16_t> Mesh::identifyHorizonFaces(const glm::vec3& axis, const std::vector<std::vector<uint32_t>>& adjacencies) const
{
    ScopedTimer("Horizon face identification");

    std::vector<uint16_t> status(m_faces.faceCount(), std::numeric_limits<uint16_t>::max());

    float dp1 = 0, dp2 = 0;

    // Identify horizon faces (faces with normals opposite to adjacent faces, based on the provided axis)
    for (uint32_t i = 0; i < m_faces.faceCount(); i++) {
        if (status[i] != std::numeric_limits<uint16_t>::max()) continue;

        status[i] = 0;

        for (uint32_t j = 0; j < adjacencies[i].size(); j++) {
            if (adjacencies[i][j] != std::numeric_limits<uint32_t>::max() && status[adjacencies[i][j]] != 0) {
                dp1 = glm::dot(axis, m_faceNormals[i]);
                dp2 = glm::dot(axis, m_faceNormals[adjacencies[i][j]]);

                if (dp1 * dp2 < 0) {
                    status[adjacencies[i][j]] = 1;
                    status[i] = 1;
                    break;
                }
            }
        }
    }

    return status;
}

float Mesh::volume() const
{
    float sum = 0;
    for (uint32_t i = 0; i < triangleCount(); i++) {
        sum += glm::dot(m_vertices[m_indices[i * STRIDE]], glm::cross(m_vertices[m_indices[i * STRIDE + 1]], m_vertices[m_indices[i * STRIDE + 2]]));
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

    std::vector<glm::vec3> vertices(count);
    for (uint32_t i = 0; i < count; i++) vertices[i] = m_vertices[indices[i]];

    return faceArea(vertices);
}

float Mesh::faceArea(const std::vector<glm::vec3>& vertices)
{
    glm::vec3 total = {0, 0, 0};
    for (uint32_t i = 0; i < vertices.size(); i++) {
        glm::vec3 vi1 = vertices[i];
        glm::vec3 vi2 = (i == vertices.size() - 1) ? vertices[0] : vertices[i + 1];

        total += glm::cross(vi1, vi2);
    }

    return std::abs(glm::dot(total, glm::normalize(glm::cross(vertices[1] - vertices[0], vertices[2] - vertices[0]))) / 2);
}