//
// Created by Cam on 2024-10-20.
//

#include "Mesh.h"

#include <QVector3D>

// Mesh manipulation

#include <fstream>
#include <iostream>
#include <glm.hpp>
#include <utility>
//#include <glm/gtx/norm.hpp>

#include "core/Timer.h"

Mesh::Mesh(float vertices[], uint32_t vertexCount, uint32_t indices[], uint32_t indexCount)
    : Mesh(VertexArray(vertices, vertexCount), FaceArray(indices, indexCount))
{
}

Mesh::Mesh(const ConvexHull& hull, bool applyColorPattern)
    : Mesh(VertexArray(hull.vertices()), hull.faces())
{

    // Convex hull renders default to alternating yellow-orange color pattern
    if (applyColorPattern) {
        setFaceColor(0, { 0.8f, 0.8f, 0.1f});

        for (uint32_t i = 0; i < faceCount(); i+= 2) {
            setFaceColor(i, {0.8f, 0.6f, 0.1f});
        }
    }
}

Mesh::Mesh(const float *vertices, uint32_t vertexCount, const uint32_t *faceIndices, const uint32_t *faceSizes, uint32_t faceCount)
    : Mesh(VertexArray(vertices, vertexCount), FaceArray(faceIndices, faceSizes, faceCount))
{
}

Mesh::Mesh(const std::vector<glm::vec3>& vertices, const std::vector<Triangle>& faces)
    : Mesh(VertexArray(vertices), FaceArray(faces))
{
}

Mesh::Mesh(const std::string& filename)
    : Mesh(VertexArray(nullptr, 0), FaceArray(nullptr, nullptr, 0))
{
    Serializable::deserialize(filename);
    initialize();
}

Mesh::Mesh(std::ifstream& file)
    : Mesh(VertexArray(nullptr, 0), FaceArray(nullptr, nullptr, 0))
{
    if (Mesh::deserialize(file)) initialize();
    else std::cerr << "Failed to deserialize mesh properly!\n";
}

Mesh::Mesh(const VertexArray& vertices, const FaceArray& faces)
    : m_initialized(false)
    , m_vertices(vertices)
    , m_faces(faces)
    , m_vertexNormals(nullptr, 0)
    , m_colorOverride(false)
    , m_baseColor(1.0f, 1.0f, 1.0f)
    , m_adjacencyOK(false)
{
    initialize();
}

Mesh::Mesh(uint32_t vertexCount, uint32_t faceCount, uint32_t indexCount)
    : m_initialized(false)
    , m_vertices(vertexCount)
    , m_faces(faceCount, indexCount)
    , m_vertexNormals(nullptr, 0)
    , m_colorOverride(false)
    , m_baseColor(1.0f, 1.0f, 1.0f)
    , m_adjacencyOK(false)
{

}

void Mesh::initialize()
{
    if (!m_initialized) {
        m_faces.calculateNormals(m_vertices.vertices());
        m_faces.triangulate(m_vertices.vertices());
        calculateVertexNormals();

        m_initialized = true;
    }
}

bool Mesh::serialize(const std::string& filename)
{
    return Serializable::serialize(filename);
}
bool Mesh::serialize(std::ofstream& file)
{
    if (m_vertices.serialize(file) && m_faces.serialize(file)) {

        Serializer::writeBool(file, m_colorOverride);
        Serializer::writeVec3(file, m_baseColor);
        // TODO serialization
//        if (vertexColorsAssigned()) return m_colors.serialize(file);

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
    m_vertices = VertexArray(file);
    m_faces = FaceArray::deserialize(file);

    if (m_vertices.vertexCount() > 0 && m_faces.faceCount() > 0) {

        m_colorOverride = Serializer::readBool(file);
        m_baseColor = Serializer::readVec3(file);
//
//        if (faceColors) {
//            m_colors = VertexArray::deserialize(file);
//        }

        return true;
    }

    return false;
}

void Mesh::calculateVertexNormals()
{
    std::vector<glm::vec3> normals(m_vertices.size(), glm::vec3{});

    for (uint32_t i = 0; i < m_faces.faceCount(); i++) {
        auto [start, count] = m_faces.triangleLookup(i);
        for (uint32_t j = 0; j < count; j++) {
            const Triangle& triangle = m_faces.triangles()[start + j];
            const glm::vec3& faceNormal = m_faces.normal(i);

            normals[triangle.I0] += faceNormal;
            normals[triangle.I1] += faceNormal;
            normals[triangle.I2] += faceNormal;
        }
    }

    for (glm::vec3& normal : normals) {
        normal = glm::normalize(normal);
    }

    m_vertexNormals = VertexArray(normals);
}

void Mesh::print() const
{
    auto center = centroid(), sc = boundedOffset();

    std::cout << "[MESH] Vertices: " << m_vertices.vertexCount() << ", Faces: " << m_faces.faceCount()
                << "\ncentroid: (" << center.x << ", " << center.y << ", " << center.z << ")"
                << ", span-center: (" << sc.x << ", " << sc.y << ", " << sc.z << ")"
                << "\nspan: (" << xSpan() << ", " << ySpan() << ", " << zSpan() << ")" << "\n";

    m_vertices.print();

    m_faces.print();

}

void Mesh::scale(float scalar)
{
    m_vertices.scale(scalar);
}

void Mesh::scale(const glm::vec3& scale)
{
    m_vertices.scale(scale);
}

void Mesh::translate(const glm::vec3& translation)
{
    m_vertices.translate(translation);
}

void Mesh::rotate(const glm::vec3& axis, float theta)
{
    m_vertices.rotate(axis, theta);
}

void Mesh::normalize(float scalar)
{
    float dx = xSpan(), dy = ySpan(), dz = zSpan();
    float maxSpan = std::max(dx, std::max(dy, dz));
    scale(scalar / maxSpan);
}

void Mesh::center()
{
    translate(boundedOffset());
}

void Mesh::zero()
{
//    translate(-ConvexHull(m_vertices).center());
    translate(-centroid());

}

bool Mesh::isInitialized() const
{
    return m_initialized;
}

void Mesh::xExtents(float &near, float &far) const
{
    return extents({1, 0, 0}, near, far);
}
void Mesh::yExtents(float &near, float &far) const
{
    return extents({0, 1, 0}, near, far);
}
void Mesh::zExtents(float &near, float &far) const
{
    return extents({0, 0, 1}, near, far);
}

void Mesh::extents(const glm::vec3& axis, float &near, float &far) const
{
    m_vertices.extents(axis, near, far);
}

float Mesh::xSpan() const
{
    return m_vertices.span({ 1, 0, 0 });
}
float Mesh::ySpan() const
{
    return m_vertices.span({ 0, 1, 0 });
}
float Mesh::zSpan() const
{
    return m_vertices.span({ 0, 0, 1 });
}

void Mesh::overrideColor(bool enable)
{
    m_colorOverride = enable;
}

void Mesh::setBaseColor(const glm::vec3& color)
{
    m_baseColor = color;
}

void Mesh::setVertexColor(const glm::vec3& color)
{
    m_vertexColors = std::vector<glm::vec3>(m_vertices.vertexCount(), color);
}

void Mesh::setVertexColor(uint32_t vertexIdx, const glm::vec3& color)
{
    if (vertexIdx >= m_vertices.vertexCount()) throw std::runtime_error("[Mesh] Out of bounds array access!");
    if (m_vertexColors.empty()) m_vertexColors = std::vector<glm::vec3>(m_vertices.vertexCount(), m_baseColor);
    else m_vertexColors[vertexIdx] = color;
}

void Mesh::setFaceColor(const glm::vec3& color)
{
    m_faces.setColor(color);
}

void Mesh::setFaceColor(uint32_t faceIdx, const glm::vec3& color)
{
    m_faces.setColor(faceIdx, color);
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

const VertexArray& Mesh::vertexNormals() const
{
    if (!m_initialized) throw std::runtime_error("[Mesh] Can not access vertex normals. Mesh has yet to be initialized!");
    return m_vertexNormals;
}

const std::vector<glm::vec3>& Mesh::vertexColors() const
{
    return m_vertexColors;
}

const glm::vec3& Mesh::baseColor() const
{
    return m_baseColor;
}

bool Mesh::colorsAssigned() const
{
    return faceColorsAssigned() || vertexColorsAssigned();
}

bool Mesh::faceColorsAssigned() const
{
    return !m_faces.colors().empty();
}
bool Mesh::vertexColorsAssigned() const
{
    return !m_vertexColors.empty();
}

bool Mesh::useBaseColor() const
{
    return m_colorOverride || !colorsAssigned();
}

uint32_t Mesh::triangleCount() const
{
    return m_faces.triangleCount();
}

const uint32_t* Mesh::indices() const
{
    return (uint32_t*)m_faces.triangles().data();
}

uint32_t Mesh::faceCount() const
{
    return m_faces.faceCount();
}

const FaceArray& Mesh::faces() const
{
    return m_faces;
}

uint32_t Mesh::matchFace(const glm::vec3& axis)
{
    return m_faces.matchFace(axis);
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
                dp1 = glm::dot(axis, m_faces.normal(i));
                dp2 = glm::dot(axis, m_faces.normal(adjacencies[i][j]));

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

float Mesh::surfaceArea() const
{
    float sum = 0;
    for (uint32_t i = 0; i < m_faces.size(); i++) sum += faceArea(i);

    return sum;
}

float Mesh::volume() const
{
    return m_faces.volume(m_vertices.vertices());
}

float Mesh::tetrahedronVolume(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
{
    return glm::dot(a, glm::cross(b, c));
}

glm::vec3 Mesh::centroid() const
{
    float sum = 0;
    glm::vec3 centroid(0.0f);

    for (const Triangle& tri : m_faces.triangles()) {
        glm::vec3 a = m_vertices[tri.I0];
        glm::vec3 b = m_vertices[tri.I1];
        glm::vec3 c = m_vertices[tri.I2];

        glm::vec3 center = (a + b + c) * (1.0f / 3);
        float area = Triangle::area(a, b, c);

        centroid += center * area;
        sum += area;
    }

    return centroid / sum;
}

glm::vec3 Mesh::boundedOffset() const
{
    glm::vec3 offset = {};
    float near, far;

    xExtents(near, far);
    offset.x = -(near + far) / 2;

    yExtents(near, far);
    offset.y = -(near + far) / 2;

    zExtents(near, far);
    offset.z = -(near + far) / 2;

    return offset;
}

glm::mat3x3 Mesh::inertiaTensor() const
{
    glm::mat3x3 it(0.0f);

    for (const Triangle& tri : m_faces.triangles()) {
        it += tetrahedronInertiaTensor(m_vertices[tri.I0], m_vertices[tri.I1], m_vertices[tri.I2]);
    }

    // Apply Parallel Axis Theorem to change frame to the centroid
    glm::vec3 c = centroid();
    float v = volume();

    float Ixy = it[0][1] + v * c.x * c.y, Ixz = it[0][2] + v * c.x * c.z, Iyz = it[1][2] + v * c.y * c.z;

    it = {
            it[0][0] - v * (c.y*c.y + c.z*c.z), Ixy, Ixz,
            Ixy, it[1][1] - v * (c.x*c.x + c.z*c.z), Iyz,
            Ixz, Iyz, it[2][2] - v * (c.x*c.x + c.y*c.y)
    };

    return it;
}

glm::mat3x3 Mesh::tetrahedronInertiaTensor(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
{
    glm::vec3 t = a + b + c;

    float px = (a.x * t.x + b.x * b.x + b.x * c.x + c.x * c.x) / 60;
    float py = (a.y * t.y + b.y * b.y + b.y * c.y + c.y * c.y) / 60;
    float pz = (a.z * t.z + b.z * b.z + b.z * c.z + c.z * c.z) / 60;


    float ap = -(t.y * t.z + a.y * a.z + b.y * b.z + c.y * c.z) / 120;
    float bp = -(t.x * t.z + a.x * a.z + b.x * b.z + c.x * c.z) / 120;
    float cp = -(t.x * t.y + a.x * a.y + b.x * b.y + c.x * c.y) / 120;

    return tetrahedronVolume(a, b, c) * glm::mat3x3(
            py + pz, bp, cp,
            bp, px + pz, ap,
            cp, ap, px + py
    );
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
    const uint32_t *indices = m_faces[faceIdx], count = m_faces.faceSizes()[faceIdx];
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