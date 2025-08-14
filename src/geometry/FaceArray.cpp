//
// Created by Cam on 2024-11-12.
//

#include "FaceArray.h"
#include "core/Timer.h"
#include "fileIO/Serializable.h"
#include "geometry/poly/Polygon.h"

#include <glm.hpp>

#include <utility>
#include <unordered_map>
#include <map>

#include <iostream>
#include <limits>
#include <cassert>

FaceArray::FaceArray(uint32_t faceCount, uint32_t indexCount)
        : m_faceCount(faceCount)
        , m_indexCount(indexCount)
        , m_faceSizes(faceCount)
        , m_faces(indexCount)
        , m_normals(faceCount)
{
}

//FaceArray::FaceArray(const uint32_t* indices, uint32_t triangleCount)
//    : FaceArray(triangleCount, 3 * triangleCount)
//{
//    std::copy(indices, indices + 3 * triangleCount, m_faces.begin());
//    std::fill(m_faceSizes.begin(), m_faceSizes.end(), 3);
//}

FaceArray::FaceArray(const std::vector<TriIndex>& faces)
        : FaceArray(faces.size(), sizeof(TriIndex) * faces.size())
{
    uint32_t idx = 0;
    for (const TriIndex& tri : faces) {
        m_faces[idx++] = tri.I0;
        m_faces[idx++] = tri.I1;
        m_faces[idx++] = tri.I2;
    }

    std::fill(m_faceSizes.begin(), m_faceSizes.end(), 3);
}

//FaceArray::FaceArray(const uint32_t* faces, const uint32_t* faceSizes, uint32_t faceCount)
//        : m_faceSizes(faceCount)
//        , m_faceCount(faceCount)
//        , m_indexCount(0)
//        , m_normals(faceCount)
//{
//    std::cout << "FA " << faceCount << " " << faces << " " << faceSizes << "\n";
//
//
//    std::copy(faceSizes, faceSizes + faceCount, m_faceSizes.begin());
//
//    for (uint32_t i = 0; i < m_faceCount; i++) m_indexCount += m_faceSizes[i];
//    m_faces = std::vector<uint32_t>(faces, faces + m_indexCount);
//
//}

FaceArray::FaceArray(const std::vector<std::vector<uint32_t>>& indices)
        : m_faceSizes(indices.size())
        , m_faceCount(indices.size())
        , m_indexCount(0)
        , m_normals(indices.size())
{
    uint32_t idx = 0;
    for (const std::vector<uint32_t>& face : indices) {
        m_faceSizes[idx++] = face.size();
        m_indexCount += face.size();
    }

    m_faces = std::vector<uint32_t>(m_indexCount);
    auto ptr = &m_faces[0];

    for (const std::vector<uint32_t>& face : indices) {
        memcpy(ptr, face.data(), face.size() * sizeof(uint32_t));
        ptr += face.size();
    }
}

bool FaceArray::serialize(std::ofstream& file)
{
    Serializer::writeUint(file, m_faceCount);
    Serializer::writeUint(file, m_indexCount);

    file.write(reinterpret_cast<const char*>(m_faceSizes.data()), m_faceCount * sizeof(uint32_t));
    file.write(reinterpret_cast<const char*>(m_faces.data()), m_indexCount * sizeof(uint32_t));

    return true;
}
FaceArray FaceArray::deserialize(std::ifstream& file)
{
    // Read face details
    uint32_t faceCount = Serializer::readUint(file), indexCount = Serializer::readUint(file);

    // Read face data
    FaceArray faces(faceCount, indexCount);
    file.read(reinterpret_cast<char*>(faces.m_faceSizes.data()), faceCount * sizeof(uint32_t));
    file.read(reinterpret_cast<char*>(faces.m_faces.data()), indexCount * sizeof(uint32_t));

    return faces;
}

void FaceArray::addFace(std::vector<uint32_t> face)
{
    // Reject faces malformed faces (Multiple indices)
    for (uint32_t i = 0; i < face.size() - 1; i++) {
        if (face[i] == face[i + 1]) { // Allow duplicate adjacent vertices, but remove
            if (face.size() < 4) return;

            face.erase(face.begin() + i + 1);
            i--;
            continue;
        }

        for (uint32_t j = i + 1; j < face.size(); j++) {
            if (face[i] == face[j]) {
//                throw std::runtime_error("[FaceArray] Malformed face. Can not attach");
                return;
            }
        }
    }

    m_faces.insert(m_faces.end(), face.begin(), face.end());
    m_faceSizes.emplace_back(face.size());
    m_faceCount++;

    m_colors.emplace_back(1, 1, 1);
}

void FaceArray::addFace(const std::vector<uint32_t>& face, const glm::dvec3& color)
{
    uint32_t count = m_faceCount;

    addFace(face);

    if (count < m_faceCount) {
        m_colors.back() = color;
    }
}

void FaceArray::assignNormals(const std::vector<glm::dvec3>& normals)
{
    m_normals = normals;
    m_normals.resize(m_faceCount);
}

glm::dvec3 FaceArray::calculateNormal(const std::vector<glm::dvec3>& boundary)
{
    if (boundary.size() < 3) throw std::runtime_error("[FaceArray] Can not calculate the normal of boundary! Inadequate size");

    if (boundary.size() == 3) return Triangle3D::normal(boundary[0], boundary[1], boundary[2]);

    glm::dvec3 normal = {};
    for (uint32_t i = 0; i < boundary.size(); i++) {
        const glm::dvec3& current = boundary[i];
        const glm::dvec3& next = boundary[(i + 1) % boundary.size()];

        normal += glm::dvec3{
                (current.y - next.y) * (current.z + next.z),
                (current.z - next.z) * (current.x + next.x),
                (current.x - next.x) * (current.y + next.y)
        };
    }

    return glm::normalize(normal);
}

void FaceArray::calculateNormals(const std::vector<glm::dvec3>& vertices)
{

    m_normals = std::vector<glm::dvec3>(m_faceCount);

    uint32_t *idxPtr = &m_faces[0];
    for (uint32_t i = 0; i < m_faceCount; i++) {
        if (m_faceSizes[i] == 3) { // Simple cross-product normal
            m_normals[i] = Triangle3D::normal(vertices[*idxPtr], vertices[*(idxPtr + 1)], vertices[*(idxPtr + 2)]);
        } else { // Otherwise, evaluate the normal using Newell's method
            m_normals[i] = { 0, 0, 0 };
            for (uint32_t j = 0; j < m_faceSizes[i]; j++) {
                const glm::dvec3& current = vertices[*(idxPtr + j)];
                const glm::dvec3& next = vertices[*(idxPtr + (j + 1) % m_faceSizes[i])];

                m_normals[i] += glm::dvec3{
                        (current.y - next.y) * (current.z + next.z),
                        (current.z - next.z) * (current.x + next.x),
                        (current.x - next.x) * (current.y + next.y)
                };
            }

            m_normals[i] = glm::normalize(m_normals[i]);
        }

        idxPtr += m_faceSizes[i];
    }
}

// Triangulates the faces based on edges created from the provided vertices. May crash if called on an invalid object
void FaceArray::triangulate(const std::vector<glm::dvec3>& vertices)
{
//    std::cout << "Triangulating... " << m_normals.size() << " " << m_faceCount << "\n"; //TODO verify
//    if (!m_normals.empty() && m_normals.size() != m_faceCount) throw std::runtime_error("[FaceArray] Mystery normal error");
//    if (m_normals.size() < m_faceCount) calculateNormals(vertices);
//    calculateNormals(vertices);

    m_triangles.clear();
    m_triFaceLookup.clear();

    uint32_t *idxPtr = &m_faces[0];
    for (uint32_t i = 0; i < m_faceCount; i++) {
        if (m_faceSizes[i] == 3) {
            m_triFaceLookup.emplace_back(m_triangles.size());
            m_triangles.emplace_back(*idxPtr, *(idxPtr + 1), *(idxPtr + 2));
        } else if (m_faceSizes[i] > 3 && inRange(idxPtr, m_faceSizes[i], vertices.size())){

            // Prepare to project the face into 2D space
            std::vector<glm::dvec3> border(m_faceSizes[i]);
            for (uint32_t j = 0; j < m_faceSizes[i]; j++) border[j] = vertices[*(idxPtr + j)];

            // Find the Delaunay triangulation of the projected 2D polygon
            auto triangles = Polygon::triangulate(VertexArray::project(border, m_normals[i]));
            if (triangles.empty()) {
                throw std::runtime_error("[FaceArray] Face triangulation failed");
            }

            // Map indices to the original set
            m_triFaceLookup.emplace_back(m_triangles.size());
            for (TriIndex& tri : triangles) {
                m_triangles.emplace_back(
                        *(idxPtr + tri.I0),
                        *(idxPtr + tri.I1),
                        *(idxPtr + tri.I2)
                        );
            }
        }

        idxPtr += m_faceSizes[i];
    }
}

void FaceArray::setColor(const glm::dvec3& color)
{
    m_colors = std::vector<glm::dvec3>(m_faceCount, color);
}

void FaceArray::setColor(uint32_t idx, const glm::dvec3& color)
{
    if (idx >= m_faceCount) throw std::runtime_error("[FaceArray] Out of bounds array access!");
    if (m_colors.empty()) setColor(color);
    else m_colors[idx] = color;
}

// Prevent improper array access
bool FaceArray::inRange(const uint32_t *idx, uint32_t count, uint32_t limit)
{
    for (uint32_t i = 0; i < count; i++) if (*(idx + i) >= limit) return false;

    return true;
}

uint32_t* FaceArray::operator[](uint32_t idx)
{
    return idxPtr(idx);
}

const uint32_t* FaceArray::operator[](uint32_t idx) const
{
    return idxPtr(idx);
}

uint32_t* FaceArray::idxPtr(uint32_t idx)
{
    if (idx >= m_faceCount) throw std::runtime_error("[FaceArray] Out of bounds array access!");

    uint32_t *ptr = &m_faces[0];
    for (uint32_t i = 0; i < idx; i++) ptr += m_faceSizes[i];

    return ptr;
}

const uint32_t* FaceArray::idxPtr(uint32_t idx) const
{
    if (idx >= m_faceCount) throw std::runtime_error("[FaceArray] Out of bounds array access!");

    const uint32_t *ptr = &m_faces[0];
    for (uint32_t i = 0; i < idx; i++) ptr += m_faceSizes[i];

    return ptr;
}

uint32_t FaceArray::faceCount() const
{
    return m_faceCount;
}

uint32_t FaceArray::indexCount() const
{
    return m_indexCount;
}

const std::vector<uint32_t>& FaceArray::faces() const
{
    return m_faces;
}

const std::vector<uint32_t>& FaceArray::faceSizes() const
{
    return m_faceSizes;
}

uint32_t* FaceArray::faceSizes()
{
    return &m_faceSizes[0];
}

const std::vector<glm::dvec3>& FaceArray::normals() const
{
    return m_normals;
}

glm::dvec3 FaceArray::normal(uint32_t idx) const
{
    if (idx >= m_normals.size()) return {};
    return m_normals[idx];
}

const std::vector<glm::dvec3>& FaceArray::colors() const
{
    return m_colors;
}
glm::dvec3 FaceArray::color(uint32_t idx) const
{
    if (idx >= m_colors.size()) return {};
    return m_colors[idx];
}

uint32_t FaceArray::triangleCount() const
{
    return m_triangles.size();
}

const std::vector<TriIndex>& FaceArray::triangles() const
{
    return m_triangles;
}

// Returns the index of the first triangle belonging to the specified face, and the number of triangles belonging to that face
std::tuple<uint32_t, uint32_t> FaceArray::triangleLookup(uint32_t faceIdx) const
{
    assert(faceIdx < m_faceCount);

    uint32_t last = faceIdx + 1 < m_faceCount ? m_triFaceLookup[faceIdx + 1] : m_triangles.size();
    return { m_triFaceLookup[faceIdx], last - m_triFaceLookup[faceIdx] };
}

// Returns the index of the face to which the specified triangle index belongs
uint32_t FaceArray::faceLookup(uint32_t triIdx) const
{
    assert(triIdx < m_triangles.size());

    for (uint32_t i = 1; i < m_triFaceLookup.size(); i++) {
        if (triIdx < m_triFaceLookup[i]) return i - 1;
    }

    return m_triFaceLookup.size() - 1;
}

std::vector<glm::dvec3> FaceArray::faceBorder(uint32_t faceIdx, const std::vector<glm::dvec3>& vertices) const
{
    assert(faceIdx < m_faceCount);

    std::vector<glm::dvec3> border;

    auto *ptr = idxPtr(faceIdx);
    for (uint32_t i = 0; i < m_faceSizes[faceIdx]; i++) {
        if (*ptr >= vertices.size()) throw std::runtime_error("[FaceArray] Invalid vertex element access when generating border");
        border.push_back(vertices[*ptr++]);
    }

    return border;
}

double FaceArray::volume(const std::vector<glm::dvec3>& vertices) const
{
    double sum = 0;
    for (const TriIndex& tri : m_triangles) {

        // Add contribution of tetrahedron to volume
        sum += glm::dot(vertices[tri.I0], glm::cross(vertices[tri.I1], vertices[tri.I2]));
    }

    return sum / 6.0f;
}

uint32_t FaceArray::matchFace(const glm::dvec3& axis) const
{
    auto match = std::max_element(m_normals.begin(), m_normals.end(), [axis](const glm::dvec3& lhs, const glm::dvec3& rhs){
        return glm::dot(lhs, axis) > glm::dot(rhs, axis);
    });

    return std::distance(m_normals.begin(), match);
}

std::vector<std::vector<uint32_t>> FaceArray::edgeList() const
{
    std::vector<uint64_t> pairs;

    const uint32_t *ptr = &m_faces[0];
    uint32_t pre = m_faces[0];

    // Generate a complete list of edges by vertex index
    for (uint32_t i = 0; i < m_faceCount; i++) {
        for (uint32_t j = 0; j < m_faceSizes[i] - 1; j++) {
            uint64_t value = (uint64_t)*ptr++ << 32;
            pairs.emplace_back(value | *ptr);
        }

        pairs.emplace_back(((uint64_t)*ptr++ << 32) | pre);
        pre = *ptr;
    }

    // Sort doubly-connected edge list for easier access
    std::sort(pairs.begin(), pairs.end());

    // Convert to a more convenient format
    std::vector<std::vector<uint32_t>> edges((pairs[pairs.size() - 1] >> 32) + 1);
    for (uint64_t pair : pairs) {
        edges[pair >> 32].emplace_back(pair & 0xFFFF);
    }

    return edges;
}

std::vector<std::vector<uint32_t>> FaceArray::adjacencies() const
{
    ScopedTimer timer("Face adjacency calculation");
    // TODO Improve speed - takes up to 15s for a representative mesh (ogre)

    std::unordered_map<uint64_t, std::pair<uint32_t, uint32_t>> links;
    std::vector<std::vector<uint32_t>> neighbors;

    for (uint32_t i = 0; i < m_faceCount; i++) {
        neighbors.emplace_back(m_faceSizes[i], std::numeric_limits<uint32_t>::max());

        auto ptr = idxPtr(i);
        uint32_t current = m_faceSizes[i] - 1, next = 0;
        for (; next < m_faceSizes[i]; next++) {
            uint64_t key = linkKey(ptr[current], ptr[next]);

            if (links.find(key) != links.end()) { // Generate link when the edge (key) has already been seen
                auto link = links[key];

                neighbors[link.first][link.second] = i;
                neighbors[i][current] = link.first;
            } else { // Record triangle with key for future reference
                links[key] = {i, current};
            }
            current = next;
        }
    }

    std::cout << "[[[[[[[[ " << links.size() << " " << neighbors.size() << "\n";

    return neighbors;
}

std::vector<uint32_t> FaceArray::instances(uint32_t maxIndex) const
{
    std::vector<uint32_t> count(maxIndex, 0);
    for (uint32_t idx : m_faces) count[idx]++;

    return count;
}

bool FaceArray::hasFreeIndices(uint32_t maxIndex) const
{
    const auto& counts = instances(maxIndex);
    for (uint32_t count : counts) {
        if (count == 0) return true;
    }

    return false;
}

inline uint64_t FaceArray::linkKey(uint32_t I0, uint32_t I1)
{
    if (I0 < I1) return ((uint64_t)I0 << 32) + I1;
    return ((uint64_t)I1 << 32) + I0;
}

uint32_t FaceArray::size() const
{
    return (m_indexCount + m_faceCount + 2) * sizeof(uint32_t);
}

bool FaceArray::empty() const
{
    return m_faceCount == 0;
}

bool FaceArray::isValid() const
{
    if (m_faceCount == 0 || m_faces.empty() || (m_faceCount != m_faceSizes.size())) return false;

    uint32_t sum = 0;
    for (uint32_t i = 0; i < m_faceCount; i++) {
        if (m_faceSizes[i] < 3) return false;
        sum += m_faceSizes[i];
    }

    return sum <= m_faces.size();
}

bool FaceArray::isValid(uint32_t maxIndex) const
{
    if (isValid()) {
        for (uint32_t idx : m_faces) if (idx > maxIndex) return false;
        return true;
    }

    return false;
}

void FaceArray::print() const
{
    uint32_t idx = 0;

    std::cout << "\n~~~~~ Faces (" << m_faceCount << " " << m_faceSizes.size() << "|" << triangleCount() << " " << m_faces.size() << ") ~~~~~\n";
    for (uint32_t i = 0; i < m_faceCount; i++) {
        std::cout << i << "| ";
        for (uint32_t j = 0; j < m_faceSizes[i]; j++) {
            std::cout << m_faces[idx++] << " ";
        }
        std::cout << "\n";
    }
}