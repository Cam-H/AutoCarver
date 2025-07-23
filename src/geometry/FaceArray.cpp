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

FaceArray::FaceArray(uint32_t faceCount, uint32_t indexCount)
        : m_faceCount(faceCount)
        , m_indexCount(indexCount)
        , m_faceSizes(new uint32_t[faceCount])
        , m_faces(new uint32_t[indexCount])
{
}

FaceArray::FaceArray(const uint32_t* indices, uint32_t triangleCount)
    : FaceArray(triangleCount, 3 * triangleCount)
{
    memcpy(m_faces, indices, 3 * triangleCount * sizeof(uint32_t));
    for (uint32_t i = 0; i < m_faceCount; i++) m_faceSizes[i] = 3;
}

FaceArray::FaceArray(const std::vector<Triangle>& faces)
        : FaceArray(faces.size(), 3 * faces.size())
{
    memcpy(m_faces, faces.data(), m_faceCount * sizeof(Triangle));
    for (uint32_t i = 0; i < m_faceCount; i++) m_faceSizes[i] = 3;
}

FaceArray::FaceArray(const uint32_t* faces, const uint32_t* faceSizes, uint32_t faceCount)
        : m_faceSizes(new uint32_t[faceCount])
        , m_faceCount(faceCount)
        , m_indexCount(0)
{
    memcpy(m_faceSizes, faceSizes, faceCount * sizeof(uint32_t));

    for (uint32_t i = 0; i < m_faceCount; i++) m_indexCount += m_faceSizes[i];
    m_faces = new uint32_t[m_indexCount];

    memcpy(m_faces, faces, m_indexCount * sizeof(uint32_t));

}



FaceArray::FaceArray(const std::vector<std::vector<uint32_t>>& indices)
        : m_faceSizes(new uint32_t[indices.size()])
        , m_faceCount(indices.size())
        , m_indexCount(0)
{
    uint32_t idx = 0;
    for (const std::vector<uint32_t>& face : indices) {
        m_faceSizes[idx++] = face.size();
        m_indexCount += face.size();
    }

    m_faces = new uint32_t[m_indexCount];
    auto ptr = m_faces;

    for (const std::vector<uint32_t>& face : indices) {
        memcpy(ptr, face.data(), face.size() * sizeof(uint32_t));
        ptr += face.size();
    }
}

FaceArray::FaceArray(const FaceArray& other)
        : FaceArray(other.m_faces, other.m_faceSizes, other.m_faceCount)
{
    m_normals = other.m_normals;
    m_colors = other.m_colors;

    m_triangles = other.m_triangles;
    m_triFaceLookup = other.m_triFaceLookup;
}

FaceArray& FaceArray::operator=(const FaceArray& other)
{

    if (this == &other)
        return *this;

    FaceArray temp(other);
    std::swap(m_faces, temp.m_faces);
    std::swap(m_faceSizes, temp.m_faceSizes);
    m_faceCount = temp.m_faceCount;
    m_indexCount = temp.m_indexCount;

    return *this;
}

FaceArray::FaceArray(FaceArray&& other) noexcept
        : m_faces(std::exchange(other.m_faces, nullptr))
        , m_faceSizes(std::exchange(other.m_faceSizes, nullptr))
        , m_faceCount(other.m_faceCount)
        , m_indexCount(other.m_indexCount)
{

}

FaceArray& FaceArray::operator=(FaceArray&& other) noexcept
{
    FaceArray temp(std::move(other));
    std::swap(m_faces, temp.m_faces);
    std::swap(m_faceSizes, temp.m_faceSizes);
    m_faceCount = temp.m_faceCount;
    m_indexCount = temp.m_indexCount;

    return *this;
}

FaceArray::~FaceArray()
{
    delete[] m_faces;
    delete[] m_faceSizes;
}

bool FaceArray::serialize(std::ofstream& file)
{
    Serializer::writeUint(file, m_faceCount);
    Serializer::writeUint(file, m_indexCount);

//    file.write(reinterpret_cast<const char*>(&m_faceCount), sizeof(uint32_t));
//    file.write(reinterpret_cast<const char*>(&m_indexCount), sizeof(uint32_t));
    file.write(reinterpret_cast<const char*>(m_faceSizes), m_faceCount * sizeof(uint32_t));
    file.write(reinterpret_cast<const char*>(m_faces), m_indexCount * sizeof(uint32_t));

    return true;
}
FaceArray FaceArray::deserialize(std::ifstream& file)
{
    // Read face details
    uint32_t faceCount = Serializer::readUint(file), indexCount = Serializer::readUint(file);

    // Read face data
    FaceArray fa(faceCount, indexCount);
    file.read(reinterpret_cast<char*>(fa.m_faceSizes), faceCount * sizeof(uint32_t));
    file.read(reinterpret_cast<char*>(fa.m_faces), indexCount * sizeof(uint32_t));

    return fa;
}

glm::vec3 FaceArray::calculateNormal(const std::vector<glm::vec3>& boundary)
{
    if (boundary.size() < 3) throw std::runtime_error("[FaceArray] Can not calculate the normal of boundary! Inadequate size");

    if (boundary.size() == 3) return Triangle::normal(boundary[0], boundary[1], boundary[2]);

    glm::vec3 normal = {};
    for (uint32_t i = 0; i < boundary.size(); i++) {
        const glm::vec3& current = boundary[i];
        const glm::vec3& next = boundary[(i + 1) % boundary.size()];

        normal += glm::vec3{
                (current.y - next.y) * (current.z + next.z),
                (current.z - next.z) * (current.x + next.x),
                (current.x - next.x) * (current.y + next.y)
        };
    }

    return glm::normalize(normal);
}

void FaceArray::calculateNormals(const std::vector<glm::vec3>& vertices)
{
    m_normals = std::vector<glm::vec3>(m_faceCount);

    uint32_t *idxPtr = m_faces;
    for (uint32_t i = 0; i < m_faceCount; i++) {
        if (m_faceSizes[i] == 3) { // Simple cross-product normal
            m_normals[i] = Triangle::normal(vertices[*idxPtr], vertices[*(idxPtr + 1)], vertices[*(idxPtr + 2)]);
        } else { // Otherwise, evaluate the normal using Newell's method
            m_normals[i] = { 0, 0, 0 };
            for (uint32_t j = 0; j < m_faceSizes[i]; j++) {
                const glm::vec3& current = vertices[*(idxPtr + j)];
                const glm::vec3& next = vertices[*(idxPtr + (j + 1) % m_faceSizes[i])];

                m_normals[i] += glm::vec3{
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

void FaceArray::triangulate(const std::vector<glm::vec3>& vertices)
{
//    std::cout << "Triangulating... " << m_normals.size() << " " << m_faceCount << "\n"; TODO verify
//    if (!m_normals.empty() && m_normals.size() != m_faceCount) throw std::runtime_error("[FaceArray] Mystery normal error");
//    if (m_normals.size() < m_faceCount) calculateNormals(vertices);
    calculateNormals(vertices);

    m_triangles.clear();
    m_triFaceLookup.clear();

    uint32_t *idxPtr = m_faces;
    for (uint32_t i = 0; i < m_faceCount; i++) {
        if (m_faceSizes[i] == 3) {
            m_triFaceLookup.emplace_back(m_triangles.size());
            m_triangles.emplace_back(*idxPtr, *(idxPtr + 1), *(idxPtr + 2));
        } else if (m_faceSizes[i] > 3 && inRange(idxPtr, m_faceSizes[i], vertices.size())){

            // Prepare to project the face into 2D space
            std::vector<glm::vec3> border(m_faceSizes[i]);
            for (uint32_t j = 0; j < m_faceSizes[i]; j++) border[j] = vertices[*(idxPtr + j)];

            // Find the Delaunay triangulation of the projected 2D polygon
            auto triangles = Polygon::triangulate(VertexArray::project(border, m_normals[i]));
            if (triangles.empty()) {
                throw std::runtime_error("[FaceArray] Face triangulation failed");
            }

            // Map indices to the original set
            m_triFaceLookup.emplace_back(m_triangles.size());
            for (Triangle& tri : triangles) {
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

void FaceArray::setColor(const glm::vec3& color)
{
    m_colors = std::vector<glm::vec3>(m_faceCount, color);
}

void FaceArray::setColor(uint32_t idx, const glm::vec3& color)
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
    if (idx >= m_faceCount) return nullptr;

    uint32_t *ptr = m_faces;
    for (uint32_t i = 0; i < idx; i++) ptr += m_faceSizes[i];

    return ptr;
}

uint32_t* FaceArray::idxPtr(uint32_t idx) const
{
    if (idx >= m_faceCount) return nullptr;

    uint32_t *ptr = m_faces;
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

const uint32_t* FaceArray::faces() const
{
    return m_faces;
}

uint32_t* FaceArray::faceSizes()
{
    return m_faceSizes;
}
const uint32_t* FaceArray::faceSizes() const
{
    return m_faceSizes;
}

const std::vector<glm::vec3>& FaceArray::normals() const
{
    return m_normals;
}

glm::vec3 FaceArray::normal(uint32_t idx) const
{
    if (idx >= m_normals.size()) return {};
    return m_normals[idx];
}

const std::vector<glm::vec3>& FaceArray::colors() const
{
    return m_colors;
}
glm::vec3 FaceArray::color(uint32_t idx) const
{
    if (idx >= m_colors.size()) return {};
    return m_colors[idx];
}

uint32_t FaceArray::triangleCount() const
{
    return m_triangles.size();
}

const std::vector<Triangle>& FaceArray::triangles() const
{
    return m_triangles;
}

std::tuple<uint32_t, uint32_t> FaceArray::triangleLookup(uint32_t faceIdx) const
{
    if (faceIdx >= m_faceCount) return {};
    else if (faceIdx + 1 == m_faceCount)return { m_triFaceLookup[faceIdx], m_triangles.size() - m_triFaceLookup[faceIdx] };
    else return { m_triFaceLookup[faceIdx], m_triFaceLookup[faceIdx + 1] - m_triFaceLookup[faceIdx] };
}

std::vector<glm::vec3> FaceArray::faceBorder(uint32_t idx, const std::vector<glm::vec3>& vertices) const
{
    if (idx >= m_faceCount) throw std::runtime_error("[FaceArray] Invalid element access when generating border");

    std::vector<glm::vec3> border;

    auto *ptr = idxPtr(idx);
    for (uint32_t i = 0; i < m_faceSizes[idx]; i++) {
        if (*ptr >= vertices.size()) throw std::runtime_error("[FaceArray] Invalid vertex element access when generating border");
        border.push_back(vertices[*ptr++]);
    }

    return border;
}

float FaceArray::volume(const std::vector<glm::vec3>& vertices) const
{
    float sum = 0;
    for (const Triangle& tri : m_triangles) {

        // Add contribution of tetrahedron to volume
        sum += glm::dot(vertices[tri.I0], glm::cross(vertices[tri.I1], vertices[tri.I2]));
    }

    return sum / 6.0f;
}

uint32_t FaceArray::matchFace(const glm::vec3& axis)
{
    auto match = std::max_element(m_normals.begin(), m_normals.end(), [axis](const glm::vec3& lhs, const glm::vec3& rhs){
        return glm::dot(lhs, axis) > glm::dot(rhs, axis);
    });

    return std::distance(m_normals.begin(), match);
}

std::vector<std::vector<uint32_t>> FaceArray::edgeList() const
{
    std::vector<uint64_t> pairs;
    uint32_t pre = m_faces[0], *ptr = m_faces;

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

    uint32_t *ptr = m_faces;
    for (uint32_t i = 0; i < m_faceCount; i++) {
        for (uint32_t j = 0; j < m_faceSizes[i]; j++) count[*ptr++]++;
    }

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

void FaceArray::print() const
{
    auto ptr = m_faces;

    std::cout << "\n~~~~~ Faces (" << m_faceCount << "|" << triangleCount() << ") ~~~~~\n";
    for (uint32_t i = 0; i < m_faceCount; i++) {
        std::cout << i << "| ";
        for (uint32_t j = 0; j < m_faceSizes[i]; j++) {
            std::cout << *ptr++ << " ";
        }
        std::cout << "\n";
    }
}