//
// Created by Cam on 2024-11-12.
//

#include "FaceArray.h"
#include "core/Timer.h"
#include "fileIO/Serializable.h"

#include <glm/glm.hpp>

#include <utility>
#include <unordered_map>
#include <map>

#include <iostream>
#include <limits>

FaceArray::FaceArray(const uint32_t* indices, uint32_t triangleCount)
    : m_faces(new uint32_t[3 * triangleCount])
    , m_faceSizes(new uint32_t[triangleCount])
    , m_faceCount(triangleCount)
    , m_indexCount(3 * triangleCount)
{
    memcpy(m_faces, indices, 3 * triangleCount * sizeof(uint32_t));
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

FaceArray::FaceArray(const std::vector<Triangle>& faces)
    : m_faceSizes(new uint32_t[faces.size()])
    , m_faceCount(faces.size())
    , m_indexCount(3 * faces.size())
{
    m_faces = new uint32_t[m_indexCount];

    for (uint32_t i = 0; i < m_faceCount; i++) m_faceSizes[i] = 3;
    memcpy(m_faces, faces.data(), m_faceCount * sizeof(Triangle));
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

FaceArray::FaceArray(uint32_t faceCount, uint32_t indexCount)
    : m_faceCount(faceCount)
    , m_indexCount(indexCount)
    , m_faceSizes(new uint32_t[faceCount])
    , m_faces(new uint32_t[indexCount])
{
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

uint32_t* FaceArray::operator[](uint32_t idx)
{
    return idxPtr(idx);
}
uint32_t* FaceArray::operator[](uint32_t idx) const
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

const uint32_t* FaceArray::faces() const
{
    return m_faces;
}
const uint32_t* FaceArray::faceSizes() const
{
    return m_faceSizes;
}
uint32_t FaceArray::faceCount() const
{
    return m_faceCount;
}

glm::vec3 FaceArray::normal(uint32_t idx, const VertexArray& vertices) const
{
    uint32_t* ptr = idxPtr(idx);
    if (ptr == nullptr) return {};

    glm::vec3 normal = vertices[ptr[0]];
    return glm::normalize(glm::cross(vertices[ptr[1]] - normal, vertices[ptr[2]] - normal));
}

// Requires that the faces are convex TODO handle non-convex faces
void FaceArray::triangulation(uint32_t* indices)
{
    uint32_t idx = 0;
    auto idxPtr = indices;
    for (uint32_t i = 0; i < m_faceCount; i++) {

        // Triangulate facet, splitting in a fan-shape from the first vertex
        for (uint32_t j = 0; j < m_faceSizes[i] - 2; j++) {
            *idxPtr++ = m_faces[idx];
            *idxPtr++ = m_faces[idx + j + 1];
            *idxPtr++ = m_faces[idx + j + 2];
        }

        idx += m_faceSizes[i];
    }
}
uint32_t FaceArray::triangleCount() const
{
    return m_indexCount - 2 * m_faceCount;
}

uint32_t FaceArray::indexCount() const
{
    return m_indexCount;
}

FaceArray FaceArray::triangulated()
{
    uint32_t count = triangleCount();
    auto *faces = new uint32_t[3 * count];

    triangulation(faces);

    return {faces, count};
}

// TODO non-convex triangulation
//    std::vector<QVector3D> loop;
//    uint32_t idx = 0, offset = 0, *idxPtr = m_indices;
//    for (uint32_t i = 0; i < m_faceCount; i++) {
//        std::cout << i << " " << idx << " " << idxPtr - m_indices << "\n";
//        for (uint32_t j = 0; j < m_faceSizes[i]; j++) {
//            std::cout << "Vertex: " << m_faces[idx] << " = " <<m_vertices[m_faces[idx]][0] << " " << m_vertices[m_faces[idx]][1] << " " << m_vertices[m_faces[idx]][2] << "\n";
//            loop.emplace_back(m_vertices[m_faces[idx]][0], m_vertices[m_faces[idx]][1], m_vertices[m_faces[idx]][2]);
//            idx++;
//        }
//        std::cout << "loop: " << loop.size() << "\n";
//        CompositePolygon poly(loop);
//        std::cout << "creation\n";
//        std::vector<Triangle> triangles = poly.triangulation();
//        std::cout << "triangles: " << triangles.size() << "\n";
//        offset = idx - m_faceSizes[i];
//        for (const Triangle& tri : triangles) {
//            std::cout << tri.m_I0 << " " << tri.m_I1 << " " << tri.m_I2 << "\n";
//            *idxPtr++ = m_faces[offset + tri.m_I0];
//            *idxPtr++ = m_faces[offset + tri.m_I1];
//            *idxPtr++ = m_faces[offset + tri.m_I2];
//        }
//        std::cout << "============================\n";
//
//        loop.clear();
//    }

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