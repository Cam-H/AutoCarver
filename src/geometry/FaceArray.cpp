//
// Created by Cam on 2024-11-12.
//

#include "FaceArray.h"

#include <utility>

#include <iostream>

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

FaceArray::~FaceArray()
{
    delete[] m_faces;
    delete[] m_faceSizes;
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

// Requires that the faces are convex
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

uint32_t FaceArray::size() const
{
    return (m_indexCount + m_faceCount + 2) * sizeof(uint32_t);
}