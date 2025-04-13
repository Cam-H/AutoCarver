//
// Created by Cam on 2024-11-12.
//

#ifndef AUTOCARVER_FACEARRAY_H
#define AUTOCARVER_FACEARRAY_H

#include <cstdint>
#include <vector>

#include "VertexArray.h"
#include "Triangle.h"

class FaceArray {
public:
    FaceArray(const uint32_t* indices, uint32_t triangleCount);
    FaceArray(const uint32_t* faces, const uint32_t* faceSizes, uint32_t faceCount);

    explicit FaceArray(const std::vector<Triangle>& faces);
    explicit FaceArray(const std::vector<std::vector<uint32_t>>& indices);

    FaceArray(const FaceArray &);
    FaceArray& operator=(const FaceArray &);

    FaceArray(FaceArray &&) noexcept;
    FaceArray& operator=(FaceArray &&) noexcept;

    ~FaceArray();

    uint32_t* operator[](uint32_t idx);
    uint32_t* operator[](uint32_t idx) const;

    [[nodiscard]] const uint32_t* faces() const;
    [[nodiscard]] const uint32_t* faceSizes() const;
    [[nodiscard]] uint32_t faceCount() const;

    [[nodiscard]] glm::vec3 normal(uint32_t idx, const VertexArray& vertices) const;

    void triangulation(uint32_t* indices); // Triangulates faces, presumes each face is convex
    [[nodiscard]] uint32_t triangleCount() const;

    [[nodiscard]] uint32_t indexCount() const;

    FaceArray triangulated();

    [[nodiscard]] std::vector<std::vector<uint32_t>> edgeList() const;
    [[nodiscard]] std::vector<std::vector<uint32_t>> adjacencies() const;

    [[nodiscard]] uint32_t size() const; // Get the size of the array in bytes
    [[nodiscard]] bool empty() const;

    void print() const;

private:

    uint32_t* idxPtr(uint32_t idx);
    uint32_t* idxPtr(uint32_t idx) const;

    inline static uint64_t linkKey(uint32_t I0, uint32_t I1);

private:
    uint32_t *m_faces; // Loops of vertex indices that compose each face
    uint32_t *m_faceSizes; // Size of each individual face in vertices

    uint32_t m_faceCount; // Total number of faces
    uint32_t m_indexCount; // Total number of indices, the sum of all face sizes
};


#endif //AUTOCARVER_FACEARRAY_H
