//
// Created by Cam on 2024-11-12.
//

#ifndef AUTOCARVER_FACEARRAY_H
#define AUTOCARVER_FACEARRAY_H

#include <cstdint>
#include <vector>
#include <fstream>

#include "VertexArray.h"
#include "geometry/shape/Triangle.h"

class FaceArray {
public:
    FaceArray(uint32_t faceCount, uint32_t indexCount);

    FaceArray(const uint32_t* indices, uint32_t triangleCount);
    explicit FaceArray(const std::vector<Triangle>& faces);

    FaceArray(const uint32_t* faces, const uint32_t* faceSizes, uint32_t faceCount);

    explicit FaceArray(const std::vector<std::vector<uint32_t>>& indices);

    FaceArray(const FaceArray &);
    FaceArray& operator=(const FaceArray &);

    FaceArray(FaceArray &&) noexcept;
    FaceArray& operator=(FaceArray &&) noexcept;

    ~FaceArray();

    bool serialize(std::ofstream& file);
    static FaceArray deserialize(std::ifstream& file);

    static glm::vec3 calculateNormal(const std::vector<glm::vec3>& boundary);
    void calculateNormals(const std::vector<glm::vec3>& vertices);
    void triangulate(const std::vector<glm::vec3>& vertices);

    void setColor(const glm::vec3& color);
    void setColor(uint32_t idx, const glm::vec3& color);

    uint32_t* operator[](uint32_t idx);
    const uint32_t* operator[](uint32_t idx) const;

    [[nodiscard]] uint32_t faceCount() const;
    [[nodiscard]] uint32_t indexCount() const;

    [[nodiscard]] const uint32_t* faces() const;

    [[nodiscard]] uint32_t* faceSizes();
    [[nodiscard]] const uint32_t* faceSizes() const;

    [[nodiscard]] const std::vector<glm::vec3>& normals() const;
    [[nodiscard]] glm::vec3 normal(uint32_t idx) const;

    [[nodiscard]] const std::vector<glm::vec3>& colors() const;
    [[nodiscard]] glm::vec3 color(uint32_t idx) const;

    [[nodiscard]] uint32_t triangleCount() const;
    [[nodiscard]] const std::vector<Triangle>& triangles() const;

    [[nodiscard]] std::tuple<uint32_t, uint32_t> triangleLookup(uint32_t faceIdx) const;

    [[nodiscard]] std::vector<glm::vec3> faceBorder(uint32_t idx, const std::vector<glm::vec3>& vertices) const;

//    FaceArray triangulated();

    // Selects the face with the normal nearest to the specified axis
    [[nodiscard]] uint32_t matchFace(const glm::vec3& axis);

    [[nodiscard]] std::vector<std::vector<uint32_t>> edgeList() const;
    [[nodiscard]] std::vector<std::vector<uint32_t>> adjacencies() const;

    // Calculates the number of instances of every vertex in a face
    [[nodiscard]] std::vector<uint32_t> instances(uint32_t maxIndex) const;

    [[nodiscard]] uint32_t size() const; // Get the size of the array in bytes
    [[nodiscard]] bool empty() const;

    void print() const;

private:

    static bool inRange(const uint32_t *idx, uint32_t count, uint32_t limit);

    uint32_t* idxPtr(uint32_t idx);
    uint32_t* idxPtr(uint32_t idx) const;

    inline static uint64_t linkKey(uint32_t I0, uint32_t I1);

private:
    uint32_t *m_faces; // Loops of vertex indices that compose each face
    uint32_t *m_faceSizes; // Size of each individual face in vertices

    std::vector<glm::vec3> m_normals;
    std::vector<glm::vec3> m_colors;

    std::vector<Triangle> m_triangles;
    std::vector<uint32_t> m_triFaceLookup;

    uint32_t m_faceCount; // Total number of faces
    uint32_t m_indexCount; // Total number of indices, the sum of all face sizes
};


#endif //AUTOCARVER_FACEARRAY_H
