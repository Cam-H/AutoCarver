//
// Created by Cam on 2024-10-20.
//

#ifndef AUTOCARVER_MESH_H
#define AUTOCARVER_MESH_H

// Mesh manipulation

#include <cstdint>

#include "ConvexHull.h"

#include "VertexArray.h"
#include "FaceArray.h"

class Mesh {
public:

    explicit Mesh(float vertices[], uint32_t vertexCount, uint32_t indices[], uint32_t indexCount);
    explicit Mesh(const ConvexHull& hull);

    explicit Mesh(const float *vertices, uint32_t vertexCount, const uint32_t *faceIndices, const uint32_t *faceSizes, uint32_t faceCount);
    explicit Mesh(const VertexArray& vertices, const FaceArray& faces);

    ~Mesh();

    void scale(float scalar);
    void scale(float x, float y, float z);
    void translate(float x, float y, float z);
    void rotate(float x, float y, float z, float theta);

    void xExtent(float &near, float &far);
    void yExtent(float &near, float &far);
    void zExtent(float &near, float &far);

    void setBaseColor(const glm::vec3& color);
    void setFaceColor(uint32_t faceIdx, const glm::vec3& color);

    void calculateAdjacencies();

    [[nodiscard]] uint32_t vertexCount() const;
    [[nodiscard]] const VertexArray& vertices() const;

    [[nodiscard]] const VertexArray& faceNormals() const;
    [[nodiscard]] const VertexArray& vertexNormals() const;

    [[nodiscard]] const VertexArray& colors() const;
    [[nodiscard]] const glm::vec3& baseColor() const;

    [[nodiscard]] uint32_t triangleCount() const;
    uint32_t* indices();

    [[nodiscard]] uint32_t faceCount() const;
    [[nodiscard]] const FaceArray& faces() const;

    [[nodiscard]] std::vector<uint32_t> outline(const glm::vec3& axis);

    [[nodiscard]] float volume() const;
    [[nodiscard]] float surfaceArea() const;

    std::vector<uint32_t> sharedFaces(const std::shared_ptr<Mesh>& reference) const;

    void directRepresentation(float *vertices, float *normals, float *colors);

    void print() const;

private:

    void calculateFaceNormals();
    void calculateVertexNormals();

    std::vector<uint16_t> identifyHorizonFaces(const glm::vec3& axis, const std::vector<std::vector<uint32_t>>& adjacencies) const;

    [[nodiscard]] float faceArea(uint32_t faceIdx) const;
    [[nodiscard]] static float faceArea(const std::vector<glm::vec3>& vertices) ;

private:

    VertexArray m_vertices;

    FaceArray m_faces;

    VertexArray m_faceNormals;
    VertexArray m_vertexNormals;

    uint32_t *m_indices;
    uint32_t m_indexCount;

    VertexArray m_colors;
    glm::vec3 m_baseColor;

    bool m_adjacencyOK;
    std::vector<std::vector<uint32_t>> m_adjacencies;

//    float *m_colors;

    const static uint8_t STRIDE = 3;

};

#endif //AUTOCARVER_MESH_H
