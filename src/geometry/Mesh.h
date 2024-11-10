//
// Created by Cam on 2024-10-20.
//

#ifndef AUTOCARVER_MESH_H
#define AUTOCARVER_MESH_H

// Mesh manipulation

#include <cstdint>

#include "ConvexHull.h"
#include <QColor>

class Mesh {
public:

    explicit Mesh(float vertices[], uint32_t vertexCount, uint32_t indices[], uint32_t indexCount);
    explicit Mesh(const ConvexHull& hull);
    ~Mesh();

    void setBaseColor(QColor color);
    void setFaceColor(uint32_t faceIdx, QColor color);

    uint32_t vertexCount() const;
    [[nodiscard]] float* vertices() const;

    [[nodiscard]] float* normals() const;
    [[nodiscard]] float* colors() const;

    uint32_t triangleCount() const;
    uint32_t* indices();

    uint32_t faceCount() const;

    void directRepresentation(float *vertices, float *normals);

private:

    void calculateFaceNormals();
    void calculateVertexNormals();

private:

    float *m_vertices;
    uint32_t m_vertexCount;

    uint32_t *m_indices;
    uint32_t m_indexCount;

    float *m_triNormals;
    float *m_normals;

    uint32_t *m_faces; // Triangle indices for each face
    uint32_t *m_faceSizes; // Size of each individual face in triangles
    uint32_t m_faceCount; // Total number of faces

    float *m_colors;

    const static uint8_t STRIDE = 3;

};

#endif //AUTOCARVER_MESH_H
