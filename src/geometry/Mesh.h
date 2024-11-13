//
// Created by Cam on 2024-10-20.
//

#ifndef AUTOCARVER_MESH_H
#define AUTOCARVER_MESH_H

// Mesh manipulation

#include <cstdint>

#include "ConvexHull.h"
#include <QColor>

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

    void setBaseColor(QColor color);
    void setFaceColor(uint32_t faceIdx, QColor color);

    [[nodiscard]] uint32_t vertexCount() const;
    [[nodiscard]] const float* vertices() const;

    [[nodiscard]] float* normals() const;
    [[nodiscard]] float* colors() const;

    [[nodiscard]] uint32_t triangleCount() const;
    uint32_t* indices();

    [[nodiscard]] uint32_t faceCount() const;
    [[nodiscard]] FaceArray faces() const;


    [[nodiscard]] float volume() const;

    void directRepresentation(float *vertices, float *normals);

private:

    void calculateFaceNormals();
    void calculateVertexNormals();

private:

    VertexArray m_vertices;

    FaceArray m_faces;


    uint32_t *m_indices;
    uint32_t m_indexCount;

    float *m_triNormals;
    float *m_normals;

    float *m_colors;

    const static uint8_t STRIDE = 3;

};

#endif //AUTOCARVER_MESH_H
