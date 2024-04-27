//
// Created by cameronh on 24/04/24.
//

#ifndef AUTOCARVER_TESSELATION_H
#define AUTOCARVER_TESSELATION_H

#include <QVector3D>
#include <QVector2D>

#include <cstdint>
#include <vector>

#include <string>
#include <iostream>

#include "Triangle.h"

struct Edge {
    uint32_t I0;
    uint32_t I1;

    uint64_t index();
    static uint64_t index(uint32_t a, uint32_t b);
};

class Tesselation
{
public:

    // Construction

    Tesselation();

    void append(const std::vector<QVector3D> &vertices, const std::vector<Triangle> &triangles);
    std::vector<uint32_t> horizon(const QVector3D &dir);
    void horizon(const QVector3D &dir, std::vector<uint32_t> &set);
    void horizon(const QVector3D &dir, std::vector<uint32_t> &set, uint32_t current, uint32_t mark);

    // Tesselation information getters

    size_t getVertexCount() const;
    size_t getEdgeCount() const;
    size_t getTriangleCount() const;

    const std::vector<QVector3D>& getVertices() const;
    const std::vector<QVector3D>& getVertexNormals() const;
    const std::unordered_map<uint64_t, Edge>& getEdges();
    const std::vector<Triangle>& getTriangles() const;
    const std::vector<QVector3D>& getNormals() const;

private:
    void calculateVertexNormals();

    static std::pair<uint32_t, uint32_t> getVertices(uint64_t edge);
private:

    // Tesselation information
    std::vector<QVector3D> m_vertices;
    std::vector<QVector3D> m_vertexNormals;

    std::unordered_map<uint64_t, Edge> m_edges;

    std::vector<Triangle> m_triangles;
    std::vector<QVector3D> m_faceNormals;

    float m_epsilon;
};

#endif //AUTOCARVER_TESSELATION_H
