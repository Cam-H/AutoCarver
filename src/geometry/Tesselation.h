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

class Tesselation
{
public:

    // Construction

    Tesselation();

    void append(const std::vector<QVector3D>& vertices, const std::vector<Triangle>& triangles);

    // Tesselation information getters

    size_t getVertexCount() const;
    size_t getEdgeCount() const;
    size_t getTriangleCount() const;

    const std::vector<QVector3D>& getVertices() const;
    const std::unordered_map<uint64_t, std::pair<uint32_t, uint32_t>>& getEdges();
    const std::vector<Triangle>& getTriangles() const;
    const std::vector<QVector3D>& getNormals() const;

private:

    static uint64_t getIndex(uint32_t a, uint32_t b);
    std::pair<uint32_t, uint32_t>& getEdge(uint32_t a, uint32_t b);
    static std::pair<uint32_t, uint32_t> getVertices(uint64_t edge);
private:

    // Tesselation information
    std::vector<QVector3D> m_vertices;
    std::unordered_map<uint64_t, std::pair<uint32_t, uint32_t>> m_edges;
    std::vector<Triangle> m_triangles;
    std::vector<QVector3D> m_normals;
};

#endif //AUTOCARVER_TESSELATION_H
