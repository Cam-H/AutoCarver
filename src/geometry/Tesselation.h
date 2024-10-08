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

class Surface;

struct Edge {
    uint32_t I0;
    uint32_t I1;

    Edge();

    static uint64_t index(uint32_t a, uint32_t b);
};

class Tesselation
{
public:

    // Construction

    Tesselation();
    Tesselation(const Tesselation& rhs) = default;
    Tesselation& operator=(const Tesselation& rhs) = default;

    void append(const std::vector<QVector3D> &vertices, const std::vector<Triangle> &triangles);
    void clear();

    int64_t pickVertex(const QVector3D& origin, const QVector3D& direction, bool occlusion = false);


    std::vector<uint32_t> horizon(const QVector3D &dir);
    void horizon(const QVector3D &dir, std::vector<uint32_t> &set);
    void horizon(const QVector3D &dir, std::vector<uint32_t> &set, uint32_t current, uint32_t mark);

    void slice(const QVector3D &origin, const QVector3D &normal, Tesselation &body);
    void slice(const QVector3D &origin, const QVector3D &normal, Tesselation &bodyA, Tesselation &bodyB);

    // Tesselation information getters

    size_t getVertexCount() const;
    size_t getEdgeCount() const;
    size_t getTriangleCount() const;

    const std::vector<QVector3D>& getVertices() const;
    const std::vector<QVector3D>& getVertexNormals() const;
    const std::unordered_map<uint64_t, Edge>& getEdges();
    const std::vector<Triangle>& getTriangles() const;
    const std::vector<QVector3D>& getNormals() const;

    void planarLoops(std::vector<std::vector<uint32_t>> &loops, std::vector<QVector3D> &normals) const;
    std::vector<Surface> surface() const;

    Surface surface(std::vector<uint32_t> &loop, const QVector3D &normal) const;
    static void enforceVertexOrder(std::vector<std::vector<QVector3D>> &loops, const QVector3D &normal) ;

private:
    void calculateVertexNormals();

    void sliceBounds(const QVector3D &origin, const QVector3D &normal, std::vector<bool> &vertices, std::vector<uint16_t> &triangles);
    void mend();
    std::vector<uint64_t> getBoundary(uint64_t start);

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
