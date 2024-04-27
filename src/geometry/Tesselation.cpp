//
// Created by cameronh on 24/04/24.
//

#include "Tesselation.h"

#include <stack>

uint64_t Edge::index()
{
    return index(I0, I1);
}

uint64_t Edge::index(uint32_t a, uint32_t b)
{
    return (a < b) ? ((uint64_t)a << 32) + b : ((uint64_t)b << 32) + a;
}

Tesselation::Tesselation()
    : m_epsilon(std::numeric_limits<float>::epsilon())
{
}

void Tesselation::append(const std::vector<QVector3D>& vertices, const std::vector<Triangle>& triangles){
    m_vertices.insert(m_vertices.end(), vertices.begin(), vertices.end());
    m_triangles.insert(m_triangles.end(), triangles.begin(), triangles.end());

    for(uint32_t i = 0; i < m_vertices.size(); i++){
        m_vertexNormals.emplace_back(0, 0, 1);
    }

    int cc = 0;
    for(uint32_t i = 0; i < triangles.size(); i++){
        uint32_t index = m_triangles.size() - triangles.size() + i;

        // Calculate normals for every triangle
        m_faceNormals.push_back(QVector3D::crossProduct(
                        vertices[triangles[i].m_I1] - vertices[triangles[i].m_I0],
                        vertices[triangles[i].m_I2] - vertices[triangles[i].m_I0])
        .normalized());

        // Identify topology
        for(uint32_t j = 0; j < 3; j++){
            uint64_t edgeIdx = Edge::index(triangles[i][j], triangles[i][(j + 1) % 3]);
            Edge& edge = m_edges[edgeIdx];
            if(edge.I0 == 0){
                edge.I0 = index;
            }else if(edge.I1 == 0){//TODO
                edge.I1 = index;
            }else{
//                std::cout << "TOPOLOGY ERROR!\n";
                cc++;
            }

//            std::cout << edge.first << " " << edge.second << " " << m_Edges.size() << " T\n";
        }
//        std::cout << "\n";
    }

    calculateVertexNormals();

    std::cout << "TP ERR: " << cc << " / " << m_edges.size() << " | " << m_vertices.size() << " " << m_triangles.size() << "\n";
}
std::vector<uint32_t> Tesselation::horizon(const QVector3D& dir)
{
    std::vector<uint32_t> set(m_triangles.size(), 0);
    horizon(dir, set);

    return set;
}
void Tesselation::horizon(const QVector3D &dir, std::vector<uint32_t> &set)
{
    uint32_t count = 2;
    for (uint32_t i = 0; i < m_triangles.size(); i++) {
        if (set[i] == 0) {
            set[i] = QVector3D::dotProduct(dir, m_faceNormals[i]) >= -m_epsilon;
            if (!set[i]) {
                horizon(dir, set, i, count);
                count += 2;
            }
        }
    }
}

void Tesselation::horizon(const QVector3D &dir, std::vector<uint32_t> &set, uint32_t current, uint32_t mark)
{
    std::stack<uint32_t> steps;
    steps.push(current);

    while (!steps.empty()) {
        current = steps.top();
        set[current] = mark;
        steps.pop();

        // Identify edges of the current triangle
        std::vector<Edge> edges = {
                m_edges[Edge::index(m_triangles[current].m_I0, m_triangles[current].m_I1)],
                m_edges[Edge::index(m_triangles[current].m_I1, m_triangles[current].m_I2)],
                m_edges[Edge::index(m_triangles[current].m_I2, m_triangles[current].m_I0)]
        };

        std::vector<uint32_t> neighbors = { edges[0].I0, edges[0].I1, edges[1].I0, edges[1].I1, edges[2].I0, edges[2].I1 };
        for (uint32_t next : neighbors) {
            if (set[next] == 0) { // Only try hopping to unvisited triangles
                if (QVector3D::dotProduct(dir, m_faceNormals[next]) >= -m_epsilon) {
                    set[next] = 1;
                } else {
                    steps.push(next);
                }
            }

            // Special indication for boundary triangles
            if (set[next] == 1) set[current] = mark + 1;
        }
    }

//    set[current] = mark;
//
//    std::vector<Edge> edges = {
//            m_edges[Edge::index(m_triangles[current].m_I0, m_triangles[current].m_I1)],
//            m_edges[Edge::index(m_triangles[current].m_I1, m_triangles[current].m_I2)],
//            m_edges[Edge::index(m_triangles[current].m_I2, m_triangles[current].m_I0)]
//    };
//
//    std::vector<uint32_t> neighbors = { edges[0].I0, edges[0].I1, edges[1].I0, edges[1].I1, edges[2].I0, edges[2].I1 };
//    for (uint32_t next : neighbors) {
//        if (set[next] == 0) { // Only try hopping to unvisited triangles
//            if (QVector3D::dotProduct(dir, m_faceNormals[next]) >= -m_epsilon) {
//                set[next] = 1;
//                continue;
//            }
//
//            horizon(dir, set, next, mark);
//        }
//    }
}

void Tesselation::calculateVertexNormals()
{
    m_vertexNormals = std::vector<QVector3D>(m_vertices.size(), {0, 0, 0});
    for (uint32_t i = 0; i < m_triangles.size(); i++) {
        m_vertexNormals[m_triangles[i].m_I0] += m_faceNormals[i];
        m_vertexNormals[m_triangles[i].m_I1] += m_faceNormals[i];
        m_vertexNormals[m_triangles[i].m_I2] += m_faceNormals[i];
    }

    for (QVector3D &normal : m_vertexNormals) {
        normal = normal.normalized();
    }
}

size_t Tesselation::getVertexCount() const {
    return m_vertices.size();
}

size_t Tesselation::getEdgeCount() const {
    return m_edges.size();
}

size_t Tesselation::getTriangleCount() const {
    return m_triangles.size();
}

const std::vector<QVector3D>& Tesselation::getVertices() const {
    return m_vertices;
}

const std::vector<QVector3D>& Tesselation::getVertexNormals() const {
    return m_vertexNormals;
}

const std::unordered_map<uint64_t, Edge>& Tesselation::getEdges() {
    return m_edges;
}

const std::vector<Triangle>& Tesselation::getTriangles() const {
    return m_triangles;
}

const std::vector<QVector3D>& Tesselation::getNormals() const {
    return m_faceNormals;
}

std::pair<uint32_t, uint32_t> Tesselation::getVertices(uint64_t edge){
    return {edge >> 32, edge & 0x0000FFFF};
}