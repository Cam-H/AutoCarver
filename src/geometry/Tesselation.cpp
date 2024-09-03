//
// Created by cameronh on 24/04/24.
//

#include "Tesselation.h"

#include <stack>

Edge::Edge() : I0(std::numeric_limits<uint32_t>::max()), I1(std::numeric_limits<uint32_t>::max()) {}

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
    uint32_t vc = m_vertices.size();

    m_vertices.insert(m_vertices.end(), vertices.begin(), vertices.end());
    m_triangles.insert(m_triangles.end(), triangles.begin(), triangles.end());

    int cc = 0;
    for(uint32_t i = 0; i < triangles.size(); i++){
        uint32_t index = m_triangles.size() - triangles.size() + i;

        // Calculate normals for every triangle
        m_faceNormals.push_back(QVector3D::crossProduct(
                        m_vertices[triangles[i].m_I1 + vc] - m_vertices[triangles[i].m_I0 + vc],
                        m_vertices[triangles[i].m_I2 + vc] - m_vertices[triangles[i].m_I0 + vc])
        .normalized());

        // Identify topology
        for(uint32_t j = 0; j < 3; j++){
            uint64_t edgeIdx = Edge::index(triangles[i][j], triangles[i][(j + 1) % 3]);
            Edge& edge = m_edges[edgeIdx];
            if(edge.I0 == std::numeric_limits<uint32_t>::max()){
                edge.I0 = index;
            }else if(edge.I1 == std::numeric_limits<uint32_t>::max()){//TODO
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
}

void Tesselation::slice(const QVector3D& origin, const QVector3D& normal, Tesselation& body)
{
    // Identify all vertices outside of bounds
    std::vector<bool> vertexRelease(m_vertices.size(), false);
    std::vector<uint16_t> triangleRelease(m_triangles.size(), 0);
    sliceBounds(origin, normal, vertexRelease, triangleRelease);

    // All vertices outside cutting plane -> remove all triangles -> do nothing
    if (triangleRelease.empty()) return;

    // All vertices within cutting plane -> tesselation is not affected
    if (vertexRelease.empty()) {
        body = *this;
        return;
    }

    // Copy and map remaining vertices
    std::vector<uint32_t> indexMap(m_vertices.size(), 0);
    for (uint32_t i = 0; i < m_vertices.size(); i++) {
        if (!vertexRelease[i]) {
            indexMap[i] = body.m_vertices.size();
            body.m_vertices.push_back(m_vertices[i]);
        }
    }

    // Insert new vertices at the intersection of edges and the cutting plane
    std::unordered_map<uint64_t, uint32_t> edgeMap;
    for (const auto& edge : m_edges) {
        if (triangleRelease[edge.second.I0] > 0 && triangleRelease[edge.second.I1] > 0) {// Pre-condition for edge intersection
            std::pair<uint32_t, uint32_t> idx = getVertices(edge.first);
            if (vertexRelease[idx.first] != vertexRelease[idx.second]) {
                QVector3D vec = m_vertices[idx.second] - m_vertices[idx.first];
                float t = (QVector3D::dotProduct(normal, origin) - QVector3D::dotProduct(normal, m_vertices[idx.first])) / QVector3D::dotProduct(normal, vec);

                edgeMap[edge.first] = body.m_vertices.size();
                body.m_vertices.push_back(m_vertices[idx.first] + t * vec);
            }
        }
    }

    // Sort triangles according to # of vertices outside of bounds (0 -> 2 -> 1 -> 3)
    std::vector<Triangle> triangles;
    for (uint32_t i = 0; i < m_triangles.size(); i++) {
        switch(triangleRelease[i]) {
            case 0: // Triangle not affected by cutting plane -> Need only remap vertex indices
                triangles.emplace_back(
                        indexMap[m_triangles[i].m_I0],
                        indexMap[m_triangles[i].m_I1],
                        indexMap[m_triangles[i].m_I2]
                        );
                break;
            case 1: // One vertex affected -> Need to generate two triangles to span resultant trapezoid
            {
                uint32_t j = 0;
                if (vertexRelease[m_triangles[i][j + 1]]) j += 2;
                if (vertexRelease[m_triangles[i][j]]) j++;

                triangles.emplace_back(
                        indexMap[m_triangles[i][j]],
                        indexMap[m_triangles[i][j + 1]],
                        edgeMap[Edge::index(m_triangles[i][(j + 1) % 3], m_triangles[i][(j + 2) % 3])]
                );

                triangles.emplace_back(
                        indexMap[m_triangles[i][j]],
                        edgeMap[Edge::index(m_triangles[i][(j + 1) % 3], m_triangles[i][(j + 2) % 3])],
                        edgeMap[Edge::index(m_triangles[i][(j + 2) % 3], m_triangles[i][j])]
                );
            }
                break;
            case 2: // Two vertices affected -> Triangle between remaining vertex and intersections of edge / plane
            {
                uint32_t j = 0;
                for (; j < 3; j++) {
                    if (!vertexRelease[m_triangles[i][j]]) break;
                }

                triangles.emplace_back(
                        indexMap[m_triangles[i][j]],
                        edgeMap[Edge::index(m_triangles[i][j], m_triangles[i][(j + 1) % 3])],
                        edgeMap[Edge::index(m_triangles[i][(j + 2) % 3], m_triangles[i][j])]
                        );
            }
                break;
        }
    }
    body.append(std::vector<QVector3D>(), triangles);

    // Heal boundary resulting from the cut
    body.healBoundaries();

    std::cout << "slice complete!\n";
//    std::cout << "-> " << count << "\n";
}

void Tesselation::healBoundaries()
{

    for (auto const& [key, edge] : m_edges) {
        if(edge.I0 == std::numeric_limits<uint32_t>::max() || edge.I1 == std::numeric_limits<uint32_t>::max()) {
            std::cout << key << " missing!\n";
            std::vector<uint64_t> boundary = getBoundary(key);

            std::pair<uint32_t, uint32_t> idx = getVertices(boundary[0]);
            std::vector<uint32_t> loop = { idx.first, idx.second };

            idx = getVertices(boundary[1]);
            if (loop[1] != idx.first && loop[1] != idx.second) {
                uint32_t swap = loop[1];
                loop[1] = loop[0];
                loop[0] = swap;
            }

            for (int i = 1; i < boundary.size() - 1; i++) {
                idx = getVertices(boundary[i]);
                if (loop[i] == idx.first) loop.push_back(idx.second);
                else loop.push_back(idx.first);
            }

//            for(int i = 0; i < loop.size(); i++) {
//                std::cout << loop[i] << " ";
//            }
//            std::cout << "\n";
//
//            std::cout << boundary.size() << "<< \n";
        }
    }
}

std::vector<uint64_t> Tesselation::getBoundary(uint64_t start)
{
    std::vector<uint64_t> boundary = { start };
    std::pair<uint32_t, uint32_t> idx = getVertices(start);
    uint32_t triIndex = m_edges[start].I0 == std::numeric_limits<uint32_t>::max() ? m_edges[start].I1 : m_edges[start].I0;
    uint32_t end = idx.first, pivot = idx.second, kick = m_triangles[triIndex].last(idx.first, idx.second);
    uint64_t current = Edge::index(pivot, kick);

    int limit = 0;
    do {

//        std::cout << "T: " << triIndex << " " << limit << "\n";

        // Select next triangle connected to pivot
        const Edge &edge = m_edges[current];
        triIndex = edge.I0 == triIndex ? edge.I1 : edge.I0;

        // Move pivot if the selected triangle is on the boundary
        kick = m_triangles[triIndex].last(pivot, kick);
        current = Edge::index(pivot, kick);
        if ((m_edges[current].I0 == std::numeric_limits<uint32_t>::max())
            + (m_edges[current].I1 == std::numeric_limits<uint32_t>::max()) == 1) {
            boundary.push_back(current);

            uint32_t temp = pivot;
            pivot = kick;
            kick = m_triangles[triIndex].last(pivot, temp);

            current = Edge::index(pivot, kick);
        }
    } while (pivot != end && limit++ < 10000);

    return boundary;
}

void Tesselation::slice(const QVector3D &origin, const QVector3D &normal, Tesselation &bodyA, Tesselation &bodyB)
{
    //TODO
}

void Tesselation::sliceBounds(const QVector3D &origin, const QVector3D &normal, std::vector<bool> &vertices, std::vector<uint16_t> &triangles)
{
    uint32_t releaseCount = 0;
    for (uint32_t i = 0; i < m_vertices.size(); i++) {
        vertices[i] = QVector3D::dotProduct(normal, m_vertices[i] - origin) > m_epsilon;
        releaseCount += vertices[i];
    }

    if (releaseCount == 0) {
        vertices.clear();
        return;
    }

    if (releaseCount == m_vertices.size()) {
        triangles.clear();
        return;
    }

    for (int i = 0; i < m_triangles.size(); i++) {
        triangles[i] = vertices[m_triangles[i].m_I0] + vertices[m_triangles[i].m_I1] + vertices[m_triangles[i].m_I2];
    }
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