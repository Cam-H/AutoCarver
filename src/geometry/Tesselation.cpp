//
// Created by cameronh on 24/04/24.
//

#include "Tesselation.h"

Tesselation::Tesselation()
{
}

void Tesselation::append(const std::vector<QVector3D>& vertices, const std::vector<Triangle>& triangles){
    m_vertices.insert(m_vertices.end(), vertices.begin(), vertices.end());
    m_triangles.insert(m_triangles.end(), triangles.begin(), triangles.end());

    for(uint32_t i = 0; i < m_vertices.size(); i++){
        m_normals.emplace_back(0, 0, 1);
    }

//    int cc = 0;
//    for(uint32_t i = 0; i < triangles.size(); i++){
//        uint32_t index = m_triangles.size() - triangles.size() + i;
//
//        // Calculate normals for every triangle
//        m_normals.push_back(QVector3D::crossProduct(
//                        vertices[triangles[i].m_I1] - vertices[triangles[i].m_I0],
//                        vertices[triangles[i].m_I2] - vertices[triangles[i].m_I0])
//        .normalized());
//
//        // Identify topology
//        for(uint32_t j = 0; j < 3; j++){
//            std::pair<uint32_t, uint32_t>& edge = getEdge(triangles[i][j], triangles[i][(j + 1) % 3]);
//            if(edge.first == 0){
//                edge.first = index;
//            }else if(edge.second == 0){//TODO
//                edge.second = index;
//            }else{
////                std::cout << "TOPOLOGY ERROR!\n";
//                cc++;
//            }
//
////            std::cout << edge.first << " " << edge.second << " " << m_Edges.size() << " T\n";
//        }
////        std::cout << "\n";
//    }

//    std::cout << "TP ERR: " << cc << " / " << m_edges.size() << " | " << m_vertices.size() << " " << m_triangles.size() << "\n";
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

const std::unordered_map<uint64_t, std::pair<uint32_t, uint32_t>>& Tesselation::getEdges() {
    return m_edges;
}

const std::vector<Triangle>& Tesselation::getTriangles() const {
    return m_triangles;
}

const std::vector<QVector3D>& Tesselation::getNormals() const {
    return m_normals;
}

uint64_t Tesselation::getIndex(uint32_t a, uint32_t b){
    return (a < b) ? ((uint64_t)a << 32) + b : ((uint64_t)b << 32) + a;
}

std::pair<uint32_t, uint32_t>& Tesselation::getEdge(uint32_t a, uint32_t b){
    return m_edges[getIndex(a, b)];
}

std::pair<uint32_t, uint32_t> Tesselation::getVertices(uint64_t edge){
    return {edge >> 32, edge & 0x0000FFFF};
}