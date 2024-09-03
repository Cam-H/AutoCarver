//
// Created by cameronh on 02/06/24.
//

#ifndef AUTOCARVER_POLYGON_H
#define AUTOCARVER_POLYGON_H

#include <QVector2D>
#include <QVector3D>

#include <vector>

#include "Triangle.h"

class Polygon {
public:

    Polygon(const std::vector<QVector2D> &border);

    struct IndexedBorder {
        std::vector<uint32_t> vertices;
    };

    std::vector<IndexedBorder> partition();

    std::vector<Triangle> tesselate();

    IndexedBorder hull();

private:

    enum class VertexType {
        NORMAL = 0, START, END, SPLIT, MERGE
    };

    template <typename VertexType>
    auto asInteger(VertexType const value) -> typename std::underlying_type<VertexType>::type{
        return static_cast<typename std::underlying_type<VertexType>::type>(value);
    }

    struct Vertex{
        QVector2D p;
        int index;
        VertexType type;

        int prev = -1;
        int next = -1;

        std::vector<int> diagonals;
    };

    void identifyVertexTypes();

    void handleRegularVertex(const Vertex& current);
    void handleStartVertex(const Vertex& current);
    void handleEndVertex(const Vertex& current);
    void handleSplitVertex(const Vertex& current);
    void handleMergeVertex(const Vertex& current);

    void tryMergeDiagonalInsertion(const Vertex& current, int index);
    void insertInterval(const Vertex& current, const Vertex& end);
    void removeInterval(int index);
    int getLeftNeighborIndex(const Vertex& current);

    void partition(std::vector<std::pair<int, int>> boundaries);

    static bool compare(const Vertex& v1,const Vertex& v2);

    static bool right(const Vertex& current, const Vertex& prev);

    static bool angle(const QVector2D &a, const QVector2D &b, const QVector2D &c);
    static float cross(const QVector2D &a, const QVector2D &b, const QVector2D &c);
    static float cross(const QVector2D &v1, const QVector2D &v2);

    static std::vector<Triangle> tesselate(const Polygon &partition);

private:

    std::vector<Vertex> m_hull;
    std::vector<Vertex> m_eventQueue;
    std::vector<std::pair<int, int>> intervals;
    std::vector<int> helper;
    std::vector<std::pair<int, int>> diagonals;

    std::vector<IndexedBorder> m_partitions;

};


#endif //AUTOCARVER_POLYGON_H
