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
    Polygon(const std::vector<std::vector<QVector2D>> &loops);

    struct IndexedBorder {
        std::vector<uint32_t> vertices;
    };

    void invalidate();

    void removeVertex(uint32_t index, bool maintainIntegrity = false);
    void insertVertex(uint32_t reference, QVector2D vertex);
    void positionVertex(uint32_t index, QVector2D position, bool maintainIntegrity = false);

    uint32_t vertexCount();
    uint32_t loopCount();
    uint32_t loopLength(uint32_t index);
    uint32_t identifyParentLoop(uint32_t index);

    QVector2D getVertex(uint32_t index);
    std::vector<uint32_t> getLoop(uint32_t index);

    std::vector<IndexedBorder> partition();

    std::vector<Triangle> tesselate();

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

        int prev = -1;
        int next = -1;

        VertexType type = VertexType::NORMAL;
        bool internal = false;
    };

    void initializeLoop(const std::vector<QVector2D> &border, bool internal = false);

    void identifyVertexTypes();
    void identifyVertexType(const QVector2D &prev, Vertex &vertex, const QVector2D &next);

    void handleRegularVertex(const Vertex& current);
    void handleStartVertex(const Vertex& current);
    void handleEndVertex(const Vertex& current);
    void handleSplitVertex(const Vertex& current);
    void handleMergeVertex(const Vertex& current);

    void tryMergeDiagonalInsertion(const Vertex& current, int index);
    void insertInterval(const Vertex& current, const Vertex& end);
    void removeInterval(int index);
    int getLeftNeighborIndex(const Vertex& current);
    static float interpolate(const QVector2D &start, const QVector2D &end, float dy);

    void partition(std::vector<std::pair<int, int>> boundaries);

    static bool compare(const Vertex& v1,const Vertex& v2);

    static bool right(const Vertex& current, const Vertex& prev);

    static bool angle(const QVector2D &pivot, const QVector2D &a, const QVector2D &b);
    static float internalAngle(const QVector2D &pivot, const QVector2D &a, const QVector2D &b);

    static float dot(const QVector2D &pivot, const QVector2D &a, const QVector2D &b);
    static float dot(const QVector2D &a, const QVector2D &b);

    static float cross(const QVector2D &pivot, const QVector2D &a, const QVector2D &b);
    static float cross(const QVector2D &v1, const QVector2D &v2);

    bool intersects(const Vertex &a, const Vertex &b);

    static bool segmentIntersection(const QVector2D &a, const QVector2D &b, const QVector2D &c, const QVector2D &d);
    static float triArea(const QVector2D &a, const QVector2D &b, const QVector2D &c);

    void diagonalize();
    void tesselate(const IndexedBorder &partition);


private:

    std::vector<Vertex> m_hull;

    // Index to the 'first' vertex on each loop
    std::vector<uint32_t> m_loops;

    std::vector<Vertex> m_eventQueue;
    std::vector<std::pair<int, int>> intervals;
    std::vector<int> helper;

    std::vector<std::pair<int, int>> m_diagonals;
    bool m_diagonalized;

    std::vector<IndexedBorder> m_partitions;
    std::vector<Triangle> m_triangles;

};


#endif //AUTOCARVER_POLYGON_H
