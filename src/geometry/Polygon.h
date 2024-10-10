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

    explicit Polygon(const std::vector<QVector2D> &border);
    explicit Polygon(const std::vector<std::vector<QVector2D>> &loops);

    explicit Polygon(const std::vector<QVector3D> &border, const QVector3D &normal);
    explicit Polygon(const std::vector<std::vector<QVector3D>> &loops, const QVector3D &normal);

    struct IndexedBorder {
        std::vector<uint32_t> vertices;
    };

    void invalidate();

    void removeVertex(uint32_t index, bool maintainIntegrity = false);
    void insertVertex(uint32_t reference, QVector2D vertex);
    void positionVertex(uint32_t index, QVector2D position, bool maintainIntegrity = false);

    void translate(QVector2D delta);

    void addLoop(const std::vector<QVector2D> &loop);

    uint32_t loopCount();
    uint32_t loopLength(uint32_t index);
    uint32_t identifyParentLoop(uint32_t index);
    std::vector<uint32_t> getLoop(uint32_t index);

    uint32_t vertexCount();
    QVector2D getVertex(uint32_t index);

    const std::vector<std::pair<int, int>>& diagonals();
    const std::vector<IndexedBorder>& partitions();
    const std::vector<Triangle>& triangulation();

    const std::vector<std::pair<int, int>>& diagonals() const;
    const std::vector<IndexedBorder>& partitions() const;
    const std::vector<Triangle>& triangulation() const;

    bool encloses(const QVector2D &p);

    float area() const;
    static float area(const std::vector<QVector2D> &border);

    static std::vector<QVector2D> reduce(const std::vector<QVector3D> &loop, const QVector3D &normal);

private:

    enum class VertexType {
        NORMAL = 0, START, END, SPLIT, MERGE
    };

    template <typename VertexType>
    auto asInteger(VertexType const value) -> typename std::underlying_type<VertexType>::type{
        return static_cast<typename std::underlying_type<VertexType>::type>(value);
    }

    struct Vertex {
        QVector2D p;

        int index = -1;

        int prev = -1;
        int next = -1;

        VertexType type = VertexType::NORMAL;
        bool internal = false;
    };

    // TODO separate into its own class when needed
    struct System {
        QVector3D origin;
        QVector3D xAxis;
        QVector3D yAxis;
    };

    void initializeLoop(const std::vector<QVector2D> &border, bool internal = false);
    static std::vector<QVector2D> reduce(const std::vector<QVector3D> &loop, const QVector3D &normal, const System &sys);

    void diagonalize();

    void identifyVertexTypes();
    static void identifyVertexType(const QVector2D &prev, Vertex &vertex, const QVector2D &next);

    void handleRegularVertex(const Vertex& current);
    void handleStartVertex(const Vertex& current);
    void handleEndVertex(const Vertex& current);
    void handleSplitVertex(const Vertex& current);
    void handleMergeVertex(const Vertex& current);

    void tryMergeDiagonalInsertion(const Vertex& current, int index);
    void insertInterval(const Vertex& current, const Vertex& end);
    void removeInterval(int index);
    int getLeftNeighborIndex(const Vertex& current);

    void partition(const std::vector<std::pair<int, int>>& boundaries);
    void triangulate(const IndexedBorder &partition);
    Triangle ccwTriangle(uint32_t I0, uint32_t I1, uint32_t I2);

    bool intersects(const Vertex &a, const Vertex &b);

    static bool compare(const Vertex& v1,const Vertex& v2);

    static bool angle(const QVector2D &pivot, const QVector2D &a, const QVector2D &b);
    static float internalAngle(const QVector2D &pivot, const QVector2D &a, const QVector2D &b);

    static float dot(const QVector2D &pivot, const QVector2D &a, const QVector2D &b);
    static float dot(const QVector2D &a, const QVector2D &b);

    static float interpolate(const QVector2D &start, const QVector2D &end, float dy);

    static bool segmentIntersection(const QVector2D &a, const QVector2D &b, const QVector2D &c, const QVector2D &d);

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
