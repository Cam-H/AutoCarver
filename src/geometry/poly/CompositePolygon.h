//
// Created by Cam on 2024-10-10.
//

#ifndef AUTOCARVER_COMPOSITEPOLYGON_H
#define AUTOCARVER_COMPOSITEPOLYGON_H

#include "Polygon.h"

#include <QVector2D>
#include <QVector3D>

#include <vector>

#include "geometry/Triangle.h"

class CompositePolygon : public Polygon {
public:

    explicit CompositePolygon(const std::vector<QVector2D> &border);
    explicit CompositePolygon(const std::vector<std::vector<QVector2D>> &loops);

    explicit CompositePolygon(const std::vector<QVector3D> &border, const QVector3D &normal);
    explicit CompositePolygon(const std::vector<std::vector<QVector3D>> &loops, const QVector3D &normal);

    struct IndexedBorder {
        std::vector<uint32_t> vertices;
    };

    virtual void invalidate() override;

    virtual void removeVertex(uint32_t index, bool maintainIntegrity = false) override;
    virtual void insertVertex(uint32_t reference, QVector2D vertex) override;
    virtual void positionVertex(uint32_t index, QVector2D position, bool maintainIntegrity = false) override;

    virtual void boolean(const Polygon &poly, BooleanOperation operation);

    const std::vector<std::pair<int, int>>& diagonals();
    const std::vector<IndexedBorder>& partitions();
    const std::vector<Triangle>& triangulation();

    const std::vector<std::pair<int, int>>& diagonals() const;
    const std::vector<IndexedBorder>& partitions() const;
    const std::vector<Triangle>& triangulation() const;

private:

    void diagonalize();

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

    void partition(const std::vector<std::pair<int, int>>& boundaries);
    void triangulate(const IndexedBorder &partition);

    bool compare(const Vertex& v1,const Vertex& v2);

    static bool angle(const QVector2D &pivot, const QVector2D &a, const QVector2D &b);
    static float internalAngle(const QVector2D &pivot, const QVector2D &a, const QVector2D &b);

    static float dot(const QVector2D &pivot, const QVector2D &a, const QVector2D &b);
    static float dot(const QVector2D &a, const QVector2D &b);


private:

    std::vector<Vertex> m_eventQueue;
    std::vector<std::pair<int, int>> intervals;
    std::vector<int> helper;

    std::vector<std::pair<int, int>> m_diagonals;
    bool m_diagonalized;

    std::vector<IndexedBorder> m_partitions;
    std::vector<Triangle> m_triangles;
};


#endif //AUTOCARVER_COMPOSITEPOLYGON_H
