//
// Created by cameronh on 02/06/24.
//

#ifndef AUTOCARVER_POLYGON_H
#define AUTOCARVER_POLYGON_H

#include <QVector2D>
#include <QVector3D>

#include <vector>

#include "geometry/Triangle.h"

class Polygon {
public:

    explicit Polygon(const std::vector<QVector2D> &border);
    explicit Polygon(const std::vector<std::vector<QVector2D>> &loops);

    explicit Polygon(const std::vector<QVector3D> &border, const QVector3D &normal);
    explicit Polygon(const std::vector<std::vector<QVector3D>> &loops, const QVector3D &normal);

    virtual void invalidate();

    virtual void removeVertex(uint32_t index, bool maintainIntegrity);
    virtual void insertVertex(uint32_t reference, QVector2D vertex);
    virtual void positionVertex(uint32_t index, QVector2D position, bool maintainIntegrity);

    void translate(QVector2D delta);
    void rotate(QVector2D center, float theta);

    enum class BooleanOperation {
        INTERSECTION = 0, UNION, DIFFERENCE
    };

    virtual void boolean(const Polygon &poly, BooleanOperation operation) = 0;

    uint32_t loopCount();
    uint32_t loopLength(uint32_t index);
    uint32_t identifyParentLoop(uint32_t index);
    std::vector<uint32_t> getLoop(uint32_t index);

    uint32_t vertexCount();
    QVector2D getVertex(uint32_t index);

    bool encloses(const QVector2D &p);

    float area();
    float area() const;

    float area(uint32_t loopIndex);
    static float area(const std::vector<QVector2D> &border);

    static std::vector<QVector2D> reduce(const std::vector<QVector3D> &loop, const QVector3D &normal);
//    static std::vector<QVector3D> expand(const std::vector<QVector3D> &loop, const QVector3D &normal);

protected:

    enum class VertexType {
        NORMAL = 0, START, END, SPLIT, MERGE
    };

    template <typename VertexType>
    auto asInteger(VertexType const value) -> typename std::underlying_type<VertexType>::type{
        return static_cast<typename std::underlying_type<VertexType>::type>(value);
    }

    struct Vertex {
        int index = -1;

        int prev = -1;
        int next = -1;

        VertexType type = VertexType::NORMAL;
    };

    // TODO separate into its own class when needed
    struct System {
        QVector3D origin;
        QVector3D xAxis;
        QVector3D yAxis;
    };

    void initializeLoop(const std::vector<QVector2D> &border);
    static std::vector<QVector2D> reduce(const std::vector<QVector3D> &loop, const QVector3D &normal, const System &sys);

    Triangle ccwTriangle(uint32_t I0, uint32_t I1, uint32_t I2);

    static float interpolate(const QVector2D &start, const QVector2D &end, float dy);

    bool intersects(const Vertex &a, const Vertex &b);
    static bool segmentIntersection(const QVector2D &a, const QVector2D &b, const QVector2D &c, const QVector2D &d);

protected:

    std::vector<QVector2D> m_vertices; // Plain list of the vertex coordinates that compose the polygon
    std::vector<Vertex> m_links; // Doubly-connected list of vertex links that compose the polygon

    std::vector<uint32_t> m_loops; // Index to the 'first' vertex on each loop

    float m_area;
    bool m_areaOK;

    float m_boundingRadius;
    QVector2D m_boundingCenter;
    bool m_boundingRadiusOK;


};


#endif //AUTOCARVER_POLYGON_H
