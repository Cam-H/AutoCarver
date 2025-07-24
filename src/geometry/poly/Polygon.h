//
// Created by cameronh on 02/06/24.
//

#ifndef AUTOCARVER_POLYGON_H
#define AUTOCARVER_POLYGON_H

#include <vector>
#include <glm.hpp>

#include "geometry/primitives/Triangle.h"

class Polygon {
public:

    explicit Polygon(const std::vector<glm::dvec2> &border, bool enforceCCWWinding = true);

    void removeVertex(uint32_t index);

    void insertVertex(const glm::dvec2& vertex);
    void insertVertex(uint32_t index, const glm::dvec2& vertex);

    void positionVertex(uint32_t index, const glm::dvec2& position);

    void translate(const glm::dvec2& translation);
    void scale(double scalar);
    void scale(const glm::dvec2& anchor, double scalar);
    void centerScale(double scalar);

    void correctWinding();
    virtual void inverseWinding();

    void clean();
    static std::vector<glm::dvec2> clean(const std::vector<glm::dvec2>& vertices);

    uint32_t vertexCount() const;
    const std::vector<glm::dvec2>& border() const;

    bool isCW() const;
    bool isCCW() const;

    double xSpan() const;
    double ySpan() const;
    glm::dvec2 spanCenter() const;

    void xExtents(double& near, double& far) const;
    void yExtents(double& near, double& far) const;

    static void cullCollinear(std::vector<glm::dvec2>& vertices, double tolerance = 1e-3);

    std::vector<glm::dvec3> projected3D(const glm::dvec3& xAxis, const glm::dvec3& yAxis, const glm::dvec3& offset = {}) const;
    static std::vector<glm::dvec3> projected3D(const std::vector<glm::dvec2>& vertices, const glm::dvec3& xAxis, const glm::dvec3& yAxis, const glm::dvec3& offset = {});

    std::vector<uint32_t> hull() const;
    static std::vector<uint32_t> hull(const std::vector<glm::dvec2>& vertices);

    std::vector<Triangle> triangulate() const;
    static std::vector<Triangle> triangulate(const std::vector<glm::dvec2>& vertices);

    [[nodiscard]] virtual std::vector<std::pair<glm::dvec2, glm::dvec2>> debugEdges() const;

protected:

    static double cross(const glm::dvec2& origin, const glm::dvec2& a, const glm::dvec2& b);

private:

    // Edge represented as unordered pair (with sorting)
    struct Edge {
        uint32_t u, v;
        Edge(uint32_t a, uint32_t b) : u(std::min(a,b)), v(std::max(a,b)) {}
        bool operator<(const Edge& e) const {
            return std::tie(u,v) < std::tie(e.u,e.v);
        }
    };

protected:

    std::vector<glm::dvec2> m_vertices; // Plain list of the vertex coordinates that compose the polygon

};


#endif //AUTOCARVER_POLYGON_H
