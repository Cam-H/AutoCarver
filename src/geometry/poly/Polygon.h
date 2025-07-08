//
// Created by cameronh on 02/06/24.
//

#ifndef AUTOCARVER_POLYGON_H
#define AUTOCARVER_POLYGON_H

#include <vector>
#include <glm/glm.hpp>

#include "geometry/Triangle.h"

class Polygon {
public:

    explicit Polygon(const std::vector<glm::vec2> &border, bool enforceCCWWinding = true);

    void removeVertex(uint32_t index);

    void insertVertex(const glm::vec2& vertex);
    void insertVertex(uint32_t index, const glm::vec2& vertex);

    void positionVertex(uint32_t index, const glm::vec2& position);

    void translate(const glm::vec2& translation);
    void scale(float scalar);
    void scale(const glm::vec2& anchor, float scalar);
    void centerScale(float scalar);

    void correctWinding();
    virtual void inverseWinding();

    uint32_t vertexCount() const;
    const std::vector<glm::vec2>& border() const;

    bool isCW() const;
    bool isCCW() const;

    float xSpan() const;
    float ySpan() const;

    void xExtents(float& near, float& far) const;
    void yExtents(float& near, float& far) const;

    static void cullCollinear(std::vector<glm::vec2>& vertices, float tolerance = 1e-3);

    std::vector<glm::vec3> projected3D(const glm::vec3& xAxis, const glm::vec3& yAxis, const glm::vec3& offset = {}) const;

    std::vector<uint32_t> hull() const;
    static std::vector<uint32_t> hull(const std::vector<glm::vec2>& vertices);

    std::vector<Triangle> tesselate();
    std::vector<Triangle> bowyerWatson(bool cullTriangles = true, float direction = 1);
    static std::vector<Triangle> bowyerWatson(const std::vector<glm::vec2>& border, bool cullTriangles = true, float direction = 1);

    [[nodiscard]] virtual std::vector<std::pair<glm::vec2, glm::vec2>> debugEdges() const;

protected:

    static float cross(const glm::vec2& origin, const glm::vec2& a, const glm::vec2& b);

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

    std::vector<glm::vec2> m_vertices; // Plain list of the vertex coordinates that compose the polygon

};


#endif //AUTOCARVER_POLYGON_H
