//
// Created by cameronh on 02/06/24.
//

#ifndef AUTOCARVER_POLYGON_H
#define AUTOCARVER_POLYGON_H

#include <vector>
#include <glm.hpp>

#include "fileIO/Serializable.h"

#include "geometry/primitives/Triangle.h"
#include "geometry/primitives/Circle.h"

class Polygon : public Serializable {
public:

    explicit Polygon(const std::vector<glm::dvec2> &border, bool enforceCCWWinding = true);
    Polygon(const glm::dvec2& center, double width, double height, double rotation = 0);

    explicit Polygon(const std::string& filename);
    explicit Polygon(std::ifstream& file);

    bool serialize(std::ofstream& file) const override;
    bool deserialize(std::ifstream& file) override;

    void removeVertex(uint32_t index);

    void insertVertex(const glm::dvec2& vertex);
    void insertVertex(uint32_t index, const glm::dvec2& vertex);

    virtual void positionVertex(uint32_t index, const glm::dvec2& position);

    void translate(const glm::dvec2& translation);

    void rotate(double theta);
    void rotate(const glm::dvec2& anchor, double theta);

    void scale(double scalar);
    void scale(const glm::dvec2& anchor, double scalar);
    void centerScale(double scalar);

    void correctWinding();
    virtual void inverseWinding();

    void convexify();

    void clean();
    static std::vector<glm::dvec2> clean(const std::vector<glm::dvec2>& vertices);

    [[nodiscard]] uint32_t vertexCount() const;
    [[nodiscard]] const std::vector<glm::dvec2>& border() const;

    [[nodiscard]] virtual uint32_t prevVertex(uint32_t vertexIndex) const;
    [[nodiscard]] virtual uint32_t nextVertex(uint32_t vertexIndex) const;

    [[nodiscard]] const Circle& bounds() const;

    [[nodiscard]] bool isCW() const;
    [[nodiscard]] bool isCCW() const;

    [[nodiscard]] double xSpan() const;
    [[nodiscard]] double ySpan() const;
    [[nodiscard]] glm::dvec2 spanCenter() const;

    [[nodiscard]] std::tuple<double, double> xExtents() const;
    [[nodiscard]] std::tuple<double, double> yExtents() const;

    [[nodiscard]] bool isValid() const;

    [[nodiscard]] glm::vec2 start() const;
    [[nodiscard]] uint32_t supportIndex(const glm::dvec2& axis, uint32_t startIndex = 0) const;
    [[nodiscard]] std::tuple<uint32_t, glm::dvec2> extreme(const glm::dvec2& axis, uint32_t startIndex = 0) const;

    static void cullCollinear(std::vector<glm::dvec2>& vertices, double tolerance = 1e-3);

    std::vector<glm::dvec3> projected3D(const glm::dvec3& xAxis, const glm::dvec3& yAxis, const glm::dvec3& offset = {}) const;
    static std::vector<glm::dvec3> projected3D(const std::vector<glm::dvec2>& vertices, const glm::dvec3& xAxis, const glm::dvec3& yAxis, const glm::dvec3& offset = {});
    static inline glm::dvec3 projected3D(const glm::dvec2& vertex, const glm::dvec3& xAxis, const glm::dvec3& yAxis, const glm::dvec3& offset) { return offset + xAxis * vertex.x + yAxis * vertex.y; };

    [[nodiscard]] std::vector<uint32_t> hull() const;
    static std::vector<uint32_t> hull(const std::vector<glm::dvec2>& vertices);

    [[nodiscard]] std::vector<TriIndex> triangulate() const;
    static std::vector<TriIndex> triangulate(const std::vector<glm::dvec2>& vertices);

    [[nodiscard]] virtual std::vector<Polygon> decompose() const;

    [[nodiscard]] bool test(const glm::dvec2& vertex) const;

    [[nodiscard]] double clear(double clearance, glm::dvec3& location) const;

    [[nodiscard]] virtual std::vector<std::pair<glm::dvec2, glm::dvec2>> debugEdges() const;

    virtual void print() const;

protected:

    static double cross(const glm::dvec2& origin, const glm::dvec2& a, const glm::dvec2& b);

    static std::tuple<bool, double> intersection(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c, const glm::dvec2& d);

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
    Circle m_bounds;
};


#endif //AUTOCARVER_POLYGON_H
