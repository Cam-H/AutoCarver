//
// Created by cameronh on 02/06/24.
//

#include "Polygon.h"

#include <iostream>
#include <cmath>
#include <set>
#include <tuple>
#include <limits>
#include <algorithm>

#include <CDT.h>

#include "geometry/primitives/Circle.h"
#include "geometry/VertexArray.h"

Polygon::Polygon(const std::vector<glm::dvec2>& border, bool enforceCCWWinding)
    : m_vertices(border)
{
    if (enforceCCWWinding) correctWinding();
}

Polygon::Polygon(const std::string& filename)
        : Polygon(std::vector<glm::dvec2>{})
{
    Serializable::load(filename);
}

Polygon::Polygon(std::ifstream& file)
        : Polygon(std::vector<glm::dvec2>{})
{
    if (!Polygon::deserialize(file)) std::cerr << "Failed to deserialize profile properly!\n";
}

bool Polygon::serialize(std::ofstream& file) const
{
    Serializer::writeVectorDVec2(file, m_vertices);

    return true;
}

bool Polygon::deserialize(std::ifstream& file)
{
    m_vertices = Serializer::readVectorDVec2(file);

    return true;
}

void Polygon::removeVertex(uint32_t index)
{
    if (index >= m_vertices.size()) throw std::runtime_error("[Polygon] Index out of bounds error!");
    m_vertices.erase(m_vertices.begin() + index);
}

void Polygon::insertVertex(const glm::dvec2& vertex)
{
    m_vertices.push_back(vertex);
}

void Polygon::insertVertex(uint32_t index, const glm::dvec2& vertex)
{
    if (index >= m_vertices.size()) m_vertices.push_back(vertex);
    else m_vertices.insert(m_vertices.begin() + index, vertex);
}

void Polygon::positionVertex(uint32_t index, const glm::dvec2& position)
{
    if (index >= m_vertices.size()) throw std::runtime_error("[Polygon] Index out of bounds error!");

    m_vertices[index] = position;
}

void Polygon::translate(const glm::dvec2& translation)
{
    for (glm::dvec2& vertex : m_vertices) vertex += translation;
}

void Polygon::scale(double scalar)
{
    for (glm::dvec2& vertex : m_vertices) vertex *= scalar;
}

void Polygon::scale(const glm::dvec2& anchor, double scalar)
{
    glm::dvec2 delta;
    for (glm::dvec2& vertex : m_vertices) {
        delta = vertex - anchor;
        vertex = anchor + delta * scalar;
    }
}

void Polygon::centerScale(double scalar)
{
    glm::dvec2 offset = { xSpan(), ySpan() };
    scale(scalar);

    offset *= -0.5f * (scalar - 1);
    translate(offset);
}

void Polygon::correctWinding()
{
    if(isCW()) inverseWinding();
}

void Polygon::inverseWinding()
{
    std::reverse(m_vertices.begin(), m_vertices.end());
}

bool Polygon::isCW() const
{
    return !isCCW();
}
bool Polygon::isCCW() const
{
    double sum = 0;

    for (uint32_t i = 0; i < m_vertices.size(); i++) {
        const glm::dvec2& current = m_vertices[i];
        const glm::dvec2& next = m_vertices[(i + 1) % m_vertices.size()];

        sum += (current.x - next.x) * (current.y + next.y);
    }

    return sum > 0;
}

double Polygon::xSpan() const
{
    double near, far;
    xExtents(near, far);
    return far - near;
}
double Polygon::ySpan() const
{
    double near, far;
    yExtents(near, far);
    return far - near;
}

glm::dvec2 Polygon::spanCenter() const
{
    double near, far, x;
    xExtents(near, far);
    x = (near + far) / 2;
    yExtents(near, far);
    return { x, (near + far) / 2 };
}

void Polygon::xExtents(double& near, double& far) const
{
    auto [max, min] = std::minmax_element(m_vertices.begin(), m_vertices.end(), [](const glm::dvec2& lhs, const glm::dvec2& rhs){
        return lhs.x > rhs.x;
    });

    near = min->x;
    far = max->x;
}
void Polygon::yExtents(double& near, double& far) const
{
    auto [max, min] = std::minmax_element(m_vertices.begin(), m_vertices.end(), [](const glm::dvec2& lhs, const glm::dvec2& rhs){
        return lhs.y > rhs.y;
    });

    near = min->y;
    far = max->y;
}

uint32_t Polygon::vertexCount() const
{
    return m_vertices.size();
}

const std::vector<glm::dvec2>& Polygon::border() const
{
    return m_vertices;
}

void Polygon::cullCollinear(std::vector<glm::dvec2>& vertices, double tolerance)
{
    glm::dvec2 prev = glm::normalize(vertices[vertices.size() - 1] - vertices[0]);
    for (uint32_t i = 0; i < vertices.size(); i++) {
        glm::dvec2 next = glm::normalize(vertices[(i + 1) % vertices.size()] - vertices[i]);
        if (glm::dot(prev, next) < -1 + tolerance) {
            vertices.erase(vertices.begin() + i);
            i--;
        } else prev = -next;
    }
}

std::vector<glm::dvec3> Polygon::projected3D(const glm::dvec3& xAxis, const glm::dvec3& yAxis, const glm::dvec3& offset) const
{
    return Polygon::projected3D(m_vertices, xAxis, yAxis, offset);
}

std::vector<glm::dvec3> Polygon::projected3D(const std::vector<glm::dvec2>& vertices, const glm::dvec3& xAxis, const glm::dvec3& yAxis, const glm::dvec3& offset)
{
    std::vector<glm::dvec3> projection;
    projection.reserve(vertices.size());

    for (const glm::dvec2& vertex : vertices) {
        projection.emplace_back(projected3D(vertex, xAxis, yAxis, offset));
    }

    return projection;
}

std::vector<uint32_t> Polygon::hull() const
{
    return hull(m_vertices);
}

std::vector<uint32_t> Polygon::hull(const std::vector<glm::dvec2>& vertices)
{

    uint32_t n = vertices.size(), k = 0;
    if (n <= 3) return { 0, 1, 2 };

    struct Vertex {
        glm::dvec2 p;
        uint32_t idx;

        bool operator<(const Vertex& b) const {
            return p.x < b.p.x || (p.x == b.p.x && p.y < b.p.y);
        }
    };

    uint32_t idx = 0;
    std::vector<Vertex> points;
    points.reserve(vertices.size());
    for (const glm::dvec2& vertex : vertices) points.push_back({ vertex, idx++ });

    std::sort(points.begin(), points.end());
    std::vector<uint32_t> hull(2 * n);


    // Lower hull
    for (uint32_t i = 0; i < vertices.size(); i++) {
        while (k >= 2 && cross(points[hull[k - 2]].p, points[hull[k - 1]].p, points[i].p) <= 0) k--;
        hull[k++] = i;
    }

    // Upper hull
    for (int i = n - 2, t = k + 1; i >= 0; i--) {
        while (k >= t && cross(points[hull[k - 2]].p, points[hull[k - 1]].p, points[i].p) <= 0) k--;
        hull[k++] = i;
    }

    hull.resize(k - 1); // Remove duplicate end point
    for (uint32_t& vertex : hull) vertex = points[vertex].idx;

    return hull;
}

double Polygon::cross(const glm::dvec2& origin, const glm::dvec2& a, const glm::dvec2& b)
{
    return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
}

std::vector<TriIndex> Polygon::triangulate() const
{
    return triangulate(m_vertices);
}

std::vector<TriIndex> Polygon::triangulate(const std::vector<glm::dvec2>& vertices)
{
    CDT::Triangulation<double> cdt;

    // Convert polygon data into appropriate CDT types
    std::vector<CDT::V2d<double>> cVertices;
    cVertices.reserve(vertices.size());
    for (const glm::dvec2& vertex : vertices) cVertices.emplace_back(vertex.x, vertex.y);

    std::vector<CDT::Edge> edges;
    edges.reserve(vertices.size());
    for (uint32_t i = 0; i < vertices.size() - 1; i++) edges.emplace_back(i, i + 1);
    edges.emplace_back(vertices.size() - 1, 0);

    std::vector<TriIndex> triangles;

    try {

        // Attach polygon data
        cdt.insertVertices(cVertices);
        cdt.insertEdges(edges);

        cdt.eraseOuterTriangles();

        // Convert output into a usable format
        for (const CDT::Triangle& triangle : cdt.triangles) {
            triangles.emplace_back(triangle.vertices[0], triangle.vertices[1], triangle.vertices[2]);
        }

    } catch (CDT::DuplicateVertexError& e) {

        // Generate a new polygon without the duplicate vertices
        auto cleaned = clean(vertices);

        // Try triangulating again
        if (cleaned.size() != vertices.size()) return triangulate(cleaned);
        else std::cout << "[Polygon] Failed to triangulate due to duplicate vertices #" << e.v1() << " and #" << e.v2() << "\n";

    }  catch (std::exception& e) {
        std::cout << e.what() << "\n";
    }

    return triangles;
}

void Polygon::clean()
{
    m_vertices = clean(m_vertices);
}

// Remove duplicate vertices from the polygon - Only applies to adjacent vertices
std::vector<glm::dvec2> Polygon::clean(const std::vector<glm::dvec2>& vertices)
{
    if (vertices.empty()) return {};

    std::vector<glm::dvec2> cleaned = { vertices[0] };
    cleaned.reserve(vertices.size());

    glm::dvec2 delta;
    for (uint32_t i = 1; i < vertices.size(); i++) {
        delta = vertices[i] - vertices[i - 1];
        if (glm::dot(delta, delta) > 1e-12) { // Non-duplicate vertex = sufficiently far from previous vertex
            cleaned.push_back(vertices[i]);
        }
    }

    // Consider endpoints
    delta = cleaned[cleaned.size() - 1] - cleaned[0];
    if (glm::dot(delta, delta) < 1e-12) cleaned.pop_back();

    return cleaned;
}

std::vector<std::pair<glm::dvec2, glm::dvec2>> Polygon::debugEdges() const
{
    return {};
}