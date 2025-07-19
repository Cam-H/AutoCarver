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

#include "geometry/shape/Circle.h"
#include "geometry/VertexArray.h"

Polygon::Polygon(const std::vector<glm::vec2>& border, bool enforceCCWWinding)
    : m_vertices(border)
{
    if (enforceCCWWinding) correctWinding();
}

void Polygon::removeVertex(uint32_t index)
{
    if (index >= m_vertices.size()) throw std::runtime_error("[Polygon] Index out of bounds error!");
    m_vertices.erase(m_vertices.begin() + index);
}

void Polygon::insertVertex(const glm::vec2& vertex)
{
    m_vertices.push_back(vertex);
}

void Polygon::insertVertex(uint32_t index, const glm::vec2& vertex)
{
    if (index >= m_vertices.size()) m_vertices.push_back(vertex);
    else m_vertices.insert(m_vertices.begin() + index, vertex);
}

void Polygon::positionVertex(uint32_t index, const glm::vec2& position)
{
    if (index >= m_vertices.size()) throw std::runtime_error("[Polygon] Index out of bounds error!");

    m_vertices[index] = position;
}

void Polygon::translate(const glm::vec2& translation)
{
    for (glm::vec2& vertex : m_vertices) vertex += translation;
}

void Polygon::scale(float scalar)
{
    for (glm::vec2& vertex : m_vertices) vertex *= scalar;
}

void Polygon::scale(const glm::vec2& anchor, float scalar)
{
    glm::vec2 delta;
    for (glm::vec2& vertex : m_vertices) {
        delta = vertex - anchor;
        vertex = anchor + delta * scalar;
    }
}

void Polygon::centerScale(float scalar)
{
    glm::vec2 offset = { xSpan(), ySpan() };
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
    float sum = 0;

    for (uint32_t i = 0; i < m_vertices.size(); i++) {
        const glm::vec2& current = m_vertices[i];
        const glm::vec2& next = m_vertices[(i + 1) % m_vertices.size()];

        sum += (current.x - next.x) * (current.y + next.y);
    }

    return sum > 0;
}

float Polygon::xSpan() const
{
    float near, far;
    xExtents(near, far);
    return far - near;
}
float Polygon::ySpan() const
{
    float near, far;
    yExtents(near, far);
    return far - near;
}

glm::vec2 Polygon::spanCenter() const
{
    float near, far, x;
    xExtents(near, far);
    x = (near + far) / 2;
    yExtents(near, far);
    return { x, (near + far) / 2 };
}

void Polygon::xExtents(float& near, float& far) const
{
    auto [max, min] = std::minmax_element(m_vertices.begin(), m_vertices.end(), [](const glm::vec2& lhs, const glm::vec2& rhs){
        return lhs.x > rhs.x;
    });

    near = min->x;
    far = max->x;
}
void Polygon::yExtents(float& near, float& far) const
{
    auto [max, min] = std::minmax_element(m_vertices.begin(), m_vertices.end(), [](const glm::vec2& lhs, const glm::vec2& rhs){
        return lhs.y > rhs.y;
    });

    near = min->y;
    far = max->y;
}

uint32_t Polygon::vertexCount() const
{
    return m_vertices.size();
}

const std::vector<glm::vec2>& Polygon::border() const
{
    return m_vertices;
}

void Polygon::cullCollinear(std::vector<glm::vec2>& vertices, float tolerance)
{
    glm::vec2 prev = glm::normalize(vertices[vertices.size() - 1] - vertices[0]);
    for (uint32_t i = 0; i < vertices.size(); i++) {
        glm::vec2 next = glm::normalize(vertices[(i + 1) % vertices.size()] - vertices[i]);
        if (glm::dot(prev, next) < -1 + tolerance) {
            vertices.erase(vertices.begin() + i);
            i--;
        } else prev = -next;
    }
}

std::vector<glm::vec3> Polygon::projected3D(const glm::vec3& xAxis, const glm::vec3& yAxis, const glm::vec3& offset) const
{
    return Polygon::projected3D(m_vertices, xAxis, yAxis, offset);
}

std::vector<glm::vec3> Polygon::projected3D(const std::vector<glm::vec2>& vertices, const glm::vec3& xAxis, const glm::vec3& yAxis, const glm::vec3& offset)
{
    std::vector<glm::vec3> projection;
    projection.reserve(vertices.size());

    for (const glm::vec2& vertex : vertices) {
        projection.emplace_back(offset + xAxis * vertex.x + yAxis * vertex.y);
    }

    return projection;
}

std::vector<uint32_t> Polygon::hull() const
{
    return hull(m_vertices);
}

std::vector<uint32_t> Polygon::hull(const std::vector<glm::vec2>& vertices)
{

    uint32_t n = vertices.size(), k = 0;
    if (n <= 3) return { 0, 1, 2 };

    struct Vertex {
        glm::vec2 p;
        uint32_t idx;

        bool operator<(const Vertex& b) const {
            return p.x < b.p.x || (p.x == b.p.x && p.y < b.p.y);
        }
    };

    uint32_t idx = 0;
    std::vector<Vertex> points;
    points.reserve(vertices.size());
    for (const glm::vec2& vertex : vertices) points.push_back({ vertex, idx++ });

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

float Polygon::cross(const glm::vec2& origin, const glm::vec2& a, const glm::vec2& b)
{
    return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
}

std::vector<Triangle> Polygon::triangulate() const
{
    return triangulate(m_vertices);
}

std::vector<Triangle> Polygon::triangulate(const std::vector<glm::vec2>& vertices)
{
    CDT::Triangulation<float> cdt;

    // Convert polygon data into appropriate CDT types
    std::vector<CDT::V2d<float>> cVertices;
    cVertices.reserve(vertices.size());
    for (const glm::vec2& vertex : vertices) cVertices.emplace_back(vertex.x, vertex.y);

    std::vector<CDT::Edge> edges;
    edges.reserve(vertices.size());
    for (uint32_t i = 0; i < vertices.size() - 1; i++) edges.emplace_back(i, i + 1);
    edges.emplace_back(vertices.size() - 1, 0);

    std::vector<Triangle> triangles;

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
std::vector<glm::vec2> Polygon::clean(const std::vector<glm::vec2>& vertices)
{
    if (vertices.empty()) return {};

    std::vector<glm::vec2> cleaned = { vertices[0] };
    cleaned.reserve(vertices.size());

    glm::vec2 delta;
    for (uint32_t i = 1; i < vertices.size(); i++) {
        delta = vertices[i] - vertices[i - 1];
        if (glm::dot(delta, delta) > 1e-6) { // Non-duplicate vertex = sufficiently far from previous vertex
            cleaned.push_back(vertices[i]);
        }
    }

    // Consider endpoints
    delta = cleaned[cleaned.size() - 1] - cleaned[0];
    if (glm::dot(delta, delta) < 1e-6) cleaned.pop_back();

    return cleaned;
}

std::vector<std::pair<glm::vec2, glm::vec2>> Polygon::debugEdges() const
{
    return {};
}