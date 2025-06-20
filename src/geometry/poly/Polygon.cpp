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

#include "geometry/Circle.h"

Polygon::Polygon(const std::vector<glm::vec2>& border)
    : m_vertices(border)
{

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
    std::vector<glm::vec3> vertices;
    vertices.reserve(m_vertices.size());

    for (const glm::vec2& vertex : m_vertices) {
        vertices.emplace_back(offset + xAxis * vertex.x + yAxis * vertex.y);
    }

    return vertices;
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

std::vector<Triangle> Polygon::tesselate()
{
    return bowyerWatson();
}

// Find the Delaunay triangulation of the polygon
std::vector<Triangle> Polygon::bowyerWatson(bool cullTriangles, float direction)
{
    return Polygon::bowyerWatson(m_vertices, cullTriangles, direction);
}

// Find the Delaunay triangulation of the provided border
std::vector<Triangle> Polygon::bowyerWatson(const std::vector<glm::vec2>& border, bool cullTriangles, float direction) {
    uint32_t n = border.size();
    float minX = std::numeric_limits<float>::max(), maxX = std::numeric_limits<float>::lowest();
    float minY = minX, maxY = maxX;

    for (const auto& vertex : border) {
        minX = std::min(minX, vertex.x);
        maxX = std::max(maxX, vertex.x);
        minY = std::min(minY, vertex.y);
        maxY = std::max(maxY, vertex.y);
    }

    float dx = maxX - minX, dy = maxY - minY;
    float delta = std::max(dx, dy);
    float midx = (minX + maxX) / 2;
    float midy = (minY + maxY) / 2;

    std::vector<glm::vec2> vertices = border;
    // Super-triangle points
    vertices.emplace_back(midx - 20.0f * delta, midy - delta);
    vertices.emplace_back(midx, midy + 20.0f * delta);
    vertices.emplace_back(midx + 20.0f * delta, midy - delta);

    std::vector<Triangle> triangles = {
            {n, n + 1, n + 2}
    };

    try {
        for (int i = 0; i < n; i++) {
            const auto& p = vertices[i];
            std::vector<Triangle> bad;
            std::vector<Edge> boundary;

            // Find triangles whose circumcircle contains the point
            for (const auto& tri : triangles) {
                auto circle = Circle::triangleCircumcircle(vertices[tri.I0], vertices[tri.I1], vertices[tri.I2]);
                if (circle.encloses(p)) {
                    bad.push_back(tri);
                }
            }

            // Find boundary (edges not shared by two bad triangles)
            std::multiset<Edge> edges;
            for (const auto& tri : bad) {
                edges.insert(Edge(tri.I0, tri.I1));
                edges.insert(Edge(tri.I1, tri.I2));
                edges.insert(Edge(tri.I2, tri.I0));
            }

            std::set<Edge> boundaryEdges;
            for (const auto& e : edges) {
                if (edges.count(e) == 1)
                    boundaryEdges.insert(e);
            }

            // Remove bad triangles
            triangles.erase(remove_if(triangles.begin(), triangles.end(), [&](const Triangle& t) {
                return find(bad.begin(), bad.end(), t) != bad.end();
            }), triangles.end());

            // Re-triangulate the hole
            for (const auto& e : boundaryEdges) {
                triangles.emplace_back(e.u, e.v, i);
            }
        }
    } catch (const std::runtime_error& e) {
        std::cerr << "Caught exception: " << e.what() << ". This face will not be triangulated\n";
        return {};
    }

    // Remove triangles connected to super triangle vertices
    triangles.erase(remove_if(triangles.begin(), triangles.end(), [&](const Triangle& tri) {
        return tri.I0 >= n || tri.I1 >= n || tri.I2 >= n;
    }), triangles.end());

    // Remove triangles outside the border (marked by inverted cross-product)
    if (cullTriangles) {
        triangles.erase(remove_if(triangles.begin(), triangles.end(), [&](const Triangle& tri) {
            glm::vec2 ab = border[tri.I1] - border[tri.I0], ac = border[tri.I2] - border[tri.I0];
            return direction * (ab.x * ac.y - ab.y * ac.x) < 0;
        }), triangles.end());
    }


    return triangles;
}

std::vector<std::pair<glm::vec2, glm::vec2>> Polygon::debugEdges() const
{
    return {};
}