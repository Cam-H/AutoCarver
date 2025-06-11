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

// Find the Delaunay triangulation of the polygon
std::vector<Triangle> Polygon::bowyerWatson()
{
    return Polygon::bowyerWatson(m_vertices);
}

// Find the Delaunay triangulation of the provided border
std::vector<Triangle> Polygon::bowyerWatson(const std::vector<glm::vec2>& border) {
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
    triangles.erase(remove_if(triangles.begin(), triangles.end(), [&](const Triangle& tri) {
        glm::vec2 ab = border[tri.I1] - border[tri.I0], ac = border[tri.I2] - border[tri.I0];
        return (ab.x * ac.y - ab.y * ac.x) < 0;
    }), triangles.end());

    return triangles;
}