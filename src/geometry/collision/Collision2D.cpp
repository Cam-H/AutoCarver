//
// Created by cjhat on 2025-09-21.
//

#include "Collision2D.h"

#include "geometry/primitives/Circle.h"
#include "geometry/primitives/Line.h"
#include "geometry/poly/Polygon.h"

bool Collision2D::test(const Circle& bodyA, const Line& bodyB)
{
    glm::dvec2 axis = glm::normalize(bodyB.second - bodyB.first);
    axis = { axis.y, -axis.x };

    return std::abs(glm::dot(axis, bodyA.center - bodyB.first)) < bodyA.radius;
}
bool Collision2D::test(const Polygon& bodyA, const Line& bodyB)
{
    if (test(bodyA.bounds(), bodyB)) {
        try {
            return gjkTest(bodyA, bodyB);
        } catch (std::exception& e) {
            std::cout << "\033[93m[Collision2D] Failed to test collision of polygon and line\n" << e.what() << "\033[0m\n";
//            static int i = 0;
//            bodyA.save("..\out\C2D-SET" + std::to_string(i++) + ".bin");
//            std::cout << "LINE: " << bodyB.first.x << " " << bodyB.first.y << " | " << bodyB.second.x << " " << bodyB.second.y << "\n";
        }
    }

    return false;
}

bool Collision2D::test(const Circle& bodyA, const Circle& bodyB)
{
    glm::dvec2 delta = bodyA.center - bodyB.center;
    return glm::dot(delta, delta) < pow(bodyA.radius + bodyB.radius, 2);
}
bool Collision2D::test(const Polygon& bodyA, const Circle& bodyB)
{
    if (test(bodyA.bounds(), bodyB)) {
        try {
            return gjkTest(bodyA, bodyB);
        } catch (std::exception& e) {
            std::cout << "\033[93m[Collision2D] Failed to test collision of circle and polygon\n" << e.what() << "\033[0m\n";
        }
    }

    return false;
}

bool Collision2D::test(const Circle& bodyA, const Polygon& bodyB)
{
    return test(bodyB, bodyA);
}

bool Collision2D::test(const Polygon& bodyA, const Polygon& bodyB)
{
    if (test(bodyA.bounds(), bodyB.bounds())) {
        try {
            return gjkTest(bodyA, bodyB);
        } catch (std::exception& e) {
            std::cout << "\033[93m[Collision2D] Failed to test collision of two polygons\n" << e.what() << "\033[0m\n";
            return edgeTest(bodyA, bodyB);
        }
    }

    return false;
}

bool Collision2D::edgeTest(const Polygon& bodyA, const Polygon& bodyB)
{
    return false; // TODO if needed
}

bool Collision2D::encloses(const Circle& body, const glm::dvec2& vertex)
{
    glm::dvec2 delta = vertex - body.center;
    return glm::dot(delta, delta) - body.radius * body.radius < 1e-12;
}

// Returns true if the line crosses the axis of the ray. t is the multiple for the intersection. The value is negative when
// the line is behind the ray
std::tuple<bool, double> Collision2D::intersection(const Line& bodyA, const Ray<2>& bodyB)
{
    auto v = bodyB.origin + bodyB.axis;
    auto a1 = Triangle2D::signedArea(bodyB.origin, v, bodyA.second);
    auto a2 = Triangle2D::signedArea(bodyB.origin, v, bodyA.first);

    if (a1 * a2 < 0) {
        double a3 = Triangle2D::signedArea(bodyA.first, bodyA.second, bodyB.origin), a4 = a3 + a2 - a1;
        return { true, a3 / (a3 - a4) };
    }

    return { false, -1.0 };
}

// Returns true if the polygon passes the axis of the ray. Also returns tMin and tMax, the distance from the ray origin of the most extreme intersections
std::tuple<bool, double, double> Collision2D::intersection(const Polygon& bodyA, const Ray<2>& bodyB)
{
   if (bodyA.vertexCount() >= 3) {
       glm::dvec2 last = bodyA.border().back();
       double min = std::numeric_limits<double>::max(), max = std::numeric_limits<double>::lowest();
       bool intersects = false;

        for (uint32_t i = 0; i < bodyA.vertexCount(); i++) {
            Line edge(last, bodyA.border()[i]);
            auto [hit, t] = intersection(edge, bodyB);
            if (hit) {
                if (t < min) min = t;
                if (t > max) max = t;
                intersects = true;
            }

            last = edge.second;
        }

        return { intersects, min, max };
   }

    return { false, -1.0, -1.0 };
}

// TODO Expand - currently only supports convex polygons
Polygon Collision2D::fragment(const Polygon& poly, const Ray<2>& lim)
{
    std::vector<glm::dvec2> border;

    const Ray<2> norm = lim.normal();

    glm::dvec2 prev = poly.border().back();
    bool prevUp = glm::dot(prev - lim.origin, lim.axis) > 0;

    for (const glm::dvec2& vertex : poly.border()) {
        bool vUp = glm::dot(vertex - lim.origin, lim.axis) > 0;

        if (prevUp ^ vUp) { // Edge intersects limit
            auto [hit, t] = intersection({ prev, vertex}, norm);
            border.emplace_back(t * norm);
        }

        if (vUp) border.emplace_back(vertex);

        prev = vertex;
        prevUp = vUp;
    }

    return Polygon(border);
}