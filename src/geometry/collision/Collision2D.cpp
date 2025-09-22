//
// Created by cjhat on 2025-09-21.
//

#include "Collision2D.h"

#include "geometry/primitives/Circle.h"
#include "geometry/poly/Polygon.h"

bool Collision2D::test(const Circle& bodyA, const glm::dvec2& a, const glm::dvec2& b)
{
    glm::dvec2 axis = glm::normalize(b - a);
    axis = { axis.y, -axis.x };

    return std::abs(glm::dot(axis, bodyA.center - a)) < bodyA.radius;
}
bool Collision2D::test(const Polygon& bodyA, const glm::dvec2& a, const glm::dvec2& b)
{
    return test(bodyA.bounds(), a, b);
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