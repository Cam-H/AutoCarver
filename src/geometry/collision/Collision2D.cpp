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
        return true;
    }

    return false;
}
bool Collision2D::test(const Polygon& bodyA, const Polygon& bodyB)
{
    if (test(bodyA.bounds(), bodyB.bounds())) {
        return true;
    }

    return false;
}

bool Collision2D::encloses(const Circle& body, const glm::dvec2& vertex)
{
    glm::dvec2 delta = vertex - body.center;
    return glm::dot(delta, delta) - body.radius * body.radius < 1e-12;
}