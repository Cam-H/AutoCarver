//
// Created by Cam on 2025-06-11.
//

#include "Circle.h"

#include <stdexcept>


Circle::Circle() : center(0, 0), radius(-1.0f) {}

Circle::Circle(const glm::vec2& center, float radius) : center(center), radius(radius) {}

bool Circle::isValid() const
{
    return radius >= 0;
}


glm::vec2 Circle::start() const
{
    return center;
}

//uint32_t Circle::supportIndex(const glm::vec2& axis) const
//{
//    return 0;
//}
//
//std::tuple<uint32_t, glm::vec2> Circle::extreme(const glm::vec2& axis) const
//{
//    return { 0, center };
//}

// Compute circumcenter and radius
Circle Circle::triangleCircumcircle(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c) {
    float d = 2 * (a.x*(b.y - c.y) + b.x*(c.y - a.y) + c.x*(a.y - b.y));
    if (std::abs(d) < 1e-9) throw std::runtime_error("Degenerate triangle");

    glm::vec2 ab = b - a;
    glm::vec2 ac = c - a;
    glm::vec2 bc = c - b;

    float la = glm::dot(a, a), lb = glm::dot(b, b), lc = glm::dot(c, c);

    float ux = (-la * bc.y + lb * ac.y - lc * ab.y) / d;
    float uy = ( la * bc.x - lb * ac.x + lc * ab.x) / d;

    return {
        glm::vec2(ux, uy),
        (a.x - ux) * (a.x - ux) + (a.y - uy) * (a.y - uy)
    };
}

bool Circle::encloses(const glm::vec2& vertex, float tolerance) const
{
    glm::vec2 delta = center - vertex;
    return glm::dot(delta, delta) <= radius + tolerance;
}