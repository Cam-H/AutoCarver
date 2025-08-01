//
// Created by cjhat on 2025-07-18.
//

#include "Ray.h"

Ray::Ray(const glm::dvec3& origin, const glm::dvec3& axis)
    : origin(origin)
    , axis(axis)
{

}

bool Ray::isValid() const
{
    double len = glm::dot(axis, axis);
    return 1 - 1e-12 < len && len < 1 + 1e-12;
}

double Ray::squareDistance(const glm::dvec3& vertex) const
{
    glm::dvec3 delta = vertex - origin;
    delta -= axis * glm::dot(axis, delta);
    return glm::dot(delta, delta);
}