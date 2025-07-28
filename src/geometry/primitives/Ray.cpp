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
    return glm::dot(axis, axis) > 1e-12;
}