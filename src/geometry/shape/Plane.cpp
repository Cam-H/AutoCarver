//
// Created by cjhat on 2025-07-18.
//

#include "Plane.h"

Plane::Plane() : origin(0, 0, 0), normal(0, 1, 0) {}

Plane::Plane(const glm::vec3& origin, const glm::vec3& normal)
    : origin(origin)
    , normal(normal)
{
}

glm::vec3 Plane::project(const glm::vec3& vertex) const
{
    return project(*this, vertex);
}

glm::vec3 Plane::project(const glm::vec3& origin, const glm::vec3& normal, const glm::vec3& vertex)
{
    return vertex - normal * glm::dot(normal, vertex - origin);
}
glm::vec3 Plane::project(const Plane& plane, const glm::vec3& vertex)
{
    return vertex - plane.normal * glm::dot(plane.normal, vertex - plane.origin);
}