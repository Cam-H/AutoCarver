//
// Created by cjhat on 2025-07-18.
//

#include "Plane.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/quaternion.hpp>

#include <iostream>

Plane::Plane() : origin(0, 0, 0), normal(0, 1, 0) {}

Plane::Plane(const glm::vec3& origin, const glm::vec3& normal)
    : origin(origin)
    , normal(normal)
{
}

void Plane::rotate(const glm::vec3& axis, float theta)
{
    normal = normal * glm::angleAxis(theta, axis);
}

bool Plane::isValid() const
{
    return glm::dot(normal, normal) > 1e-6;
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

void Plane::print() const
{
    std::cout << "[Plane] origin: (" << origin.x << ", " << origin.y << ", " << origin.z << "), normal: ("
            << normal.x << ", " << normal.y << ", " << normal.z << ")\n";
}