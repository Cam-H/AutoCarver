//
// Created by cjhat on 2025-07-18.
//

#include "Plane.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/quaternion.hpp>

#include <iostream>

Plane::Plane() : origin(0, 0, 0), normal(0, 1, 0) {}

Plane::Plane(const glm::dvec3& origin, const glm::dvec3& normal)
    : origin(origin)
    , normal(normal)
{
}

void Plane::rotate(const glm::dvec3& axis, double theta)
{
    normal = normal * glm::angleAxis(theta, axis);
}

void Plane::rotate(const glm::dquat& rotation)
{
    normal = normal * rotation;
}

void Plane::invert()
{
    normal = -normal;
}

bool Plane::isValid() const
{
    return glm::dot(normal, normal) > 1e-12;
}

Plane Plane::rotated(const glm::dvec3& axis, double theta) const
{
    return { origin, normal * glm::angleAxis(theta, axis) };
}

Plane Plane::rotated(const glm::dquat& rotation) const
{
    return { origin, normal * rotation };
}

Plane Plane::inverted() const
{
    return { origin, -normal };
}

glm::dvec3 Plane::project(const glm::dvec3& vertex) const
{
    return project(*this, vertex);
}

glm::dvec3 Plane::project(const glm::dvec3& origin, const glm::dvec3& normal, const glm::dvec3& vertex)
{
    return vertex - normal * glm::dot(normal, vertex - origin);
}
glm::dvec3 Plane::project(const Plane& plane, const glm::dvec3& vertex)
{
    return vertex - plane.normal * glm::dot(plane.normal, vertex - plane.origin);
}

void Plane::print() const
{
    std::cout << "[Plane] origin: (" << origin.x << ", " << origin.y << ", " << origin.z << "), normal: ("
            << normal.x << ", " << normal.y << ", " << normal.z << ")\n";
}