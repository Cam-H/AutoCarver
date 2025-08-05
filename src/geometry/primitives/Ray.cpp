//
// Created by cjhat on 2025-07-18.
//

#include "Ray.h"
#include "geometry/Transformable.h"

#include <iostream>

Ray::Ray()
    : origin({})
    , axis({ 0, 1, 0 })
{

}

Ray::Ray(const glm::dvec3& origin, const glm::dvec3& axis)
    : origin(origin)
    , axis(axis)
{

}

glm::dvec3 operator*(const Ray& ray, double t)
{
    return ray.origin + t * ray.axis;
}
glm::dvec3 operator*(double t, const Ray& ray)
{
    return ray.origin + t * ray.axis;
}

Ray operator*(const glm::dmat4& transform, const Ray& ray)
{
    glm::dvec4 transformed = transform * glm::dvec4(ray.origin.x, ray.origin.y, ray.origin.z, 1);
    return { glm::dvec3(transformed.x, transformed.y, transformed.z), Transformable::rotation(transform) * ray.axis };
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

void Ray::print() const
{
    std::cout << "[Ray] origin: (" << origin.x << ", " << origin.y << ", " << origin.z << "), axis: (" << axis.x << ", " << axis.y << ", " << axis.z << ")\n";
}