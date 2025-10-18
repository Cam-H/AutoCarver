//
// Created by cjhat on 2025-07-18.
//

#include "Ray.h"
#include "geometry/Transformable.h"

#include <iostream>

template<typename Derived, uint16_t dims>
RayBase<Derived, dims>::RayBase(const vec& origin, const vec& axis)
        : origin(origin)
        , axis(axis)
{

}

template<typename Derived, uint16_t dims>
void RayBase<Derived, dims>::reverse()
{
    axis = -axis;
}

template<typename Derived, uint16_t dims>
bool RayBase<Derived, dims>::isValid() const
{
    double len = glm::dot(axis, axis);
    return 1 - 1e-12 < len && len < 1 + 1e-12;
}

template<typename Derived, uint16_t dims>
double RayBase<Derived, dims>::squareDistance(const vec& vertex) const
{
    vec delta = vertex - origin;
    delta -= axis * glm::dot(axis, delta);
    return glm::dot(delta, delta);
}



Ray<2>::Ray() : RayBase<Ray<2>, 2>({}, { 0, 1 }) {}

// Creates a forward pointing ray. The origin (x, y) and location.z represents the angle from x+
Ray<2>::Ray(const glm::dvec3& location) : RayBase<Ray<2>, 2>({ location.x, location.y }, { cos(location.z), sin(location.z) }) {}

Ray<2>::Ray(const glm::dvec2& origin, const glm::dvec2& axis) : RayBase<Ray<2>, 2>(origin, axis) {}

double Ray<2>::angle() const
{
    return atan2(axis.y, axis.x);
}

Ray<2> Ray<2>::normal() const
{
    return { origin, { axis.y, -axis.x }};
}

void Ray<2>::print() const
{
    std::cout << "[RayD] origin: (" << origin.x << ", " << origin.y
              << "), axis: (" << axis.x << ", " << axis.y << ")\n";
}



Ray<3>::Ray() : RayBase<Ray<3>, 3>({}, { 0, 0, 1 }) {}
Ray<3>::Ray(const glm::dvec3& origin, const glm::dvec3& axis) : RayBase<Ray<3>, 3>(origin, axis) {}

Ray<3> operator*(const glm::dmat4& transform, const Ray<3>& ray)
{
    glm::dvec4 transformed = transform * glm::dvec4(ray.origin.x, ray.origin.y, ray.origin.z, 1);
    return { { transformed.x, transformed.y, transformed.z }, Transformable::rotation(transform) * ray.axis };
}

double Ray<3>::axialRotation(const glm::dvec3& reference, const glm::dvec3& axis)
{
    glm::dvec3 proj = axis - reference * glm::dot(axis, reference);
    return atan2(proj.x, -proj.z);
}

// Calculate the rotation of the ray axis about the specified reference axis
double Ray<3>::axialRotation(const glm::dvec3& reference) const
{
    return axialRotation(reference, axis);
}

void Ray<3>::print() const
{
    std::cout << "[Ray3D] origin: (" << origin.x << ", " << origin.y << ", " << origin.z
                     << "), axis: (" << axis.x << ", " << axis.y << ", " << axis.z << ")\n";
}