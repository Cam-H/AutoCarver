//
// Created by cjhat on 2025-07-21.
//

#include "Axis3D.h"

#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/quaternion.hpp>

Axis3D::Axis3D()
    : xAxis(1, 0, 0)
    , yAxis(0, 1, 0)
    , zAxis(0, 0, 1)
{

}

Axis3D::Axis3D(const glm::dvec3& axis)
    : zAxis(axis)
{
    xAxis = axis.x * axis.x > 0.99f ? glm::dvec3(0, 0, 1) : glm::dvec3(1, 0, 0);
    xAxis = glm::normalize(glm::cross(zAxis, xAxis));

    yAxis = glm::normalize(glm::cross(zAxis, xAxis));
}

Axis3D::Axis3D(const glm::dvec3& xAxis, const glm::dvec3& yAxis)
    : xAxis(xAxis)
    , yAxis(yAxis)
    , zAxis(glm::normalize(glm::cross(xAxis, yAxis)))
{

}
Axis3D::Axis3D(const glm::dvec3& xAxis, const glm::dvec3& yAxis, const glm::dvec3& zAxis)
    : xAxis(xAxis)
    , yAxis(yAxis)
    , zAxis(zAxis)
{

}

glm::dmat3 Axis3D::toTransform() const
{
    return {
        xAxis, yAxis, zAxis
    };
}

glm::dquat Axis3D::toQuat() const
{
    return glm::quat_cast(toTransform());
}

void Axis3D::print() const
{
    std::cout << "[Axis3D] x-axis: (" << xAxis.x << ", " << xAxis.y << ", " << xAxis.z
                    << "), y-axis: (" << yAxis.x << ", " << yAxis.y << ", " << yAxis.z
                    << "), z-axis: (" << zAxis.x << ", " << zAxis.y << ", " << zAxis.z << ")\n";
}