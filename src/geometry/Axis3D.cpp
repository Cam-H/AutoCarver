//
// Created by cjhat on 2025-07-21.
//

#include "Axis3D.h"

#include <iostream>

Axis3D::Axis3D()
    : xAxis(1, 0, 0)
    , yAxis(0, 1, 0)
    , zAxis(0, 0, 1)
{

}

Axis3D::Axis3D(const glm::vec3& axis)
    : zAxis(axis)
{
    xAxis = axis.x * axis.x > 0.99f ? glm::vec3(0, 0, 1) : glm::vec3(1, 0, 0);
    xAxis = glm::normalize(glm::cross(zAxis, xAxis));

    yAxis = glm::normalize(glm::cross(zAxis, xAxis));
}

Axis3D::Axis3D(const glm::vec3& xAxis, const glm::vec3& yAxis)
    : xAxis(xAxis)
    , yAxis(yAxis)
    , zAxis(glm::normalize(glm::cross(xAxis, yAxis)))
{

}
Axis3D::Axis3D(const glm::vec3& xAxis, const glm::vec3& yAxis, const glm::vec3& zAxis)
    : xAxis(xAxis)
    , yAxis(yAxis)
    , zAxis(zAxis)
{

}

void Axis3D::print() const
{
    std::cout << "[Axis3D] x-axis: (" << xAxis.x << ", " << xAxis.y << ", " << xAxis.z
                    << "), y-axis: (" << yAxis.x << ", " << yAxis.y << ", " << yAxis.z
                    << "), z-axis: (" << zAxis.x << ", " << zAxis.y << ", " << zAxis.z << ")\n";
}