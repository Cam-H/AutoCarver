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

Axis3D::Axis3D(const glm::dmat3& matrix)
    : xAxis(matrix[0][0], matrix[0][1], matrix[0][2])
    , yAxis(matrix[1][0], matrix[1][1], matrix[1][2])
    , zAxis(matrix[2][0], matrix[2][1], matrix[2][2])
{

}

Axis3D Axis3D::operator*(const glm::dquat& rotation) const
{
    Axis3D axes = *this;
    axes.rotate(rotation);
    return axes;
}

// Rotates the system about the Y-axis by -90dg
void Axis3D::rotateY()
{
    glm::dvec3 newX = -zAxis;
    zAxis = xAxis;
    xAxis = newX;
}

void Axis3D::rotateX(double theta)
{
    rotate(xAxis, theta);
}
void Axis3D::rotateY(double theta)
{
    rotate(yAxis, theta);
}
void Axis3D::rotateZ(double theta)
{
    rotate(zAxis, theta);
}

void Axis3D::rotate(const glm::dvec3& axis, double theta)
{
    rotate(glm::angleAxis(theta, axis));
}

void Axis3D::rotate(const glm::dquat& rotation)
{
    xAxis = xAxis * rotation;
    yAxis = yAxis * rotation;
    zAxis = zAxis * rotation;
}

bool Axis3D::isValid() const
{
    glm::dvec3 cross = glm::cross(xAxis, yAxis), len = {
            glm::dot(xAxis, xAxis),
            glm::dot(yAxis, yAxis),
            glm::dot(zAxis, zAxis)
    };

    double cLen = glm::dot(cross, zAxis);

    return 1 - 1e-12 < len.x && len.x < 1 + 1e-12
        && 1 - 1e-12 < len.y && len.y < 1 + 1e-12
        && 1 - 1e-12 < len.z && len.z < 1 + 1e-12
        && 1 - 1e-12 < cLen  && cLen  < 1 + 1e-12;
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