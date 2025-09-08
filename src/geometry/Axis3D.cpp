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

bool operator==(const Axis3D& lhs, const Axis3D& rhs)
{
    return Axis3D::compare(lhs, rhs, 1e-12);
}
bool operator!=(const Axis3D& lhs, const Axis3D& rhs)
{
    return !(lhs == rhs);
}

Axis3D operator*(const Axis3D& axes, const glm::dmat3& rotation)
{
    return axes * glm::quat_cast(rotation);
}

Axis3D operator*(const Axis3D& axes, const glm::dquat& rotation)
{
    Axis3D system = axes;
    system.rotate(rotation);
    return system;
}

// Develop axes aligned to a face [by face normal] (x along cut direction, y normal to plane, z along blade)
Axis3D Axis3D::faceAligned(const glm::dvec3& normal, const glm::dvec3& direction, bool alignHorizontal)
{
    Axis3D axes(normal);
    axes.rotateX(-M_PI / 2);

    // Bring the zAxis into the horizontal plane
    if (alignHorizontal) {
        double planeAngle = atan2(-glm::dot(axes.zAxis, UP), glm::dot(axes.xAxis, UP));
        axes.rotateY(-planeAngle);
    }

    // Ensure zAxis points in the correct direction
    if (glm::dot(axes.zAxis, direction) < 0) axes.flipXZ();

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

void Axis3D::flipXY()
{
    xAxis = -xAxis;
    yAxis = -yAxis;
}

void Axis3D::flipXZ()
{
    xAxis = -xAxis;
    zAxis = -zAxis;
}
void Axis3D::flipYZ()
{
    yAxis = -yAxis;
    zAxis = -zAxis;
}

bool Axis3D::isValid() const
{
    return checkAxis(xAxis, xAxis, 1e-12)
        && checkAxis(yAxis, yAxis, 1e-12)
        && checkAxis(zAxis, zAxis, 1e-12)
        && checkAxis(glm::cross(xAxis, yAxis), zAxis, 1e-12);
}

bool Axis3D::compare(const Axis3D& lhs, const Axis3D& rhs, double tolerance)
{
    return checkAxis(lhs.xAxis, rhs.xAxis, tolerance)
        && checkAxis(lhs.yAxis, rhs.yAxis, tolerance)
        && checkAxis(lhs.zAxis, rhs.zAxis, tolerance);
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

glm::dquat Axis3D::relative(const Axis3D& axes) const
{
    return axes.toTransform() * glm::transpose(toTransform());
}

// Expresses the provided vertex in terms of the coordinate system
glm::dvec3 Axis3D::localize(const glm::dvec3& vertex) const
{
    return {
        glm::dot(vertex, xAxis),
        glm::dot(vertex, yAxis),
        glm::dot(vertex, zAxis)
    };
}

// Provided a vertex in this coordinate system, recovers coordinates in the world (I) system
glm::dvec3 Axis3D::delocalize(const glm::dvec3& vertex) const
{
    return xAxis * vertex.x + yAxis * vertex.y + zAxis * vertex.z;
}

void Axis3D::print() const
{
    std::cout << "[Axis3D] x-axis: (" << xAxis.x << ", " << xAxis.y << ", " << xAxis.z
                    << "), y-axis: (" << yAxis.x << ", " << yAxis.y << ", " << yAxis.z
                    << "), z-axis: (" << zAxis.x << ", " << zAxis.y << ", " << zAxis.z << ")\n";
}