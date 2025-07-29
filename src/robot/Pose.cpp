//
// Created by cjhat on 2025-07-28.
//

#include "Pose.h"

#include "geometry/Transformable.h"

Pose::Pose(const glm::dvec3& position, const Axis3D& axes)
    : position(position)
    , axes(axes)
{

}

bool operator==(const Pose& lhs, const Pose& rhs)
{
    glm::dvec3 delta = lhs.position - rhs.position;
    return glm::dot(delta, delta) < 1e-12 && lhs.axes == rhs.axes;
}
bool operator!=(const Pose& lhs, const Pose& rhs)
{
    return !(lhs == rhs);
}

Pose operator*(const glm::dmat4& lhs, const Pose& rhs)
{
    glm::dvec4 transformed = lhs * glm::dvec4(rhs.position.x, rhs.position.y, rhs.position.z, 1);
    return { glm::dvec3(transformed.x, transformed.y, transformed.z), rhs.axes * Transformable::rotation(lhs) };
}
// TODO validate/complete
//Pose operator*(const Pose& lhs, const glm::dmat4& rhs)
//{
//
//}


bool Pose::oriented(const Pose& pose, double tolerance) const
{
    return oriented(pose.axes, tolerance);
}
bool Pose::oriented(const Axis3D& system, double tolerance) const
{
    return Axis3D::compare(axes, system, tolerance);
}

Pose Pose::translated(const glm::dvec3& translation) const
{
    return { position + translation, axes };
}