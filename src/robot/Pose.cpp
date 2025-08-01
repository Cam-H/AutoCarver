//
// Created by cjhat on 2025-07-28.
//

#include "Pose.h"

#include "geometry/Transformable.h"
#include <iostream>

Pose::Pose(const glm::dvec3& position, const Axis3D& axes)
    : position(position)
    , axes(axes)
{

}

void Pose::localTranslate(const glm::dvec3& translation)
{
    position += axes.delocalize(translation);
}
void Pose::globalTranslate(const glm::dvec3& translation)
{
    position += translation;
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

void Pose::print() const
{
    std::cout << "[Pose] pos: (" << position.x << ", " << position.y << ", " << position.z << ")\n\t";
    axes.print();
}