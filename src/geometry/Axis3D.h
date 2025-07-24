//
// Created by cjhat on 2025-07-21.
//

#ifndef AUTOCARVER_AXIS3D_H
#define AUTOCARVER_AXIS3D_H

#include <glm.hpp>

class Axis3D {
public:

    Axis3D();
    explicit Axis3D(const glm::dvec3& axis);
    Axis3D(const glm::dvec3& xAxis, const glm::dvec3& yAxis);
    Axis3D(const glm::dvec3& xAxis, const glm::dvec3& yAxis, const glm::dvec3& zAxis);

    [[nodiscard]] glm::dmat3 toTransform() const;
    [[nodiscard]] glm::dquat toQuat() const;

    void print() const;

public:
    glm::dvec3 xAxis;
    glm::dvec3 yAxis;
    glm::dvec3 zAxis;

};


#endif //AUTOCARVER_AXIS3D_H
