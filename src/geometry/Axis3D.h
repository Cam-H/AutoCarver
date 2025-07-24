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
    Axis3D(const glm::dmat3& matrix);

    Axis3D operator*(const glm::dquat& rotation) const;


    void rotateY();

    void rotateX(double theta);
    void rotateY(double theta);
    void rotateZ(double theta);

    void rotate(const glm::dvec3& axis, double theta);
    void rotate(const glm::dquat& rotation);

    [[nodiscard]] bool isValid() const;

    [[nodiscard]] glm::dmat3 toTransform() const;
    [[nodiscard]] glm::dquat toQuat() const;

    void print() const;

public:
    glm::dvec3 xAxis;
    glm::dvec3 yAxis;
    glm::dvec3 zAxis;

};


#endif //AUTOCARVER_AXIS3D_H
