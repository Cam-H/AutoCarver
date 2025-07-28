//
// Created by cjhat on 2025-07-18.
//

#ifndef AUTOCARVER_PLANE_H
#define AUTOCARVER_PLANE_H

#include "glm.hpp"

class Plane {
public:

    Plane();
    Plane(const glm::dvec3& origin, const glm::dvec3& normal);

    void rotate(const glm::dvec3& axis, double theta);
    void invert();

    [[nodiscard]] bool isValid() const;

    [[nodiscard]] double axialRotation(const glm::dvec3& axis) const;

    [[nodiscard]] Plane rotated(const glm::dvec3& axis, double theta) const;
    [[nodiscard]] Plane inverted() const;

    [[nodiscard]] glm::dvec3 project(const glm::dvec3& vertex) const;
    inline static glm::dvec3 project(const glm::dvec3& origin, const glm::dvec3& normal, const glm::dvec3& vertex);
    inline static glm::dvec3 project(const Plane& plane, const glm::dvec3& vertex);

    [[nodiscard]] inline double d() const { return glm::dot(origin, normal); }

    void print() const;

public:
    glm::dvec3 origin;
    glm::dvec3 normal;
};


#endif //AUTOCARVER_PLANE_H
