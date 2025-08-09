//
// Created by cjhat on 2025-07-18.
//

#ifndef AUTOCARVER_RAY_H
#define AUTOCARVER_RAY_H

#include <glm.hpp>

class Ray {
public:

    Ray();
    Ray(const glm::dvec3& origin, const glm::dvec3& axis);

    friend glm::dvec3 operator*(const Ray& ray, double t);
    friend glm::dvec3 operator*(double t, const Ray& ray);

    friend Ray operator*(const glm::dmat4& transform, const Ray& ray);

    [[nodiscard]] bool isValid() const;

    [[nodiscard]] double axialRotation(const glm::dvec3& axis) const;
    static double axialRotation(const glm::dvec3& reference, const glm::dvec3& axis);

    [[nodiscard]] double squareDistance(const glm::dvec3& vertex) const;

    void print() const;

public:
    glm::dvec3 origin;
    glm::dvec3 axis;

};


#endif //AUTOCARVER_RAY_H
