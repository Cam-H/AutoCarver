//
// Created by cjhat on 2025-07-18.
//

#ifndef AUTOCARVER_RAY_H
#define AUTOCARVER_RAY_H

#include <glm.hpp>

class Ray {
public:

    Ray(const glm::dvec3& origin, const glm::dvec3& axis);

    [[nodiscard]] bool isValid() const;

public:
    glm::dvec3 origin;
    glm::dvec3 axis;

};


#endif //AUTOCARVER_RAY_H
