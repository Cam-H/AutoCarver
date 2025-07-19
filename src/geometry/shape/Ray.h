//
// Created by cjhat on 2025-07-18.
//

#ifndef AUTOCARVER_RAY_H
#define AUTOCARVER_RAY_H

#include <glm.hpp>

class Ray {
public:

    Ray(const glm::vec3& origin, const glm::vec3& axis);

public:
    glm::vec3 origin;
    glm::vec3 axis;

};


#endif //AUTOCARVER_RAY_H
