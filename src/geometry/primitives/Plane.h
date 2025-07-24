//
// Created by cjhat on 2025-07-18.
//

#ifndef AUTOCARVER_PLANE_H
#define AUTOCARVER_PLANE_H

#include "glm.hpp"

class Plane {
public:

    Plane();
    Plane(const glm::vec3& origin, const glm::vec3& normal);

    void rotate(const glm::vec3& axis, float theta);

    [[nodiscard]] bool isValid() const;

    [[nodiscard]] glm::vec3 project(const glm::vec3& vertex) const;
    inline static glm::vec3 project(const glm::vec3& origin, const glm::vec3& normal, const glm::vec3& vertex);
    inline static glm::vec3 project(const Plane& plane, const glm::vec3& vertex);

    [[nodiscard]] inline float d() const { return glm::dot(origin, normal); }

    void print() const;

public:
    glm::vec3 origin;
    glm::vec3 normal;
};


#endif //AUTOCARVER_PLANE_H
