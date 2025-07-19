//
// Created by cjhat on 2025-07-18.
//

#ifndef AUTOCARVER_AABB_H
#define AUTOCARVER_AABB_H

#include <glm.hpp>

class ConvexHull;

class AABB {
public:

    AABB(const glm::vec3& min, float sideLength);
    AABB(const glm::vec3& min, const glm::vec3& max);
    AABB(const ConvexHull& hull);

    float xLength() const;
    float yLength() const;
    float zLength() const;

    [[nodiscard]] glm::vec3 center() const;

    [[nodiscard]] glm::vec3 vertex(uint32_t index) const;
    [[nodiscard]] glm::vec3 extreme(const glm::vec3& axis) const;

public:
    glm::vec3 min;
    glm::vec3 max;
};


#endif //AUTOCARVER_AABB_H
