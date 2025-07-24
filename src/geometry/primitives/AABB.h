//
// Created by cjhat on 2025-07-18.
//

#ifndef AUTOCARVER_AABB_H
#define AUTOCARVER_AABB_H

#include <glm.hpp>

class ConvexHull;

class AABB {
public:

    AABB(const glm::dvec3& min, double sideLength);
    AABB(const glm::dvec3& min, const glm::dvec3& max);

    explicit AABB(const ConvexHull& hull);
    explicit AABB(const std::vector<glm::dvec3>& vertices);

    [[nodiscard]] bool isValid() const;

    [[nodiscard]] double xLength() const;
    [[nodiscard]] double yLength() const;
    [[nodiscard]] double zLength() const;
    [[nodiscard]] double maxLength() const;

    [[nodiscard]] glm::dvec3 center() const;
    [[nodiscard]] glm::dvec3 vertex(uint32_t index) const;

    // Required functions for GJK collision tests
    [[nodiscard]] glm::dvec3 start() const;
    [[nodiscard]] uint32_t supportIndex(const glm::dvec3& axis, uint32_t startIndex = 0) const;
    [[nodiscard]] std::tuple<uint32_t, glm::dvec3> extreme(const glm::dvec3& axis, uint32_t startIndex = 0) const;

    void print() const;

public:
    glm::dvec3 min;
    glm::dvec3 max;
};


#endif //AUTOCARVER_AABB_H
