//
// Created by cjhat on 2025-07-18.
//

#include "AABB.h"

#include "geometry/ConvexHull.h"
#include "geometry/VertexArray.h"

AABB::AABB(const glm::vec3& min, float sideLength)
    : AABB(min, min + glm::vec3{ sideLength, sideLength, sideLength})
{

}

AABB::AABB(const glm::vec3& min, const glm::vec3& max)
    : min(min)
    , max(max)
{

}
AABB::AABB(const ConvexHull& hull)
    : min()
    , max()
{
    float near, far;

    VertexArray::extents(hull.vertices(), { 1, 0, 0 }, near, far);
    min.x = near;
    max.x = far;

    VertexArray::extents(hull.vertices(), { 0, 1, 0 }, near, far);
    min.y = near;
    max.y = far;

    VertexArray::extents(hull.vertices(), { 0, 0, 1 }, near, far);
    min.z = near;
    max.z = far;

}

glm::vec3 AABB::vertex(uint32_t index) const
{
    switch (index) {
        case 0: return min;
        case 1: return { max.x, min.y, min.z };
        case 2: return { max.x, min.y, max.z };
        case 3: return { min.x, min.y, max.z };
        case 4: return { min.x, max.y, min.z };
        case 5: return { max.x, max.y, min.z };
        case 6: return { max.x, max.y, max.z };
        case 7: return { min.x, max.y, max.z };
        default: throw std::runtime_error("[AABB] Invalid vertex index!");
    }
}

glm::vec3 AABB::extreme(const glm::vec3& axis) const
{
    return {
        axis.x < 0 ? min.x : max.x,
        axis.y < 0 ? min.y : max.y,
        axis.z < 0 ? min.z : max.z,
    };
}