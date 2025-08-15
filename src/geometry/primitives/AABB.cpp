//
// Created by cjhat on 2025-07-18.
//

#include "AABB.h"

#include "ConvexHull.h"
#include "geometry/VertexArray.h"

// Creates a cube, centered on the origin with the given side length
AABB::AABB(double sideLength)
    : min(-0.5 * sideLength, -0.5 * sideLength, -0.5 * sideLength)
    , max( 0.5 * sideLength,  0.5 * sideLength,  0.5 * sideLength)
{

}

AABB::AABB(const glm::dvec3& min, double sideLength)
    : AABB(min, min + glm::dvec3{ sideLength, sideLength, sideLength})
{

}

AABB::AABB(const glm::dvec3& min, const glm::dvec3& max)
    : min(min)
    , max(max)
{

}
AABB::AABB(const ConvexHull& hull)
        : min()
        , max()
{
    double near, far;

    hull.extents({ 1, 0, 0 }, near, far);
    min.x = near;
    max.x = far;

    hull.extents({ 0, 1, 0 }, near, far);
    min.y = near;
    max.y = far;

    hull.extents({ 0, 0, 1 }, near, far);
    min.z = near;
    max.z = far;
}

AABB::AABB(const std::vector<glm::dvec3>& vertices)
        : min()
        , max()
{
    double near, far;

    VertexArray::extents(vertices, { 1, 0, 0 }, near, far);
    min.x = near;
    max.x = far;

    VertexArray::extents(vertices, { 0, 1, 0 }, near, far);
    min.y = near;
    max.y = far;

    VertexArray::extents(vertices, { 0, 0, 1 }, near, far);
    min.z = near;
    max.z = far;
}

bool AABB::isValid() const
{
    return min.x < max.x && min.y < max.y && min.z < max.z;
}

double AABB::xLength() const
{
    return max.x - min.x;
}
double AABB::yLength() const
{
    return max.y - min.y;
}
double AABB::zLength() const
{
    return max.z - min.z;
}

double AABB::maxLength() const
{
    return std::max(xLength(), std::max(yLength(), zLength()));
}

glm::dvec3 AABB::center() const
{
    return 0.5 * (min + max);
}

glm::dvec3 AABB::vertex(uint32_t index) const
{
    switch (index) {
        case 0: return min;
        case 1: return { min.x, min.y, max.z };
        case 2: return { min.x, max.y, min.z };
        case 3: return { min.x, max.y, max.z };
        case 4: return { max.x, min.y, min.z };
        case 5: return { max.x, min.y, max.z };
        case 6: return { max.x, max.y, min.z };
        case 7: return { max.x, max.y, max.z };
        default: throw std::runtime_error("[AABB] Invalid vertex index!");
    }
}


glm::dvec3 AABB::start() const
{
    return center();
}

uint32_t AABB::supportIndex(const glm::dvec3& axis, uint32_t startIndex) const
{
    return 4 * (axis.x < 0) + 2 * (axis.y < 0) + (axis.z < 0);
}

std::tuple<uint32_t, glm::dvec3> AABB::extreme(const glm::dvec3& axis, uint32_t startIndex) const
{
    auto index = supportIndex(axis);
    return { index, vertex(index) };
//    return {
//        axis.x < 0 ? min.x : max.x,
//        axis.y < 0 ? min.y : max.y,
//        axis.z < 0 ? min.z : max.z,
//    };
}

void AABB::print() const
{
    std::cout << "[AABB] x-span: (" << min.x << ", " << max.x
                  << "), y-span: (" << min.y << ", " << max.y
                  << "), z-span: (" << min.z << ", " << max.z << ")\n";
}