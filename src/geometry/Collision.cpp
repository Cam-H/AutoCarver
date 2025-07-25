//
// Created by cjhat on 2025-07-18.
//

#include "Collision.h"

#include "shape/Ray.h"
#include "shape/Plane.h"
#include "shape/Triangle.h"
#include "shape/Circle.h"
#include "shape/Sphere.h"
#include "shape/AABB.h"
#include "ConvexHull.h"

bool Collision::test(const Plane& bodyA, const Plane& bodyB)
{
    auto cross = glm::cross(bodyA.normal, bodyB.normal);
    if (glm::dot(cross, cross) > 1e-6) return true; // Non-parallel plane case

    return glm::dot(bodyA.normal, bodyB.origin - bodyA.origin) < 1e-6; // Coplanar case
}

bool Collision::test(const Sphere& bodyA, const Sphere& bodyB)
{
    glm::vec3 delta = bodyA.center - bodyB.center;
    return glm::dot(delta, delta) < pow(bodyA.radius + bodyB.radius, 2);
}

bool Collision::test(const AABB& bodyA, const AABB& bodyB)
{
    return bodyA.max.x > bodyB.min.x && bodyB.max.x > bodyA.min.x
        && bodyA.max.y > bodyB.min.y && bodyB.max.y > bodyA.min.y
        && bodyA.max.z > bodyB.min.z && bodyB.max.z > bodyA.min.z;
}

bool Collision::test(const ConvexHull& bodyA, const ConvexHull& bodyB)
{
    auto simplex = gjk(bodyA, bodyB);
    return simplex.colliding();
}

bool Collision::test(const Plane& bodyA, const Sphere& bodyB)
{
    return test(bodyB, bodyA);
}
bool Collision::test(const Sphere& bodyA, const Plane& bodyB)
{
    glm::vec3 delta = bodyA.center - bodyB.origin;
    return glm::dot(delta, delta) < bodyA.radius * bodyA.radius;
}

bool Collision::test(const Plane& bodyA, const AABB& bodyB)
{
    return test(bodyB, bodyA);
}
bool Collision::test(const AABB& bodyA, const Plane& bodyB)
{
    auto center = 0.5f * (bodyA.min + bodyA.max);
    auto extent = bodyA.max - center;

    float r = std::abs(bodyB.normal.x) * extent.x
            + std::abs(bodyB.normal.y) * extent.y
            + std::abs(bodyB.normal.z) * extent.z;

    return std::abs(glm::dot(bodyB.normal, center) - bodyB.d()) <= r;
}

bool Collision::test(const Plane& bodyA, const ConvexHull& bodyB)
{
    return test(bodyB, bodyA);
}
bool Collision::test(const ConvexHull& bodyA, const Plane& bodyB)
{
    if (bodyA.empty()) return false;

    // Determine whether vertices exist on both sides of the plane
    bool ref = glm::dot(bodyB.normal, bodyA.vertices()[0] - bodyB.origin) > -1e-6;
    for (uint32_t i = 1; i < bodyA.vertexCount(); i++) {
        if ((glm::dot(bodyB.normal, bodyA.vertices()[i] - bodyB.origin) > -1e-6) != ref) return true;
    }

    return false;
}

bool Collision::test(const Sphere& bodyA, const AABB& bodyB)
{
    return test(bodyB, bodyA);
}
bool Collision::test(const AABB& bodyA, const Sphere& bodyB)
{
    glm::vec3 delta = bodyB.center - nearest(bodyA, bodyB.center);
    return glm::dot(delta, delta) < bodyB.radius * bodyB.radius;
}

bool Collision::test(const Sphere& bodyA, const ConvexHull& bodyB)
{
    return test(bodyB, bodyA);
}
bool Collision::test(const ConvexHull& bodyA, const Sphere& bodyB)
{
    auto simplex = gjk(bodyA, bodyB);
    return simplex.colliding();
}

bool Collision::test(const AABB& bodyA, const ConvexHull& bodyB)
{
    return test(bodyB, bodyA);
}
bool Collision::test(const ConvexHull& bodyA, const AABB& bodyB)
{
    auto simplex = gjk(bodyA, bodyB);
    return simplex.colliding();
}

glm::vec3 Collision::nearest(const AABB& body, const glm::vec3& reference)
{
    return {
            std::clamp(reference.x, body.min.x, body.max.x),
            std::clamp(reference.y, body.min.y, body.max.y),
            std::clamp(reference.z, body.min.z, body.max.z)
    };
}

glm::vec3 Collision::farthest(const AABB& body, const glm::vec3& reference)
{
    glm::vec3 center = 0.5f * (body.min + body.max);

    return {
            (center.x > reference.x ? body.max.x : body.min.x),
            (center.y > reference.y ? body.max.y : body.min.y),
            (center.z > reference.z ? body.max.z : body.min.z)
    };
}

bool Collision::encloses(const Sphere& body, const glm::vec3& vertex)
{
    glm::vec3 delta = vertex - body.center;
    return glm::dot(delta, delta) - body.radius * body.radius < 1e-6;
}

bool Collision::encloses(const Sphere& bodyA, const Sphere& bodyB)
{
    glm::vec3 delta = bodyA.center - bodyB.center;
    return glm::dot(delta, delta) < pow(bodyA.radius - bodyB.radius, 2);
}


bool Collision::encloses(const Sphere& bodyA, const AABB& bodyB)
{
    return encloses(bodyB, bodyA);

}
bool Collision::encloses(const AABB& bodyA, const Sphere& bodyB)
{
    glm::vec3 delta = bodyB.center - farthest(bodyA, bodyB.center);
    return glm::dot(delta, delta) < bodyB.radius * bodyB.radius;
}

glm::vec3 Collision::initialAxis(const ConvexHull& bodyA, const ConvexHull& bodyB, std::pair<uint32_t, uint32_t>& idx)
{
    if (idx.first != std::numeric_limits<uint32_t>::max() && idx.second != std::numeric_limits<uint32_t>::max()) {
        return bodyA.vertices()[idx.first] - bodyB.vertices()[idx.second];
    }

    idx = { 0, 0 };

    return bodyA.center() - bodyB.center();
}

glm::vec3 Collision::initialAxis(const Sphere& bodyA, const ConvexHull& bodyB, std::pair<uint32_t, uint32_t>& idx)
{
    idx = { idx.second, idx.first };
    auto axis = -initialAxis(bodyB, bodyA, idx);
    idx = { idx.second, idx.first };

    return axis;
}
glm::vec3 Collision::initialAxis(const ConvexHull& bodyA, const Sphere& bodyB, std::pair<uint32_t, uint32_t>& idx)
{
    if (idx.first != std::numeric_limits<uint32_t>::max()) {
        return bodyA.vertices()[idx.first] - bodyB.center;
    }

    idx = { 0, 0 };

    return bodyA.center() - bodyB.center;
}

glm::vec3 Collision::initialAxis(const AABB& bodyA, const ConvexHull& bodyB, std::pair<uint32_t, uint32_t>& idx)
{
    idx = { idx.second, idx.first };
    auto axis = -initialAxis(bodyB, bodyA, idx);
    idx = { idx.second, idx.first };

    return axis;
}
glm::vec3 Collision::initialAxis(const ConvexHull& bodyA, const AABB& bodyB, std::pair<uint32_t, uint32_t>& idx)
{
    if (idx.first != std::numeric_limits<uint32_t>::max()) {
        return bodyA.vertices()[idx.first] - bodyB.min;
    }

    idx = { 0, 0 };

    return bodyA.center() - bodyB.min;
}

glm::vec3 Collision::gjkSupport(const Sphere& bodyA, const ConvexHull& bodyB, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx)
{
    idx = { idx.second, idx.first };
    auto support = -gjkSupport(bodyB, bodyA, transform, axis, idx);
    idx = { idx.second, idx.first };

    return support;
}

glm::vec3 Collision::gjkSupport(const ConvexHull& bodyA, const Sphere& bodyB, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx)
{
    idx.first = bodyA.walk(axis, idx.first);
//    idx.second = 0;

    glm::vec4 vec = transform * glm::vec4(bodyB.center.x, bodyB.center.y, bodyB.center.z, 1.0f);
    glm::vec3 nearest = glm::vec3{ vec.x, vec.y, vec.z } - glm::normalize(axis) * bodyB.radius;

    return bodyA.vertices()[idx.first] - nearest;
}

glm::vec3 Collision::gjkSupport(const AABB& bodyA, const ConvexHull& bodyB, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx)
{
    idx = { idx.second, idx.first };
    auto support = -gjkSupport(bodyB, bodyA, transform, axis, idx);
    idx = { idx.second, idx.first };

    return support;
}
glm::vec3 Collision::gjkSupport(const ConvexHull& bodyA, const AABB& bodyB, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx)
{
    idx.first = bodyA.walk(axis, idx.first);

    glm::vec3 vertex = bodyB.extreme(-axis * glm::mat3(transform));
    glm::vec4 vec = transform * glm::vec4(vertex.x, vertex.y, vertex.z, 1.0f);

    return bodyA.vertices()[idx.first] - glm::vec3{ vec.x, vec.y, vec.z };
}

glm::vec3 Collision::gjkSupport(const ConvexHull& bodyA, const ConvexHull& bodyB, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx)
{
    idx.first = bodyA.walk(axis, idx.first);
    idx.second = bodyB.walk(-axis * glm::mat3(transform), idx.second);

    glm::vec4 vec = transform * glm::vec4(bodyB.vertices()[idx.second].x, bodyB.vertices()[idx.second].y, bodyB.vertices()[idx.second].z, 1.0f);

    return bodyA.vertices()[idx.first] - glm::vec3{ vec.x, vec.y, vec.z };
}

std::tuple<bool, float, glm::vec3> Collision::intersection(const AABB& body, const Ray& ray)
{
    float tMin = std::numeric_limits<float>::lowest(), tMax = std::numeric_limits<float>::max();

    for (uint8_t i = 0; i < 3; i++) {
        if (ray.axis[i] * ray.axis[i] < 1e-6) { // Handle parallel ray case
            if (ray.origin[i] < body.min[i] || ray.origin[i] > body.max[i]) return { false, 0, {} };
        } else {
            float ood = 1.0f / ray.axis[i];
            float t1 = (body.min[i] - ray.origin[i]) * ood;
            float t2 = (body.max[i] - ray.origin[i]) * ood;
            if (t1 > t2) std::swap(t1, t2);

            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);

            if (tMin > tMax) return { false, 0, {} };
        }
    }

    // Ray intersects
    return { true, tMin, ray.origin + ray.axis * tMin };
}

// TODO Improvement: Leverage walk to calculate intersection in-place
std::vector<glm::vec3> Collision::intersection(const ConvexHull& hull, const Plane& plane)
{
    if (hull.vertexCount() < 4) return {}; // Early exit when the hull is poorly formed

    const auto& [above, divided] = partition(hull, plane);
    if (!divided) return {}; // Early exit when intersection is impossible (hull does not pass through the plane)

    return intersection(hull, plane, above);
}

std::vector<glm::vec3> Collision::intersection(const ConvexHull& hull, const Plane& plane, const std::vector<bool>& partition)
{
    float d = plane.d();

    // Find vertices on the cut plane
    std::vector<glm::vec3> intersection;
    for (uint32_t i = 0; i < hull.vertexCount(); i++) {
        for (uint32_t j : hull.neighbors(i)) {
            if (i < j && partition[i] != partition[j]) { // Skip repeated edges, edges that do not cross the plane
                glm::vec3 vertex = hull.vertices()[i], edge = hull.vertices()[j] - vertex;
                float t = (d - glm::dot(plane.normal, vertex)) / glm::dot(plane.normal, edge);
                intersection.emplace_back(vertex + edge * t);
            }
        }
    }

    return intersection;
}

bool Collision::above(const ConvexHull& hull, const Plane& plane)
{
    const auto& [above, divided] = partition(hull, plane);
    return !divided && above[0];
}
bool Collision::below(const ConvexHull& hull, const Plane& plane)
{
    const auto& [below, divided] = partition(hull, { plane.origin, -plane.normal });
    return !divided && below[0];
}

// Indicates whether each vertex is above or below the plane in the returned vector
// Returns false if all vertices are either above or below the plane
std::tuple<std::vector<bool>, bool> Collision::partition(const ConvexHull& hull, const Plane& plane)
{
    uint32_t sum = 0;

    std::tuple<std::vector<bool>, bool> out = { std::vector<bool>(hull.vertexCount()), false };
    auto& above = std::get<0>(out);

    // Determine which vertices are above the cut plane
    const auto& vertices = hull.vertices();
    for (uint32_t i = 0; i < vertices.size(); i++) {
        above[i] = glm::dot(plane.normal, vertices[i] - plane.origin) > -1e-6;
        sum += above[i];
    }

    // Indicate spread of the vertices
    std::get<1>(out) = sum != 0 && sum != above.size();

    return out;
}

ConvexHull Collision::fragment(const ConvexHull& hull, const Plane& plane)
{
    if (hull.empty()) return {};

    const auto& [above, divided] = partition(hull, plane);
    if (!divided) {

        // Exit if no edges intersect with the plane
        if (above[0]) return hull;
        else return {};
    }

    // Find vertex intersections of hull with plane
    std::vector<glm::vec3> set = intersection(hull, plane, above);

    // Attach vertices on the positive side of the plane
    for (uint32_t i = 0; i < above.size(); i++) if (above[i]) set.push_back(hull.vertices()[i]);

    return { VertexArray::clean(set) };
}
std::pair<ConvexHull, ConvexHull> Collision::fragments(const ConvexHull& hull, const Plane& plane)
{
    if (hull.empty()) return {};

    const auto& [above, divided] = partition(hull, plane);
    if (!divided) {

        // Exit if no edges intersect with the plane
        if (above[0]) return { hull, {} };
        else return { {}, hull };
    }

    std::vector<glm::vec3> setA = intersection(hull, plane, above);

    // Duplicate intersection vertices in other set
    std::vector<glm::vec3> setB = setA;

    // Attach original vertices to respective fragments
    for (uint32_t i = 0; i < above.size(); i++) {
        if (above[i]) setA.push_back(hull.vertices()[i]);
        else setB.push_back(hull.vertices()[i]);
    }

    return { VertexArray::clean(setA), VertexArray::clean(setB) };
}

//template<class T1, class T2>
//bool CollisionPair<T1, T2>::intersects() const
//{
//    return Collision::intersects(first, second);
//}


