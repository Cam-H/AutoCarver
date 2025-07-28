//
// Created by cjhat on 2025-07-18.
//

#include "Collision.h"

#include "geometry/primitives/Ray.h"
#include "geometry/primitives/Plane.h"
#include "geometry/primitives/Triangle.h"
#include "geometry/primitives/Circle.h"
#include "geometry/primitives/Sphere.h"
#include "geometry/primitives/AABB.h"
#include "geometry/primitives/ConvexHull.h"
#include "EPA.h"

bool Collision::test(const Plane& bodyA, const Plane& bodyB)
{
    auto cross = glm::cross(bodyA.normal, bodyB.normal);
    if (glm::dot(cross, cross) > 1e-12) return true; // Non-parallel plane case

    return glm::dot(bodyA.normal, bodyB.origin - bodyA.origin) < 1e-12; // Coplanar case
}

bool Collision::test(const Sphere& bodyA, const Sphere& bodyB)
{
    glm::dvec3 delta = bodyA.center - bodyB.center;
    return glm::dot(delta, delta) < pow(bodyA.radius + bodyB.radius, 2);
}

bool Collision::test(const AABB& bodyA, const AABB& bodyB)
{
    return bodyA.max.x > bodyB.min.x && bodyB.max.x > bodyA.min.x
        && bodyA.max.y > bodyB.min.y && bodyB.max.y > bodyA.min.y
        && bodyA.max.z > bodyB.min.z && bodyB.max.z > bodyA.min.z;
}

bool Collision::test(const Plane& bodyA, const Sphere& bodyB)
{
    return test(bodyB, bodyA);
}
bool Collision::test(const Sphere& bodyA, const Plane& bodyB)
{
    glm::dvec3 delta = bodyA.center - bodyB.origin;
    return glm::dot(delta, delta) < bodyA.radius * bodyA.radius;
}

bool Collision::test(const Plane& bodyA, const AABB& bodyB)
{
    return test(bodyB, bodyA);
}
bool Collision::test(const AABB& bodyA, const Plane& bodyB)
{
    auto center = 0.5 * (bodyA.min + bodyA.max);
    auto extent = bodyA.max - center;

    double r = std::abs(bodyB.normal.x) * extent.x
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
    bool ref = glm::dot(bodyB.normal, bodyA.vertices()[0] - bodyB.origin) > -1e-12;
    for (uint32_t i = 1; i < bodyA.vertexCount(); i++) {
        if ((glm::dot(bodyB.normal, bodyA.vertices()[i] - bodyB.origin) > -1e-12) != ref) return true;
    }

    return false;
}

bool Collision::test(const Sphere& bodyA, const AABB& bodyB)
{
    return test(bodyB, bodyA);
}
bool Collision::test(const AABB& bodyA, const Sphere& bodyB)
{
    glm::dvec3 delta = bodyB.center - nearest(bodyA, bodyB.center);
    return glm::dot(delta, delta) < bodyB.radius * bodyB.radius;
}

glm::dvec3 Collision::nearest(const AABB& body, const glm::dvec3& reference)
{
    return {
            std::clamp(reference.x, body.min.x, body.max.x),
            std::clamp(reference.y, body.min.y, body.max.y),
            std::clamp(reference.z, body.min.z, body.max.z)
    };
}

glm::dvec3 Collision::farthest(const AABB& body, const glm::dvec3& reference)
{
    glm::dvec3 center = 0.5 * (body.min + body.max);

    return {
            (center.x > reference.x ? body.max.x : body.min.x),
            (center.y > reference.y ? body.max.y : body.min.y),
            (center.z > reference.z ? body.max.z : body.min.z)
    };
}

bool Collision::encloses(const Sphere& body, const glm::dvec3& vertex)
{
    glm::dvec3 delta = vertex - body.center;
    return glm::dot(delta, delta) - body.radius * body.radius < 1e-12;
}

bool Collision::encloses(const AABB& body, const glm::dvec3& vertex)
{
    return body.min.x < vertex.x && body.max.x > vertex.x
        && body.min.y < vertex.y && body.max.y > vertex.y
        && body.min.z < vertex.z && body.max.z > vertex.z;
}

bool Collision::encloses(const ConvexHull& body, const glm::dvec3& vertex)
{
    for (uint32_t i = 0; i < body.facetCount(); i++) {
        const Plane& plane = body.facePlane(i);
        if (glm::dot(plane.normal, vertex - plane.origin) > 0) return false;
    }

    return true;
}

bool Collision::encloses(const Sphere& bodyA, const Sphere& bodyB)
{
    glm::dvec3 delta = bodyA.center - bodyB.center;
    return glm::dot(delta, delta) < pow(bodyA.radius - bodyB.radius, 2);
}

// Test whether bodyA encloses bodyB
bool Collision::encloses(const AABB& bodyA, const AABB& bodyB)
{
    return bodyA.min.x < bodyB.min.x && bodyA.max.x > bodyB.max.x
        && bodyA.min.y < bodyB.min.y && bodyA.max.y > bodyB.max.y
        && bodyA.min.z < bodyB.min.z && bodyA.max.z > bodyB.max.z;
}

// Test whether the sphere encloses the AABB
bool Collision::encloses(const Sphere& bodyA, const AABB& bodyB)
{
    glm::dvec3 delta = bodyA.center - farthest(bodyB, bodyA.center);
    return glm::dot(delta, delta) < bodyA.radius * bodyA.radius;
}
// Test whether the AABB encloses the sphere
bool Collision::encloses(const AABB& bodyA, const Sphere& bodyB)
{
    glm::dvec3 delta = bodyB.center - bodyA.center();
    return std::abs(delta.x) < 0.5f * bodyA.xLength() - bodyB.radius
        && std::abs(delta.y) < 0.5f * bodyA.yLength() - bodyB.radius
        && std::abs(delta.z) < 0.5f * bodyA.zLength() - bodyB.radius;
}

bool Collision::encloses(const Sphere& bodyA, const ConvexHull& bodyB)
{
    return encloses(bodyA, bodyB.vertices());
}

bool Collision::encloses(const AABB& bodyA, const ConvexHull& bodyB)
{
    return encloses(bodyA, AABB(bodyB));
}

bool Collision::encloses(const ConvexHull& bodyA, const Sphere& bodyB)
{
    for (uint32_t i = 0; i < bodyA.facetCount(); i++) {
        const Plane& plane = bodyA.facePlane(i);
        if (glm::dot(-plane.normal, bodyB.center - plane.origin) < bodyB.radius) return false;
    }

    return true;
}
bool Collision::encloses(const ConvexHull& bodyA, const AABB& bodyB)
{
    return encloses(bodyA, bodyB.min)
        && encloses(bodyA, bodyB.max)
        && encloses(bodyA, bodyB.vertex(1))
        && encloses(bodyA, bodyB.vertex(2))
        && encloses(bodyA, bodyB.vertex(3))
        && encloses(bodyA, bodyB.vertex(4))
        && encloses(bodyA, bodyB.vertex(5))
        && encloses(bodyA, bodyB.vertex(6));
}

std::tuple<bool, double, glm::dvec3> Collision::intersection(const AABB& body, const Ray& ray)
{
    double tMin = std::numeric_limits<double>::lowest(), tMax = std::numeric_limits<double>::max();

    for (uint8_t i = 0; i < 3; i++) {
        if (ray.axis[i] * ray.axis[i] < 1e-12) { // Handle parallel ray case
            if (ray.origin[i] < body.min[i] || ray.origin[i] > body.max[i]) return { false, 0, {} };
        } else {
            double ood = 1.0f / ray.axis[i];
            double t1 = (body.min[i] - ray.origin[i]) * ood;
            double t2 = (body.max[i] - ray.origin[i]) * ood;
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
std::vector<glm::dvec3> Collision::intersection(const ConvexHull& hull, const Plane& plane)
{
    if (hull.vertexCount() < 4) return {}; // Early exit when the hull is poorly formed

    const auto& [above, divided] = partition(hull, plane);
    std::cout << above[0] << " " << above[1] << " " << divided << "STATE\n";
    if (!divided) return {}; // Early exit when intersection is impossible (hull does not pass through the plane)

    return intersection(hull, plane, above);
}

std::vector<glm::dvec3> Collision::intersection(const ConvexHull& hull, const Plane& plane, const std::vector<bool>& partition)
{
    double d = plane.d();

    // Find vertices on the cut plane
    std::vector<glm::dvec3> intersection;
    for (uint32_t i = 0; i < hull.vertexCount(); i++) {
        for (uint32_t j : hull.neighbors(i)) {
            if (i < j && partition[i] != partition[j]) { // Skip repeated edges, edges that do not cross the plane
                glm::dvec3 vertex = hull.vertices()[i], edge = hull.vertices()[j] - vertex;
                double t = (d - glm::dot(plane.normal, vertex)) / glm::dot(plane.normal, edge);
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
        std::cout << glm::dot(plane.normal, vertices[i] - plane.origin) << "~\n";
        above[i] = glm::dot(plane.normal, vertices[i] - plane.origin) > -1e-12;
        sum += above[i];
    }

    std::cout << "Partition " << sum << ":\n";
    for (auto i : above) std::cout << i << " ";
    std::cout << "\n";

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
    std::vector<glm::dvec3> set = intersection(hull, plane, above);

    // Attach vertices on the positive side of the plane
    for (uint32_t i = 0; i < above.size(); i++) if (above[i]) set.push_back(hull.vertices()[i]);

    return ConvexHull(set);
//    return ConvexHull(VertexArray::clean(set));
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

    std::vector<glm::dvec3> setA = intersection(hull, plane, above);

    // Duplicate intersection vertices in other set
    std::vector<glm::dvec3> setB = setA;

    // Attach original vertices to respective fragments
    for (uint32_t i = 0; i < above.size(); i++) {
        if (above[i]) setA.push_back(hull.vertices()[i]);
        else setB.push_back(hull.vertices()[i]);
    }

    return { ConvexHull(setA), ConvexHull(setB) };
}

//template<class T1, class T2>
//bool CollisionPair<T1, T2>::intersects() const
//{
//    return Collision::intersects(first, second);
//}


