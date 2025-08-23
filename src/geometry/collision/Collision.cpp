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

std::tuple<bool, double> Collision::raycast(const Plane& plane, const Ray& ray)
{
    double den = glm::dot(plane.normal, ray.axis);
    if (std::abs(den) < 1e-12) return { false, 0 }; // Parallel case - No intersection
//    if (den < -1e-12) return { false, 0 }; // Parallel case + Behind plane case - No intersection

    double t = glm::dot(plane.normal, plane.origin - ray.origin) / den;
    return { t >= 0, t };
}
std::tuple<bool, double> Collision::raycast(const Triangle3D& triangle, const Ray& ray)
{
    glm::dvec3 AB = triangle.b - triangle.a, AC = triangle.c - triangle.a;

    // Calculate determinant
    glm::dvec3 h = glm::cross(ray.axis, AC);
    double a = glm::dot(AB, h);

    if (std::abs(a) < 1e-12) return { false, 0 }; // Ray is parallel to the triangle

    // Calculate barycentric coordinates
    glm::dvec3 s = ray.origin - triangle.a;
    double f = 1.0 / a, u = f * glm::dot(s, h);
    if (u < 0.0 || u > 1.0) return { false, 0 };

    glm::dvec3 q = glm::cross(s, AB);
    double v = f * glm::dot(ray.axis, q);
    if (v < 0.0 || u + v > 1.0) return { false, 0 };

    double t = f * glm::dot(AC, q);
    return { t >= 0, t };
}

// Solved via substitution with the analytic sphere formula. Ray is assumed to be normalized for quadratic calculation
std::tuple<bool, double> Collision::raycast(const Sphere& sphere, const Ray& ray)
{
    glm::dvec3 delta = ray.origin - sphere.center;
    double b = 2.0 * glm::dot(delta, ray.axis);
    double c = glm::dot(delta, delta) - sphere.radius*sphere.radius;

    double discriminant = b*b - 4*c;
    if (discriminant < 0.0) return { false, 0 };

    double dist = sqrt(discriminant);
    double t0 = -b - dist, t1 = -b + dist;

    if (t0 >= 0.0) return { true, t0 / 2.0 };
    if (t1 >= 0.0) return { true, t1 / 2.0 };

    return { false, 0 };

}
std::tuple<bool, double> Collision::raycast(const AABB& box, const Ray& ray)
{
    double tMin = std::numeric_limits<double>::lowest(), tMax = std::numeric_limits<double>::max();

    for (uint8_t i = 0; i < 3; i++) {
        if (ray.axis[i] * ray.axis[i] < 1e-12) { // Handle parallel ray case
            if (ray.origin[i] < box.min[i] || ray.origin[i] > box.max[i]) return { false, 0 };
        } else {
            double ood = 1.0f / ray.axis[i];
            double t1 = (box.min[i] - ray.origin[i]) * ood;
            double t2 = (box.max[i] - ray.origin[i]) * ood;
            if (t1 > t2) std::swap(t1, t2);

            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);

            if (tMin > tMax) return { false, 0 };
        }
    }

    return { true, tMin };
}
std::tuple<bool, double> Collision::raycast(const ConvexHull& hull, const Ray& ray)
{
    auto [hit, t, idx] = pickFace(hull, ray);
    return { hit, t };
}

std::tuple<bool, double, uint32_t> Collision::pickFace(const ConvexHull& hull, const Ray& ray)
{
    const std::vector<glm::dvec3>& vertices = hull.vertices();
    const FaceArray& fa = hull.faces();
    uint32_t idxOffset = 0, count, idx = 0;

    double tMin = -1;

    for (uint32_t i = 0; i < fa.faceCount(); i++) {
        count = fa.faceSizes()[i];

//        if (count == 3) { //
//            Triangle3D triangle(
//                    hull.vertices()[fa.faces()[idxOffset]],
//                    hull.vertices()[fa.faces()[idxOffset + 1]],
//                    hull.vertices()[fa.faces()[idxOffset + 2]]);
//            auto [hit, t] = raycast(triangle, ray);
//            if (hit && (tMin < 0 || t < tMin)) tMin = t;
//            continue;
//        }

        // Test whether the intersection of the ray with the face plane is enclosed by the face (Based on the face being a convex polygon)
        Plane plane(vertices[fa.faces()[idxOffset]], fa.normals()[i]);
        auto [hit, t] = raycast(plane, ray);
        if (hit) {
            glm::dvec3 vertex = ray.origin + t * ray.axis;

            // Individually test that vertex is interior to each edge
            for (uint32_t j = 0; j < count; j++) {
                glm::dvec3 edge = vertices[fa.faces()[idxOffset + ((j + 1) % count)]] - vertices[fa.faces()[idxOffset + j]];
                if (glm::dot(glm::cross(plane.normal, edge), vertex - vertices[fa.faces()[idxOffset + j]]) < -1e-12) {
                    hit = false;
                    break;
                }
            }

            // Record the face nearest to the ray origin, if it fully encloses the ray-plane intersection
            if (hit && (tMin < 0 || t < tMin)) {
                tMin = t;
                idx = i;
            }
        }

        idxOffset += count;
    }


    return { tMin != -1, tMin, idx };
}

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
    auto [hit, t] = raycast(body, ray);
    if (!hit) return { false, 0, {} };
    return { hit, t, ray.origin + ray.axis * t };
}

// TODO Improvement: Leverage walk to calculate intersection in-place
std::vector<glm::dvec3> Collision::intersection(const ConvexHull& hull, const Plane& plane)
{
    if (hull.vertexCount() < 4) return {}; // Early exit when the hull is poorly formed

    const auto& state = partition(hull, plane);
//    std::cout << (uint32_t)state[0] << " " << (uint32_t)state[1] << " " << (uint32_t)state.back() << "STATE\n";
    if (state.back() != 0) return {}; // Early exit when intersection is impossible (hull does not pass through the plane)

    return intersection(hull, plane, state);
}

// Returns a list of vertices formed from the intersection of edges and the plane (Vertices are not sequential)
std::vector<glm::dvec3> Collision::intersection(const ConvexHull& hull, const Plane& plane, const std::vector<uint8_t>& partition)
{
    double d = plane.d();

    // Find vertices on the cut plane
    std::vector<glm::dvec3> intersection;
    for (uint32_t i = 0; i < hull.vertexCount(); i++) {
        if (partition[i] == 0) { // Vertex is on the plane
            intersection.push_back(hull.vertices()[i]);
            continue;
        }

        for (uint32_t j : hull.neighbors(i)) {
            if (partition[j] == 0) continue;

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
    return partition(hull, plane).back() == 1;
}
bool Collision::below(const ConvexHull& hull, const Plane& plane)
{
    return partition(hull, plane).back() == 2;
}

// Indicates where vertices are in relation to the plane (0 = on, 1 = above, 2 = below)
// Last element indicates overall state (0 = divided, 1 = entirely above, 2 = entirely below)
std::vector<uint8_t> Collision::partition(const ConvexHull& hull, const Plane& plane)
{
    std::vector<uint8_t> state(hull.vertexCount() + 1);

    // Determine which vertices are above the cut plane
    bool appearance[3] = { false, false, false }; // Track which positions have shown up
    const auto& vertices = hull.vertices();
    for (uint32_t i = 0; i < vertices.size(); i++) {
//        std::cout << glm::dot(plane.normal, vertices[i] - plane.origin) << "~\n";
        double dot = glm::dot(plane.normal, vertices[i] - plane.origin);
        state[i] = (dot > 1e-12) + 2 * (dot < -1e-12);
        appearance[state[i]] = true;
    }

    // Indicate spread of the vertices
    state.back() = 3 - 2 * appearance[1] - appearance[2];

//    std::cout << "Partition:\n";
//    for (auto i : state) std::cout << (uint32_t)i << " ";
//    std::cout << "\n";

    return state;
}

ConvexHull Collision::fragment(const ConvexHull& hull, const Plane& plane)
{
    if (hull.empty()) return {};

    const auto& state = partition(hull, plane);
    if (state.back() != 0) {

        // Exit if no edges intersect with the plane
        if (state.back() == 1) return hull;
        else return {};
    }

    // Find vertex intersections of hull with plane
    std::vector<glm::dvec3> set = intersection(hull, plane, state);

    // Attach vertices on the positive side of the plane
    for (uint32_t i = 0; i < state.size() - 1; i++) if (state[i] == 1) set.push_back(hull.vertices()[i]);

    return ConvexHull(set);
//    return ConvexHull(VertexArray::clean(set));
}
std::pair<ConvexHull, ConvexHull> Collision::fragments(const ConvexHull& hull, const Plane& plane)
{
    if (hull.empty()) return {};

    const auto& state = partition(hull, plane);
    if (state.back() != 0) {

        // Exit if no edges intersect with the plane
        if (state.back() == 1) return { hull, {} };
        else return { {}, hull };
    }

    std::vector<glm::dvec3> setA = intersection(hull, plane, state);

    // Duplicate intersection vertices in other set
    std::vector<glm::dvec3> setB = setA;

    // Attach original vertices to respective fragments
    for (uint32_t i = 0; i < state.size(); i++) {
        if (state[i] == 1) setA.push_back(hull.vertices()[i]);
        else if (state[i] == 2) setB.push_back(hull.vertices()[i]);
    }

    return { ConvexHull(setA), ConvexHull(setB) };
}

//template<class T1, class T2>
//bool CollisionPair<T1, T2>::intersects() const
//{
//    return Collision::intersects(first, second);
//}


