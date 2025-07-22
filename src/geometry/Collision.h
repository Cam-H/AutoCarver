//
// Created by cjhat on 2025-07-18.
//

#ifndef AUTOCARVER_COLLISIONPAIR_H
#define AUTOCARVER_COLLISIONPAIR_H

#include <iostream>

#include <glm.hpp>

class Ray;
class Plane;
class Triangle;
class Circle;
class Sphere;
class AABB;
class ConvexHull;

class EPA;

#include "Simplex.h"

// An array of static collision detection methods for primitive shapes
class Collision {
public:

    // Determine if there is an intersection between the specified bodies
    static bool test(const Plane& bodyA, const Plane& bodyB);
    static bool test(const Sphere& bodyA, const Sphere& bodyB);
    static bool test(const AABB& bodyA, const AABB& bodyB);
    static bool test(const ConvexHull& bodyA, const ConvexHull& bodyB);

    // Tests against plane & other bodies
    static bool test(const Plane& bodyA, const Sphere& bodyB);
    static bool test(const Sphere& bodyA, const Plane& bodyB);

    static bool test(const Plane& bodyA, const AABB& bodyB);
    static bool test(const AABB& bodyA, const Plane& bodyB);

    static bool test(const Plane& bodyA, const ConvexHull& bodyB);
    static bool test(const ConvexHull& bodyA, const Plane& bodyB);

    // Tests against sphere & remaining bodies
    static bool test(const Sphere& bodyA, const AABB& bodyB);
    static bool test(const AABB& bodyA, const Sphere& bodyB);

    static bool test(const Sphere& bodyA, const ConvexHull& bodyB);
    static bool test(const ConvexHull& bodyA, const Sphere& bodyB);

    // Tests against AABB & remaining bodies
    static bool test(const AABB& bodyA, const ConvexHull& bodyB);
    static bool test(const ConvexHull& bodyA, const AABB& bodyB);

    template<class T1, class T2>
    static Simplex gjk(const T1& bodyA, const T2& bodyB)
    {
        if (!bodyA.isValid() || !bodyB.isValid()) throw std::runtime_error("[Collision] Invalid bodies. Can not calculate GJK");

        std::pair<uint32_t, uint32_t> idx = { std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max() };

        glm::vec3 axis = initialAxis(bodyA, bodyB, idx), next;
        Simplex simplex({ gjkSupport(bodyA, bodyB, axis, idx), idx });

        axis = -simplex[0].val;

//        int limit = (int)(vertexCount() + body.vertexCount());
        int limit = 1e6;
        while (limit-- > 0) {
            next = gjkSupport(bodyA, bodyB, axis, idx);

            // Check whether cs is impossible
            if (glm::dot (axis, next) <= 0) return simplex;

            simplex.add({ next, idx });

            // Check whether the cs is certain
            if (simplex.evaluate(axis)) return simplex;
        }

        std::cout << "[Collision] GJK Error!\n";
        return Simplex(Simplex::Vertex{});
    }

    template<class T1, class T2>
    static Simplex gjk(const T1& bodyA, const T2& bodyB, const glm::mat4& transform, std::pair<uint32_t, uint32_t>& idx)
    {
        if (!bodyA.isValid() || !bodyB.isValid()) throw std::runtime_error("[Collision] Invalid bodies. Can not calculate GJK");

        glm::vec3 axis = initialAxis(bodyA, bodyB, idx), next;
        Simplex simplex({ gjkSupport(bodyA, bodyB, transform, axis, idx), idx });

        axis = -simplex[0].val;

//        int limit = (int)(vertexCount() + body.vertexCount());
        int limit = 1e6;
        while (limit-- > 0) {
            next = gjkSupport(bodyA, bodyB, transform, axis, idx);

            // Check whether cs is impossible
            if (glm::dot (axis, next) <= 0) return simplex;

            simplex.add({ next, idx });

            // Check whether the cs is certain
            if (simplex.evaluate(axis)) return simplex;
        }

        std::cout << "[Collision] GJK Error!\n";
        return Simplex(Simplex::Vertex{});
    }

    // Determine the closest vertex on the body to the reference
    static glm::vec3 nearest(const AABB& body, const glm::vec3& reference);

    // Determine the furthest vertex on the body to the reference
    static glm::vec3 farthest(const AABB& body, const glm::vec3& reference);

    // Determine if the first body (A) encloses the second body (B)
    static bool encloses(const Sphere& body, const glm::vec3& vertex);
    static bool encloses(const AABB& body, const glm::vec3& vertex);

    template<class T>
    static bool encloses(const T& body, const std::vector<glm::vec3>& vertices)
    {
        for (const glm::vec3& vertex : vertices) {
            if (!encloses(body, vertex)) return false;
        }

        return true;
    }

    static bool encloses(const Sphere& bodyA, const Sphere& bodyB);
    static bool encloses(const AABB& bodyA, const AABB& bodyB);

    static bool encloses(const Sphere& bodyA, const AABB& bodyB);
    static bool encloses(const AABB& bodyA, const Sphere& bodyB);

    static bool encloses(const Sphere& bodyA, const ConvexHull& bodyB);
    static bool encloses(const AABB& bodyA, const ConvexHull& bodyB);

    // Determine the intersection between the specified bodies
    static std::tuple<bool, float, glm::vec3> intersection(const AABB& body, const Ray& ray);

    static std::vector<glm::vec3> intersection(const ConvexHull& hull, const Plane& plane);
    static EPA intersection(const ConvexHull& bodyA, const ConvexHull& bodyB);

    // Determine the relative position of the specified body to the plane
    static bool above(const ConvexHull& hull, const Plane& plane);
    static bool below(const ConvexHull& hull, const Plane& plane);


    // Create fragments of the body against the cutting body
    static ConvexHull fragment(const ConvexHull& hull, const Plane& plane);
    static std::pair<ConvexHull, ConvexHull> fragments(const ConvexHull& hull, const Plane& plane);

private:

    static glm::vec3 initialAxis(const ConvexHull& bodyA, const ConvexHull& bodyB, std::pair<uint32_t, uint32_t>& idx);

    static glm::vec3 initialAxis(const Sphere& bodyA, const ConvexHull& bodyB, std::pair<uint32_t, uint32_t>& idx);
    static glm::vec3 initialAxis(const ConvexHull& bodyA, const Sphere& bodyB, std::pair<uint32_t, uint32_t>& idx);

    static glm::vec3 initialAxis(const AABB& bodyA, const ConvexHull& bodyB, std::pair<uint32_t, uint32_t>& idx);
    static glm::vec3 initialAxis(const ConvexHull& bodyA, const AABB& bodyB, std::pair<uint32_t, uint32_t>& idx);

    static glm::vec3 gjkSupport(const ConvexHull& bodyA, const ConvexHull& bodyB, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx);

    template<class T1, class T2>
    static glm::vec3 gjkSupport(const T1& bodyA, const T2& bodyB, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx)
    {
        return bodyA.extreme(axis) - bodyB.extreme(-axis);
    }


    static glm::vec3 gjkSupport(const Sphere& bodyA, const ConvexHull& bodyB, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx);
    static glm::vec3 gjkSupport(const ConvexHull& bodyA, const Sphere& bodyB, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx);

    static glm::vec3 gjkSupport(const AABB& bodyA, const ConvexHull& bodyB, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx);
    static glm::vec3 gjkSupport(const ConvexHull& bodyA, const AABB& bodyB, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx);

    static glm::vec3 gjkSupport(const ConvexHull& bodyA, const ConvexHull& bodyB, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx);


    static std::vector<glm::vec3> intersection(const ConvexHull& hull, const Plane& plane, const std::vector<bool>& partition);
    static std::tuple<std::vector<bool>, bool> partition(const ConvexHull& hull, const Plane& plane);

};

//template<class T1, class T2>
//class CollisionPair {
//public:
//
//    CollisionPair(T1 bodyA, T2 bodyB) : first(bodyA), second(bodyB) {}
//
//    bool intersects() const;
//
//private:
//    T1 first;
//    T2 second;
//};
//
//#include "Collision.tpp"

#endif //AUTOCARVER_COLLISIONPAIR_H
