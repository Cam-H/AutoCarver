//
// Created by cjhat on 2025-09-22.
//

#ifndef AUTOCARVER_GJK_H
#define AUTOCARVER_GJK_H

#include "Simplex.h"

class GJK {
public:

    template<class T1, class T2>
    static std::tuple<uint32_t, uint32_t, glm::dvec2> support(const T1& bodyA, const T2& bodyB, const glm::dvec2& axis, const std::pair<uint32_t, uint32_t>& idx)
    {
        auto [aIdx, aVertex] = bodyA.extreme(axis, idx.first);
        auto [bIdx, bVertex] = bodyB.extreme(-axis, idx.second);

        return { aIdx, bIdx, aVertex - bVertex };
    }

    template<class T1, class T2>
    static std::tuple<uint32_t, uint32_t, glm::dvec3> support(const T1& bodyA, const T2& bodyB, const glm::dvec3& axis, const std::pair<uint32_t, uint32_t>& idx)
    {
        auto [aIdx, aVertex] = bodyA.extreme(axis, idx.first);
        auto [bIdx, bVertex] = bodyB.extreme(-axis, idx.second);

        return { aIdx, bIdx, aVertex - bVertex };
    }

    template<class T1, class T2>
    static std::tuple<uint32_t, uint32_t, glm::dvec3> support(const T1& bodyA, const T2& bodyB, const glm::dvec3& axis, const glm::dmat4& transform, const std::pair<uint32_t, uint32_t>& idx)
    {
        auto [aIdx, aVertex] = bodyA.extreme(axis, idx.first);
        auto [bIdx, bVertex] = bodyB.extreme(-axis * glm::mat3(transform), idx.second);

        glm::dvec4 vec = transform * glm::dvec4(bVertex.x, bVertex.y, bVertex.z, 1.0);
        return { aIdx, bIdx, aVertex - glm::dvec3{ vec.x, vec.y, vec.z }};
    }


    template<class T1, class T2>
    static Simplex<2> solve2D(const T1& bodyA, const T2& bodyB, const std::pair<uint32_t, uint32_t>& idx)
    {
        if (!bodyA.isValid() || !bodyB.isValid()) throw std::runtime_error("[GJK2D] Invalid bodies. Can not calculate");

        glm::dvec2 axis = { 1, 0 };
        Simplex<2> simplex(support(bodyA, bodyB, axis, idx));

        axis = -simplex[0].val;

        int limit = 1e3;
        while (limit-- > 0) {
            const auto& [aIdx, bIdx, next] = support(bodyA, bodyB, axis, simplex[0].idx); // Use most recent idx

            // Check whether cs is impossible
            if (glm::dot (axis, next) < 1e-12) return simplex;

            simplex.add(Simplex<2>::Vertex({ aIdx, bIdx }, next));

            // Check whether the cs is certain
            if (simplex.evaluate(axis)) return simplex;
        }

        throw std::runtime_error("[GJK2D] Failed to terminate when calculating");
    }

    template<class T1, class T2>
    static Simplex<3> solve3D(const T1& bodyA, const T2& bodyB, const std::pair<uint32_t, uint32_t>& idx)
    {
        if (!bodyA.isValid() || !bodyB.isValid()) throw std::runtime_error("[GJK3D] Invalid bodies. Can not calculate");

        glm::dvec3 axis = { 1, 0, 0 };
        Simplex<3> simplex(support(bodyA, bodyB, axis, idx));

        axis = -simplex[0].val;

        int limit = 1e3;
        while (limit-- > 0) {
            const auto& [aIdx, bIdx, next] = support(bodyA, bodyB, axis, simplex[0].idx); // Use most recent idx

            // Check whether cs is impossible
            if (glm::dot (axis, next) < 1e-12) return simplex;

            simplex.add(Simplex<3>::Vertex({ aIdx, bIdx }, next));

            // Check whether the cs is certain
            if (simplex.evaluate(axis)) return simplex;
        }

        throw std::runtime_error("[GJK3D] Failed to terminate when calculating");
    }

    template<class T1, class T2>
    static Simplex<3> solve3D(const T1& bodyA, const T2& bodyB, const glm::dmat4& transform, const std::pair<uint32_t, uint32_t>& idx)
    {
        if (!bodyA.isValid() || !bodyB.isValid()) throw std::runtime_error("[GJK3D] Invalid bodies. Can not calculate");

        glm::dvec3 axis = { 1, 0, 0 };
        Simplex<3> simplex(support(bodyA, bodyB, axis, transform, idx));

        axis = -simplex[0].val;

        int limit = 1e3;
        while (limit-- > 0) {
            const auto& [aIdx, bIdx, next] = support(bodyA, bodyB, axis, transform, simplex[0].idx); // Use most recent idx

            // Check whether cs is impossible
            if (glm::dot(axis, next) < 1e-12) return simplex;

            simplex.add(Simplex<3>::Vertex({ aIdx, bIdx }, next));

            // Check whether the cs is certain
            if (simplex.evaluate(axis)) return simplex;
        }

        throw std::runtime_error("[GJK3D] Failed to terminate when calculating");
    }
};

#endif //AUTOCARVER_GJK_H
