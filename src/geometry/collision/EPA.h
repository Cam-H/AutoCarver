//
// Created by Cam on 2025-04-10.
// Implementation of the Expanding Polytope Algorithm
//

#ifndef AUTOCARVER_EPA_H
#define AUTOCARVER_EPA_H

#include <string>
#include <vector>
#include <glm.hpp>

class ConvexHull;

#include "GJK.h"

#include "geometry/primitives/Triangle.h"
#include "Simplex.h"

static const double TOLERANCE = 1e-12;

class EPA {
public:

    EPA();

    template<class T1, class T2>
    EPA(const T1& a, const T2& b, Simplex<3> simplex);

    template<class T1, class T2>
    EPA(const T1& a, const T2& b, Simplex<3> simplex, const glm::dmat4& relative);

    void setWorldTransform(const glm::dmat4& transform);

    [[nodiscard]] bool colliding() const;

    [[nodiscard]] const glm::dvec3& delta() const;
    [[nodiscard]] glm::dvec3 overlap() const;
    [[nodiscard]] glm::dvec3 offset() const;

    [[nodiscard]] const std::pair<uint32_t, uint32_t>& nearest() const;

    [[nodiscard]] const glm::dvec3& colliderAClosestLocal() const;
    [[nodiscard]] const glm::dvec3& colliderAClosest() const;

    [[nodiscard]] const glm::dvec3& colliderBClosestLocal() const;
    [[nodiscard]] const glm::dvec3& colliderBClosest() const;

    [[nodiscard]] double distance() const;

private:

    struct Facet {
        TriIndex triangle;
        glm::dvec3 normal;
        double value;

        std::vector<uint32_t> neighbors;
        bool onHull;
    };

    glm::dvec3 normal(uint32_t a, uint32_t b, uint32_t c);

    template<class T1, class T2>
    static void expandSimplex(const T1& a, const T2& b, const glm::dmat4& relative, Simplex<3>& simplex);

    static bool isValid(const Simplex<3>& simplex);

    void prepareFacets(const Simplex<3>& simplex, std::vector<std::pair<double, uint32_t>>& order);
    void prepareFacets(const std::vector<uint32_t>& horizon, std::vector<std::pair<double, uint32_t>>& order);

    void calculateHorizon(const glm::dvec3& apex, int64_t last, uint32_t current, std::vector<uint32_t>& horizon);

    [[nodiscard]] glm::dvec3 barycentric(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, const glm::dvec3& p);
    [[nodiscard]] static glm::dvec3 fromBarycentric(const Triangle3D& triangle, const glm::dvec3& bary);

    void exportState(const std::string& path);
private:

    std::vector<Facet> facets;
    std::vector<Simplex<3>::Vertex> vertices;

    bool m_colliding;

    std::pair<uint32_t, uint32_t> m_nearest;

    glm::dvec3 m_localOffset;
    glm::dvec3 m_worldOffset;

    glm::dvec3 m_aLocal;
    glm::dvec3 m_aWorld;

    glm::dvec3 m_bLocal;
    glm::dvec3 m_bWorld;

};

#include "EPA.tpp"

#endif //AUTOCARVER_EPA_H
