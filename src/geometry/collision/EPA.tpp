#include "EPA.h"

#include <iostream>

#include <gtc/quaternion.hpp>
#include "geometry/Axis3D.h"

template<class T1, class T2>
EPA::EPA(const T1& a, const T2& b, Simplex simplex)
    : EPA(a, b, simplex, glm::dmat4(1.0))
{
    // TODO implement without transformations to optimize
}

template<class T1, class T2>
EPA::EPA(const T1& a, const T2& b, Simplex simplex, const glm::dmat4& relative)
    : EPA()
{
    std::vector<std::pair<double, uint32_t>> order;

//    simplex.purgeDuplicates();

    // Expand the input simplex, ensuring the algorithm starts with a tetrahedron
    expandSimplex(a, b, relative, simplex);
    if (!isValid(simplex)) return;

    // Prepare the initial set of facets from the simplex
    prepareFacets(simplex, order);

    while (true) {
        Facet& facet = facets[order[0].second];

        // Skip facet if it has already been eliminated from the hull
        if (!facets[order[0].second].onHull) {
            order.erase(order.begin());
            continue;
        }

        // Find next vertex to expand to, based on current shortest overlap. Verify it protrudes from the reference facet
        Simplex::Vertex vertex(gjkSupport(a, b, facet.normal, relative, vertices[facet.triangle.I0].idx));
        if (glm::dot(facet.normal, vertex.val - vertices[facet.triangle.I0].val) <= TOLERANCE) break;

        vertices.push_back(vertex);

        // Calculate the horizon visible relative to the newest vertex
        std::vector<uint32_t> horizon;
        calculateHorizon(vertices[vertices.size() - 1].val, -1, order[0].second, horizon);

        prepareFacets(horizon, order);// Generate a strip of new facets between the apex and the horizon
    }

    uint32_t idx = order[0].second;
    const TriIndex& tri = facets[idx].triangle;


    // Record cs results
    m_colliding = order[0].first < 0;

    // Calculate the projection of the origin on to the nearest facet
    glm::dvec3 proj = glm::dot(facets[idx].normal, vertices[tri.I0].val) * facets[idx].normal;

    // TODO - Barycentric does not always give a reliable result for closest points. Need to evaluate closest of triangle-line or triangle-triangle
    // Calculate the barycentric co-ordinates of the intersection point
    glm::dvec3 bary = Triangle3D::clampedBarycentric(vertices[tri.I0].val, vertices[tri.I1].val, vertices[tri.I2].val, proj);

    TriIndex triA = { vertices[tri.I0].idx.first, vertices[tri.I1].idx.first, vertices[tri.I2].idx.first };
    TriIndex triB = { vertices[tri.I0].idx.second, vertices[tri.I1].idx.second, vertices[tri.I2].idx.second };

    // Calculate the closest points (locally) on the colliders
    m_aWorld = m_aLocal = fromBarycentric(Triangle3D(a.vertices(), triA), bary);
    m_bWorld = m_bLocal = fromBarycentric(Triangle3D(b.vertices(), triB), bary); // TODO fix bLocal

    m_worldOffset = m_localOffset = -proj;

    m_nearest = vertices[tri.I0].idx;

}

template<class T1, class T2>
void EPA::expandSimplex(const T1& a, const T2& b, const glm::dmat4& relative, Simplex& simplex)
{
    switch (simplex.size()) {
        case 1: // Point case
        {
            static const glm::dvec3 searchDirections[6] = {
//                                axis,
//                                -axis,
                    {  1.0,  0.0,  0.0 },
                    { -1.0,  0.0,  0.0 },
                    {  0.0,  1.0,  0.0 },
                    {  0.0, -1.0,  0.0 },
                    {  0.0,  0.0,  1.0 },
                    {  0.0,  0.0, -1.0 }
            };

            for (const glm::dvec3& direction : searchDirections) {
                Simplex::Vertex vertex(gjkSupport(a, b, direction, relative, simplex[0].idx));

                glm::dvec3 delta = vertex.val - simplex[0].val;
                if (glm::dot(delta, delta) >= TOLERANCE) {
                    simplex.add(vertex);
                    break;
                }
            }
        }
            if (simplex.size() < 2) break; // Stop if for some reason the point case failed
        case 2: // Line case
        {
            glm::dvec3 line = glm::normalize(simplex[1].val - simplex[0].val);

            // Create reasonable initial search direction
            Axis3D system(line);
            glm::dvec3 axis = system.xAxis;

            glm::dquat rotation = glm::angleAxis(2.0 / 3.0  * M_PI, line);

            // Find a vertex off the line to expand to
            for (uint32_t i = 0; i < 3; i++) {
                Simplex::Vertex vertex(gjkSupport(a, b, axis, relative, simplex[0].idx));

                if (std::abs(glm::dot(axis, vertex.val - simplex[0].val)) >= TOLERANCE) {
                    simplex.add(vertex);
                    break;
                }

                // Prepare to try a new search direction (rotated by 60dg)
                axis = rotation * axis;
            }
        }
            if (simplex.size() < 3) break; // Stop if for some reason the line case failed
        case 3: // Triangle case
        {
            glm::dvec3 axis = glm::normalize(glm::cross(simplex[1].val - simplex[0].val, simplex[2].val - simplex[0].val));

            Simplex::Vertex vertex(gjkSupport(a, b, axis, relative, simplex[0].idx));
            double delta = std::abs(glm::dot(axis, vertex.val - simplex[0].val));

            // Try the opposite direction of the plane in case the triangle is extreme
            if (delta <= TOLERANCE) {
                vertex = Simplex::Vertex(gjkSupport(a, b, -axis, relative, simplex[0].idx));
                delta = std::abs(glm::dot(axis, vertex.val - simplex[0].val));
            }

            // Confirm selected vertex is off the plane
            if (delta >= TOLERANCE) {
                simplex.add(vertex);
            }
        }
    }

    simplex.correctWinding();
}

template<class T1, class T2>
std::tuple<uint32_t, uint32_t, glm::dvec3> EPA::gjkSupport(const T1& bodyA, const T2& bodyB, const glm::dvec3& axis, const std::pair<uint32_t, uint32_t>& idx)
{
    auto [aIdx, aVertex] = bodyA.extreme(axis, idx.first);
    auto [bIdx, bVertex] = bodyB.extreme(-axis, idx.second);

    return { aIdx, bIdx, aVertex - bVertex };
}

template<class T1, class T2>
std::tuple<uint32_t, uint32_t, glm::dvec3> EPA::gjkSupport(const T1& bodyA, const T2& bodyB, const glm::dvec3& axis, const glm::dmat4& transform, const std::pair<uint32_t, uint32_t>& idx)
{
    auto [aIdx, aVertex] = bodyA.extreme(axis, idx.first);
    auto [bIdx, bVertex] = bodyB.extreme(-axis * glm::mat3(transform), idx.second);

    glm::dvec4 vec = transform * glm::dvec4(bVertex.x, bVertex.y, bVertex.z, 1.0);
    return { aIdx, bIdx, aVertex - glm::dvec3{ vec.x, vec.y, vec.z }};
}