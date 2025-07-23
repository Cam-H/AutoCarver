#include "EPA.h"

#include <iostream>

#include <gtc/quaternion.hpp>
#include "geometry/Axis3D.h"

template<class T1, class T2>
EPA::EPA(const T1& a, const T2& b, Simplex simplex)
    : EPA(a, b, simplex, glm::mat4(1.0f))
{
    // TODO implement without transformations to optimize
}

template<class T1, class T2>
EPA::EPA(const T1& a, const T2& b, Simplex simplex, const glm::mat4& relative)
    : EPA()
{
    std::vector<std::pair<float, uint32_t>> order;

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
        if (glm::dot(facet.normal, vertex.val - vertices[facet.triangle.I0].val) <= 1e-6) break;

        vertices.push_back(vertex);

        // Calculate the horizon visible relative to the newest vertex
        std::vector<uint32_t> horizon;
        calculateHorizon(vertices[vertices.size() - 1].val, -1, order[0].second, horizon);

        prepareFacets(horizon, order);// Generate a strip of new facets between the apex and the horizon
    }

    uint32_t idx = order[0].second;
    const Triangle& tri = facets[idx].triangle;


    // Record cs results
    m_colliding = order[0].first < 0;

    // Calculate the projection of the origin on to the nearest facet
    glm::vec3 proj = glm::dot(facets[idx].normal, vertices[tri.I0].val) * facets[idx].normal;

    // TODO - Barycentric does not always give a reliable result for closest points. Need to evaluate closest of triangle-line or triangle-triangle
    // Calculate the barycentric co-ordinates of the intersection point
    glm::vec3 bary = Triangle::clampedBarycentric(vertices[tri.I0].val, vertices[tri.I1].val, vertices[tri.I2].val, proj);

    Triangle triA = { vertices[tri.I0].idx.first, vertices[tri.I1].idx.first, vertices[tri.I2].idx.first };
    Triangle triB = { vertices[tri.I0].idx.second, vertices[tri.I1].idx.second, vertices[tri.I2].idx.second };

    // Calculate the closest points (locally) on the colliders
    m_aWorld = m_aLocal = fromBarycentric(a.vertices(), triA, bary);
    m_bWorld = m_bLocal = fromBarycentric(b.vertices(), triB, bary); // TODO fix bLocal

    m_worldOffset = m_localOffset = -proj;

    m_nearest = vertices[tri.I0].idx;

}

template<class T1, class T2>
void EPA::expandSimplex(const T1& a, const T2& b, const glm::mat4& relative, Simplex& simplex)
{
    switch (simplex.size()) {
        case 1: // Point case
        {
            static const glm::vec3 searchDirections[6] = {
//                                axis,
//                                -axis,
                    {  1.0f,  0.0f,  0.0f },
                    { -1.0f,  0.0f,  0.0f },
                    {  0.0f,  1.0f,  0.0f },
                    {  0.0f, -1.0f,  0.0f },
                    {  0.0f,  0.0f,  1.0f },
                    {  0.0f,  0.0f, -1.0f }
            };

            for (const glm::vec3& direction : searchDirections) {
                Simplex::Vertex vertex(gjkSupport(a, b, direction, relative, simplex[0].idx));

                glm::vec3 delta = vertex.val - simplex[0].val;
                if (glm::dot(delta, delta) >= 1e-6) {
                    simplex.add(vertex);
                    break;
                }
            }
        }
            if (simplex.size() < 2) break; // Stop if for some reason the point case failed
        case 2: // Line case
        {
            glm::vec3 line = glm::normalize(simplex[1].val - simplex[0].val);

            // Create reasonable initial search direction
            Axis3D system(line);
            glm::vec3 axis = system.xAxis;

            glm::quat rotation = glm::angleAxis(2.0f / 3.0f  * (float)M_PI, line);

            // Find a vertex off the line to expand to
            for (uint32_t i = 0; i < 3; i++) {
                Simplex::Vertex vertex(gjkSupport(a, b, axis, relative, simplex[0].idx));

                if (std::abs(glm::dot(axis, vertex.val - simplex[0].val)) >= 1e-6) {
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
            glm::vec3 axis = glm::normalize(glm::cross(simplex[1].val - simplex[0].val, simplex[2].val - simplex[0].val));

            Simplex::Vertex vertex(gjkSupport(a, b, axis, relative, simplex[0].idx));
            float delta = std::abs(glm::dot(axis, vertex.val - simplex[0].val));

            // Try the opposite direction of the plane in case the triangle is extreme
            if (delta <= 1e-6) {
                vertex = Simplex::Vertex(gjkSupport(a, b, -axis, relative, simplex[0].idx));
                delta = std::abs(glm::dot(axis, vertex.val - simplex[0].val));
            }

            // Confirm selected vertex is off the plane
            if (delta >= 1e-6) {
                simplex.add(vertex);
            }
        }
    }

//    simplex.correctWinding();
}

template<class T1, class T2>
std::tuple<uint32_t, uint32_t, glm::vec3> EPA::gjkSupport(const T1& bodyA, const T2& bodyB, const glm::vec3& axis, const std::pair<uint32_t, uint32_t>& idx)
{
    auto [aIdx, aVertex] = bodyA.extreme(axis, idx.first);
    auto [bIdx, bVertex] = bodyB.extreme(-axis, idx.second);

    return { aIdx, bIdx, aVertex - bVertex };
}

template<class T1, class T2>
std::tuple<uint32_t, uint32_t, glm::vec3> EPA::gjkSupport(const T1& bodyA, const T2& bodyB, const glm::vec3& axis, const glm::mat4& transform, const std::pair<uint32_t, uint32_t>& idx)
{
    auto [aIdx, aVertex] = bodyA.extreme(axis, idx.first);
    auto [bIdx, bVertex] = bodyB.extreme(-axis * glm::mat3(transform), idx.second);

    glm::vec4 vec = transform * glm::vec4(bVertex.x, bVertex.y, bVertex.z, 1.0f);
    return { aIdx, bIdx, aVertex - glm::vec3{ vec.x, vec.y, vec.z }};
}