//
// Created by Cam on 2025-04-10.
//

#include "EPA.h"

#include "glm/gtc/quaternion.hpp"

#include "ConvexHull.h"
#include "Simplex.h"
#include "fileIO/MeshHandler.h"

EPA::EPA()
    : m_colliding(false)
    , m_offset()
    , m_nearest(std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max())
    , m_aLocal()
    , m_aWorld()
    , m_bLocal()
    , m_bWorld()
{

}

EPA::EPA(const ConvexHull& a, const ConvexHull& b, const glm::mat4& transform, const glm::mat4& relativeTransform, Simplex simplex)
    : m_colliding(false)
    , m_offset()
    , m_nearest(std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max())
    , m_aLocal()
    , m_aWorld()
    , m_bLocal()
    , m_bWorld()
{
    std::vector<std::pair<float, uint32_t>> order;

    // Expand the input simplex, ensuring the algorithm starts with a tetrahedron
    expandSimplex(a, b, relativeTransform, simplex);
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
        auto vertex = support(a, b, relativeTransform, facet.normal, vertices[facet.triangle.I0].idx);
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
    m_offset = glm::mat3(transform) * -proj;

    // TODO - Barycentric does not always give a reliable result for closest points. Need to evaluate closest of triangle-line or triangle-triangle
    // Calculate the barycentric co-ordinates of the intersection point
    glm::vec3 bary = Triangle::clampedBarycentric(vertices[tri.I0].val, vertices[tri.I1].val, vertices[tri.I2].val, proj);

    Triangle triA = { vertices[tri.I0].idx.first, vertices[tri.I1].idx.first, vertices[tri.I2].idx.first };
    Triangle triB = { vertices[tri.I0].idx.second, vertices[tri.I1].idx.second, vertices[tri.I2].idx.second };

    // Calculate the closest points (locally) on the colliders
    m_aLocal = fromBarycentric(a.vertices(), triA, bary);
    m_bLocal = fromBarycentric(b.vertices(), triB, bary);

    // Calculate the world positions from the local points
    glm::vec4 temp =  transform * glm::vec4(m_aLocal.x, m_aLocal.y, m_aLocal.z, 1.0);
    m_aWorld = { temp.x, temp.y, temp.z };

    temp = transform * relativeTransform * glm::vec4(m_bLocal.x, m_bLocal.y, m_bLocal.z, 1.0);
    m_bWorld = { temp.x, temp.y, temp.z };

    m_nearest = vertices[tri.I0].idx;
}

glm::vec3 EPA::fromBarycentric(const VertexArray& va, const Triangle& triangle, const glm::vec3& bary)
{
    return bary.x * va[triangle.I0] + bary.y * va[triangle.I1] + bary.z * va[triangle.I2];
}

void EPA::exportState(const std::string& path)
{
    float *vx = new float[3 * vertices.size()], *vPtr = vx;
    for (const Simplex::Vertex& vertex : vertices) {
        *vPtr++ = vertex.val.x;
        *vPtr++ = vertex.val.y;
        *vPtr++ = vertex.val.z;
    }

    uint32_t *fx = new uint32_t[3 * facets.size()], *fPtr = fx;
    for (const Facet& facet : facets) {
        *fPtr++ = facet.triangle.I0;
        *fPtr++ = facet.triangle.I1;
        *fPtr++ = facet.triangle.I2;
    }

    MeshHandler::exportMesh(std::make_shared<Mesh>(vx, vertices.size(), fx, facets.size()), path);
}

Simplex::Vertex EPA::support(const ConvexHull& a, const ConvexHull& b, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t> idx)
{
    idx.first = a.walk(axis, idx.first);
    idx.second = b.walk(-axis * glm::mat3(transform), idx.second);

    glm::vec4 vec = transform * glm::vec4(b.vertices()[idx.second].x, b.vertices()[idx.second].y, b.vertices()[idx.second].z, 1.0f);


    return { a.vertices()[idx.first] - glm::vec3{ vec.x, vec.y, vec.z }, idx };
}

glm::vec3 EPA::normal(uint32_t a, uint32_t b, uint32_t c)
{
    return glm::normalize(glm::cross(vertices[b].val - vertices[a].val, vertices[c].val - vertices[a].val));
}

void EPA::expandSimplex(const ConvexHull& a, const ConvexHull& b, const glm::mat4& transform, Simplex& simplex)
{
    switch (simplex.size()) {
        case 1: // Point case
        {
//            glm::vec4 temp = transform * glm::vec4(b.center().x, b.center().y, b.center().z, 1.0f);
//            glm::vec3 axis = glm::vec3{temp.x, temp.y, temp.z} - a.center();
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
                Simplex::Vertex vertex = support(a, b, transform, direction, simplex[0].idx);

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
            glm::vec3 ref = line.x * line.x == 1 ? glm::vec3{ 0, 1, 0 } : glm::vec3{ 1, 0, 0 };
            glm::vec3 axis = glm::normalize(glm::cross(line, ref));

            glm::quat rotation = glm::angleAxis(2.0f / 3.0f  * (float)M_PI, line);

            // Find a vertex off the line to expand to
            for (uint32_t i = 0; i < 3; i++) {
                Simplex::Vertex vertex = support(a, b, transform, axis, simplex[0].idx);

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

            Simplex::Vertex vertex = support(a, b, transform, axis, simplex[0].idx);
            float delta = std::abs(glm::dot(axis, vertex.val - simplex[0].val));

            // Try the opposite direction of the plane in case the triangle is extreme
            if (delta <= 1e-6) {
                vertex = support(a, b, transform, -axis, simplex[0].idx);
                delta = std::abs(glm::dot(axis, vertex.val - simplex[0].val));
            }

            // Confirm selected vertex is off the plane
            if (delta >= 1e-6) {
                simplex.add(vertex);
            }
        }
    }

    simplex.correctWinding();
}

// Debug method to check correctness of algorithm input
bool EPA::isValid(const Simplex& simplex)
{
    if (simplex.size() != 4) {
        std::cout << "IS: " << simplex.size() << "\n";
        for (uint32_t i = 0; i < simplex.size(); i++) {
            std::cout << simplex[i].idx.first << " " << simplex[i].idx.second << " (" << simplex[i].val.x << ", " << simplex[i].val.y << ", " << simplex[i].val.z << ")\n";
        }
    }

    return simplex.size() == 4;
}

void EPA::prepareFacets(const Simplex& simplex, std::vector<std::pair<float, uint32_t>>& order)
{
    for (uint32_t i = 0; i < 4; i++) vertices.push_back(simplex[i]);

    std::array<Triangle, 4> indices = {
            Triangle{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {3, 2, 1}
    };

    std::vector<std::vector<uint32_t>> neighbors = {{2, 3, 1}, {0, 3, 2}, {1, 3, 0}, {1, 0, 2}};

    for (uint32_t i = 0; i < 4; i++) {
        glm::vec3 norm = normal(indices[i].I0, indices[i].I1, indices[i].I2);
        facets.push_back({ indices[i], norm, glm::dot(norm, -vertices[indices[i].I0].val), neighbors[i], true });
    }

    order = {
            {facets[0].value, 0}, {facets[1].value, 1}, {facets[2].value, 2}, {facets[3].value, 3}
    };

    std::sort(order.begin(), order.end(), [](const std::pair<float, uint32_t>& a, const std::pair<float, uint32_t>& b) {
        return a.first > b.first;
    });
}

void EPA::prepareFacets(const std::vector<uint32_t>& horizon, std::vector<std::pair<float, uint32_t>>& order) {

    order.erase(order.begin()); // Remove the current facet from consideration

    // Create a chain of triangular facets between the apex and its respective horizon
    uint32_t lastVertex = horizon[horizon.size() - 1], lastFacet = facets.size() + (horizon.size() / 2) - 1, start = facets.size();
    for(uint32_t j = 0; j < horizon.size(); j += 2){

        // Identify neighbors for linking
        uint32_t currentFacet = facets.size(), nextFacet = (currentFacet + 1 - start) % (horizon.size() / 2) + start;
        uint32_t z = (facets[horizon[j]].triangle[1] == horizon[j + 1]) + 2 * (facets[horizon[j]].triangle[2] == horizon[j + 1]);
        facets[horizon[j]].neighbors[z] = currentFacet;// Link against existing facet beyond the horizon

        // Prepare the new facet and link
        Triangle triangle = {(uint32_t)vertices.size() - 1, lastVertex, horizon[j + 1] };
        glm::vec3 norm = normal(triangle.I0, triangle.I1, triangle.I2);

        Facet facet = {triangle, norm, glm::dot(norm, -vertices[triangle.I0].val), {lastFacet, horizon[j], nextFacet}, true };
        order.emplace_back(facet.value, facets.size());

        if (std::isnan(facet.value)) {
            std::cout << "EPA BR " << norm.x << " " << norm.y << " " << norm.z << " | " << triangle.I0 << " " << triangle.I1 << " " << triangle.I2
                << "\n" << lastFacet << " " << horizon[j] << " " << nextFacet << "\n";
        }

        facets.push_back(facet);
        lastVertex = horizon[j + 1];
        lastFacet = currentFacet;
    }

    // TODO better than sort
    std::sort(order.begin(), order.end(), [](const std::pair<float, uint32_t>& a, const std::pair<float, uint32_t>& b) {
        return a.first > b.first;
    });
}

void EPA::calculateHorizon(const glm::vec3& apex, int64_t last, uint32_t current, std::vector<uint32_t>& horizon){
    if(!facets[current].onHull){
        return;
    }

    float test = glm::dot(facets[current].normal, apex - vertices[facets[current].triangle.I0].val);

    if(test > 1e-6){ // Check whether facet is visible to apex
        facets[current].onHull = false;

        // Identify neighbors to visit
        std::vector<uint32_t> neighbors = {0, 0, 0};
        uint32_t start = last == -1 ? 0 : std::find(facets[current].neighbors.begin(), facets[current].neighbors.end(), last) - facets[current].neighbors.begin();
        neighbors[0] = facets[current].neighbors[start];
        neighbors[1] = facets[current].neighbors[(start + 1) % 3];
        neighbors[2] = facets[current].neighbors[(start + 2) % 3];

        for(uint32_t neighbor : neighbors){
            calculateHorizon(apex, current, neighbor, horizon);
        }

        return;
    }

    uint32_t index = std::find(facets[current].neighbors.begin(), facets[current].neighbors.end(), last) - facets[current].neighbors.begin();

    horizon.push_back(current);// Encode information about adjacent facet for linking purposes
    horizon.push_back(facets[current].triangle[index]);// Encode next vertex on horizon
}

bool EPA::colliding() const
{
    return m_colliding;
}

glm::vec3 EPA::overlap() const
{
    return m_colliding ? m_offset : glm::vec3();
}
glm::vec3 EPA::offset() const
{
    return m_colliding ? glm::vec3() : m_offset;
}

const std::pair<uint32_t, uint32_t>& EPA::nearest() const
{
    return m_nearest;
}

const glm::vec3& EPA::colliderAClosestLocal() const
{
    return m_aLocal;
}
const glm::vec3& EPA::colliderAClosest() const
{
    return m_aWorld;
}

const glm::vec3& EPA::colliderBClosestLocal() const
{
    return m_bLocal;
}
const glm::vec3& EPA::colliderBClosest() const
{
    return m_bWorld;
}

float EPA::distance() const
{
    return glm::length(m_aWorld - m_bWorld);
}