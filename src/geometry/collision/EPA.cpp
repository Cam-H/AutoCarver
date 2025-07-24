//
// Created by Cam on 2025-04-10.
//

#include "EPA.h"

#include "geometry/primitives/ConvexHull.h"
#include "Simplex.h"
#include "fileIO/MeshHandler.h"

EPA::EPA()
    : m_colliding(false)
    , m_nearest(std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max())
    , m_localOffset()
    , m_worldOffset()
    , m_aLocal()
    , m_aWorld()
    , m_bLocal()
    , m_bWorld()
{

}

void EPA::setWorldTransform(const glm::dmat4& transform)
{
    m_worldOffset = glm::mat3(transform) * m_localOffset;

    // Calculate the world positions from the local points
    glm::vec4 temp =  transform * glm::vec4(m_aLocal.x, m_aLocal.y, m_aLocal.z, 1.0);
    m_aWorld = { temp.x, temp.y, temp.z };

    temp = transform * glm::vec4(m_bLocal.x, m_bLocal.y, m_bLocal.z, 1.0);
    m_bWorld = { temp.x, temp.y, temp.z };
}

glm::dvec3 EPA::fromBarycentric(const std::vector<glm::dvec3>& va, const Triangle& triangle, const glm::dvec3& bary)
{
    return bary.x * va[triangle.I0] + bary.y * va[triangle.I1] + bary.z * va[triangle.I2];
}

void EPA::exportState(const std::string& path)
{
    double *vx = new double[3 * vertices.size()], *vPtr = vx;
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

glm::dvec3 EPA::normal(uint32_t a, uint32_t b, uint32_t c)
{
    return glm::normalize(glm::cross(vertices[b].val - vertices[a].val, vertices[c].val - vertices[a].val));
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

void EPA::prepareFacets(const Simplex& simplex, std::vector<std::pair<double, uint32_t>>& order)
{
    for (uint32_t i = 0; i < 4; i++) vertices.push_back(simplex[i]);

    std::array<Triangle, 4> indices = {
            Triangle{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {3, 2, 1}
    };

    std::vector<std::vector<uint32_t>> neighbors = {{2, 3, 1}, {0, 3, 2}, {1, 3, 0}, {1, 0, 2}};

    for (uint32_t i = 0; i < 4; i++) {
        glm::dvec3 norm = normal(indices[i].I0, indices[i].I1, indices[i].I2);
        facets.push_back({ indices[i], norm, glm::dot(norm, -vertices[indices[i].I0].val), neighbors[i], true });
    }

    order = {
            {facets[0].value, 0}, {facets[1].value, 1}, {facets[2].value, 2}, {facets[3].value, 3}
    };

    std::sort(order.begin(), order.end(), [](const std::pair<double, uint32_t>& a, const std::pair<double, uint32_t>& b) {
        return a.first > b.first;
    });
}

void EPA::prepareFacets(const std::vector<uint32_t>& horizon, std::vector<std::pair<double, uint32_t>>& order) {

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
        glm::dvec3 norm = normal(triangle.I0, triangle.I1, triangle.I2);

        Facet facet = {triangle, norm, glm::dot(norm, -vertices[triangle.I0].val), {lastFacet, horizon[j], nextFacet}, true };
        order.emplace_back(facet.value, facets.size());

        if (std::isnan(facet.value)) {
            std::cout << "EPA BR " << norm.x << " " << norm.y << " " << norm.z << " | " << triangle.I0 << " " << triangle.I1 << " " << triangle.I2
                << "\n" << lastFacet << " " << horizon[j] << " " << nextFacet << "\n";
            throw std::runtime_error("[EPA] Error preparing facets for algorithm");
        }

        facets.push_back(facet);
        lastVertex = horizon[j + 1];
        lastFacet = currentFacet;
    }

    // TODO better than sort
    std::sort(order.begin(), order.end(), [](const std::pair<double, uint32_t>& a, const std::pair<double, uint32_t>& b) {
        return a.first > b.first;
    });
}

void EPA::calculateHorizon(const glm::dvec3& apex, int64_t last, uint32_t current, std::vector<uint32_t>& horizon){
    if(!facets[current].onHull){
        return;
    }

    double test = glm::dot(facets[current].normal, apex - vertices[facets[current].triangle.I0].val);

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

glm::dvec3 EPA::overlap() const
{
    return m_colliding ? m_worldOffset : glm::dvec3();
}
glm::dvec3 EPA::offset() const
{
    return m_colliding ? glm::dvec3() : m_worldOffset;
}

const std::pair<uint32_t, uint32_t>& EPA::nearest() const
{
    return m_nearest;
}

const glm::dvec3& EPA::colliderAClosestLocal() const
{
    return m_aLocal;
}
const glm::dvec3& EPA::colliderAClosest() const
{
    return m_aWorld;
}

const glm::dvec3& EPA::colliderBClosestLocal() const
{
    return m_bLocal;
}
const glm::dvec3& EPA::colliderBClosest() const
{
    return m_bWorld;
}

double EPA::distance() const
{
    return glm::length(m_aWorld - m_bWorld);
}