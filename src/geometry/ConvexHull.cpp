//
// Created by cameronh on 24/08/23.
//
//#define ENABLE_VHACD_IMPLEMENTATION 1

#include "ConvexHull.h"

#include "Mesh.h"
#include "EPA.h"


#include <QVector3D>
#include <glm.hpp>

#include "Core/Timer.h"

#include <thread>
#include <iostream>
#include <numeric>
#include <utility>

ConvexHull::ConvexHull()
    : m_center()
    , m_faces(nullptr, nullptr, 0)
{
}

ConvexHull::ConvexHull(const VertexArray&  cloud)
    : ConvexHull(cloud.vertices())
{

}

ConvexHull::ConvexHull(const std::vector<glm::vec3>& cloud)
    : m_center()
    , m_cloud(cloud)
    , m_faces(nullptr, nullptr, 0)

{
    initialize();
}

ConvexHull::ConvexHull(const std::shared_ptr<Mesh>& mesh)
        : m_center(0)
        , m_cloud(std::vector<glm::vec3>())
        , m_faces(nullptr, nullptr, 0)
{
    m_cloud.reserve(mesh->vertexCount());

    // Excludes orphan vertices that may be part of the mesh
    std::vector<uint32_t> instances = mesh->faces().instances(mesh->vertexCount());
    for (uint32_t i = 0; i < instances.size(); i++) {
        if (instances[i] > 0) m_cloud.push_back(mesh->vertices()[i]);
    }

    initialize();
}

void ConvexHull::initialize()
{
    if (m_cloud.size() < 4) {
//        std::cout << "\033[31mERROR! Can not generate a 3D convex hull with fewer than 4 vertices\033[0m\n";
        return;
    }

    // Default to minimum hull size that will not introduce potential bugs
    m_vertices.reserve(m_cloud.size());

    std::vector<Triangle> triangles = initialApproximation();

    if (triangles.empty()) {
//        m_cloud.clear();
//        m_cloud.shrink_to_fit();
//        m_vertices.shrink_to_fit();
        return;
    }

    // Initial pass to sort point cloud between tetrahedral facets
    prepareFacets(triangles);

    for (uint32_t i = 0; i < facets.size(); i++) {
        if (facets[i].onHull && !facets[i].outside.empty()) {
            m_vertices.push_back(m_cloud[facets[i].outside[0]]);

            std::vector<uint32_t> horizon, set;
            calculateHorizon(m_vertices[m_vertices.size() - 1], -1, i, horizon, set);
            prepareFacets(horizon, set);// Generate a strip of new facets between the apex and the horizon
        }
    }

    // Attach all remaining triangles to the convex hull
    uint32_t idx = 0, count = 0;

    std::vector<std::vector<uint32_t>> faces;

    for(Facet& facet : facets){
        if(facet.onHull){
            uint32_t last = facet.triangle.I0;

            faces.emplace_back();
            std::vector<uint32_t> &face = faces[faces.size() - 1];

            std::vector<std::pair<uint32_t, uint32_t>> neighbors = {
                    {idx, facet.neighbors[0]}, {idx, facet.neighbors[1]}, {idx, facet.neighbors[2]}
            };

            // Identify adjacent facets with collinear normals to record as one face
            facet.onHull = false;
            for(uint32_t i = 0; i < neighbors.size(); i++){
                Facet& neighbor = facets[neighbors[i].second];

                if(neighbor.onHull && glm::dot(neighbor.normal, facet.normal) > 1 - 2 * std::numeric_limits<float>::epsilon()){

                    uint32_t ref = std::find(neighbor.neighbors.begin(), neighbor.neighbors.end(), neighbors[i].first) - neighbor.neighbors.begin();

                    neighbors.insert(neighbors.begin() + i + 1, {neighbors[i].second, neighbor.neighbors[(ref + 2) % 3]});
                    neighbors[i] = {neighbors[i].second, neighbor.neighbors[(ref + 1) % 3]};
                    neighbor.onHull = false;
                    i--;
                }
            }

            for (const auto &neighbor : neighbors) {
                const Triangle &tri = facets[neighbor.second].triangle;
                uint32_t ref = tri.I0 == last ? 0 : (tri.I1 == last ? 1 : 2);
                last = tri[(ref + 2) % 3];
                face.push_back(last);
            }

            count += face.size();
        }

        idx++;
    }

    // TODO record vertex connections

    // Do away with working memory
    facets.clear();

    m_faces = FaceArray(faces.size(), count);

    uint32_t *idxPtr = m_faces[0], *sizePtr = m_faces.faceSizes();
    for (const auto& face : faces) {
        *sizePtr++ = face.size();
        for (uint32_t index : face) *idxPtr++ = index;
    }


    // Calculate convex hull center
    for (uint32_t i = 0; i < m_vertices.size(); i++) {
        m_center += m_vertices[i];
    }
    m_center = m_center * (1 / (float)m_vertices.size());

    m_walks = m_faces.edgeList();
}

uint32_t ConvexHull::vertexCount() const
{
    return m_vertices.size();
}

const std::vector<glm::vec3>& ConvexHull::vertices() const
{
    return m_vertices;
}

uint32_t ConvexHull::facetCount() const
{
    return m_faces.faceCount();
}

const FaceArray& ConvexHull::faces() const
{
    return m_faces;
}

glm::vec3 ConvexHull::facetNormal(uint32_t idx) const
{
    const uint32_t *loop = m_faces[idx];
    if (loop == nullptr) return {};

    return Triangle::normal(m_vertices[loop[0]], m_vertices[loop[1]], m_vertices[loop[2]]);
}

glm::vec3 ConvexHull::center() const
{
    return m_center;
}

bool ConvexHull::empty() const
{
    return m_vertices.empty();
}

EPA ConvexHull::epaIntersection(const ConvexHull& body, const glm::mat4& transform, const glm::mat4& relativeTransform, std::pair<uint32_t, uint32_t>& idx) const
{
    return { *this, body, transform, relativeTransform, gjkIntersection(body, relativeTransform, idx) };
}

glm::vec3 ConvexHull::initialAxis(const Sphere& body, std::pair<uint32_t, uint32_t>& idx) const
{
    if (idx.first != std::numeric_limits<uint32_t>::max()) {
        return m_vertices[idx.first] - body.center;
    }

    idx = { 0, 0 };

    return m_center - body.center;
}

glm::vec3 ConvexHull::initialAxis(const ConvexHull& body, std::pair<uint32_t, uint32_t>& idx) const
{
    if (idx.first != std::numeric_limits<uint32_t>::max() && idx.second != std::numeric_limits<uint32_t>::max()) {
        return m_vertices[idx.first] - body.m_vertices[idx.second];
    }

    idx = { 0, 0 };

    return m_center - body.m_center;
}

glm::vec3 ConvexHull::gjkSupport(const Sphere& body, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx) const
{
    idx.first = walk(axis, idx.first);
//    idx.second = 0;

    glm::vec4 vec = transform * glm::vec4(body.center.x, body.center.y, body.center.z, 1.0f);
    glm::vec3 nearest = glm::vec3{ vec.x, vec.y, vec.z } - glm::normalize(axis) * body.radius;

    return m_vertices[idx.first] - nearest;
}

glm::vec3 ConvexHull::gjkSupport(const ConvexHull& body, const glm::mat4& transform, const glm::vec3& axis, std::pair<uint32_t, uint32_t>& idx) const
{
    idx.first = walk(axis, idx.first);
    idx.second = body.walk(-axis * glm::mat3(transform), idx.second);

    glm::vec4 vec = transform * glm::vec4(body.m_vertices[idx.second].x, body.m_vertices[idx.second].y, body.m_vertices[idx.second].z, 1.0f);

    return m_vertices[idx.first] - glm::vec3{ vec.x, vec.y, vec.z };
}

uint32_t ConvexHull::walk(const glm::vec3& axis, uint32_t index) const
{
    uint32_t i = 0;
    while (i == 0) { // Only true when arriving at a fresh vertex
        for (; i < m_walks[index].size(); i++) { // Iterate through adjacent vertices
            if (glm::dot(axis, m_vertices[m_walks[index][i]] - m_vertices[index]) > 1e-6) { // Move to the first adjacent vertex along the axis
                index = m_walks[index][i];
                i = 0;
                break;
            }
        }
    }

    return index;
}

std::vector<glm::vec3> ConvexHull::border(const glm::vec3& faceNormal) const
{
    float value = std::numeric_limits<float>::lowest();
    uint32_t faceIndex = 0;

    for (uint32_t i = 0; i < m_faces.faceCount(); i++) {
        float test = glm::dot(faceNormal, m_faces.normal(i));
        if (value < test) {
            faceIndex = i;
            value = test;
        }
    }

    auto *ptr = m_faces[faceIndex];
    std::vector<glm::vec3> border;
    border.reserve(m_faces.faceSizes()[faceIndex]);

    for (uint32_t i = 0; i < m_faces.faceSizes()[faceIndex]; i++) {
        border.emplace_back(m_vertices[*ptr++]);
    }

    return border;
}

std::vector<uint32_t> ConvexHull::horizon(const glm::vec3& axis) const
{
    bool xN = axis.x * axis.x == 1;
    return horizon(axis, glm::vec3 { !xN, xN, 0});
}
std::vector<uint32_t> ConvexHull::horizon(const glm::vec3& axis, const glm::vec3& support) const
{

    uint32_t idx = 0;
    std::vector<uint32_t> boundary(3);
    VertexArray::extremes(m_vertices, support, boundary[1], boundary[0]);

    boundary[2] = boundary[0]; // Duplicate first vertex so the algorithm wraps

    while (idx < boundary.size() - 1) {
        glm::vec3 vec = glm::normalize(glm::cross(m_vertices[boundary[idx + 1]] - m_vertices[boundary[idx]], axis));
        uint32_t next = walk(vec, boundary[idx]);
        if (next != boundary[idx] && next != boundary[idx + 1]) {
            boundary.insert(boundary.begin() + idx + 1, next);
        } else idx++;

        // Check
        if (boundary.size() > m_vertices.size()) throw std::runtime_error("[ConvexHull] Failed to find a horizon!");
    }

    // Remove duplicate vertex (First and last are the same). Chooses initial vertex to begin from top-left position
    if (glm::dot(support, m_vertices[boundary[1]] - m_vertices[boundary[0]]) > -1e-6)
        boundary.erase(boundary.begin());
    else boundary.pop_back();

    return boundary;
}

std::vector<Triangle> ConvexHull::initialApproximation(){
    std::vector<Triangle> triangles;

    // Find a pair of extreme points of the point cloud
    std::vector<uint32_t> extremes = { 0, 0, 0, 0 };
    glm::vec3 axes[3] = { glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1) };
    for (const glm::vec3& axis : axes) {
        if (VertexArray::extremes(m_cloud, axis, extremes[0], extremes[1])) {
            break;
        }
    }

    // Prepare a maximal tetrahedron - exits early if the cloud is degenerate in some dimension
    if (extremes[0] == extremes[1]) {
        std::cout << "\033[31mERROR! Can not generate a 3D convex hull from the cloud! The cloud is 0D degenerate\033[0m\n";
        return {};
    }

    if (!VertexArray::extreme(m_cloud, extremes[0], extremes[1], extremes[2])) {
        std::cout << "\033[31mERROR! Can not generate a 3D convex hull from the cloud! The cloud is 1D degenerate\033[0m\n";
        return {};
    }

    if (!VertexArray::extreme(m_cloud, extremes[0], extremes[1], extremes[2], extremes[3])) {
        std::cout << "\033[31mERROR! Can not generate a 3D convex hull from the cloud! The cloud is 2D degenerate\033[0m\n";
        return {};
    }

    // Add extremities to the convex hull
    for (uint32_t extreme : extremes) {
        m_vertices.push_back(m_cloud[extreme]);
    }

    // Erase extreme vertices from cloud to prevent consideration later on (can otherwise introduce precision errors)
    std::sort(extremes.begin(), extremes.end(), [](uint32_t a, uint32_t b){ return b < a; });
    for (uint32_t extreme : extremes) m_cloud.erase(m_cloud.begin() + extreme);

    // Swap vertices if needed to ensure proper winding
    auto ref = m_vertices[3] - m_vertices[0];
    auto normal = Triangle::normal(m_vertices[0], m_vertices[1], m_vertices[2]);

    if (glm::dot(ref, normal) > 0) {
        std::swap(m_vertices[0], m_vertices[1]);
    }

    // Prepare initial triangle approximation (tetrahedron)
    triangles.emplace_back(0, 1, 2);
    triangles.emplace_back(0, 2, 3);
    triangles.emplace_back(0, 3, 1);
    triangles.emplace_back(3, 2, 1);

    return triangles;
}

glm::vec3 ConvexHull::wNormal(const Triangle& triangle)
{
    return Triangle::normal(m_vertices[triangle.I0], m_vertices[triangle.I1], m_vertices[triangle.I2]);
}

void ConvexHull::prepareFacets(const std::vector<Triangle>& triangles){
    std::vector<uint32_t> free(m_cloud.size());
    std::iota(free.begin(), free.end(), 0);

    std::vector<std::vector<uint32_t>> neighbors = { { 2, 3, 1 }, { 0, 3, 2 }, { 1, 3, 0 }, { 1, 0, 2 } };
    for (uint32_t i = 0; i < triangles.size(); i++) {

        glm::vec3 normal = wNormal(triangles[i]);

        Facet facet = {triangles[i], normal, {}, neighbors[i], true};
        sortCloud(free, facet);
        facets.push_back(facet);
    }
}

void ConvexHull::prepareFacets(const std::vector<uint32_t>& horizon, std::vector<uint32_t>& set) {

    // Create a chain of triangular facets between the apex and its respective horizon
    uint32_t lastVertex = horizon[horizon.size() - 1], lastFacet = facets.size() + (horizon.size() / 2) - 1, start = facets.size();
    for (uint32_t j = 0; j < horizon.size(); j += 2) {

        // Identify neighbors for linking
        uint32_t currentFacet = facets.size(), nextFacet = (currentFacet + 1 - start) % (horizon.size() / 2) + start;
        uint32_t z = (facets[horizon[j]].triangle[1] == horizon[j + 1]) + 2 * (facets[horizon[j]].triangle[2] == horizon[j + 1]);
        facets[horizon[j]].neighbors[z] = currentFacet;// Link against existing facet beyond the horizon

        // Prepare the new facet and link
        Triangle triangle = { (uint32_t)m_vertices.size() - 1, lastVertex, horizon[j + 1] };
        glm::vec3 normal = wNormal(triangle);

        Facet facet = {triangle, normal, {}, {lastFacet, horizon[j], nextFacet}, true};
        sortCloud(set, facet);

        facets.push_back(facet);
        lastVertex = horizon[j + 1];
        lastFacet = currentFacet;
    }
}

void ConvexHull::sortCloud(std::vector<uint32_t>& free, Facet& facet){
    float value = -1;
    int64_t peak = -1;

    for (uint32_t i = 0; i < free.size(); i++) { // Iterate through unsorted cloud
        float test = glm::dot(facet.normal, m_cloud[free[i]] - m_vertices[facet.triangle.I0]);
        if (test > 1e-6) {//std::numeric_limits<float>::epsilon()
            if (test > value) {
                value = test;
                peak = (int64_t)facet.outside.size();
            }

            // Assign the free vertex to this facet
            facet.outside.push_back(free[i]);
            std::swap(free[i--], free[free.size() - 1]);
            free.pop_back();
        }
    }

    // Identify the furthest point in the cloud (for refinement step)
    if (peak > 0) {
        std::swap(facet.outside[peak], facet.outside[0]);
    }
}

void ConvexHull::calculateHorizon(const glm::vec3& apex, int64_t last, uint32_t current, std::vector<uint32_t>& horizon, std::vector<uint32_t>& set){
    if (!facets[current].onHull) {
        return;
    }

    float test = glm::dot(facets[current].normal, apex - m_vertices[facets[current].triangle.I0]);

    if (test > 1e-6) { // Check whether facet is visible to apex
        set.insert(set.end(), facets[current].outside.begin(), facets[current].outside.end());
        facets[current].outside.clear();
        facets[current].onHull = false;

        // Identify neighbors to visit
        std::vector<uint32_t> neighbors = { 0, 0, 0 };
        uint32_t start = last == -1 ? 0 : std::find(facets[current].neighbors.begin(), facets[current].neighbors.end(), last) - facets[current].neighbors.begin();
        neighbors[0] = facets[current].neighbors[start];
        neighbors[1] = facets[current].neighbors[(start + 1) % 3];
        neighbors[2] = facets[current].neighbors[(start + 2) % 3];

        for (uint32_t neighbor : neighbors) {
            calculateHorizon(apex, current, neighbor, horizon, set);
        }

        return;
    }


    uint32_t index = std::find(facets[current].neighbors.begin(), facets[current].neighbors.end(), last) - facets[current].neighbors.begin();

    horizon.push_back(current);// Encode information about adjacent facet for linking purposes
    horizon.push_back(facets[current].triangle[index]);// Encode next vertex on horizon
}

std::vector<glm::vec3> ConvexHull::intersection(const glm::vec3& origin, const glm::vec3& normal) const
{
    if (m_vertices.empty()) return {};

    std::vector<bool> above(m_vertices.size());
    if (partition(origin, normal, above)) return {};

    return intersection(origin, normal, above);
}

bool ConvexHull::above(const glm::vec3& origin, const glm::vec3& normal) const
{
    if (m_vertices.empty()) return false;

    std::vector<bool> above(m_vertices.size());
    return partition(origin, normal, above) && above[0];
}
bool ConvexHull::below(const glm::vec3& origin, const glm::vec3& normal) const
{
    if (m_vertices.empty()) return false;

    std::vector<bool> below(m_vertices.size());
    return partition(origin, -normal, below) && below[0];
}

bool ConvexHull::intersects(const glm::vec3& origin, const glm::vec3& normal) const
{
    if (m_vertices.empty()) return false;

    // Determine whether vertices exist on both sides of the cutting plane
    bool ref = glm::dot(normal, m_vertices[0] - origin) > -1e-6;
    for (uint32_t i = 1; i < m_vertices.size(); i++) {
        if ((glm::dot(normal, m_vertices[i] - origin) > -1e-6) != ref) return true;
    }

    return false;
}
bool ConvexHull::intersects(const Sphere& sphere) const
{
    std::pair<uint32_t, uint32_t> indices = { 0, 0 };
    auto simplex = gjkIntersection(sphere, glm::mat4(1.0f), indices);
    return simplex.colliding();
}


bool ConvexHull::partition(const glm::vec3& origin, const glm::vec3& normal, std::vector<bool>& above) const
{
    uint32_t sum = 0;

    // Determine which vertices are above the cut plane
    for (uint32_t i = 0; i < m_vertices.size(); i++) {
        above[i] = glm::dot(normal, m_vertices[i] - origin) > -1e-6;
        sum += above[i];
    }

    // Indicate when all vertices are either below or above the plane
    return sum == 0 || sum == above.size();
}

ConvexHull ConvexHull::fragment(const glm::vec3& origin, const glm::vec3& normal) const
{
    if (m_vertices.empty()) return {};

    std::vector<bool> above(m_vertices.size());
    if (partition(origin, normal, above)) { // Identify vertices above and below the plane

        // Exit if no edges intersect with the plane
        if (above[0]) return *this;
        else return {};
    }

    // Find vertex intersections of hull with plane
    std::vector<glm::vec3> set = intersection(origin, normal, above);


    // Attach vertices on the positive side of the plane
    for (uint32_t i = 0; i < above.size(); i++) if (above[i]) set.push_back(m_vertices[i]);

    return { VertexArray::clean(set) };
}

std::pair<ConvexHull, ConvexHull> ConvexHull::fragments(const glm::vec3& origin, const glm::vec3& normal) const
{
    if (m_vertices.empty()) return {};

    std::vector<bool> above(m_vertices.size());
    if (partition(origin, normal, above)) { // Identify vertices above and below the plane

        // Exit if no edges intersect with the plane
        if (above[0]) return { *this, {} };
        else return { {}, *this };
    }

    std::vector<glm::vec3> setA = intersection(origin, normal, above);

    // Duplicate intersection vertices in other set
    std::vector<glm::vec3> setB = setA;

    // Attach original vertices to respective fragments
    for (uint32_t i = 0; i < above.size(); i++) {
        if (above[i]) setA.push_back(m_vertices[i]);
        else setB.push_back(m_vertices[i]);
    }

    return { VertexArray::clean(setA), VertexArray::clean(setB) };
}

// TODO Improvement: Leverage walk to calculate intersection in-place
std::vector<glm::vec3> ConvexHull::intersection(const glm::vec3& origin, const glm::vec3& normal, const std::vector<bool>& above) const
{

    float d = glm::dot(origin, normal);

    // Find vertices on the cut plane
    std::vector<glm::vec3> intersection;
    for (uint32_t i = 0; i < m_vertices.size(); i++) {
        for (uint32_t j : m_walks[i]) {
            if (i < j && above[i] != above[j]) { // Skip repeated edges, edges that do not cross the plane
                glm::vec3 vertex = m_vertices[i], edge = m_vertices[j] - vertex;
                float t = (d - glm::dot(normal, vertex)) / glm::dot(normal, edge);
                intersection.emplace_back(vertex + edge * t);
            }
        }
    }

    return intersection;
}

uint32_t ConvexHull::step(const glm::vec3& normal, const glm::vec3& axis, uint32_t index) const
{
    uint32_t next = std::numeric_limits<uint32_t>::max();
//    float
    for (uint32_t walk : m_walks[index]) {
        if (glm::dot(normal, m_vertices[walk] - m_vertices[index]) > 0 ) {
            if (next == std::numeric_limits<uint32_t>::max()) {

            }
        }
    }

    return next;
}

//bool ConvexHull::raycast(const glm::vec3& origin, const glm::vec3& direction, float& t) const {
//    bool hit = false;
//
//    for (const Triangle& tri: m_Triangles) {
//        glm::vec2 intersection;
//        float test;
//
//        if (glm::intersectRayTriangle(origin, direction, m_Vertices[tri.m_I0], m_Vertices[tri.m_I1], m_Vertices[tri.m_I2], intersection, test)) {
//            if (test >= 0 && test < t) {
//                t = test;
//                hit = true;
//            }
//        }
//    }
//
//    return hit;
//}
//
//bool ConvexHull::raycast(const glm::vec3& origin, const glm::vec3& direction) const {
//    for (const Triangle& tri: m_Triangles) {
//        glm::vec2 intersection;
//        float test;
//
//        if (glm::intersectRayTriangle(origin, direction, m_Vertices[tri.m_I0], m_Vertices[tri.m_I1], m_Vertices[tri.m_I2], intersection, test)) {
//            return true;
//        }
//    }
//
//    return false;
//}