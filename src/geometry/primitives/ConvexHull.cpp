//
// Created by cameronh on 24/08/23.
//
//#define ENABLE_VHACD_IMPLEMENTATION 1

#include "ConvexHull.h"

#include "geometry/Mesh.h"

#include <glm.hpp>

#include "geometry/collision/Collision.h"
#include "geometry/primitives/Plane.h"
#include "geometry/poly/Polygon.h"

#include <thread>
#include <iostream>

ConvexHull::ConvexHull()
    : m_initialized(false)
    , m_faces(0, 0)
    , m_volume(-1.0)
    , m_walkStart(0)
    , m_orphans(false)
{
}

ConvexHull::ConvexHull(std::ifstream& file)
    : ConvexHull()
{
    ConvexHull::deserialize(file);
}

ConvexHull::ConvexHull(const VertexArray& cloud)
    : ConvexHull()
{
    auto builder = HullBuilder(cloud.vertices());

    // TODO remove: temporary tracking if a cloud with degen points was somehow created (Should not happen within the program)
    builder.clean();
    build(builder);
}

ConvexHull::ConvexHull(const std::shared_ptr<Mesh>& mesh)
        : ConvexHull(mesh->vertices())
{
    if (mesh->faces().hasFreeIndices(mesh->vertexCount())) {
        std::cout << "\033[93m[ConvexHull] Generated from a mesh with orphan vertices! Result may noy be as expected\033[0m\n";
    }
}

// Unlike when passing a Mesh or VertexArray, a raw point cloud is not assumed to be properly formed. The cloud will be cleaned before processing
ConvexHull::ConvexHull(const std::vector<glm::dvec3>& cloud)
    : ConvexHull()
{
    auto builder = HullBuilder(cloud);
    builder.clean();
    build(builder);
}

void ConvexHull::build(HullBuilder& builder)
{
    builder.initialize();
    builder.solve();

    m_vertices = builder.getVertices();

    m_faces = builder.getFaces();

    initialize();
}

void ConvexHull::initialize()
{
    if (m_vertices.empty() || m_faces.empty()) return;

    // Calculate convex hull center
    m_bounds = Sphere::enclose(m_vertices);

    m_walks = m_faces.edgeList();

    // Verify that the hull is well-formed (If not walk() defaults to a less efficient method)
    for (const std::vector<uint32_t>& walks : m_walks) {
        if (walks.empty()) {
            m_orphans = true;
            break;
        }
    }

    m_initialized = true;
}

bool ConvexHull::serialize(std::ofstream& file) const
{
    Serializer::writeBool(file, m_initialized);

    if (m_initialized) {
        VertexArray(m_vertices).serialize(file);
        m_faces.serialize(file);
    }

    return true;
}
bool ConvexHull::deserialize(std::ifstream& file)
{
    m_initialized = Serializer::readBool(file);

    if (m_initialized) {
        m_vertices = VertexArray(file).vertices();
        m_faces = FaceArray::deserialize(file);
        initialize();
    }

    return true;
}

// Calculate additional information like volume that is normally skipped to save time
void ConvexHull::evaluate()
{
    if (m_initialized && m_volume < 0) {
        m_faces.triangulate(m_vertices);
        m_volume = m_faces.volume(m_vertices);
    }
}

void ConvexHull::translate(const glm::dvec3& translation)
{
    for (glm::dvec3& vertex : m_vertices) vertex += translation;
}

void ConvexHull::setWalkStart(uint32_t startIndex)
{
    if (startIndex < m_walks.size()) {
        m_walkStart = startIndex;
        if (m_walks[m_walkStart].empty()) std::cout << "\033[93m[ConvexHull] Walk start assigned to orphan vertex\033[0m\n";
    } else throw std::runtime_error("[ConvexHull] Invalid walk start assignment");
}
bool ConvexHull::getWalkStart() const
{
    return m_walkStart;
}

bool ConvexHull::isValid() const
{
    return m_initialized && !m_vertices.empty();
}

uint32_t ConvexHull::vertexCount() const
{
    return m_vertices.size();
}

const std::vector<glm::dvec3>& ConvexHull::vertices() const
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

const Sphere& ConvexHull::bounds() const
{
    return m_bounds;
}

Plane ConvexHull::facePlane(uint32_t idx) const
{
    if (idx >= m_faces.faceCount()) throw std::runtime_error("[ConvexHull] Index out of bounds. Can not generate plane");
    return { m_vertices[m_faces[idx][0]], m_faces.normal(idx) };
}

bool ConvexHull::empty() const
{
    return m_vertices.empty();
}

uint32_t ConvexHull::walk(const glm::dvec3& axis) const
{
    uint32_t index = m_walkStart;

    // Skip over any orphan vertices
//    while (index < m_walks.size() && m_walks[index].empty()) index++;

    return walk(axis, m_walkStart);
}

uint32_t ConvexHull::walk(const glm::dvec3& axis, uint32_t index) const
{
    if (index >= m_walks.size()) {
        std::cout << "HULL: " << index << " " << m_walks.size() << "\n";
        print();
        throw std::runtime_error("[ConvexHull] Index out of bounds. Can not walk");
    } else if (m_orphans) {
//        std::cout << "\033[93m[ConvexHull] Orphaned start point. Can not walk. Using fallback method\033[0m\n";
        VertexArray::extreme(m_vertices, axis, index);
        return index;
    }

    uint32_t i = 0;
    while (i == 0) { // Only true when arriving at a fresh vertex
        for (; i < m_walks[index].size(); i++) { // Iterate through adjacent vertices
            if (glm::dot(axis, m_vertices[m_walks[index][i]] - m_vertices[index]) > 1e-12) { // Move to the first adjacent vertex along the axis
                index = m_walks[index][i];
                i = 0;
                break;
            }
        }
    }

    return index;
}

std::tuple<uint32_t, uint32_t> ConvexHull::extremes(const glm::dvec3& axis) const
{
    return { walk(-axis), walk(axis) };
}

double ConvexHull::span(const glm::dvec3& axis) const
{
    auto [min, max] = extremes(axis);
    return glm::dot(axis, m_vertices[max] - m_vertices[min]);
}

double ConvexHull::far(const glm::dvec3& axis) const
{
    auto [idx, vertex] = extreme(axis);
    return glm::dot(axis, vertex);
}

const glm::dvec3& ConvexHull::start() const
{
    return m_vertices[m_walkStart];
}

uint32_t ConvexHull::supportIndex(const glm::dvec3& axis) const
{
    return supportIndex(axis, m_walkStart);
}
std::tuple<uint32_t, glm::dvec3> ConvexHull::extreme(const glm::dvec3& axis) const
{
    return extreme(axis, m_walkStart);
}

uint32_t ConvexHull::supportIndex(const glm::dvec3& axis, uint32_t startIndex) const
{
    return walk(axis, startIndex);
}

std::tuple<uint32_t, glm::dvec3> ConvexHull::extreme(const glm::dvec3& axis, uint32_t startIndex) const
{
    auto index = supportIndex(axis, startIndex);
    return { index, m_vertices[index] };
}


void ConvexHull::extents(const glm::dvec3& axis, double& near, double& far) const
{
    {
        auto [idx, vertex] = extreme(-axis);
        near = glm::dot(axis, vertex);
    }

    {
        auto [idx, vertex] = extreme(axis);
        far = glm::dot(axis, vertex);
    }
}

const std::vector<uint32_t>& ConvexHull::neighbors(uint32_t index) const
{
    if (index >= m_walks.size()) throw std::runtime_error("[ConvexHull] Invalid walk access");
    return m_walks[index];
}

std::vector<glm::dvec3> ConvexHull::border(const glm::dvec3& faceNormal) const
{
    double value = std::numeric_limits<double>::lowest();
    uint32_t faceIndex = 0;

    for (uint32_t i = 0; i < m_faces.faceCount(); i++) {
        double test = glm::dot(faceNormal, m_faces.normal(i));
        if (value < test) {
            faceIndex = i;
            value = test;
        }
    }

    auto *ptr = m_faces[faceIndex];
    std::vector<glm::dvec3> border;
    border.reserve(m_faces.faceSizes()[faceIndex]);

    for (uint32_t i = 0; i < m_faces.faceSizes()[faceIndex]; i++) {
        border.emplace_back(m_vertices[*ptr++]);
    }

    return border;
}

std::vector<uint32_t> ConvexHull::horizon(const glm::dvec3& axis) const
{
    bool xN = axis.x * axis.x == 1;
    return horizon(axis, glm::dvec3 { !xN, xN, 0});
}
std::vector<uint32_t> ConvexHull::horizon(const glm::dvec3& axis, const glm::dvec3& support) const
{

    uint32_t idx = 0;
    std::vector<uint32_t> boundary(3);
    VertexArray::extremes(m_vertices, support, boundary[1], boundary[0]);

    boundary[2] = boundary[0]; // Duplicate first vertex so the algorithm wraps

    while (idx < boundary.size() - 1) {
        glm::dvec3 vec = glm::normalize(glm::cross(m_vertices[boundary[idx + 1]] - m_vertices[boundary[idx]], axis));
        uint32_t next = walk(vec, boundary[idx]);
        if (next != boundary[idx] && next != boundary[idx + 1]) {
            boundary.insert(boundary.begin() + idx + 1, next);
        } else idx++;

        // Check
        if (boundary.size() > m_vertices.size()) throw std::runtime_error("[ConvexHull] Failed to find a horizon!");
    }

    // Remove duplicate vertex (First and last are the same). Chooses initial vertex to begin from top-left position
    if (glm::dot(support, m_vertices[boundary[1]] - m_vertices[boundary[0]]) > -1e-12)
        boundary.erase(boundary.begin());
    else boundary.pop_back();

    return boundary;
}

uint32_t ConvexHull::step(const glm::dvec3& normal, const glm::dvec3& axis, uint32_t index) const
{
    uint32_t next = std::numeric_limits<uint32_t>::max();
//    double
    for (uint32_t walk : m_walks[index]) {
        if (glm::dot(normal, m_vertices[walk] - m_vertices[index]) > 0 ) {
            if (next == std::numeric_limits<uint32_t>::max()) {

            }
        }
    }

    return next;
}

double ConvexHull::volume() const
{
    if (m_volume < 0) {
        std::cout << "\033[93m[ConvexHull] Volume calculation result not recorded. Use evaluate() to store result for repeated uses of volume()\033[0m\n";
        auto faces = m_faces;
        faces.triangulate(m_vertices);
        return faces.volume(m_vertices);
    }

    return m_volume;
}

ConvexHull ConvexHull::unite(const ConvexHull& hullA, const ConvexHull& hullB)
{
    auto vertices = hullA.vertices();
    vertices.insert(vertices.end(), hullB.vertices().begin(), hullB.vertices().end());
    return ConvexHull(vertices);
}
std::tuple<bool, ConvexHull> ConvexHull::tryMerge(const ConvexHull& hullA, const ConvexHull& hullB)
{
    auto collision = Collision::intersection(hullA, hullB, { 0, 0 });
    glm::dvec3 delta = collision.colliding() ? collision.overlap() : collision.offset();

//    std::cout << collision.colliding() << " " << glm::length(collision.offset()) << " " << glm::length(collision.overlap()) << "}{{}\n";
    if (glm::dot(delta, delta) < 1e-12) {
        auto hull = unite(hullA, hullB);
        hull.evaluate();

//        std::cout << "Volumes: " << hull.volume() << " " << hullA.volume() << " " << hullB.volume() << " = " << (hullA.volume() + hullB.volume()) << "\n";
        if (std::abs(hull.volume() - hullA.volume() - hullB.volume()) < 1e-12) return { true, hull };
    }
    return { false, {} };
}

void ConvexHull::print() const
{
    std::cout << "[ConvexHull] Vertices: " << m_vertices.size() << ", Faces: " << m_faces.faceCount() << "\nVertices:\n";
    for (const glm::dvec3& vertex : m_vertices) std::cout << vertex.x << " " << vertex.y << " " << vertex.z << "\n";

    m_faces.print();

    std::cout << "Edges:\n";
    for (const auto& walk : m_walks) {
        std::cout << &walk - &m_walks[0] << ": ";
        for (auto w : walk) std::cout << w << " ";
        std::cout << "\n";
    }
}

//bool ConvexHull::raycast(const glm::dvec3& origin, const glm::dvec3& direction, double& t) const {
//    bool hit = false;
//
//    for (const Triangle& tri: m_Triangles) {
//        glm::vec2 intersection;
//        double test;
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
//bool ConvexHull::raycast(const glm::dvec3& origin, const glm::dvec3& direction) const {
//    for (const Triangle& tri: m_Triangles) {
//        glm::vec2 intersection;
//        double test;
//
//        if (glm::intersectRayTriangle(origin, direction, m_Vertices[tri.m_I0], m_Vertices[tri.m_I1], m_Vertices[tri.m_I2], intersection, test)) {
//            return true;
//        }
//    }
//
//    return false;
//}

HullBuilder::HullBuilder(const std::vector<glm::dvec3>& cloud)
        : m_cloud(cloud)
        , m_initialized(false)
        , m_iteration(0)
{
    assert(!cloud.empty());
}

void HullBuilder::clean()
{
    const uint32_t initialSize = m_cloud.size();
    m_cloud = VertexArray::clean(m_cloud);
    if (initialSize > m_cloud.size()) std::cout << "\033[93mTried to construct a hull from a cloud with duplicate vertices. " << initialSize - m_cloud.size() << " vertices removed\033[0m\n";
}

void HullBuilder::initialize()
{
    if (m_cloud.size() < 4) return;

    std::vector<TriIndex> triangles = initialApproximation();

    // Only proceed if approximation was successful
    if (!triangles.empty()) {

        m_vertices.reserve(4 + m_cloud.size());

        // Initial pass to sort point cloud between tetrahedral facets
        prepareFacets(triangles);

        m_initialized = true;
    }
}

void HullBuilder::step()
{
    if (m_initialized && m_iteration < m_facets.size()) iterate();
}

void HullBuilder::solve()
{
    while (m_initialized && m_iteration < m_facets.size()) iterate();
}

void HullBuilder::iterate()
{
    if (m_facets[m_iteration].onHull && !m_facets[m_iteration].outside.empty()) {
        m_vertices.push_back(m_cloud[m_facets[m_iteration].outside[0]]);

        std::vector<uint32_t> horizon, set;
        calculateHorizon(m_vertices[m_vertices.size() - 1], -1, m_iteration, horizon, set);
        prepareFacets(horizon, set);// Generate a strip of new facets between the apex and the horizon
    }

    m_iteration++;
}

std::vector<TriIndex> HullBuilder::initialApproximation(){

    // Find a pair of extreme points of the point cloud
    std::vector<uint32_t> extremes = { 0, 0, 0, 0 };
    glm::dvec3 axes[3] = { glm::dvec3(1, 0, 0), glm::dvec3(0, 1, 0), glm::dvec3(0, 0, 1) };
    for (const glm::dvec3& axis : axes) {
        if (VertexArray::extremes(m_cloud, axis, extremes[0], extremes[1])) {
            break;
        }
    }

    // Prepare a maximal tetrahedron - exits early if the cloud is degenerate in some dimension
    if (extremes[0] == extremes[1]) {
//        std::cout << "\033[31mERROR! Can not generate a 3D convex hull from the cloud! The cloud is 0D degenerate\033[0m\n";
        return {};
    }

    if (!VertexArray::extreme(m_cloud, extremes[0], extremes[1], extremes[2])) {
//        std::cout << "\033[31mERROR! Can not generate a 3D convex hull from the cloud! The cloud is 1D degenerate\033[0m\n";
        return {};
    }

    if (!VertexArray::extreme(m_cloud, extremes[0], extremes[1], extremes[2], extremes[3])) {
//        std::cout << "\033[31mERROR! Can not generate a 3D convex hull from the cloud! The cloud is 2D degenerate\033[0m\n";
        return {};
    }

    // Add extremities to the convex hull
    for (uint32_t extreme : extremes) {
        m_vertices.push_back(m_cloud[extreme]);
    }

    // Erase extreme vertices from cloud to prevent consideration later on (can otherwise introduce precision errors)
    std::sort(extremes.begin(), extremes.end(), [](uint32_t a, uint32_t b){ return b < a; });
    for (uint32_t extreme : extremes) {
        std::swap(m_cloud.back(), m_cloud[extreme]);
        m_cloud.pop_back();
    }

    // Swap vertices if needed to ensure proper winding
    auto ref = m_vertices[3] - m_vertices[0];
    auto normal = Triangle3D::normal(m_vertices[0], m_vertices[1], m_vertices[2]);

    if (glm::dot(ref, normal) > 0) {
        std::swap(m_vertices[0], m_vertices[1]);
    }

    // Prepare initial triangle approximation (tetrahedron)
    return {
            { 0, 1, 2 },
            { 0, 2, 3 },
            { 0, 3, 1 },
            { 3, 2, 1 }
    };
}

//void HullBuilder::purgeOrphans(std::vector<std::vector<uint32_t>>& faces)
//{
//    std::vector<bool> orphans(m_vertices.size(), true);
//    for (const std::vector<uint32_t>& face : faces) {
//        for (uint32_t idx : face) orphans[idx] = false;
//    }
//
//    for (uint32_t i = 0; i < m_vertices.size(); i++) {
//        if (orphans[i]) {
//            m_vertices[i] = m_vertices[m_vertices.size() - 1];
//            m_vertices.pop_back();
//
//            // Correct indexing
//            for (std::vector<uint32_t>& face : faces) {
//                for (uint32_t& idx : face) {
//                    if (idx == m_vertices.size()) idx = i;
//                }
//            }
//
//            orphans[i] = orphans[m_vertices.size()];
//            i--;
//        }
//    }
//}

void HullBuilder::prepareFacets(const std::vector<TriIndex>& triangles){
    std::vector<uint32_t> free(m_cloud.size());
    std::iota(free.begin(), free.end(), 0);

    std::vector<std::vector<uint32_t>> neighbors = { { 2, 3, 1 }, { 0, 3, 2 }, { 1, 3, 0 }, { 1, 0, 2 } };
    for (uint32_t i = 0; i < triangles.size(); i++) {

        glm::dvec3 normal = Triangle3D(m_vertices, triangles[i]).normal();

        Facet facet = {triangles[i], normal, {}, neighbors[i], true};
        sortCloud(free, facet);
        m_facets.push_back(facet);
    }
}

void HullBuilder::prepareFacets(const std::vector<uint32_t>& horizon, std::vector<uint32_t>& set) {

    // Create a chain of triangular facets between the apex and its respective horizon
    uint32_t lastVertex = horizon[horizon.size() - 1], lastFacet = m_facets.size() + (horizon.size() / 2) - 1, start = m_facets.size();
    for (uint32_t j = 0; j < horizon.size(); j += 2) {

        // Identify neighbors for linking
        uint32_t currentFacet = m_facets.size(), nextFacet = (currentFacet + 1 - start) % (horizon.size() / 2) + start;
        uint32_t z = (m_facets[horizon[j]].triangle[1] == horizon[j + 1]) + 2 * (m_facets[horizon[j]].triangle[2] == horizon[j + 1]);
        m_facets[horizon[j]].neighbors[z] = currentFacet;// Link against existing facet beyond the horizon

        // Prepare the new facet and link
        TriIndex triangle = { (uint32_t)m_vertices.size() - 1, lastVertex, horizon[j + 1] };
        glm::dvec3 normal = Triangle3D(m_vertices, triangle).normal();

        Facet facet = {triangle, normal, {}, {lastFacet, horizon[j], nextFacet}, true};
        sortCloud(set, facet);

        m_facets.push_back(facet);
        lastVertex = horizon[j + 1];
        lastFacet = currentFacet;
    }
}

void HullBuilder::sortCloud(std::vector<uint32_t>& free, Facet& facet){
    double value = -1;
    int64_t peak = -1;

    for (uint32_t i = 0; i < free.size(); i++) { // Iterate through unsorted cloud
        double test = glm::dot(facet.normal, m_cloud[free[i]] - m_vertices[facet.triangle.I0]);
        if (test > 1e-12) {//std::numeric_limits<double>::epsilon()
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

void HullBuilder::calculateHorizon(const glm::dvec3& apex, int64_t last, uint32_t current, std::vector<uint32_t>& horizon, std::vector<uint32_t>& set){
    if (!m_facets[current].onHull) {
        return;
    }

    double test = glm::dot(m_facets[current].normal, apex - m_vertices[m_facets[current].triangle.I0]);

    if (test > 1e-12) { // Check whether facet is visible to apex
        set.insert(set.end(), m_facets[current].outside.begin(), m_facets[current].outside.end());
        m_facets[current].outside.clear();
        m_facets[current].onHull = false;

        // Identify neighbors to visit
        std::vector<uint32_t> neighbors = { 0, 0, 0 };
        uint32_t start = last == -1 ? 0 : std::find(m_facets[current].neighbors.begin(), m_facets[current].neighbors.end(), last) - m_facets[current].neighbors.begin();
        neighbors[0] = m_facets[current].neighbors[start];
        neighbors[1] = m_facets[current].neighbors[(start + 1) % 3];
        neighbors[2] = m_facets[current].neighbors[(start + 2) % 3];

        for (uint32_t neighbor : neighbors) {
            calculateHorizon(apex, current, neighbor, horizon, set);
        }

        return;
    }


    uint32_t index = std::find(m_facets[current].neighbors.begin(), m_facets[current].neighbors.end(), last) - m_facets[current].neighbors.begin();

    horizon.push_back(current);// Encode information about adjacent facet for linking purposes
    horizon.push_back(m_facets[current].triangle[index]);// Encode next vertex on horizon
}

uint32_t HullBuilder::getIteration() const
{
    return m_iteration;
}

bool HullBuilder::finished() const
{
    return m_initialized && m_iteration == m_facets.size();
}

FaceArray HullBuilder::getFaces() const
{
    std::vector<std::vector<uint32_t>> faces;
    std::vector<glm::dvec3> normals;

    std::vector<bool> hullCheck(m_facets.size());
    for (uint32_t i = 0; i < m_facets.size(); i++) hullCheck[i] = m_facets[i].onHull;

    for(uint32_t i = 0; i < m_facets.size(); i++){
        if(hullCheck[i]){
            const Facet& facet = m_facets[i];
            uint32_t faceIdx = faces.size();

            faces.push_back({ facet.triangle.I0, facet.triangle.I1, facet.triangle.I2 });
            normals.push_back(facet.normal);

            // Attach unique in-plane vertex indices to the face from other facets
            for (uint32_t j = &facet - &m_facets[0] + 1; j < m_facets.size(); j++) {
                if (hullCheck[j] && glm::dot(m_facets[j].normal, facet.normal) > 1 - 1e-12) {
                    for (uint8_t k = 0; k < 3; k++) {
                        if (std::find(faces[faceIdx].begin(), faces[faceIdx].end(), m_facets[j].triangle[k]) == faces[faceIdx].end()) {
                            faces[faceIdx].emplace_back(m_facets[j].triangle[k]);
                        }
                    }

                    hullCheck[j] = false;
                }
            }
        }
    }

    for (uint32_t i = 0; i < faces.size(); i++) {
        if (faces[i].size() >= 4) {
            std::vector<glm::dvec3> vertices(faces[i].size());
            for (uint32_t j = 0; j < faces[i].size(); j++) vertices[j] = m_vertices[faces[i][j]];

            auto hull = Polygon::hull(VertexArray::project(vertices, normals[i]));

            // Correct indexing
            for (uint32_t& index : hull) index = faces[i][index];
            faces[i] = hull;
        }
    }

//    purgeOrphans(faces); TODO consider - Expensive but leaving orphans affects walk operations

    FaceArray out(faces);
    out.assignNormals(normals);
    return out;
}

std::vector<glm::dvec3> HullBuilder::getVertices() const
{
    return m_vertices;
}

ConvexHull HullBuilder::getHull() const
{
    auto hull = ConvexHull();
    hull.m_vertices = getVertices();
    hull.m_faces = getFaces();
    hull.initialize();

    return hull;
}
