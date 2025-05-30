//
// Created by cameronh on 24/08/23.
//
//#define ENABLE_VHACD_IMPLEMENTATION 1

#include "ConvexHull.h"

#include "EPA.h"


#include <QVector3D>
#include <glm/glm.hpp>

#include "Core/Timer.h"

#include <thread>
#include <iostream>
#include <numeric>
#include <utility>

ConvexHull::ConvexHull()
    : m_vertices(nullptr, 0)
    , m_center()
    , m_cloud(nullptr, 0)
    , m_faces(nullptr, nullptr, 0)
{
}

ConvexHull::ConvexHull(const float* cloud, uint32_t cloudSize)
    : ConvexHull(VertexArray{cloud, cloudSize})

{
}

ConvexHull::ConvexHull(VertexArray cloud)
    : m_vertices(nullptr, 0)
    , m_center()
    , m_faces(nullptr, nullptr, 0)
    , m_cloud(std::move(cloud)){

    if (m_cloud.vertexCount() < 4) {
        std::cout << "\033[31mERROR! Can not generate a 3D convex hull with fewer than 4 vertices\033[0m\n";
        return;
    }

    // Default to minimum hull size that will not introduce potential bugs
    w_vertices.reserve(m_cloud.vertexCount());

    std::vector<Triangle> triangles = initialApproximation();

    if (triangles.empty()) {
        return;
    }

    // Initial pass to sort point cloud between tetrahedral facets
    prepareFacets(triangles);

    for (uint32_t i = 0; i < facets.size(); i++) {
        if (facets[i].onHull && !facets[i].outside.empty()) {
            w_vertices.push_back(m_cloud[facets[i].outside[0]]);

            std::vector<uint32_t> horizon, set;
            calculateHorizon(w_vertices[w_vertices.size() - 1], -1, i, horizon, set);
            prepareFacets(horizon, set);// Generate a strip of new facets between the apex and the horizon
        }
    }

    // Adjust vertex array size to calculated hull size
    m_vertices = VertexArray((float*)w_vertices.data(), w_vertices.size());

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

    auto facetSizes = new uint32_t[faces.size()];
    auto facets = new uint32_t[count];

    idx = 0;
    for (uint32_t i = 0; i < faces.size(); i++) {
        facetSizes[i] = faces[i].size();
        for (uint32_t vertex : faces[i]) facets[idx++] = vertex;
    }

    m_faces = {facets, facetSizes, (uint32_t)faces.size()};

    // Caluclate convex hull center
    for (uint32_t i = 0; i < m_vertices.vertexCount(); i++) {
        m_center += m_vertices[i];
    }
    m_center = m_center * (1 / (float)m_vertices.vertexCount());

    m_walks = m_faces.edgeList();
}

uint32_t ConvexHull::vertexCount() const
{
    return m_vertices.vertexCount();
}

const VertexArray& ConvexHull::vertices() const
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
    uint32_t *loop = m_faces[idx];
    if (loop == nullptr) return {};

    return Triangle::normal(m_vertices[loop[0]], m_vertices[loop[1]], m_vertices[loop[2]]);
}

glm::vec3 ConvexHull::center() const
{
    return m_center;
}

bool ConvexHull::isSourceConvex() const
{
    std::cout << m_vertices.vertexCount() << " " << m_cloud.vertexCount() << " |";
    return m_vertices.vertexCount() == m_cloud.vertexCount();
}

bool ConvexHull::isConvex(const VertexArray& test)
{
    return ConvexHull(test).isSourceConvex();
}

Simplex ConvexHull::gjkIntersection(const ConvexHull& body, const glm::mat4& transform, std::pair<uint32_t, uint32_t>& idx) const
{

    glm::vec3 axis = initialAxis(body, idx), next;
    Simplex simplex({ gjkSupport(body, transform, axis, idx), idx });

    axis = -simplex[0].val;

    int limit = (int)(vertexCount() + body.vertexCount());
    while (limit-- > 0) {
        next = gjkSupport(body, transform, axis, idx);

        // Check whether cs is impossible
        if (glm::dot (axis, next) <= 0) return simplex;

        simplex.add({ next, idx });

        // Check whether the cs is certain
        if (simplex.evaluate(axis)) return simplex;
    }

    std::cout << "CH GJK Error!\n";
    return Simplex(Simplex::Vertex{});
}

EPA ConvexHull::epaIntersection(const ConvexHull& body, const glm::mat4& transform, const glm::mat4& relativeTransform, std::pair<uint32_t, uint32_t>& idx) const
{
    return { *this, body, transform, relativeTransform, gjkIntersection(body, relativeTransform, idx) };
}

glm::vec3 ConvexHull::initialAxis(const ConvexHull& body, std::pair<uint32_t, uint32_t>& idx) const
{
    if (idx.first != std::numeric_limits<uint32_t>::max() && idx.second != std::numeric_limits<uint32_t>::max()) {
        return m_vertices[idx.first] - body.m_vertices[idx.second];
    }

    idx = { 0, 0 };

    return m_center - body.m_center;
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
    uint32_t i = 0, j = 0;

    while (i == 0) { // Only true when arriving at a fresh vertex
        for (; i < m_walks[index].size(); i++) { // Iterate through adjacent vertices
            if (glm::dot(axis, m_vertices[m_walks[index][i]] - m_vertices[index]) > 1e-6) { // Move to the first adjacent vertex along the axis
                index = m_walks[index][i];
                i = 0;
                break;
            }
        }

        // For debugging
        if (j++ > 10000) {
            std::cout << "\033[31mERROR! Failed to walk to a final destination!\033[0m\n";
            break;
        }
    }

//    float value = std::numeric_limits<float>::lowest(), temp;
//    for (uint32_t i = 0; i < m_vertices.vertexCount(); i++) {
//        temp = glm::dot(axis, m_vertices[i]);
//        if (temp > value) {
//            value = temp;
//            index = i;
//        }
//    }

    return index;
}

std::vector<Triangle> ConvexHull::initialApproximation(){
    std::vector<Triangle> triangles;

    // Find a pair of extreme points of the point cloud
    std::vector<uint32_t> extremes = { 0, 0, 0, 0 };
    float axes[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1}, *ptr = axes;
    for (uint8_t i = 0; i < 3; i++) {
        if (m_cloud.extremes(ptr, extremes[0], extremes[1])) {
            break;
        }

        ptr += 3;
    }

    // Prepare a maximal tetrahedron - exits early if the cloud is degenerate in some dimension
    if (extremes[0] == extremes[1]) {
        std::cout << "\033[31mERROR! Can not generate a 3D convex hull from the cloud! The cloud is 0D degenerate\033[0m\n";
        return triangles;
    }

    if (!m_cloud.extreme(extremes[0], extremes[1], extremes[2])) {
        std::cout << "\033[31mERROR! Can not generate a 3D convex hull from the cloud! The cloud is 1D degenerate\033[0m\n";
        return triangles;
    }

    if (!m_cloud.extreme(extremes[0], extremes[1], extremes[2], extremes[3])) {
        std::cout << "\033[31mERROR! Can not generate a 3D convex hull from the cloud! The cloud is 2D degenerate\033[0m\n";
        return triangles;
    }

    // Add extremities to the convex hull
    for (uint32_t extreme : extremes) {
        w_vertices.push_back(m_cloud[extreme]);
    }

    // Erase extreme vertices from cloud to prevent consideration later on (can otherwise introduce precision errors)
    std::sort(extremes.begin(), extremes.end(), [](uint32_t a, uint32_t b){ return b < a; });
    for (uint32_t extreme : extremes) m_cloud.remove(extreme);

    // Swap vertices if needed to ensure proper winding
    auto ref = w_vertices[3] - w_vertices[0];
    auto normal = Triangle::normal(w_vertices[0], w_vertices[1], w_vertices[2]);

    if (glm::dot(ref, normal) > 0) {
        std::swap(w_vertices[0], w_vertices[1]);
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
    return Triangle::normal(w_vertices[triangle.I0], w_vertices[triangle.I1], w_vertices[triangle.I2]);
}

void ConvexHull::prepareFacets(const std::vector<Triangle>& triangles){
    std::vector<uint32_t> free(m_cloud.vertexCount());
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
        Triangle triangle = { (uint32_t)w_vertices.size() - 1, lastVertex, horizon[j + 1] };
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
        float test = glm::dot(facet.normal, m_cloud[free[i]] - w_vertices[facet.triangle.I0]);
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

    float test = glm::dot(facets[current].normal, apex - w_vertices[facets[current].triangle.I0]);

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

ConvexHull ConvexHull::fragment(const glm::vec3& origin, const glm::vec3& normal) const
{

    // Find vertex intersections of hull with plane
    std::vector<bool> above(m_vertices.vertexCount());
    std::vector<glm::vec3> set = intersection(origin, normal, above);

    // Exit if no edges intersect with the plane
    if (set.empty()) {
        return *this;
    }

    // Attach vertices on the positive side of the plane
    for (uint32_t i = 0; i < above.size(); i++) if (above[i]) set.push_back(m_vertices[i]);

    return { VertexArray(set) };
}

std::pair<ConvexHull, ConvexHull> ConvexHull::fragments(const glm::vec3& origin, const glm::vec3& normal) const
{
    std::vector<bool> above(m_vertices.vertexCount());
    std::vector<glm::vec3> setA = intersection(origin, normal, above);

    // Exit if no edges intersect with the plane
    if (setA.empty()) {
        return { *this, {} };
    }

    std::vector<glm::vec3> setB = setA;

    // Attach vertices to respective fragments
    for (uint32_t i = 0; i < above.size(); i++) {
        if (above[i]) setA.push_back(m_vertices[i]);
        else setB.push_back(m_vertices[i]);
    }

    return { VertexArray(setA), VertexArray(setB) };
}

// TODO Improvement: Leverage walk to calculate intersection in-place
std::vector<glm::vec3> ConvexHull::intersection(const glm::vec3& origin, const glm::vec3& normal, std::vector<bool>& above) const
{

    float d = glm::dot(origin, normal);

    // Determine which vertices are above the cut plane
    for (uint32_t i = 0; i < m_vertices.vertexCount(); i++) {
        above[i] = glm::dot(normal, origin - m_vertices[i]) <= 0;
    }

    // Find vertices on the cut plane
    std::vector<glm::vec3> intersection;
    for (uint32_t i = 0; i < m_vertices.vertexCount(); i++) {
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                             CONVEX DECOMPOSITION                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//void ConvexDecomposition::ProcessLogger::Update(const double overallProgress, const double stageProgress, const char* const stage, const char *operation){
////    char scratch[512];
////    snprintf(scratch,sizeof(scratch),"[%-40s] : %0.0f%% : %0.0f%% : %s",stage,overallProgress,stageProgress,operation);
////
//////        if ( strcmp(stage,mCurrentStage.c_str()) == 0 )
//////        {
//////            for (uint32_t i=0; i<mLastLen; i++)
//////            {
//////                printf("%c", 8);
//////            }
//////        }
//////        else
//////        {
//////            printf("\n");
//////            mCurrentStage = std::string(stage);
//////        }
//////        mLastLen = (uint32_t)strlen(scratch);
////    printf("%s", scratch);
//    std::cout << "|" << stage << "| " << overallProgress << " : " << stageProgress << "\n";
//}
//
//void ConvexDecomposition::ProcessLogger::NotifyVHACDComplete(){
//    Log("VHACD::Complete");
//}
//
//void ConvexDecomposition::ProcessLogger::Log(const char* const msg){
//    std::cout << msg << "\n";
////    mLogMessages.push_back(std::string(msg));
//}
//
//ConvexDecomposition::ConvexDecomposition(const Settings& settings)
//        : m_Valid(false), m_Latest(true), m_ActiveSettings(settings), m_NewSettings(settings), m_Hulls({}), m_BaseColor(DEFAULT_CONVEX_HULL_COLOR), m_ColorDelta(0.1f, 0.1f, 0.1f){
//
//}
//
//ConvexDecomposition::ConvexDecomposition(const Settings& settings, const std::vector<glm::vec3>& vertices, const std::vector<Triangle>& triangles)
//        : m_Valid(false), m_Latest(true), m_ActiveSettings(settings), m_NewSettings(settings), m_BaseColor(DEFAULT_CONVEX_HULL_COLOR), m_ColorDelta(0.1f, 0.1f, 0.1f){
//
//    decompose(vertices, triangles);
//}
//
//void ConvexDecomposition::applyNewSettings(const Settings& settings){
//    if(m_NewSettings != settings){
//        m_NewSettings = settings;
//        m_Latest = false;
//    }
//
//    // Allow settings name to be directly overwritten
//    m_NewSettings.name = m_ActiveSettings.name = settings.name;
//}
//
//void ConvexDecomposition::restoreSettings(){
//    m_NewSettings = m_ActiveSettings;
//    m_Latest = true;
//}
//
//void ConvexDecomposition::invalidate(){
//    m_Valid = false;
//}
//
//void ConvexDecomposition::decompose(const std::vector<glm::vec3>& vertices, const std::vector<Triangle>& triangles, bool async) {
//
//    auto *points = new float[vertices.size() * 3];
//    auto indices = new uint32_t[triangles.size() * 3];
//
//    {
//        ScopedTimer timer("Data conversion for V-HACD");
//
//        // Convert data into a format acceptable to vhacd
//        for (uint32_t i = 0; i < vertices.size(); i++) {
//            points[3 * i] = vertices[i].x;
//            points[3 * i + 1] = vertices[i].y;
//            points[3 * i + 2] = vertices[i].z;
//        }
//
//        for (uint32_t i = 0; i < triangles.size(); i++) {
//            indices[3 * i] = triangles[i].m_I0;
//            indices[3 * i + 1] = triangles[i].m_I1;
//            indices[3 * i + 2] = triangles[i].m_I2;
//        }
//    }
//
//    decompose(points, vertices.size(), indices, triangles.size(), async);
//
//    delete[] points;
//    delete[] indices;
//}
//
//void ConvexDecomposition::decompose(float* vertices, uint32_t vertexCount, uint32_t* indices, uint32_t triangleCount, bool async){
//    m_Hulls.clear();
//
//    if(vertexCount == 0 || triangleCount == 0){
//        return;
//    }
//
//    VHACD::IVHACD::Parameters params = generateParameters(m_NewSettings);
//    params.m_asyncACD = async;
//
//    ProcessLogger logging;
//    params.m_callback = &logging;
//    params.m_logger = &logging;
//
//#if VHACD_DISABLE_THREADING
//    VHACD::IVHACD *iface = VHACD::CreateVHACD();
//#else
//    VHACD::IVHACD *iface = params.m_asyncACD ? VHACD::CreateVHACD_ASYNC() : VHACD::CreateVHACD();
//#endif
//
//    {
//        ScopedTimer timer("V-HACD process");
//        iface->Compute(vertices, vertexCount, indices, triangleCount, params);
//        while(!iface->IsReady()){
//            std::this_thread::sleep_for(std::chrono::nanoseconds(10000));
//        }
//    }
//
//    // Capture results of process and convert to the standard format
//    prepareConvexHulls(iface);
//
//    iface->Release();
//
//    m_ActiveSettings = m_NewSettings;
//    m_Valid = m_Latest = true;
//}
//
//VHACD::IVHACD::Parameters ConvexDecomposition::generateParameters(const Settings& settings){
//    VHACD::IVHACD::Parameters params;
//
//    params.m_maxConvexHulls = settings.maxHullCount;
//    params.m_maxNumVerticesPerCH = settings.maxVertexCount;
//    params.m_resolution = settings.voxelResolution;
//    params.m_minimumVolumePercentErrorAllowed = settings.volumeError;
//    params.m_minimumVolumePercentErrorAllowed = settings.recurseDepth;
//
//    params.m_fillMode = VHACD::FillMode::FLOOD_FILL;
//    params.m_shrinkWrap = true;
//
//    return params;
//}
//
//void ConvexDecomposition::prepareConvexHulls(VHACD::IVHACD* iface){
//    m_Hulls.clear();
//
//    uint32_t index = 0;
//    for (uint32_t i = 0; i < iface->GetNConvexHulls(); i++) {
//        VHACD::IVHACD::ConvexHull hull;
//        iface->GetConvexHull(i, hull);
//
//        // Extract only vertex information - vhacd lacks some information which makes it easier to just reconstruct the convex hull from a point cloud
//        std::vector<glm::vec3> hv;
//        hv.reserve(hull.m_points.size());
//        for(const VHACD::Vertex& vertex : hull.m_points) {
//            hv.emplace_back(vertex.mX, vertex.mY, vertex.mZ);
//        }
//
//        m_Hulls.emplace_back(hv);
//
//        // Color the constructed hull to make it easier to distinguish
//        glm::vec color = m_Hulls[i].getNextColor(m_BaseColor, m_ColorDelta, ++index);
//        m_Hulls[i].setColor(glm::clamp(color, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 1.0f, 1.0f)));
//    }
//}
//
//bool ConvexDecomposition::isValid() const{
//    return m_Valid;
//}
//
//bool ConvexDecomposition::isLatest() const{
//    return m_Latest;
//}
//
//const ConvexDecomposition::Settings& ConvexDecomposition::getActiveSettings() const{
//    return m_NewSettings;
//}
//
//uint32_t ConvexDecomposition::getHullCount() const{
//    return m_Hulls.size();
//}
//
//const std::vector<ConvexHull>& ConvexDecomposition::getHulls(){
//    return m_Hulls;
//}