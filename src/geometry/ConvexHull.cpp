//
// Created by cameronh on 24/08/23.
//
//#define ENABLE_VHACD_IMPLEMENTATION 1

#include "ConvexHull.h"

#include <QVector3D>

#include "Core/Timer.h"
//#include "Polygon.h"

#include <thread>
#include <iostream>
#include <numeric>

ConvexHull::ConvexHull()
    : m_vertices(nullptr)
    , m_vertexCount(0)
    , m_cloud(nullptr, 0)
    , m_facets(nullptr)
    , m_facetSizes(nullptr)
    , m_facetCount(0)
{

}

ConvexHull::ConvexHull(const float* cloud, uint32_t cloudSize)
    : m_vertices(nullptr)
    , m_vertexCount(0)
    , m_cloud(cloud, cloudSize)

{

    if(cloudSize < 4){
        std::cout << "\033[31mERROR! Can not generate a 3D convex hull with fewer than 4 vertices\033[0m\n";
        return;
    }

    m_vertices = new float[cloudSize * STRIDE];

//    std::cout << "Initial triangle approximation: (" << cloudSize << ")\n";
    std::vector<Triangle> triangles = initialApproximation();
//    for (const Triangle &tri : triangles) {
//        std::cout << tri.I0 << " " << tri.I1 << " " << tri.I2 << "\n";
//    }std::cout << "------------------------------\n";

//    std::cout << "CURRENT VERTICES (" << m_vertexCount << "): \n";
//    for (uint32_t i = 0; i < m_vertexCount; i++) {
//        std::cout << "(" << m_vertices[i * STRIDE] << ", " << m_vertices[i * STRIDE + 1] << ", " << m_vertices[i * STRIDE + 2] << ")\n";
//    }
    if(triangles.empty()){
        return;
    }

//    m_facets = new uint32_t[12] {0, 1, 2, 0, 2, 3, 0, 3, 1, 3, 2, 1};
//    m_facetSizes = new uint32_t[4] {3, 3, 3, 3};
//    m_facetCount = 4;

    // Initial pass to sort point cloud between tetrahedral facets
    prepareFacets(triangles);

//    std::cout << "PF\n";

    for(uint32_t i = 0; i < facets.size(); i++){
//        std::cout << "Facet: N" << i << " " << m_vertexCount << "\n";
        if(facets[i].onHull && !facets[i].outside.empty()){
            for (uint8_t j = 0; j < 3; j++) m_vertices[m_vertexCount * STRIDE + j] = m_cloud.vertices()[facets[i].outside[0] * STRIDE + j];
            m_vertexCount++;

            std::vector<uint32_t> horizon, set;
            calculateHorizon(&m_vertices[(m_vertexCount - 1) * STRIDE], -1, i, horizon, set);
            prepareFacets(horizon, set);// Generate a strip of new facets between the apex and the horizon
        }
    }

    std::vector<Triangle> fTri;

    // Attach all remaining triangles to the convex hull
    uint32_t idx = 0;

//    std::cout << "Vertices:\n";
//    for (uint32_t i = 0; i < m_vertexCount; i++) {
//        std::cout << m_vertices[3 * i] << " " << m_vertices[3 * i + 1] << " " << m_vertices[3 * i + 2] << "\n";
//    }
//
//    std::cout << "CHECK\n";
//    for(Facet& facet : facets) {
//        std::cout << "TRI " << facet.triangle.I0 << " " << facet.triangle.I1 << " " << facet.triangle.I2 << " -> \n";
//
////        if(!facet.onHull) continue;
////
////        for (uint32_t j = 0; j < 3; j++) {
////            std::cout << j << "|" << facet.neighbors[j] << " = ";
////            const Facet &f = facets[facet.neighbors[j]];
////            if (!f.onHull) continue;
////            std::cout << "TRI " << f.triangle.I0 << " " << f.triangle.I1 << " " << f.triangle.I2 << "\n";
////        }
//    }




//    std::cout << "FINAL V" << m_vertexCount << ":\n";
//    m_facetCount = 0;
//    for (const Facet& f : facets) m_facetCount += f.onHull;
//
//    m_facets = new uint32_t[3 * m_facetCount];
//    m_facetSizes = new uint32_t[m_facetCount];
//    for (const Facet& f : facets) {
//        if (f.onHull) {
//            m_facets[3 * idx + 0] = f.triangle.I0;
//            m_facets[3 * idx + 1] = f.triangle.I1;
//            m_facets[3 * idx + 2] = f.triangle.I2;
//            m_facetSizes[idx] = 3;
//
//            std::cout << ++idx << "|" << f.triangle.I0 << " " << f.triangle.I1 << " " << f.triangle.I2 << "|" << f.neighbors.size() << " " << f.outside.size() << "\n";
//
//
//        }
//    }


    std::cout << "MANIFOLD Test:\n" << isManifold(facets) << "\n";

    std::vector<std::vector<uint32_t>> faces;
    uint32_t count = 0;

    for(Facet& facet : facets){
//        std::cout << "F" << idx;
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
//                std::cout << "NEIGH: ";
//                for (auto n : neighbors) std::cout << n.first << "|" << n.second << " ";
//                std::cout << "| ";

                Facet& neighbor = facets[neighbors[i].second];
//                std::cout << "N" << i << "|" << neighbors[i].first << "/" << neighbors[i].second << "/" << (VertexArray::dot(neighbor.normal, facet.normal)) << "=" << (VertexArray::dot(neighbor.normal, facet.normal) > 1 - 2 * std::numeric_limits<float>::epsilon())
//                    << "(" << neighbor.neighbors[0] << " " << neighbor.neighbors[1] << " " << neighbor.neighbors[2] << ") ";

                if(neighbor.onHull && VertexArray::dot(neighbor.normal, facet.normal) > 1 - 2 * std::numeric_limits<float>::epsilon()){
//                    std::cout << "ADD";
//                    fTri.push_back(neighbor.triangle);

                    uint32_t ref = std::find(neighbor.neighbors.begin(), neighbor.neighbors.end(), neighbors[i].first) - neighbor.neighbors.begin();
                    if (ref >= 3) std::cout << "DMGGGG";
//                    neighbors[0] = facets[current].neighbors[start];
//                    neighbors[1] = facets[current].neighbors[(start + 1) % 3];
//                    neighbors[2] = facets[current].neighbors[(start + 2) % 3];

                    neighbors.insert(neighbors.begin() + i + 1, {neighbors[i].second, neighbor.neighbors[(ref + 2) % 3]});
                    neighbors[i] = {neighbors[i].second, neighbor.neighbors[(ref + 1) % 3]};
                    neighbor.onHull = false;
                    i--;
                }
//                std::cout << "\n";
            }

//            std::cout << "\n--------------------------\nFin: ";
//            for (auto n : neighbors) std::cout << n.first << "//" << n.second << " ";
            for (const auto &neighbor : neighbors) {
                const Triangle &tri = facets[neighbor.second].triangle;
                uint32_t ref = tri.I0 == last ? 0 : (tri.I1 == last ? 1 : 2);
                last = tri[(ref + 2) % 3];
                face.push_back(last);
            }

            count += face.size();

//            std::cout << "\n";
//            for (uint32_t f : face) std::cout << f << " ";
//            std::cout << "\n--------------------------\n";

        }

//        std::cout << "\n";

        idx++;
//        if(Triangle::area(m_Vertices[facet.triangle.m_I0], m_Vertices[facet.triangle.m_I1], m_Vertices[facet.triangle.m_I2]) < std::numeric_limits<float>::epsilon()){
//            std::cout << "\033[31mTODO FIX! Some precision error in convex hull creation! " << pointCloud.size() << " " << m_Vertices.size() << "\033[0m\n";
//        }
    }

//    std::cout << "Links:\n";
//    idx = 0;
//    for (const Facet &facet : facets) {
//        if (facet.neighbors[0] == 96 || facet.neighbors[1] == 96 || facet.neighbors[2] == 96)
//            std::cout << idx << " " << facet.onHull << " | " << facet.neighbors[0] << " " << facet.neighbors[1] << " " << facet.neighbors[2] << "\n";
//
//        idx++;
//    }

//    std::cout << "REPORT:\n";
    m_facetCount = faces.size();
    m_facetSizes = new uint32_t[faces.size()];
    m_facets = new uint32_t[count];

    idx = 0;
    for (uint32_t i = 0; i < faces.size(); i++) {
        std::cout << faces[i].size() << " ";
        m_facetSizes[i] = faces[i].size();
        for (uint32_t vertex : faces[i]) m_facets[idx++] = vertex;
    }
    std::cout << "\n";

    // Do away with working memory
    facets.clear();

}

uint32_t ConvexHull::vertexCount() const
{
    return m_vertexCount;
}

float* ConvexHull::vertices() const
{
    return m_vertices;
}

//uint32_t *ConvexHull::facets() const
//{
//    return m_facets;
//}
uint32_t *ConvexHull::facetSizes() const
{
    return m_facetSizes;
}

uint32_t ConvexHull::facetCount() const
{
    return m_facetCount;
}

uint32_t* ConvexHull::triangulate(uint32_t& triangleCount) const
{
    // Calculate the number of triangles required to tesselate facets
    triangleCount = 0;
    for (uint32_t i = 0; i < m_facetCount; i++) {
        triangleCount += m_facetSizes[i] - 2;
    }

    uint32_t idx = 0, fIdx = 0;
    auto triangles = new uint32_t[3 * triangleCount];
    for (uint32_t i = 0; i < m_facetCount; i++) {

        // Triangulate facet, splitting in a fan-shape from the first vertex
        for (uint32_t j = 0; j < m_facetSizes[i] - 2; j++) {
            triangles[idx++] = m_facets[fIdx];
            triangles[idx++] = m_facets[fIdx + j + 1];
            triangles[idx++] = m_facets[fIdx + j + 2];
        }

        fIdx += m_facetSizes[i];
    }

    std::cout << "Triangle count: " << triangleCount << "\n";
    return triangles;
}

std::vector<ConvexHull::Triangle> ConvexHull::initialApproximation(){
    std::vector<Triangle> triangles;

    // Find a pair of extreme points of the point cloud
    std::vector<uint32_t> extremes = {0, 0, 0, 0};
    float axes[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1}, *ptr = axes;
    for(uint8_t i = 0; i < 3; i++){
        if(m_cloud.extremes(ptr, extremes[0], extremes[1])){
            break;
        }

        ptr += 3;
    }

    // Prepare a maximal tetrahedron - exits early if the cloud is degenerate in some dimension
    if(extremes[0] == extremes[1]){
        std::cout << "\033[31mERROR! Can not generate a 3D convex hull from the cloud! The cloud is 0D degenerate\033[0m\n";
        return triangles;
    }

    if(!m_cloud.extreme(extremes[0], extremes[1], extremes[2])){
        std::cout << "\033[31mERROR! Can not generate a 3D convex hull from the cloud! The cloud is 1D degenerate\033[0m\n";
        return triangles;
    }

    if(!m_cloud.extreme(extremes[0], extremes[1], extremes[2], extremes[3])){
        std::cout << "\033[31mERROR! Can not generate a 3D convex hull from the cloud! The cloud is 2D degenerate\033[0m\n";
        return triangles;
    }

    // Add extremities to the convex hull
    for(uint32_t extreme : extremes){
        for (uint8_t j = 0; j < 3; j++) m_vertices[m_vertexCount * STRIDE + j] = m_cloud.vertices()[extreme * STRIDE + j];
        m_vertexCount++;
    }

    // Erase extreme vertices from cloud to prevent consideration later on (can otherwise introduce precision errors)
    std::sort(extremes.begin(), extremes.end(), [](uint32_t a, uint32_t b){return b < a;});
    for(uint32_t extreme : extremes) m_cloud.remove(extreme);

    // Swap vertices if needed to ensure proper winding
    auto del = VertexArray::sub(&m_vertices[9], &m_vertices[0]), norm = new float[3];
    VertexArray::normal(norm, &m_vertices[0], &m_vertices[3], &m_vertices[6]);

    if(VertexArray::dot(norm, del) > 0){
        VertexArray::swap(&m_vertices[0], &m_vertices[3]);
    }

    // Prepare initial triangle approximation (tetrahedron)
    triangles.push_back({0, 1, 2});
    triangles.push_back({0, 2, 3});
    triangles.push_back({0, 3, 1});
    triangles.push_back({3, 2, 1});

    return triangles;
}

void ConvexHull::prepareFacets(const std::vector<Triangle>& triangles){
    std::vector<uint32_t> free(m_cloud.vertexCount());
    std::iota(free.begin(), free.end(), 0);

    std::vector<std::vector<uint32_t>> neighbors = {{2, 3, 1}, {0, 3, 2}, {1, 3, 0}, {1, 0, 2}};
    for(uint32_t i = 0; i < triangles.size(); i++){
        auto normal = new float[3];
        VertexArray::normal(normal, &m_vertices[triangles[i].I0 * STRIDE], &m_vertices[triangles[i].I1 * STRIDE], &m_vertices[triangles[i].I2 * STRIDE]);
//        std::cout << "PF1 Normal: (" << normal[0] << "," << normal[1] << ", " << normal[2] << ")\n";
        Facet facet = {triangles[i], normal, {}, neighbors[i], true};
        sortCloud(free, facet);
//        std::cout << "AF1: " << triangles[i].I0 << " " << triangles[i].I1 << " " << triangles[i].I2 << "\n";
        facets.push_back(facet);
    }
}

void ConvexHull::prepareFacets(const std::vector<uint32_t>& horizon, std::vector<uint32_t>& set) {
//    std::cout << "PFH: ";
//    for (uint32_t h : horizon) std::cout << h << " ";
//    std::cout << "\n";

    // Create a chain of triangular facets between the apex and its respective horizon
    uint32_t lastVertex = horizon[horizon.size() - 1], lastFacet = facets.size() + (horizon.size() / 2) - 1, start = facets.size();
    for(uint32_t j = 0; j < horizon.size(); j += 2){

        // Identify neighbors for linking
        uint32_t currentFacet = facets.size(), nextFacet = (currentFacet + 1 - start) % (horizon.size() / 2) + start;
        uint32_t z = (facets[horizon[j]].triangle[1] == horizon[j + 1]) + 2 * (facets[horizon[j]].triangle[2] == horizon[j + 1]);
        facets[horizon[j]].neighbors[z] = currentFacet;// Link against existing facet beyond the horizon

        // Prepare the new facet and link
        Triangle triangle = {m_vertexCount - 1, lastVertex, horizon[j + 1]};
        auto normal = new float[3];
        VertexArray::normal(normal, &m_vertices[triangle.I0 * STRIDE], &m_vertices[triangle.I1 * STRIDE], &m_vertices[triangle.I2 * STRIDE]);
//        std::cout << "PF2 Normal: " << triangle.I0 << " " << triangle.I1 << " " << triangle.I2 << " (" << normal[0] << "," << normal[1] << ", " << normal[2] << ") \n";

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

    for(uint32_t i = 0; i < free.size(); i++){
        float *vec = VertexArray::sub(&m_cloud.vertices()[free[i] * STRIDE], &m_vertices[facet.triangle.I0 * STRIDE]);
        float test = VertexArray::dot(facet.normal, vec);
        if(test > std::numeric_limits<float>::epsilon()){//std::numeric_limits<float>::epsilon()
            if(test > value){
                value = test;
                peak = (int64_t)facet.outside.size();
            }

            facet.outside.push_back(free[i]);
            std::swap(free[i--], free[free.size() - 1]);
            free.pop_back();
        }
    }

    // Identify the furthest point in the cloud (for refinement step)
    if(peak > 0){
        std::swap(facet.outside[peak], facet.outside[0]);
    }
}

void ConvexHull::calculateHorizon(const float *apex, int64_t last, uint32_t current, std::vector<uint32_t>& horizon, std::vector<uint32_t>& set){
    if(!facets[current].onHull){
        return;
    }

    float *vec = VertexArray::sub(apex, &m_vertices[facets[current].triangle.I0 * STRIDE]);
    float test = VertexArray::dot(facets[current].normal, vec);

//    delete vec;

//    std::cout << "CHS: " << last << " " << current << " -> " << test << " ~ ";

    if(test > std::numeric_limits<float>::epsilon()){ // Check whether facet is visible to apex
        set.insert(set.end(), facets[current].outside.begin(), facets[current].outside.end());
        facets[current].outside.clear();
        facets[current].onHull = false;

        // Identify neighbors to visit
        std::vector<uint32_t> neighbors = {0, 0, 0};
        uint32_t start = last == -1 ? 0 : std::find(facets[current].neighbors.begin(), facets[current].neighbors.end(), last) - facets[current].neighbors.begin();
        neighbors[0] = facets[current].neighbors[start];
        neighbors[1] = facets[current].neighbors[(start + 1) % 3];
        neighbors[2] = facets[current].neighbors[(start + 2) % 3];

//        std::cout << "CHN(" << neighbors[0] << " " << neighbors[1] << " " << neighbors[2] << ")\n";

        for(uint32_t neighbor : neighbors){
//            std::cout << neighbor << "]";
            calculateHorizon(apex, current, neighbor, horizon, set);
        }

        return;
    }


    uint32_t index = std::find(facets[current].neighbors.begin(), facets[current].neighbors.end(), last) - facets[current].neighbors.begin();

//    std::cout << "|" << index << "\\" << current << "\\" << facets[current].triangle[index] << "| ";

    horizon.push_back(current);// Encode information about adjacent facet for linking purposes
    horizon.push_back(facets[current].triangle[index]);// Encode next vertex on horizon
}

bool ConvexHull::isManifold(const std::vector<Facet> &facets)
{
    bool manifold = true;

    for (uint32_t i = 0; i < facets.size(); i++) {
        if (!facets[i].onHull) continue;

        for (uint32_t neighbor : facets[i].neighbors) {
            if (facets[neighbor].onHull && !isManifold(facets[neighbor], i)) {
                std::cout << "LINK BROKEN: " << i << "|" << facets[neighbor].neighbors[0] << " " << facets[neighbor].neighbors[1] << " " << facets[neighbor].neighbors[2] << "\n";
                manifold = false;
            }
        }
    }

    return manifold;
}

bool ConvexHull::isManifold(const Facet &dest, uint32_t src)
{
    return std::find(dest.neighbors.begin(), dest.neighbors.end(), src) != dest.neighbors.end();
}

//std::vector<std::pair<glm::vec3, glm::vec3>> ConvexHull::getAxes() const {
//    std::vector<std::pair<glm::vec3, glm::vec3>> axes;
//
//    for(uint32_t i = 0; i < m_Breaks.size(); i++){
//        axes.emplace_back(m_Vertices[m_Triangles[m_Breaks[i] - 1].m_I0], m_Normals[i]);
//    }
//
//    return axes;
//}
//bool closestPointRaySegment(const glm::vec3& origin, const glm::vec3& normal, const glm::vec3& p1, const glm::vec3& p2, glm::vec3& nearest){
//    glm::vec3 direction = p2 - p1;
//    glm::vec3 r = origin - p1;
//
//    float e = glm::dot(direction, direction);
//    float f = glm::dot(direction, r);
//
//    // Assumed no degenerate
//    float c = glm::dot(normal, r);
//    float b = glm::dot(normal, direction);
//    float den = e - b * b;
//
//    float s = 0;
//    if(den != 0){
//        s = (b * f - c * e) / den;
//    }
//
//    float t = (b * s + f) / e;
//    nearest = p1 + direction * t;
//
//    return t >= 0 && t <= 1;
//}
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