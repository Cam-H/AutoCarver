//
// Created by Cam on 2025-06-19.
//

#include "Profile.h"

#include <iostream>

Profile::Profile()
    : Profile({}, { 1, 0,  0 }, { 0, 0, -1 }, { 0, 1, 0 })
{
}

Profile::Profile(const std::vector<glm::vec2>& contour, const glm::vec3& normal, const glm::vec3& xAxis, const glm::vec3& yAxis)
    : Polygon(contour)
    , m_normal(normal)
    , m_xAxis(xAxis)
    , m_yAxis(yAxis)
    , m_method(RefinementMethod::DIRECT)
    , m_next(0)
{
    initialize();
}

Profile::Profile(const std::string& filename)
        : Profile()
{
    Serializable::deserialize(filename);
}

Profile::Profile(std::ifstream& file)
        : Profile()
{
    if (!Profile::deserialize(file)) std::cerr << "Failed to deserialize profile properly!\n";
}

bool Profile::serialize(const std::string& filename)
{
    return Serializable::serialize(filename);
}
bool Profile::serialize(std::ofstream& file)
{
    Serializer::writeVectorVec2(file, m_vertices);
    Serializer::writeVec3(file, m_normal);
    Serializer::writeVec3(file, m_xAxis);
    Serializer::writeVec3(file, m_yAxis);

    return true;
}

bool Profile::deserialize(const std::string& filename)
{
    return Serializable::deserialize(filename);
}
bool Profile::deserialize(std::ifstream& file)
{
    m_vertices = Serializer::readVectorVec2(file);
    m_normal = Serializer::readVec3(file);
    m_xAxis = Serializer::readVec3(file);
    m_yAxis = Serializer::readVec3(file);

    return true;
}

void Profile::initialize()
{
    if (m_vertices.size() < 4) return;

    std::vector<uint32_t> border = hull();

    // Identify concave faces, noting boundaries with the convex hull
    for (uint32_t i = 0; i < border.size(); i++) {
        uint32_t idx = (i + 1) % border.size(), count = 1 + difference(border[i], border[idx], m_vertices.size());
        if (count > 2) m_remainder.emplace_back(border[i], count);
    }
}

void Profile::setRefinementMethod(RefinementMethod method)
{
    m_method = method;
}

std::vector<uint32_t> Profile::refine()
{

    if (m_next < m_remainder.size()) {
        if (m_remainder[m_next].second == 3) { // Easy direct triangle cutout
            m_remainder.erase(m_remainder.begin() + m_next);
            return {
                m_remainder[m_next].first,
                (uint32_t)((m_remainder[m_next].first + 1) % m_vertices.size()),
                (uint32_t)((m_remainder[m_next].first + 2) % m_vertices.size())
            };
        }

        // More involved decomposition of concave for processing
        switch (m_method) {
            case RefinementMethod::DIRECT:
                return directRefinement();
            case RefinementMethod::DELAUNEY:
                return delauneyRefinement();
            case RefinementMethod::TEST:
                return testRefinement();
        }
    }

    return {};
}

std::vector<uint32_t> Profile::directRefinement()
{
    auto cut = m_remainder[m_next];
    m_remainder.erase(m_remainder.begin() + m_next);
    return sectionIndices(cut);
}


std::vector<uint32_t> Profile::delauneyRefinement()
{
    std::vector<uint32_t> indices = sectionIndices(m_remainder[m_next]), triangle;
    std::vector<glm::vec2> vertices = sectionVertices(indices);

    // Find the Delauney triangulation of the remaining section
    auto triangles = Polygon::bowyerWatson(vertices, true, -1);

    // Select the triangle on the boundary (The only accessible tri)
    for (const Triangle& tri : triangles) {
        if (tri.has(0) && tri.has(indices.size() - 1)) {
            uint32_t splitVertex = tri.last(0, indices.size() - 1);
            triangle = { indices[0], indices[splitVertex], indices[indices.size() - 1] };
            break;
        }
    }

    // If a viable triangular cut was found
    if (!triangle.empty()) {
        uint32_t d1 = difference(triangle[0], triangle[1], m_vertices.size()),
                 d2 = difference(triangle[1], triangle[2], m_vertices.size());

        // Create new boundaries based on the cut triangle (When borders are still concave)
        if (d1 > 1) {
            m_remainder[m_next] = { triangle[0], d1 + 1};
            if (d2 > 1) m_remainder.insert(m_remainder.begin() + m_next + 1, { triangle[1], d2 + 1});
        } else if (d2 > 1) {
            m_remainder[m_next] = { triangle[1], d2 + 1};
        }
    } else {
        m_next++; // Skip over this section because it is unreachable
    }

    return triangle;
}

std::vector<uint32_t> Profile::testRefinement()
{
//    auto cut = m_remainder[m_next];
//
//    std::vector<uint32_t> section = sectionIndices(m_remainder[m_next]);
//
//    std::vector<uint32_t> splitPoints;
//    uint32_t end = (cut.first + cut.second) % m_vertices.size();
//
//    glm::vec2 axis = glm::normalize(m_vertices[end] - m_vertices[cut.first]);
//    glm::vec2 normal = { -axis.y, -axis.x };
//
//    std::vector<glm::vec2> rVertices;
//    rVertices.reserve(cut.second);
//
//    for (uint32_t i = 0; i < cut.second; i++) {
//        uint32_t idx = (cut.first + i) % m_vertices.size();
//        rVertices.emplace_back(glm::dot(axis, m_vertices[idx]), glm::dot(normal, m_vertices[idx]));
//        std::cout << "R " << i << " (" << idx << ") " << rVertices[i].x << " " << rVertices[i].y << "\n";
//    }
//
//    float xLim = std::numeric_limits<float>::lowest();
//    for (uint32_t i = 0; i < cut.second; i++) {
//        uint32_t idx = (cut.first + i) % m_vertices.size();
//        float value = glm::dot(axis, m_vertices[idx]);
//        if (xLim < value) {
//            splitPoints.emplace_back(idx);
//            xLim = value;
//        }
//    }
//
//    xLim = std::numeric_limits<float>::max();
//    for (int i = splitPoints.size() - 1; i >= 0; i--) {
//        float value = glm::dot(axis, m_vertices[splitPoints[i]]);
//        std::cout << "AA " << i << " " << splitPoints[i] << " " << value << "\n";
////            if ()
//    }
//
//    for (uint32_t p : splitPoints) std::cout << p << " ";
//    std::cout << "~SP\n";
//
//    std::cout << axis.x << " " << axis.y << " | " << normal.x << " " << normal.y << "\n";
    return {};
}

uint32_t Profile::difference(uint32_t a, uint32_t b, uint32_t max)
{
    return b < a ? max - a + b : b - a;
}

std::vector<uint32_t> Profile::sectionIndices(const std::pair<uint32_t, uint32_t>& limits) const
{
    std::vector<uint32_t> indices;
    indices.reserve(limits.second);

    for (uint32_t i = 0; i < limits.second; i++) indices.emplace_back((limits.first + i) % m_vertices.size());
    return indices;
}

std::vector<glm::vec2> Profile::sectionVertices(const std::pair<uint32_t, uint32_t>& limits) const
{
    return sectionVertices(sectionIndices(limits));
}
std::vector<glm::vec2> Profile::sectionVertices(const std::vector<uint32_t>& indices) const
{
    std::vector<glm::vec2> section(indices.size());
    for (uint32_t i = 0; i < indices.size(); i++) section[i] = m_vertices[indices[i]];
    return section;
}

// Project vertices into 3D space according to the defined axis system
std::vector<glm::vec3> Profile::projected3D(const glm::vec3& offset)
{
    return Polygon::projected3D(m_xAxis, m_yAxis, offset);
}

std::vector<std::pair<glm::vec2, glm::vec2>> Profile::debugEdges() const {
    auto edges = Polygon::debugEdges();

    for (const auto& edge : m_remainder)
        edges.emplace_back(m_vertices[edge.first], m_vertices[(edge.first + edge.second - 1) % m_vertices.size()]);

    if (m_next < m_remainder.size()) {
        std::vector<glm::vec2> vertices = sectionVertices(m_remainder[m_next]);
        auto triangles = Polygon::bowyerWatson(vertices, true, -1);

        for (const Triangle& tri: triangles) {
            edges.emplace_back(vertices[tri.I0], vertices[tri.I1]);
            edges.emplace_back(vertices[tri.I1], vertices[tri.I2]);
            edges.emplace_back(vertices[tri.I2], vertices[tri.I0]);
        }
    }


    return edges;
}
