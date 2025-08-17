//
// Created by Cam on 2025-06-19.
//

#include "Profile.h"

#include <iostream>

#include <gtc/quaternion.hpp>


Profile::Profile()
    : Profile({}, { 1, 0,  0 }, { 0, 0, -1 }, { 0, 1, 0 })
{
}

Profile::Profile(const std::vector<glm::dvec2>& contour, const glm::dvec3& normal, const glm::dvec3& xAxis, const glm::dvec3& yAxis)
    : Polygon(contour, false)
    , m_normal(normal)
    , m_xAxis(xAxis)
    , m_yAxis(yAxis)
    , m_method(RefinementMethod::DIRECT)
    , m_minimumArea(0.0001f)
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
    Serializer::writeVectorDVec2(file, m_vertices);
    Serializer::writeDVec3(file, m_normal);
    Serializer::writeDVec3(file, m_xAxis);
    Serializer::writeDVec3(file, m_yAxis);

    return true;
}

bool Profile::deserialize(const std::string& filename)
{
    return Serializable::deserialize(filename);
}
bool Profile::deserialize(std::ifstream& file)
{
    m_vertices = Serializer::readVectorDVec2(file);
    m_normal = Serializer::readDVec3(file);
    m_xAxis = Serializer::readDVec3(file);
    m_yAxis = Serializer::readDVec3(file);

    return true;
}

void Profile::initialize()
{
    if (m_vertices.size() < 4) return;

    correctWinding();

    m_hull = hull();

    // Reorganize recorded border to ensure it is in ascending order (Needed for future operations)
    uint32_t last = m_hull[0];
    for (uint32_t i = 1; i < m_hull.size(); i++) {
        if (m_hull[i] < last) {
            last = m_hull.size();
            m_hull.insert(m_hull.begin(), m_hull.end(), m_hull.end() + m_hull.size() - i);
            m_hull.erase(m_hull.begin() + last, m_hull.end());
            break;
        }
        last = m_hull[i];
    }

//    std::cout << "Border: ";
//    for (uint32_t b : m_hull) std::cout << b << " ";
//    std::cout << "\n";


    // Identify concave faces, noting boundaries with the convex hull
    for (uint32_t i = 0; i < m_hull.size(); i++) {
        uint32_t idx = (i + 1) % m_hull.size(), count = 1 + difference(m_hull[i], m_hull[idx], m_vertices.size());
        if (count > 2) emplaceRemainder(m_hull[i], count);
    }

//    std::cout << "Profile: " << isCCW() << " " << m_hull.size() << " " << m_vertices.size() << " " << m_remainder.size() << " " << "\n";
    for (auto& r : m_remainder) std::cout << r.first << " " << r.second << "\n";

//    setRefinementMethod(m_method);

}

// Returns the index of the terminal vertex for the step
uint32_t Profile::terminus(uint32_t step) const
{
    if (step >= m_remainder.size()) throw std::runtime_error("[Profile] Array index out of bounds");
    return offsetIndex(m_remainder[step].first + m_remainder[step].second - 2);
}

uint32_t Profile::offsetIndex(uint32_t idx, uint32_t offset) const
{
    return (idx + offset) % m_vertices.size();
}

void Profile::emplaceRemainder(uint32_t start, uint32_t count)
{
    auto axis = edgeNormal(start, offsetIndex(start, count - 1));
    uint32_t subCount;

    while ((subCount = subdivide(axis, start, count)) < count) {
        // Additional check to remove adjacent collinear vertices
        if (subCount >= 2) m_remainder.emplace_back(start, subCount + 1);

        // Prepare for next division
        start = offsetIndex(start, subCount);
        count -= subCount;
    }

    // Capture remaining
    if(count > 2) m_remainder.emplace_back(start, count);
}
void Profile::insertRemainder(uint32_t index, uint32_t start, uint32_t count)
{
    auto axis = edgeNormal(start, offsetIndex(start, count - 1));
    uint32_t subCount;

    while ((subCount = subdivide(axis, start, count)) < count) {
        // Additional check to remove adjacent collinear vertices
        if (subCount >= 2) m_remainder.insert(m_remainder.begin() + index, { start, subCount + 1 });

        // Prepare for next division
        start = offsetIndex(start, subCount);
        count -= subCount;
    }

    // Capture remaining
    if (count > 2) m_remainder.insert(m_remainder.begin() + index, { start, count });
}

// Splits a section into subcomponents along protrusions. This is necessary because
// later triangulation would fail due to 0 thickness sections that would otherwise result
uint32_t Profile::subdivide(const glm::dvec2& normal, uint32_t start, uint32_t count)
{
    for (uint32_t i = 1; i < count - 1; i++) {
        double result = glm::dot(normal, glm::normalize(m_vertices[offsetIndex(start, i)] - m_vertices[start]));
        if (result > -0.02f) return i;
    }

    return count;
}

glm::dvec2 Profile::edgeNormal(uint32_t start, uint32_t end)
{
    glm::dvec2 edge = m_vertices[end] - m_vertices[start];
    return glm::normalize(glm::dvec2{ edge.y, -edge.x });
}

void Profile::setRefinementMethod(RefinementMethod method)
{
    m_method = method;

    if (m_remainder.empty()) return;

    // More involved decomposition of concave for processing
    switch (m_method) {
//        case RefinementMethod::DIRECT: directRefinement(); break;
        case RefinementMethod::DELAUNEY: delauneyRefinement(); break;
//        case RefinementMethod::TEST: testRefinement(); break;
        default: throw std::runtime_error("[Profile] Unrecognized refinement method");
    }
}

void Profile::setMimimumArea(double area)
{
    m_minimumArea = area;
}
//void Profile::translate(const glm::dvec2& translation)
//{
//    Polygon::translate(translation);
//}

void Profile::translate(const glm::dvec3& translation)
{
    Polygon::translate({
        glm::dot(translation, m_xAxis),
        glm::dot(translation, m_yAxis)
    });
}

void Profile::rotateAbout(const glm::dvec3& axis, double theta)
{
    auto rotation = glm::angleAxis(theta, axis);
    m_normal = rotation * m_normal;
    m_xAxis = rotation * m_xAxis;
    m_yAxis = rotation * m_yAxis;
}

void Profile::inverseWinding()//TODO update
{
    Polygon::inverseWinding();

    for (auto& remainder : m_remainder) {

        // Offset index by count (Required because inversion causes initiation from opposite side of section)
        remainder.first += remainder.second;
        if (remainder.first >= m_vertices.size()) remainder.first -= m_vertices.size();

        remainder.first = m_vertices.size() - remainder.first; // Invert index
    }
}

// Skips the next section along with any further sections dependent on it
void Profile::skip()
{
    if (!m_sections.empty()) {
        const uint32_t count = 1 + m_sections[0].children;
        for (uint32_t i = 0; i < count; i++) m_sections.pop_front();
    }
}

TriIndex Profile::refine()
{
    if (!m_sections.empty()) {
        auto section = m_sections.front();
        m_sections.pop_front();
        return section.triangle;
    }

    return { 0, 0, 0 };
}

uint32_t Profile::remainingSections() const
{
    return m_sections.size();
}

bool Profile::complete() const
{
    return m_sections.empty();
}

bool Profile::isVertexExternal(uint32_t index) const
{
    if (index >= m_vertices.size()) return false;
    return std::binary_search(m_hull.begin(), m_hull.end(), index);
}

bool Profile::isNextExternal() const
{
    return !complete() && isVertexExternal(m_sections[0].triangle.I0)
                       && isVertexExternal(m_sections[0].triangle.I2);
}

std::pair<double, double> Profile::angles() const
{
    if (m_sections.empty()) return { 0, 0 };
    return angles(m_sections[0].triangle);
}
std::pair<double, double> Profile::clearance() const
{
    if (m_sections.empty()) return { 0, 0 };
    return clearance(m_sections[0].triangle);
}

// Returns the exterior angle formed between the cut and adjacent edges for both vertices [in radians]
std::pair<double, double> Profile::angles(const TriIndex& triangle) const
{
    if (!triangle.isValid(m_vertices.size())) return { 0, 0 };

    uint32_t prev = prevVertex(triangle.I0);
    uint32_t next = nextVertex(triangle.I2);

    return {
        angle(m_vertices[prev] - m_vertices[triangle.I0], m_vertices[triangle.I1] - m_vertices[triangle.I0]),
        angle(m_vertices[triangle.I1] - m_vertices[triangle.I2], m_vertices[next] - m_vertices[triangle.I2])
    };
}

double Profile::angle(const glm::dvec2& a, const glm::dvec2& b)
{
    double value = atan2(b.y, b.x) - atan2(a.y, a.x);
    return value + 2 * M_PI * (value < 0);
}

uint32_t Profile::prevVertex(uint32_t vertexIndex) const
{
    for (const Section& section : m_sections) {
        if (section.triangle.I2 == vertexIndex) return section.triangle.I0;
    }

    return vertexIndex == 0 ? m_vertices.size() - 1 : vertexIndex - 1;
}
uint32_t Profile::nextVertex(uint32_t vertexIndex) const
{
    for (const Section& section : m_sections) {
        if (section.triangle.I0 == vertexIndex) return section.triangle.I2;
    }

    return vertexIndex == m_vertices.size() - 1 ? 0 : vertexIndex + 1;
}

// Returns the clearance from either vertex before the first obstruction on the reversed cut direction (Margin for alignment)
std::pair<double, double> Profile::clearance(const TriIndex& triangle) const
{
    if (!triangle.isValid(m_vertices.size())) return { 0, 0 };

    auto theta = angles(triangle);

    return {
        theta.first  < M_PI ? 0 : clearance(-glm::normalize(m_vertices[triangle.I0] - m_vertices[triangle.I1]), triangle.I0),
        theta.second < M_PI ? 0 : clearance(-glm::normalize(m_vertices[triangle.I2] - m_vertices[triangle.I1]), triangle.I2)
    };
}

// Calculates clearance along the axis from the specified vertex
// Always returns DOUBLE_MAX if the vertex is on the convex hull regardless of axis direction
double Profile::clearance(const glm::dvec2& axis, uint32_t vertexIndex) const
{
    double t = std::numeric_limits<double>::max();

    if (!isVertexExternal(vertexIndex)) {
        glm::dvec2 dir = m_vertices[vertexIndex] - axis;

        auto hv = nextHullVertices(vertexIndex);

        uint32_t index = prevVertex(vertexIndex), prev = index;

        // Find nearest intersection along the upper chain (skipping first edge)
        while (prev != hv.first) {
            index = prevVertex(index);

            auto [hit, tIntersection] = intersection(m_vertices[vertexIndex], dir, m_vertices[prev], m_vertices[index]);
            if (hit) t = std::min(t, tIntersection);
            prev = index;
        }

        index = nextVertex(vertexIndex), prev = index;

        // Find nearest intersection along the lower chain (skipping first edge)
        while (prev != hv.second) {
            index = nextVertex(index);

            auto [hit, tIntersection] = intersection(m_vertices[vertexIndex], dir, m_vertices[prev], m_vertices[index]);
            if (hit) t = std::min(t, tIntersection);
            prev = index;
        }
    }

    return t;
}

std::tuple<bool, double> Profile::intersection(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c, const glm::dvec2& d)
{
    double a1 = Triangle2D::signedArea(a, b, d), a2 = Triangle2D::signedArea(a, b, c);

    if (a1 * a2 < 0) {
        double a3 = Triangle2D::signedArea(c, d, a), a4 = a3 + a2 - a1;
        if (a3 * a4 < 0) return { true, a3 / (a3 - a4) };
    }

    return { false, 0 };
}

std::pair<uint32_t, uint32_t> Profile::nextHullVertices(uint32_t vertexIndex) const
{
    auto lIt = std::lower_bound(m_hull.begin(), m_hull.end(), vertexIndex);
    auto hIt = std::upper_bound(m_hull.begin(), m_hull.end(), vertexIndex);

    return {
        lIt == m_hull.begin() ? m_hull.back() : *(lIt - 1), // index is lowest  -> wrap to the last
        hIt == m_hull.end() ? m_hull[0] : *hIt              // index is highest -> wrap to the start
    };
}

const glm::dvec3& Profile::normal() const
{
    return m_normal;
}

void Profile::addTriangle(uint32_t startIndex)
{
    auto tri = TriIndex(startIndex,offsetIndex(startIndex, 1),offsetIndex(startIndex, 2));
    if (area(tri) > m_minimumArea) m_sections.emplace_back(tri, 0);
}

void Profile::directRefinement()
{
//    auto cut = m_remainder[m_next];
//    auto indices = sectionIndices(cut);
//
//    if (isValidRefinement(indices)) {
//        m_remainder.erase(m_remainder.begin() + m_next);
//        return indices;
//    } else m_next++;
//
//    return {};
}


void Profile::delauneyRefinement()
{
    for (const std::pair<uint32_t, uint32_t>& rem : m_remainder) {

        // Easy direct triangle cutout
        if (rem.second == 3) {
            addTriangle(rem.first);
            continue;
        }

        // Generate chain of just the vertices that need to be refined in this section
        std::vector<uint32_t> indices = sectionIndices(rem);
        std::vector<glm::dvec2> vertices = sectionVertices(indices);

        // Find the Delauney triangulation of the section
        auto rawTriangles = Polygon::triangulate(vertices);

        std::vector<Section> sections = prepareSections(rawTriangles, indices);

        commitSections(sections);
    }
}

void Profile::commitSections(const std::vector<Section>& sections)
{
    for (uint32_t i = 0; i < sections.size(); i++) {
        double sum = area(sections[i].triangle);
        uint32_t j = sections[i].children * (sum == 0);

        // For small, valid sections, sum area of children
        while (sum < m_minimumArea && j < sections[i].children) {
            sum += area(sections[i + j].triangle);
        }

        // Only commit valid sections of sufficient size TODO test
        if (sum < m_minimumArea) {

            // Adjust parent counts to remove impact of now removed sections
            for (int k = (int)m_sections.size() - 1; k >= 0; k--) {
                if (m_sections[k].children == 0) break;
                m_sections[k].children -= sections[i].children;
            }

            i += sections[i].children;
        } else m_sections.push_back(sections[i]);
    }
}

std::vector<Profile::Section> Profile::prepareSections(std::vector<TriIndex>& triangles, const std::vector<uint32_t>& indexMap)
{
    std::vector<Section> sections;
    sections.reserve(triangles.size());

    struct Edge {
        Edge(uint32_t first, uint32_t last, uint32_t depth) : first(first), last(last), depth(depth) {}
        uint32_t first, last, depth;
    };

    std::vector<Edge> edges = { { 0, (uint32_t)indexMap.size() - 1, 0 } };
    std::vector<uint32_t> dIndex; // Index of last section with depth == index of element in dIndex

    // Select triangles from exterior to interior like a tree
    while (!triangles.empty()) {
        if (edges.empty()) throw std::runtime_error("[Profile] Failed to partition triangles");

        auto [valid, nextTri] = nextTriangle(triangles, edges.back().first, edges.back().last);
        uint32_t depth = edges.back().depth;
        edges.pop_back();

        if (valid) {

            // Correct indexing while constructing sections
            sections.emplace_back(TriIndex(indexMap[nextTri.I0], indexMap[nextTri.I1], indexMap[nextTri.I2]), 0);

            edges.emplace_back(nextTri.I0, nextTri.I1, depth + 1);
            edges.emplace_back(nextTri.I1, nextTri.I2, depth + 1);
        }

        // Calculate # of children whenever a similar or shallower depth is reached
        while (depth < dIndex.size()) {
            sections[dIndex.back()].children = sections.size() - dIndex.back() - 2;
            dIndex.pop_back();
        }

        dIndex.emplace_back(sections.size() - 1);
    }

    // Capture remaining (Considers the last node at each depth)
    while (!dIndex.empty()) {
        sections[dIndex.back()].children = sections.size() - dIndex.back() - 1;
        dIndex.pop_back();
    }

    return sections;
}


// Selects the triangle connected to the given indices, returns its other vertex, and removes it from the pool
std::tuple<bool, TriIndex> Profile::nextTriangle(std::vector<TriIndex>& triangles, uint32_t first, uint32_t last)
{
    for (uint32_t i = 0; i < triangles.size(); i++) {
        if (triangles[i].has(first) && triangles[i].has(last)) {
            uint32_t splitVertex = triangles[i].last(first, last);

            std::swap(triangles[i], triangles.back());
            triangles.pop_back();

            return { true, TriIndex(first, splitVertex, last) };
        }
    }

    return { false, TriIndex(0, 0, 0) };
}

void Profile::testRefinement()
{

}

double Profile::area(const TriIndex& triangle) const
{
    if (triangle.I0 == triangle.I1) return 0; // Indicates invalid triangle

    return std::abs(Triangle2D::area(m_vertices[triangle.I0], m_vertices[triangle.I1], m_vertices[triangle.I2]));
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

std::vector<glm::dvec2> Profile::sectionVertices(const std::pair<uint32_t, uint32_t>& limits) const
{
    return sectionVertices(sectionIndices(limits));
}
std::vector<glm::dvec2> Profile::sectionVertices(const std::vector<uint32_t>& indices) const
{
    std::vector<glm::dvec2> section(indices.size());
    for (uint32_t i = 0; i < indices.size(); i++) section[i] = m_vertices[indices[i]];
    return section;
}

// Project vertices into 3D space according to the defined axis system
std::vector<glm::dvec3> Profile::projected3D(const glm::dvec3& offset)
{
    return Polygon::projected3D(m_xAxis, m_yAxis, offset);
}

std::vector<glm::dvec3> Profile::projected3D(const std::vector<uint32_t>& indices, const glm::dvec3& offset)
{
    std::vector<glm::dvec2> vertices;
    vertices.reserve(indices.size());

    for (uint32_t idx : indices) {
        if (idx >= m_vertices.size()) throw std::runtime_error("[Profile] Out of bounds vertex index access!");
        vertices.emplace_back(m_vertices[idx]);
    }

    return Polygon::projected3D(vertices, m_xAxis, m_yAxis, offset);
}

std::vector<std::pair<glm::dvec2, glm::dvec2>> Profile::debugEdges() const {
    auto edges = Polygon::debugEdges();

    // Only need to show one edge of each triangle (All except boundary are shared)
    for (const Section& section: m_sections) {
        edges.emplace_back(m_vertices[section.triangle.I0], m_vertices[section.triangle.I2]);
    }

    // Draw clearances
    if (!m_sections.empty()) {
        TriIndex tri = m_sections[0].triangle;

        auto margin = clearance();
        if (margin.first  != 0) edges.emplace_back(m_vertices[tri.I0], m_vertices[tri.I0] + margin.first  * glm::normalize(m_vertices[tri.I0] - m_vertices[tri.I1]));
        if (margin.second != 0) edges.emplace_back(m_vertices[tri.I2], m_vertices[tri.I2] + margin.second * glm::normalize(m_vertices[tri.I2] - m_vertices[tri.I1]));

    }


    return edges;
}
