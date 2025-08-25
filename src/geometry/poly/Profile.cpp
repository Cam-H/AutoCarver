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
    , m_system(xAxis, yAxis,normal)
    , m_method(RefinementMethod::DIRECT)
    , m_minimumArea(0.0002)
{
    initialize();
}

Profile::Profile(const std::string& filename)
        : Profile()
{
    Serializable::load(filename);
}

Profile::Profile(std::ifstream& file)
        : Profile()
{
    if (!Profile::deserialize(file)) std::cerr << "Failed to deserialize profile properly!\n";
}

bool Profile::serialize(std::ofstream& file) const
{
    Polygon::serialize(file);

    Serializer::writeDVec3(file, m_system.xAxis);
    Serializer::writeDVec3(file, m_system.yAxis);
    Serializer::writeDVec3(file, m_system.zAxis);

    switch (m_method) {
        case RefinementMethod::DIRECT:   Serializer::writeUint(file, 0); break;
        case RefinementMethod::DELAUNEY: Serializer::writeUint(file, 1); break;
        case RefinementMethod::TEST:     Serializer::writeUint(file, 2); break;
    }

    Serializer::writeUint(file, m_hull.size());
    for (uint32_t idx : m_hull) Serializer::writeUint(file, idx);

    Serializer::writeUint(file, m_remainder.size());
    for (const auto& pair : m_remainder) {
        Serializer::writeUint(file, pair.first);
        Serializer::writeUint(file, pair.second);
    }

    Serializer::writeUint(file, m_sections.size());
    for (const Section& section : m_sections) {
        Serializer::writeUint(file, section.triangle.I0);
        Serializer::writeUint(file, section.triangle.I1);
        Serializer::writeUint(file, section.triangle.I2);
        Serializer::writeUint(file, section.children);
    }

//    std::cout << "Serialize\n";
//    print();

    return true;
}

// TODO safety checks
bool Profile::deserialize(std::ifstream& file)
{
    Polygon::deserialize(file);

    m_system.xAxis = Serializer::readDVec3(file);
    m_system.yAxis = Serializer::readDVec3(file);
    m_system.zAxis = Serializer::readDVec3(file);

    // Don't call setter because internal state should not be recalculated (Loaded instead)
    switch (Serializer::readUint(file)) {
        case 0: m_method = RefinementMethod::DIRECT; break;
        case 1: m_method = RefinementMethod::DELAUNEY; break;
        case 2: m_method = RefinementMethod::TEST; break;
    }

    m_hull = std::vector<uint32_t>(Serializer::readUint(file));
    for (uint32_t& idx : m_hull) idx = Serializer::readUint(file);

    m_remainder = std::vector<std::pair<uint32_t, uint32_t>>(Serializer::readUint(file));
    for (std::pair<uint32_t, uint32_t>& pair : m_remainder) {
        pair.first = Serializer::readUint(file);
        pair.second = Serializer::readUint(file);
    }

    m_sections.clear();
    uint32_t size = Serializer::readUint(file);
    for (uint32_t i = 0; i < size; i++) {
        uint32_t I0 = Serializer::readUint(file), I1 = Serializer::readUint(file), I2 = Serializer::readUint(file);
        m_sections.emplace_back(TriIndex(I0, I1, I2), Serializer::readUint(file));
    }

//    std::cout << "Deserialize\n";
//    print();

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

    // Identify concave faces, noting boundaries with the convex hull
    for (uint32_t i = 0; i < m_hull.size(); i++) {
        uint32_t idx = (i + 1) % m_hull.size(), count = 1 + difference(m_hull[i], m_hull[idx], m_vertices.size());
        if (count > 2) emplaceRemainder(m_hull[i], count);
    }

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
    if (m_method != method && !m_remainder.empty()) {
        m_method = method;

        // More involved decomposition of concave regions for processing
        switch (m_method) {
//        case RefinementMethod::DIRECT: directRefinement(); break;
            case RefinementMethod::DELAUNEY: delauneyRefinement(); break;
//        case RefinementMethod::TEST: testRefinement(); break;
            default: throw std::runtime_error("[Profile] Unrecognized refinement method");
        }
    }

    print();
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
        glm::dot(translation, m_system.xAxis),
        glm::dot(translation, m_system.yAxis)
    });
}

void Profile::rotateAbout(const glm::dvec3& axis, double theta)
{
    m_system.rotate(axis, theta);
    auto rotation = glm::angleAxis(theta, axis);
//    m_normal = rotation * m_normal;
//    m_xAxis = rotation * m_xAxis;
//    m_yAxis = rotation * m_yAxis;
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

void Profile::refine()
{
    if (!m_sections.empty()) m_sections.pop_front();
}

TriIndex Profile::next() const
{
    if (m_sections.empty()) return { 0, 0, 0 };
    return m_sections[0].triangle;
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

// Returns the area of the next section
double Profile::area() const
{
    if (m_sections.empty()) return 0;
    return area(m_sections[0].triangle);
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
        intersection(m_vertices[vertexIndex], dir, hv.first, prevVertex(vertexIndex), t);
        intersection(m_vertices[vertexIndex], dir, nextVertex(vertexIndex), hv.second, t);
    }

    return t;
}

bool Profile::intersects(const glm::dvec2& a, const glm::dvec2& b, uint32_t start, uint32_t end) const
{
    uint32_t index = start;
    while (start != end) {
        index = nextVertex(index);
        auto [hit, tIntersection] = intersection(a, b, m_vertices[start], m_vertices[index]);
        if (hit && 1e-12 < tIntersection && tIntersection < 1 - 1e-12) {
//            std::cout << "Intersection: " << start << " " << end << " | " << index << " " << tIntersection << "\n";
            return true;
        }
        start = index;
    }

    return false;
}

void Profile::intersection(const glm::dvec2& a, const glm::dvec2& b, uint32_t start, uint32_t end, double& t) const
{
    uint32_t index = start;
    while (start != end) {
        index = nextVertex(index);
        auto [hit, tIntersection] = intersection(a, b, m_vertices[start], m_vertices[index]);
        if (hit) t = std::min(t, tIntersection);
        start = index;
    }
}

std::tuple<bool, double> Profile::intersection(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c, const glm::dvec2& d)
{
    double a1 = Triangle2D::signedArea(a, b, d), a2 = Triangle2D::signedArea(a, b, c);

    if (a1 * a2 < 0) {
        double a3 = Triangle2D::signedArea(c, d, a), a4 = a3 + a2 - a1;
//        std::cout << "T: " << a3 / (a3-a4);
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
    return m_system.zAxis;
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
std::vector<glm::dvec3> Profile::projected3D(const glm::dvec3& offset) const
{
    return Polygon::projected3D(m_system.xAxis, m_system.yAxis, offset);
}

std::vector<glm::dvec3> Profile::projected3D(const TriIndex& triangle, const glm::dvec3& offset) const
{
    return projected3D(std::vector<uint32_t>{ triangle.I0, triangle.I1, triangle.I2 }, offset);
}

std::vector<glm::dvec3> Profile::projected3D(const std::vector<uint32_t>& indices, const glm::dvec3& offset) const
{
    std::vector<glm::dvec2> vertices;
    vertices.reserve(indices.size());

    for (uint32_t idx : indices) {
        if (idx >= m_vertices.size()) throw std::runtime_error("[Profile] Out of bounds vertex index access!");
        vertices.emplace_back(m_vertices[idx]);
    }

    return Polygon::projected3D(vertices, m_system.xAxis, m_system.yAxis, offset);
}

std::tuple<glm::dvec3, glm::dvec3, glm::dvec3, glm::dvec3> Profile::projected3D(const Profile::Relief& relief) const
{
    glm::dvec3 offset = {};

    return {
        Polygon::projected3D(relief.start,    m_system.xAxis, m_system.yAxis, offset),
        Polygon::projected3D(relief.internal, m_system.xAxis, m_system.yAxis, offset),
        Polygon::projected3D(relief.external, m_system.xAxis, m_system.yAxis, offset),
        Polygon::projected3D(relief.normal,   m_system.xAxis, m_system.yAxis, offset)
    };
}

std::vector<std::pair<glm::dvec2, glm::dvec2>> Profile::debugEdges() const {
    auto edges = Polygon::debugEdges();

    // Only need to show one edge of each triangle (All except boundary are shared)
    for (const Section& section: m_sections) {
        edges.emplace_back(m_vertices[section.triangle.I0], m_vertices[section.triangle.I2]);
    }

    // Show clearances
    if (!m_sections.empty()) {
        TriIndex tri = m_sections[0].triangle;

        double width = 0.108, thickness = 0.011; // Test values
        std::vector<uint8_t> checks;

        auto margin = clearance();
        if (margin.first  != 0) edges.emplace_back(m_vertices[tri.I0], m_vertices[tri.I0] + std::min(margin.first,  2.0) * glm::normalize(m_vertices[tri.I0] - m_vertices[tri.I1]));
        else checks.emplace_back(0);

        if (margin.second != 0) edges.emplace_back(m_vertices[tri.I2], m_vertices[tri.I2] + std::min(margin.second, 2.0) * glm::normalize(m_vertices[tri.I2] - m_vertices[tri.I1]));
        else checks.emplace_back(1);

        // Show reliefs, if appropriate
        for (uint8_t check : checks) {
            auto relief = prepareRelief(width, thickness, check);
            if (relief.valid) {
                for (uint32_t i = 1; i < relief.edges.size(); i++) edges.emplace_back(relief.edges[i - 1], relief.edges[i]);

                auto step = relief.step();
                auto depths = relief.depths();
                glm::dvec2 pos = relief.start + relief.extLength * relief.external, normal = { relief.normal.y, -relief.normal.x };
                glm::dvec2 in = { relief.internal.y, -relief.internal.x };
                if (check == 1) normal = -normal;
                for (double depth : depths) {
                    glm::dvec2 terminus = pos - relief.normal * depth, corner = terminus + normal * thickness;
                    std::cout << "ERR: " << glm::dot(in, terminus - relief.start) << "(" << (100.0 * glm::dot(in, terminus - relief.start) / depth) << "%)"
                            << " " << glm::dot(in, 0.5 * (corner + terminus) - relief.start) << " "
                            << " " << glm::dot(in, corner - relief.start) << "(" << (100.0 * glm::dot(in, corner - relief.start) / depth) << "%)" << "\n";

                    edges.emplace_back(pos, terminus);
                    edges.emplace_back(terminus, corner);
                    pos -= step * relief.external;
                    edges.emplace_back(corner, pos);
                }
            }

            // Indicate desired end point
            edges.emplace_back(relief.start + relief.internal * width, relief.start + relief.internal * width + glm::dvec2{ 0, -0.2 });

            std::cout << "R" << (uint32_t)check << ": " << relief.valid << " " << relief.cutLength << " " << relief.extLength << "\n";
        }
    }


    return edges;
}

void Profile::print() const
{
    std::cout << "[Profile]\n";
    m_system.print();

    std::cout << "hull: [";
    for (uint32_t idx : m_hull) std::cout << idx << " ";

    for (uint32_t idx : m_hull) std::cout << "(" << m_vertices[idx].x << ", " << m_vertices[idx].y << ")\n";

    std::cout << "]\nrem: [";
    for (const auto& rem : m_remainder) std::cout << "(" << rem.first << ", " << rem.second << ") ";

    std::cout << "]\nsections: \n";
    for (const Section& section : m_sections) {
        std::cout << section.triangle.I0 << " " << section.triangle.I1 << " " << section.triangle.I2 << " " << section.children << "\n";
    }

    std::cout << "\n";
}

// Forms a parallelogram representing the required empty region to perform a relief cut
Profile::Relief::Relief(const glm::dvec2& start, const glm::dvec2& split, const glm::dvec2& end, const glm::dvec2& help, double width, double thickness)
    : start(start)
    , internal(split - start)
    , external(glm::normalize(end - start))
    , normal(help - start)
    , thickness(thickness)
    , sink(0)
    , valid(false)
{

    cutLength = glm::length(internal);
    internal /= cutLength;

    double normalLength = glm::length(normal);
    normal /= normalLength;

    theta = acos(glm::dot(internal, external));
    phi = acos(glm::dot(external, normal));

    double del = step(), fi = sin(M_PI - theta - phi) / sin(phi);
    extLength = std::min(cutLength, width) * fi + del;

//    std::cout << "RELIEF: " << cutLength << " -> " << extLength << " " << width << " " << (180*theta/M_PI) << " " << (180*phi/M_PI) << " " << normalLength << "\n";

    // TODO calculate sink
//    auto psi = acos(glm::dot(-normal, glm::normalize(end - split)));
//    firstDepth = extLength * del / fi - std::max(0.0, thickness / tan(psi));

    auto apex = start + external * extLength + normal * width;

    edges = {
            apex - normal * width,
            apex,
            apex - external * extLength
    };

    if (normalLength < width) edges.emplace_back(help);
}

double Profile::Relief::step() const
{
    return thickness / cos(0.5 * M_PI - phi);
}

double Profile::Relief::reduction() const
{
    return std::max(0.0, thickness / tan(M_PI - theta - phi));
}

std::vector<double> Profile::Relief::depths() const
{
    if (extLength < 0) {
//        throw std::runtime_error("[Profile::Relief] Can not evaluate depths. Relief improperly defined");
        std::cout << "\033[93m[Profile::Relief] Can not evaluate depths. Relief improperly defined\033[0m\n";
        return {};
    }

    double x = extLength, del = step(), reduc = reduction(), factor = sin(theta) / sin(M_PI - theta - phi);

    std::vector<double> depth;
    depth.reserve((size_t)std::floor(extLength / del));

    while (x > del) {
        depth.emplace_back(x * factor - reduc);
        x -= del;
    }

    depth[0] += sink;
    return depth;
}

const glm::dvec2& Profile::Relief::apex()
{
    return edges[1];
}

// Tries to find a set of relief cuts to enable the cut
// edgeIndex = 0 -> first edge, edgeIndex = 1 -> second edge
// Checks collisions against a parallelogram of side lengths bladeWidth and depth (calculated) returning true
// if there were no intersections (indicating the operation is accessible to the robot)
// TODO Currently only tries forming a relief cut with the adjacent edge as a guide. There are more (infinitely many) options. Trying against the opposite face would also capture some situations
Profile::Relief Profile::prepareRelief(double bladeWidth, double bladeThickness, uint8_t edgeIndex) const
{
    if (m_sections.empty() || edgeIndex > 1) throw std::runtime_error("[Profile] Can not evaluate relief");

    uint32_t splitIndex = m_sections[0].triangle.I1;
    auto hv = nextHullVertices(splitIndex);

    auto [start, adj, help] = system(edgeIndex);
    auto relief = Relief(start, m_vertices[splitIndex], adj, help, bladeWidth, bladeThickness);

    // Validate relief
    relief.valid = true;
    for (uint32_t i = 1; i < relief.edges.size(); i++) {
        if (intersects(relief.edges[i - 1], relief.edges[i], hv.first, hv.second)) {
            relief.valid = false;
            break;
        }
    }

    return relief;
}

std::tuple<glm::dvec2, glm::dvec2, glm::dvec2> Profile::system(uint8_t edgeIndex) const
{
    const TriIndex& tri = m_sections[0].triangle;

    if (edgeIndex == 0) return { m_vertices[tri.I0], m_vertices[tri.I2], m_vertices[prevVertex(tri.I0)] };
    return                     { m_vertices[tri.I2], m_vertices[tri.I0], m_vertices[nextVertex(tri.I2)] };
}