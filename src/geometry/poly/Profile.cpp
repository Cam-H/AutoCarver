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
    , m_next(0)
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
}

void Profile::setMimimumArea(double area)
{
    m_minimumArea = area;
}
void Profile::translate(const glm::dvec2& translation)
{
    Polygon::translate(translation);
}

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

std::vector<uint32_t> Profile::refine()
{
    if (m_next < m_remainder.size()) {
        if (m_remainder[m_next].second == 3) return triangleRefinement(); // Easy direct triangle cutout

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

bool Profile::complete() const
{
    return m_next >= m_remainder.size();
}

bool Profile::isNextExternal() const
{
//    std::cout << "EXT Check: " << m_remainder[m_next].first << " " << offsetIndex(m_remainder[m_next].first + m_remainder[m_next].second - 2) << "\n";
    return !complete()
        && std::binary_search(m_hull.begin(), m_hull.end(), m_remainder[m_next].first)
        && std::binary_search(m_hull.begin(), m_hull.end(), offsetIndex(m_remainder[m_next].first + m_remainder[m_next].second - 2));
}

const glm::dvec3& Profile::normal() const
{
    return m_normal;
}

std::vector<uint32_t> Profile::triangleRefinement()
{
    auto indices = {
            m_remainder[m_next].first,
            offsetIndex(m_remainder[m_next].first, 1),
            offsetIndex(m_remainder[m_next].first, 2)
    };

    if (isValidRefinement(indices)) {
        m_remainder.erase(m_remainder.begin() + m_next);
        return indices;
    } else m_next++;

    return {};
}

std::vector<uint32_t> Profile::directRefinement()
{
    auto cut = m_remainder[m_next];
    auto indices = sectionIndices(cut);

    if (isValidRefinement(indices)) {
        m_remainder.erase(m_remainder.begin() + m_next);
        return indices;
    } else m_next++;

    return {};
}


std::vector<uint32_t> Profile::delauneyRefinement()
{
    std::vector<uint32_t> indices = sectionIndices(m_remainder[m_next]), triangle;
    std::vector<glm::dvec2> vertices = sectionVertices(indices);

    // Find the Delauney triangulation of the remaining section
    auto triangles = Polygon::triangulate(vertices);

    // Select the triangle on the boundary (The only accessible tri)
    for (const TriIndex& tri : triangles) {
        if (tri.has(0) && tri.has(indices.size() - 1)) {
            uint32_t splitVertex = tri.last(0, indices.size() - 1);
            triangle = { indices[0], indices[splitVertex], indices[indices.size() - 1] };
            break;
        }
    }

    // If a viable triangular cut was found
    if (isValidRefinement(triangle)) {
        uint32_t d1 = difference(triangle[0], triangle[1], m_vertices.size()),
                 d2 = difference(triangle[1], triangle[2], m_vertices.size());

        // Create new boundaries based on the cut triangle (When borders are still concave)
        if (d1 > 1) {
            m_remainder[m_next] = { triangle[0], d1 + 1};
            if (d2 > 1) insertRemainder(m_next + 1, triangle[1], d2 + 1);
        } else if (d2 > 1) {
            m_remainder[m_next] = { triangle[1], d2 + 1};
        }

//        std::cout << triangle[0] << " " << triangle[1] << " " << triangle[2] << " Tri\n";

//        return { triangle[2], triangle[1], triangle[0] };
        return triangle;
    } else {
        m_next++; // Skip over this section because it is unreachable
    }

    return {};
}

std::vector<uint32_t> Profile::testRefinement()
{
    return {};
}

bool Profile::isValidRefinement(const std::vector<uint32_t>& indices) const
{
//    std::cout << "VR: " << area(indices) << "\n";
    if (area(indices) < m_minimumArea) {
        std::cout << "Invalid refinement " << area(indices) << " ~ " << m_minimumArea << "\n";
        for (auto i : indices) std::cout << i << "|" << m_vertices[i].x << " " << m_vertices[i].y << "\n";
    }
    return !indices.empty() && area(indices) > m_minimumArea;
}

double Profile::area(const std::vector<uint32_t>& indices) const
{
    if (indices.size() < 3) {
        std::cout << indices.size() << " XXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n";
        return 0;
    } else if (indices.size() == 3)
        return std::abs(Triangle2D::area(m_vertices[indices[0]], m_vertices[indices[1]], m_vertices[indices[2]]));
    else throw std::runtime_error("[Profile] Area calculation not yet developed");
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

    for (const auto& edge : m_remainder)
        edges.emplace_back(m_vertices[edge.first], m_vertices[(edge.first + edge.second - 1) % m_vertices.size()]);

    if (m_next < m_remainder.size()) {
        std::vector<glm::dvec2> vertices = sectionVertices(m_remainder[m_next]);
        auto triangles = Polygon::triangulate(vertices);

        for (const TriIndex& tri: triangles) {
            edges.emplace_back(vertices[tri.I0], vertices[tri.I1]);
            edges.emplace_back(vertices[tri.I1], vertices[tri.I2]);
            edges.emplace_back(vertices[tri.I2], vertices[tri.I0]);
        }
    }


    return edges;
}
