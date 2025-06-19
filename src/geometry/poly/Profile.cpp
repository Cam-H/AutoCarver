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
        uint32_t idx = (i + 1) % border.size(), count = border[idx] < border[i]
                            ? m_vertices.size() - border[i] + border[idx]
                            : border[idx] - border[i];

        if (count > 1) m_remainder.emplace_back(border[i], count);
    }
}

std::tuple<uint32_t, uint32_t, uint32_t> Profile::refine(float margin)
{

    // TODO verify margin

    if (m_next < m_remainder.size()) {
        auto cut = m_remainder[m_next];
        uint32_t end = (cut.first + cut.second) % m_vertices.size();

        std::cout << m_next << " " << m_remainder.size() << " " << cut.first << " " << cut.second << "\n";
        if (cut.second == 2) { // Easy direct triangle cutout
            m_remainder.erase(m_remainder.begin() + m_next);
            return { cut.first, (cut.first + 1) % m_vertices.size(), end };
        }

        std::vector<uint32_t> splitPoints;

        glm::vec2 axis = glm::normalize(m_vertices[end] - m_vertices[cut.first]);
        glm::vec2 normal = { -axis.y, -axis.x };

        float xLim = std::numeric_limits<float>::lowest();
        for (uint32_t i = 0; i < cut.second; i++) {
            uint32_t idx = (cut.first + i) % m_vertices.size();
            float value = glm::dot(axis, m_vertices[idx]);
            if (xLim < value) {
                splitPoints.emplace_back(idx);
                xLim = value;
            }
        }

        xLim = std::numeric_limits<float>::max();
        for (int i = splitPoints.size() - 1; i >= 0; i--) {
            float value = glm::dot(axis, m_vertices[splitPoints[i]]);
            std::cout << "AA " << i << " " << splitPoints[i] << " " << value << "\n";
//            if ()
        }

        for (uint32_t p : splitPoints) std::cout << p << " ";
        std::cout << "~SP\n";

        std::cout << axis.x << " " << axis.y << " | " << normal.x << " " << normal.y << "\n";
    }

    return {};
}

// Project vertices into 3D space according to the defined axis system
std::vector<glm::vec3> Profile::projected3D(const glm::vec3& offset)
{


//    for (std::pair<uint32_t, glm::vec3>& vertex : m_border) {
//        if (vertex.first == std::numeric_limits<uint32_t>::max()) { // Unmatched vertex (Probably concave)
//            vertex.second = scale * (vertex.second - glm::vec3{ offset.x, offset.y, 0 });
//            vertex.second = (m_right * vertex.second.x - m_up * vertex.second.y + origin);
//        } else { // Matched vertex (On convex hull)
//            vertex.first = hull[vertex.first];
//            vertex.second = m_hull.vertices()[vertex.first] - m_axis * glm::dot(m_axis, m_hull.vertices()[vertex.first]);
//        }
//    }

    return Polygon::projected3D(m_xAxis, m_yAxis, offset);
}

std::vector<std::pair<glm::vec2, glm::vec2>> Profile::debugEdges() const {
    auto edges = Polygon::debugEdges();

    for (const auto& edge : m_remainder)
        edges.emplace_back(m_vertices[edge.first], m_vertices[(edge.first + edge.second) % m_vertices.size()]);

    return edges;
}
