//
// Created by Cam on 2024-10-30.
//

#include "VertexArray.h"
#include "fileIO/Serializable.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/quaternion.hpp>


#include <limits>
#include <cmath>
#include <utility>

VertexArray::VertexArray(const float* vertices, uint32_t vertexCount)
    : m_vertices(vertexCount)
{
    memcpy(m_vertices.data(), vertices, vertexCount * STRIDE * sizeof(float));
}

VertexArray::VertexArray(uint32_t vertexCount)
        : m_vertices(vertexCount)
{

}

VertexArray::VertexArray(const std::vector<glm::vec3>& vertices)
    : m_vertices(vertices)
{

}

VertexArray::VertexArray(const std::string& filename)
{
    Serializable::deserialize(filename);
}
VertexArray::VertexArray(std::ifstream& file)
{
    VertexArray::deserialize(file);
}

bool VertexArray::serialize(const std::string& filename)
{
    return Serializable::serialize(filename);

}
bool VertexArray::serialize(std::ofstream& file)
{
    Serializer::writeUint(file, m_vertices.size());
//    file.write(reinterpret_cast<const char*>(&m_vertexCount), sizeof(uint32_t));
    file.write(reinterpret_cast<const char*>(m_vertices.data()), m_vertices.size() * stride());

    return true;
}

bool VertexArray::deserialize(const std::string& filename)
{
    return Serializable::deserialize(filename);
}
bool VertexArray::deserialize(std::ifstream& file)
{
    // Read vertex count
    uint32_t vertexCount = Serializer::readUint(file);

    // Read vertex data
    m_vertices.resize(vertexCount);
    file.read(reinterpret_cast<char*>(m_vertices.data()), vertexCount * VertexArray::stride());

    return true;
}

glm::vec3& VertexArray::operator[](uint32_t idx)
{
    if (idx >= m_vertices.size()) throw std::runtime_error("[VertexArray] Invalid array index access!");
    return m_vertices[idx];
}

const glm::vec3& VertexArray::operator[](uint32_t idx) const
{
    if (idx >= m_vertices.size()) throw std::runtime_error("[VertexArray] Invalid array index access!");
    return m_vertices[idx];
}

void VertexArray::scale(float scalar) {
    for (glm::vec3& vertex : m_vertices) {
        vertex *= scalar;
    }
}

void VertexArray::scale(const glm::vec3& scale)
{
    for (glm::vec3& vertex : m_vertices) {
        vertex *= scale;
    }
}

void VertexArray::translate(const glm::vec3& translation)
{
    for (glm::vec3& vertex : m_vertices) {
        vertex += translation;
    }
}

void VertexArray::rotate(const glm::vec3& axis, float theta)
{
    glm::quat rotation = glm::angleAxis(theta, axis);

    for (glm::vec3& vertex : m_vertices) {
        vertex = vertex * rotation;
    }
}

void VertexArray::replace(uint32_t idx, const glm::vec3& replacement)
{
    if (idx < m_vertices.size()) {
        m_vertices[idx] = replacement;
    }
}

void VertexArray::remove(uint32_t idx)
{
    if (idx < m_vertices.size()) {
        m_vertices.erase(m_vertices.begin() + idx);
    }
}

void VertexArray::swap(uint32_t I0, uint32_t I1)
{
    if (I0 > m_vertices.size() || I1 > m_vertices.size()) return;
    std::swap(m_vertices[I0], m_vertices[I1]);
}

std::vector<glm::vec2> VertexArray::project(const glm::vec3& normal)
{
    return project(m_vertices, normal);
}
std::vector<glm::vec2> VertexArray::project(const std::vector<glm::vec3>& vertices, const glm::vec3& normal)
{
    glm::vec3 ref = normal.x * normal.x > 0.99f ? glm::vec3{ 0, 1, 0 } : glm::vec3{ 1, 0, 0 };
    glm::vec3 xAxis = glm::normalize(glm::cross(normal, ref));

    return project(vertices, xAxis, glm::normalize(glm::cross(normal, xAxis)));
}

std::vector<glm::vec2> VertexArray::project(const std::vector<glm::vec3>& vertices, const glm::vec3& xAxis, const glm::vec3& yAxis)
{
    std::vector<glm::vec2> projection;
    projection.reserve(vertices.size());

    for (const glm::vec3& vertex : vertices) {
        projection.emplace_back(glm::dot(xAxis, vertex), glm::dot(yAxis, vertex));
    }

    return projection;
}

const std::vector<glm::vec3>& VertexArray::vertices() const
{
    return m_vertices;
}

uint32_t VertexArray::vertexCount() const
{
    return m_vertices.size();
}

uint32_t VertexArray::length() const
{
    return m_vertices.size();
}

uint32_t VertexArray::size() const
{
    return m_vertices.size() * stride() + sizeof(uint32_t);
}

bool VertexArray::empty() const
{
    return m_vertices.empty();
}

uint32_t VertexArray::stride() {
    return STRIDE * sizeof(float);
}

bool VertexArray::extremes(const glm::vec3& axis, uint32_t &min, uint32_t &max) const
{
    return extremes(m_vertices, axis, min, max);
}

bool VertexArray::extreme(uint32_t p1, uint32_t p2, uint32_t& max) const
{
    return extreme(m_vertices, p1, p2, max);
}

bool VertexArray::extreme(uint32_t p1, uint32_t p2, uint32_t p3, uint32_t& max) const
{
    return extreme(m_vertices, p1, p2, p3, max);
}

bool VertexArray::extremes(const std::vector<glm::vec3>& vertices, const glm::vec3& axis, uint32_t &min, uint32_t &max)
{
    float minValue = std::numeric_limits<float>::max();
    float maxValue = std::numeric_limits<float>::lowest();

    for(uint32_t i = 0; i < vertices.size(); i++){
        float value = glm::dot(vertices[i], axis);

        if(value < minValue){
            minValue = value;
            min = i;
        }

        if(value > maxValue){
            maxValue = value;
            max = i;
        }
    }

    return min != max;
}
bool VertexArray::extreme(const std::vector<glm::vec3>& vertices, uint32_t p1, uint32_t p2, uint32_t& max)
{
    float maxValue = std::numeric_limits<float>::lowest();

    glm::vec3 axis = glm::normalize(vertices[p2] - vertices[p1]);

    for(uint32_t i = 0; i < vertices.size(); i++){
        glm::vec3 test = glm::cross(axis, vertices[p1] - vertices[i]);
        float value = glm::dot(test, test);// Length squared;

        if(value > maxValue){
            maxValue = value;
            max = i;
        }
    }

    return maxValue > std::numeric_limits<float>::epsilon();
}
bool VertexArray::extreme(const std::vector<glm::vec3>& vertices, uint32_t p1, uint32_t p2, uint32_t p3, uint32_t& max)
{
    float maxValue = std::numeric_limits<float>::lowest();

    glm::vec3 axis = glm::normalize(glm::cross(vertices[p2] - vertices[p1], vertices[p3] - vertices[p1]));

    for(uint32_t i = 0; i < vertices.size(); i++){
        glm::vec3 vec = vertices[p1] - vertices[i];
        float value = std::abs(glm::dot(axis, vec));

        if(value > maxValue){
            maxValue = value;
            max = i;
        }
    }

    return std::abs(maxValue) > std::numeric_limits<float>::epsilon();
}

void VertexArray::extents(const glm::vec3& axis, float &near, float &far) const
{
    extents(m_vertices, axis, near, far);
}

void VertexArray::extents(const std::vector<glm::vec3>& vertices, const glm::vec3& axis, float &near, float &far)
{
    uint32_t min, max;
    extremes(vertices, axis, min, max);

    near = glm::dot(axis, vertices[min]);
    far = glm::dot(axis, vertices[max]);
}

float VertexArray::span(const glm::vec3& axis) const
{
    return span(m_vertices, axis);
}
float VertexArray::span(const std::vector<glm::vec3>& vertices, const glm::vec3& axis)
{
    float near, far;
    extents(vertices, axis, near, far);
    return far - near;
}

void VertexArray::clean()
{
    m_vertices = clean(m_vertices);
}

size_t VertexArray::hash(const glm::vec2& vec, float factor)
{
    return cantor((uint32_t)(vec.x * factor), (uint32_t)(vec.y * factor));
}

size_t VertexArray::hash(const glm::vec3& vec, float factor)
{
    return hash((uint32_t)(vec.x * factor), (uint32_t)(vec.y * factor), (uint32_t)(vec.z * factor));
}

size_t VertexArray::hash(size_t a, size_t b, size_t c)
{
    return cantor(a, cantor(b, c));
}

size_t VertexArray::cantor(size_t a, size_t b)
{
    return (a + b + 1) * (a + b) / 2 + b;
}

void VertexArray::print() const
{
    std::cout << "~~~~~ Vertices (" << m_vertices.size() << ") ~~~~~\n";
    for (const glm::vec3& vertex : m_vertices) {
        std::cout << vertex.x << ", " << vertex.y << ", " << vertex.z << "\n";
    }
}