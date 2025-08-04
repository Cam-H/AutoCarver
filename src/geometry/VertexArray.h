//
// Created by Cam on 2024-10-30.
//

#ifndef AUTOCARVER_VERTEXARRAY_H
#define AUTOCARVER_VERTEXARRAY_H

#include "fileIO/Serializable.h"

#include <cstdint>
#include <iostream>
#include <fstream>

#include <vector>

#include <vec2.hpp>
#include <vec3.hpp>

class VertexArray : public Serializable {
public:
//    VertexArray(const double* vertices, uint32_t vertexCount);

    explicit VertexArray(uint32_t vertexCount);
    explicit VertexArray(const std::vector<glm::dvec2>& vertices);
    explicit VertexArray(const std::vector<glm::dvec3>& vertices);

    explicit VertexArray(const std::string& filename);
    explicit VertexArray(std::ifstream& file);


    bool serialize(const std::string& filename) override;
    bool serialize(std::ofstream& file) override;

    bool deserialize(const std::string& filename) override;
    bool deserialize(std::ifstream& file) override;

//    static VertexArray deserialize(std::ifstream& file);

//    double* operator[](uint32_t idx);
    glm::dvec3& operator[](uint32_t idx);
    const glm::dvec3& operator[](uint32_t idx) const;


    void scale(double scalar);
    void scale(const glm::dvec3& scale);
    void translate(const glm::dvec3& translation);
    void rotate(const glm::dvec3& axis, double theta);
    void rotate(const glm::dquat& rotation);
    static void rotate(std::vector<glm::dvec3>& vertices, const glm::dvec3& axis, double theta);
    static void rotate(std::vector<glm::dvec3>& vertices, const glm::dquat& rotation);

    static std::vector<glm::dvec3> rotated(const std::vector<glm::dvec3>& vertices, const glm::dvec3& axis, double theta);
    static std::vector<glm::dvec3> rotated(const std::vector<glm::dvec3>& vertices, const glm::dquat& rotation);

    void replace(uint32_t idx, const glm::dvec3& replacement);
    void remove(uint32_t idx);
    void swap(uint32_t I0, uint32_t I1);

    std::vector<glm::dvec2> project(const glm::dvec3& normal);
    static std::vector<glm::dvec2> project(const std::vector<glm::dvec3>& vertices, const glm::dvec3& normal);
    static std::vector<glm::dvec2> project(const std::vector<glm::dvec3>& vertices, const glm::dvec3& xAxis, const glm::dvec3& yAxis);

    [[nodiscard]] const std::vector<glm::dvec3>& vertices() const;
    [[nodiscard]] uint32_t vertexCount() const;

    [[nodiscard]] uint32_t length() const;
    [[nodiscard]] uint32_t size() const; // Get the size of the array in bytes
    [[nodiscard]] bool empty() const;

    [[nodiscard]] static uint32_t stride() ;

    bool extremes(const glm::dvec3& axis, uint32_t &min, uint32_t &max) const; // Get furthest vertices (top & bottom) along axis
    bool extreme(uint32_t p1, uint32_t p2, uint32_t& max) const; // Get furthest vertex (perpendicular) from the provided axis
    bool extreme(uint32_t p1, uint32_t p2, uint32_t p3, uint32_t& max) const; // Get furthest vertex from the plane formed by provided indices

    static bool extremes(const std::vector<glm::dvec3>& vertices, const glm::dvec3& axis, uint32_t &min, uint32_t &max);
    static bool extreme(const std::vector<glm::dvec3>& vertices, uint32_t p1, uint32_t p2, uint32_t& max);
    static bool extreme(const std::vector<glm::dvec3>& vertices, uint32_t p1, uint32_t p2, uint32_t p3, uint32_t& max);


    void extents(const glm::dvec3& axis, double &near, double &far) const;
    static void extents(const std::vector<glm::dvec3>& vertices, const glm::dvec3& axis, double &near, double &far);

    double span(const glm::dvec3& axis) const;
    static double span(const std::vector<glm::dvec3>& vertices, const glm::dvec3& axis);

    // Removes duplicate vertices
    void clean();

    template<class T>
    static std::vector<T> clean(const std::vector<T>& vertices)
    {
        std::vector<T> cleaned;
        cleaned.reserve(vertices.size());

        double tolerance = 1e-3, factor = 1 / tolerance; // TODO validate such high tolerance
        std::unordered_map<size_t, uint32_t> vertexMap;

        // Only attach non-coincident vertices
        for (uint32_t i = 0; i < vertices.size(); i++) {
            size_t key = hash(vertices[i], factor);

            auto it = vertexMap.find(key);
            if (it == vertexMap.end()) {
                cleaned.push_back(vertices[i]);
                vertexMap[key] = i;
            }
        }

        cleaned.shrink_to_fit();
        return cleaned;
    }

    template<class T>
    static std::vector<uint32_t> cleanIndex(const std::vector<T>& vertices)
    {
        double tolerance = 1e-3, factor = 1 / tolerance; // TODO validate such high tolerance
        std::unordered_map<size_t, uint32_t> vertexMap;
        std::vector<uint32_t> indices(vertices.size(), std::numeric_limits<uint32_t>::max());

        // Remove coincident vertices, recording indices to use to recover vertex mapping
        for (uint32_t i = 0; i < indices.size(); i++) {
            size_t key = hash(vertices[i], factor);

            auto it = vertexMap.find(key);
            if (it == vertexMap.end()) {
                indices[i] = vertexMap[key] = i;
            } else indices[i] = it->second;
        }

        return indices;
    }

    void print() const;

private:

    [[nodiscard]] static size_t hash(const glm::vec2& vec, double factor);
    [[nodiscard]] static size_t hash(const glm::dvec3& vec, double factor);
    [[nodiscard]] static size_t hash(size_t a, size_t b, size_t c) ;
    [[nodiscard]] static size_t cantor(size_t a, size_t b) ;

private:
    std::vector<glm::dvec3> m_vertices;

    const static uint8_t STRIDE = 3;
};


#endif //AUTOCARVER_VERTEXARRAY_H
