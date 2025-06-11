//
// Created by Cam on 2024-10-30.
//

#ifndef AUTOCARVER_VERTEXARRAY_H
#define AUTOCARVER_VERTEXARRAY_H

#include <cstdint>
#include <iostream>
#include <fstream>

#include <vector>

#include <glm/vec3.hpp>

class VertexArray {
public:
    VertexArray(const float* vertices, uint32_t vertexCount);

    explicit VertexArray(const std::vector<glm::vec3>& vertices);


    bool serialize(std::ofstream& file);
    static VertexArray deserialize(std::ifstream& file);

//    float* operator[](uint32_t idx);
    const glm::vec3& operator[](uint32_t idx) const;


    void scale(float scalar);
    void scale(const glm::vec3& scale);
    void translate(const glm::vec3& translation);
    void rotate(const glm::vec3& axis, float theta);

    void replace(uint32_t idx, const glm::vec3& replacement);
    void remove(uint32_t idx);
    void swap(uint32_t I0, uint32_t I1);

    [[nodiscard]] const std::vector<glm::vec3>& vertices() const;
    [[nodiscard]] uint32_t vertexCount() const;

    [[nodiscard]] uint32_t length() const;
    [[nodiscard]] uint32_t size() const; // Get the size of the array in bytes
    [[nodiscard]] bool empty() const;

    [[nodiscard]] static uint32_t stride() ;

    bool extremes(const glm::vec3& axis, uint32_t &min, uint32_t &max) const; // Get furthest vertices (top & bottom) along axis
    bool extreme(uint32_t p1, uint32_t p2, uint32_t& max); // Get furthest vertex (perpendicular) from the provided axis
    bool extreme(uint32_t p1, uint32_t p2, uint32_t p3, uint32_t& max); // Get furthest vertex from the plane formed by provided indices

    void extents(const glm::vec3& axis, float &near, float &far) const;

    void print() const;

private:

    explicit VertexArray(uint32_t vertexCount);

private:
    std::vector<glm::vec3> m_vertices;

    const static uint8_t STRIDE = 3;
};


#endif //AUTOCARVER_VERTEXARRAY_H
