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

    VertexArray(const VertexArray &);
    VertexArray& operator=(const VertexArray &);

    VertexArray(VertexArray &&) noexcept;
    VertexArray& operator=(VertexArray &&) noexcept;

    ~VertexArray();

    bool serialize(std::ofstream& file);
    static VertexArray deserialize(std::ifstream& file);

//    float* operator[](uint32_t idx);
    glm::vec3 operator[](uint32_t idx) const;


    void scale(float scalar);
    void scale(float xScale, float yScale, float zScale);
    void translate(const float* translation);
    void rotate(const float* axis, float theta);

    void replace(uint32_t idx, const glm::vec3& replacement);
    void remove(uint32_t idx);
    void swap(uint32_t I0, uint32_t I1);

    float* sub(uint32_t I0, uint32_t I1);
    static float* sub(const float* a, const float* b);

    void sub(float* res, uint32_t I0, uint32_t I1);
    static void sub(float* res, const float* a, const float* b);

    float dot(uint32_t I0, uint32_t I1);
    static float dot(const float *a, const float *b);

    static void cross(float* res, const float *a, const float *b);

    void normal(float* res, uint32_t I0, uint32_t I1, uint32_t I2);
    static void normal(float* res, const float* a, const float *b, const float *c);
    static void normalize(float* vec);

    static float length2(const float *a);
    static float length(const float *a);

    [[nodiscard]] const float* data() const;
    [[nodiscard]] uint32_t length() const;

    [[nodiscard]] const float* vertices() const;
    [[nodiscard]] uint32_t vertexCount() const;

    [[nodiscard]] static uint32_t stride() ;
    [[nodiscard]] uint32_t size() const; // Get the size of the array in bytes
    [[nodiscard]] bool empty() const;

    [[nodiscard]] std::vector<glm::vec3> toVector() const;

    bool extremes(const float *axis, uint32_t &min, uint32_t &max); // Get furthest vertices (top & bottom) along axis
    bool extreme(uint32_t p1, uint32_t p2, uint32_t& max); // Get furthest vertex (perpendicular) from the provided axis
    bool extreme(uint32_t p1, uint32_t p2, uint32_t p3, uint32_t& max); // Get furthest vertex from the plane formed by provided indices

    void extents(const float *axis, float &near, float &far);

    void print() const;

private:

    VertexArray(uint32_t vertexCount);

private:
    float *m_vertices;
    uint32_t m_vertexCount;

    const static uint8_t STRIDE = 3;
};


#endif //AUTOCARVER_VERTEXARRAY_H
