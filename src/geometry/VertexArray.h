//
// Created by Cam on 2024-10-30.
//

#ifndef AUTOCARVER_VERTEXARRAY_H
#define AUTOCARVER_VERTEXARRAY_H

#include <iostream>

class VertexArray {
public:
    VertexArray(const float* vertices, uint32_t vertexCount);

    VertexArray(const VertexArray &);
    VertexArray& operator=(const VertexArray &);

    VertexArray(VertexArray &&) noexcept;            // 4/5: Move Ctor
    VertexArray& operator=(VertexArray &&) noexcept; // 5/5: Move Assignment

    ~VertexArray();

    void remove(uint32_t idx);
    static void swap(float *a, float *b);

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

    [[nodiscard]] const float* vertices() const;
    [[nodiscard]] uint32_t vertexCount() const;

    bool extremes(const float *axis, uint32_t &min, uint32_t &max); // Get furthest vertices (top & bottom) along axis
    bool extreme(uint32_t p1, uint32_t p2, uint32_t& max); // Get furthest vertex (perpendicular) from the provided axis
    bool extreme(uint32_t p1, uint32_t p2, uint32_t p3, uint32_t& max); // Get furthest vertex from the plane formed by provided indices

private:
    float *m_vertices;
    uint32_t m_vertexCount;

    const static uint8_t STRIDE = 3;
};


#endif //AUTOCARVER_VERTEXARRAY_H
