//
// Created by Cam on 2024-10-30.
//

#ifndef AUTOCARVER_VERTEXARRAY_H
#define AUTOCARVER_VERTEXARRAY_H

#include <cstdint>
#include <iostream>

#include <vector>

struct vec3f {
public:

    vec3f();

    vec3f(float x, float y, float z);

    [[nodiscard]] float length() const;
    static float length(const vec3f& vec);

    [[nodiscard]] float length2() const;
    static float length2(const vec3f& vec);

    void normalize();
    [[nodiscard]] vec3f normalized() const;

    [[nodiscard]] float dot(const vec3f& b) const;
    static float dot(const vec3f& a, const vec3f& b);

    [[nodiscard]] vec3f cross(const vec3f& b) const;
    static vec3f cross(const vec3f& a, const vec3f& b);

    static vec3f unitNormal(const vec3f& a, const vec3f& b, const vec3f& c);

    static float determinant(const vec3f& a, const vec3f& b, const vec3f& c);
    static bool collinear(const vec3f& a, const vec3f& b, const vec3f& c);

    friend vec3f operator+(const vec3f& a, const vec3f& b);
    vec3f& operator+=(const vec3f& rhs);

    friend vec3f operator-(const vec3f& a);
    friend vec3f operator-(const vec3f& a, const vec3f& b);
    vec3f& operator-=(const vec3f& rhs);

    friend vec3f operator*(const vec3f& a, float scalar);
    friend vec3f operator*(float scalar, const vec3f& a);
    friend vec3f operator/(const vec3f& a, float scalar);

    friend std::ostream& operator<<(std::ostream& os, const vec3f& vec);

public:
    float x;
    float y;
    float z;
};

class VertexArray {
public:
    VertexArray(const float* vertices, uint32_t vertexCount);

    explicit VertexArray(const std::vector<vec3f>& vertices);

    VertexArray(const VertexArray &);
    VertexArray& operator=(const VertexArray &);

    VertexArray(VertexArray &&) noexcept;
    VertexArray& operator=(VertexArray &&) noexcept;

    ~VertexArray();

//    float* operator[](uint32_t idx);
    vec3f operator[](uint32_t idx) const;


    void scale(float scalar);
    void scale(float xScale, float yScale, float zScale);
    void translate(const float* translation);
    void rotate(const float* axis, float theta);

    void replace(uint32_t idx, const vec3f& replacement);
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

    [[nodiscard]] uint32_t size() const; // Get the size of the array in bytes
    [[nodiscard]] bool empty() const;

    bool extremes(const float *axis, uint32_t &min, uint32_t &max); // Get furthest vertices (top & bottom) along axis
    bool extreme(uint32_t p1, uint32_t p2, uint32_t& max); // Get furthest vertex (perpendicular) from the provided axis
    bool extreme(uint32_t p1, uint32_t p2, uint32_t p3, uint32_t& max); // Get furthest vertex from the plane formed by provided indices

    void extents(const float *axis, float &near, float &far);

    void print() const;

private:
    float *m_vertices;
    uint32_t m_vertexCount;

    const static uint8_t STRIDE = 3;
};


#endif //AUTOCARVER_VERTEXARRAY_H
