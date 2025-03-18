//
// Created by Cam on 2024-10-30.
//

#include "VertexArray.h"

#include <iostream>

#include <limits>
#include <cmath>
#include <utility>

VertexArray::VertexArray(const float* vertices, uint32_t vertexCount)
    : m_vertices(new float[vertexCount * STRIDE])
    , m_vertexCount(vertexCount)
{
    memcpy(m_vertices, vertices, m_vertexCount * STRIDE * sizeof(float));
}

VertexArray::VertexArray(const std::vector<vec3f>& vertices)
    : m_vertices(new float[vertices.size() * STRIDE])
    , m_vertexCount(vertices.size())
{
    memcpy(m_vertices, vertices.data(), m_vertexCount * STRIDE * sizeof(float));
}

VertexArray::VertexArray(const VertexArray& other)
        : VertexArray(other.m_vertices, other.m_vertexCount)
{

}

VertexArray& VertexArray::operator=(const VertexArray& other)
{
//    auto local_vertices = new float[other.m_vertexCount];
//    memcpy(local_vertices, array.m_vertices, array.m_vertexCount * STRIDE * sizeof(float));
//
//    delete[] m_vertices;
//    m_vertices = local_vertices;
//    m_vertexCount = array.m_vertexCount;

    if (this == &other)
        return *this;

    VertexArray temp(other);
    std::swap(m_vertices, temp.m_vertices);
    m_vertexCount = temp.m_vertexCount;

    return *this;
}

VertexArray::VertexArray(VertexArray&& other) noexcept
    : m_vertices(std::exchange(other.m_vertices, nullptr))
    , m_vertexCount(other.m_vertexCount)
{

}

VertexArray& VertexArray::operator=(VertexArray&& other) noexcept
{
    VertexArray temp(std::move(other));
    std::swap(m_vertices, temp.m_vertices);
    m_vertexCount = temp.m_vertexCount;

    return *this;
}

VertexArray::~VertexArray()
{
    delete[] m_vertices;
}

//float* VertexArray::operator[](uint32_t idx)
//{
//    return &m_vertices[idx * STRIDE];
//}

vec3f VertexArray::operator[](uint32_t idx) const
{
    return {m_vertices[idx * STRIDE], m_vertices[idx * STRIDE + 1], m_vertices[idx * STRIDE + 2]};
}

void VertexArray::scale(float scalar) {
    for (uint32_t i = 0; i < m_vertexCount * STRIDE; i++) {
        m_vertices[i] *= scalar;
    }
}

void VertexArray::scale(float xScale, float yScale, float zScale)
{
    for (uint32_t i = 0; i < m_vertexCount * STRIDE; i+=3) {
        m_vertices[i    ] *= xScale;
        m_vertices[i + 1] *= yScale;
        m_vertices[i + 2] *= zScale;
    }
}

void VertexArray::translate(const float* translation)
{
    for (uint32_t i = 0; i < m_vertexCount * STRIDE; i++) {
        m_vertices[i] += translation[i % STRIDE];
    }
}

void VertexArray::rotate(const float* axis, float theta)
{
    float val = sinf(theta / 2), temp[3];

    float quat[4] = {cosf(theta / 2), axis[0] * val, axis[1] * val, axis[2] * val};

    float mat[3][3] = {
            {
                1 - 2 * (powf(quat[2], 2) + powf(quat[3], 2)),
                2 * (quat[1] * quat[2] - quat[3] * quat[0]),
                2 * (quat[1] * quat[3] + quat[2] * quat[0])
            }, {
                2 * (quat[1] * quat[2] + quat[3] * quat[0]),
                1 - 2 * (powf(quat[1], 2) + powf(quat[3], 2)),
                2 * (quat[2] * quat[3] - quat[1] * quat[0])
            }, {
                2 * (quat[1] * quat[3] - quat[2] * quat[0]),
                2 * (quat[2] * quat[3] + quat[1] * quat[0]),
                1 - 2 * (powf(quat[1], 2) + powf(quat[2], 2))
            }
    };

    for (uint32_t i = 0; i < m_vertexCount; i++) {
        temp[0] = m_vertices[i * STRIDE + 0] * mat[0][0] + m_vertices[i * STRIDE + 1] * mat[0][1] + m_vertices[i * STRIDE + 2] * mat[0][2];
        temp[1] = m_vertices[i * STRIDE + 0] * mat[1][0] + m_vertices[i * STRIDE + 1] * mat[1][1] + m_vertices[i * STRIDE + 2] * mat[1][2];
        temp[2] = m_vertices[i * STRIDE + 0] * mat[2][0] + m_vertices[i * STRIDE + 1] * mat[2][1] + m_vertices[i * STRIDE + 2] * mat[2][2];

        m_vertices[i * STRIDE + 0] = temp[0];
        m_vertices[i * STRIDE + 1] = temp[1];
        m_vertices[i * STRIDE + 2] = temp[2];
    }
}

void VertexArray::replace(uint32_t idx, const vec3f& replacement)
{
    if (idx < m_vertexCount) {
        m_vertices[idx * STRIDE    ] = replacement.x;
        m_vertices[idx * STRIDE + 1] = replacement.y;
        m_vertices[idx * STRIDE + 2] = replacement.z;
    }
}

void VertexArray::remove(uint32_t idx)
{
    if (idx < m_vertexCount) {
        m_vertexCount--;

        for (uint8_t i = 0; i < 3; i++) m_vertices[idx * STRIDE + i] = m_vertices[m_vertexCount * STRIDE + i];
    }
}

void VertexArray::swap(uint32_t I0, uint32_t I1)
{
    if (I0 > m_vertexCount || I1 > m_vertexCount) return;

    I0 *= STRIDE;
    I1 *= STRIDE;

    auto ptr1 = &m_vertices[I0], ptr2 = &m_vertices[I1];
    vec3f temp = {m_vertices[I0], m_vertices[I0 + 1], m_vertices[I0 + 2]};
    *ptr1++ = *ptr2++;
    *ptr1++ = *ptr2++;
    *ptr1++ = *ptr2;

    ptr2 -= 2;
    *ptr2++ = temp.x;
    *ptr2++ = temp.y;
    *ptr2++ = temp.z;
}

float* VertexArray::sub(uint32_t I0, uint32_t I1)
{
    return sub(&m_vertices[I0 * STRIDE], &m_vertices[I1 * STRIDE]);
}

float* VertexArray::sub(const float* a, const float* b)
{
    return new float[3] {
            a[0] - b[0],
            a[1] - b[1],
            a[2] - b[2]
    };
}

void VertexArray::sub(float* res, uint32_t I0, uint32_t I1)
{
    sub(res, &m_vertices[I0 * STRIDE], &m_vertices[I1 * STRIDE]);
}

void VertexArray::sub(float* res, const float* a, const float* b)
{
    res[0] = a[0] - b[0];
    res[1] = a[1] - b[1];
    res[2] = a[2] - b[2];
}

float VertexArray::dot(uint32_t I0, uint32_t I1)
{
    return dot(&m_vertices[I0 * STRIDE], &m_vertices[I1 * STRIDE]);
}

float VertexArray::dot(const float *a, const float *b)
{
    return a[0] * b[0] + a[1] * b[1]+ a[2] * b[2];
}

void VertexArray::cross(float* res, const float *a, const float *b)
{
    res[0] = a[1] * b[2] - a[2] * b[1];
    res[1] = a[2] * b[0] - a[0] * b[2];
    res[2] = a[0] * b[1] - a[1] * b[0];
}

void VertexArray::normal(float* res, uint32_t I0, uint32_t I1, uint32_t I2)
{
    normal(res, &m_vertices[I0 * STRIDE], &m_vertices[I1 * STRIDE], &m_vertices[I2 * STRIDE]);
}

void VertexArray::normal(float* res, const float* a, const float *b, const float *c)
{
    float* ab = sub(b, a);
    float* ac = sub(c, a);

    cross(res, ab, ac);
    normalize(res);

    delete ab, ac;
}

void VertexArray::normalize(float* vec)
{
    float len = 1.0f / length(vec);
    vec[0] *= len;
    vec[1] *= len;
    vec[2] *= len;
}

float VertexArray::length2(const float *a)
{
    return powf(a[0], 2) + powf(a[1], 2) + powf(a[2], 2);
}

float VertexArray::length(const float *a)
{
    return sqrtf(length2(a));
}

const float* VertexArray::data() const
{
    return m_vertices;
}

uint32_t VertexArray::length() const
{
    return m_vertexCount;
}

const float* VertexArray::vertices() const
{
    return m_vertices;
}

uint32_t VertexArray::vertexCount() const
{
    return m_vertexCount;
}

uint32_t VertexArray::size() const
{
    return (m_vertexCount * STRIDE) * sizeof(float) + sizeof(uint32_t);
}

bool VertexArray::empty() const
{
    return m_vertexCount == 0;
}

bool VertexArray::extremes(const float *axis, uint32_t &min, uint32_t &max)
{
    float minValue = std::numeric_limits<float>::max();
    float maxValue = std::numeric_limits<float>::lowest();

    for(uint32_t i = 0; i < m_vertexCount; i++){
        float value = dot(&m_vertices[i * STRIDE], axis);// TODO double check

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

bool VertexArray::extreme(uint32_t p1, uint32_t p2, uint32_t& max)
{
    float maxValue = std::numeric_limits<float>::lowest();

    float axis[3], test[3], vec[3];
    sub(axis, p2, p1);
    normalize(axis);

    for(uint32_t i = 0; i < m_vertexCount; i++){
        sub(vec, p1, i);
        cross(test, axis, vec);

        float value = length2(test);// / length;

        if(value > maxValue){
            maxValue = value;
            max = i;
        }
    }

    return maxValue > std::numeric_limits<float>::epsilon();
}

bool VertexArray::extreme(uint32_t p1, uint32_t p2, uint32_t p3, uint32_t& max)
{
    float maxValue = std::numeric_limits<float>::lowest();

    float axis[3], vec[3];
    normal(axis, p1, p2, p3);

    for(uint32_t i = 0; i < m_vertexCount; i++){
        sub(vec, p1, i);
        float value = std::abs(dot(axis, vec));

        if(value > maxValue){
            maxValue = value;
            max = i;
        }
    }

    return std::abs(maxValue) > std::numeric_limits<float>::epsilon();
}

void VertexArray::extents(const float *axis, float &near, float &far)
{
    uint32_t min, max;
    extremes(axis, min, max);

    near = VertexArray::dot(axis, &m_vertices[min * STRIDE]);
    far = VertexArray::dot(axis, &m_vertices[max * STRIDE]);
}

void VertexArray::print() const
{
    std::cout << "~~~~~ Vertices (" << m_vertexCount << ") ~~~~~\n";
    for (uint32_t i = 0; i < m_vertexCount; i++) {
        std::cout << m_vertices[i * STRIDE] << ", " << m_vertices[i * STRIDE + 1] << ", " << m_vertices[i * STRIDE + 2] << "\n";
    }
}