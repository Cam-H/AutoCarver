//
// Created by Cam on 2024-10-30.
//

#include "VertexArray.h"

#include <limits>
#include <cmath>

VertexArray::VertexArray(const float* vertices, uint32_t vertexCount)
    : m_vertices(new float[vertexCount * STRIDE])
    , m_vertexCount(vertexCount)
{
    std::copy(vertices, vertices + vertexCount * STRIDE, m_vertices);
}

void VertexArray::remove(uint32_t idx)
{
    if (idx < m_vertexCount) {
        m_vertexCount--;

        for (uint8_t i = 0; i < 3; i++) m_vertices[idx * STRIDE + i] = m_vertices[m_vertexCount * STRIDE + i];
    }
}

void VertexArray::swap(float *a, float *b)
{
    for (uint8_t i = 0; i < 3; i++) {
        float temp = a[i];

        a[i] = b[i];
        b[i] = temp;
    }
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

const float* VertexArray::vertices() const
{
    return m_vertices;
}

uint32_t VertexArray::vertexCount() const
{
    return m_vertexCount;
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

    float *axis = sub(p2, p1), test[3];
    normalize(axis);
//    float length = length2(axis);

    for(uint32_t i = 0; i < m_vertexCount; i++){
        float *vec = sub(p1, i);
        cross(test, axis, vec);

        float value = length2(test);// / length;

        if(value > maxValue){
            maxValue = value;
            max = i;
        }

//        delete vec;
    }

//    delete axis;

    return maxValue > std::numeric_limits<float>::epsilon();
}

bool VertexArray::extreme(uint32_t p1, uint32_t p2, uint32_t p3, uint32_t& max)
{
    float maxValue = std::numeric_limits<float>::lowest();

    auto axis = new float[3];
    normal(axis, p1, p2, p3);

    for(uint32_t i = 0; i < m_vertexCount; i++){
        float *vec = sub(p1, i);
        float value = std::abs(dot(axis, vec));
        delete vec;

        if(value > maxValue){
            maxValue = value;
            max = i;
        }
    }

    return std::abs(maxValue) > std::numeric_limits<float>::epsilon();
}