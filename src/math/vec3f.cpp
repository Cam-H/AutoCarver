//
// Created by Cam on 2025-03-15.
//

#include "vec3f.h"

#include <cmath>

vec3f::vec3f()
        : x(0)
        , y(0)
        , z(0)
{

}

vec3f::vec3f(float x, float y, float z)
        : x(x)
        , y(y)
        , z(z)
{

}

float vec3f::length() const
{
    return sqrtf(x * x + y * y + z * z);
}

float vec3f::length(const vec3f& vec)
{
    return sqrtf(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

float vec3f::length2() const
{
    return x * x + y * y + z * z;
}

float vec3f::length2(const vec3f& vec)
{
    return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
}

void vec3f::normalize()
{
    float len = 1 / length();
    x *= len;
    y *= len;
    z *= len;
}

vec3f vec3f::normalized() const
{
    float len = 1 / sqrtf(x * x + y * y + z * z);
    return {x * len, y * len, z * len};
}

float vec3f::dot(const vec3f& b) const
{
    return x * b.x + y * b.y + z * b.z;
}
float vec3f::dot(const vec3f& a, const vec3f& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

vec3f vec3f::cross(const vec3f& b) const
{
    return {y * b.z - b.y * z, z * b.x - x * b.z, x * b.y - y * b.x};
}
vec3f vec3f::cross(const vec3f& a, const vec3f& b)
{
    return {a.y * b.z - b.y * a.z, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

vec3f vec3f::unitNormal(const vec3f& a, const vec3f& b, const vec3f& c)
{
    return vec3f::cross(b - a, c - a).normalized();
}

float vec3f::determinant(const vec3f& a, const vec3f& b, const vec3f& c)
{
    return a.x * (b.y * c.z - b.z * c.y) - a.y * (b.x * c.z - b.z * c.x) + a.z * (b.x * c.y - b.y * c.x);
}

bool vec3f::collinear(const vec3f& a, const vec3f& b, const vec3f& c)
{
    vec3f cp = cross(b - a, c - a);

    return std::abs(cp.x) <= 1e-06 && std::abs(cp.y) <= 1e-06 && std::abs(cp.z) <= 1e-06;
}

vec3f operator+(const vec3f& a, const vec3f& b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
vec3f& vec3f::operator+=(const vec3f& rhs)
{
    this->x += rhs.x;
    this->y += rhs.y;
    this->z += rhs.z;

    return *this;
}

vec3f operator-(const vec3f& a)
{
    return {-a.x, -a.y, -a.z};
}
vec3f operator-(const vec3f& a, const vec3f& b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
vec3f& vec3f::operator-=(const vec3f& rhs)
{
    this->x -= rhs.x;
    this->y -= rhs.y;
    this->z -= rhs.z;

    return *this;
}

vec3f operator*(const vec3f& a, float scalar)
{
    return {a.x * scalar, a.y * scalar, a.z * scalar};
}
vec3f operator*(float scalar, const vec3f& a)
{
    return {a.x * scalar, a.y * scalar, a.z * scalar};
}
vec3f operator/(const vec3f& a, float scalar)
{
    return {a.x / scalar, a.y / scalar, a.z / scalar};
}

std::ostream& operator<<(std::ostream& os, const vec3f& vec)
{
    return os << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
}
