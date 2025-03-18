//
// Created by Cam on 2025-03-15.
//

#ifndef AUTOCARVER_VEC3F_H
#define AUTOCARVER_VEC3F_H

#include <iostream>

class vec3f {
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

#endif //AUTOCARVER_VEC3F_H
