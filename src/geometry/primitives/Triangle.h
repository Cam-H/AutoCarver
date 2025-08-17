//
// Created by cameronh on 26/04/24.
//

#ifndef AUTOCARVER_TRIANGLE_H
#define AUTOCARVER_TRIANGLE_H

#include <QVector2D>
#include <QVector3D>

#include <glm.hpp>

class TriIndex {
public:

    TriIndex(uint32_t I0, uint32_t I1, uint32_t I2);

    bool operator==(const TriIndex& other) const;

    uint32_t operator[](uint32_t i) const;
    uint32_t& operator[](uint32_t i);

    [[nodiscard]] bool isValid() const;
    [[nodiscard]] bool isValid(uint32_t limit) const;

    [[nodiscard]] bool has(uint32_t i) const;

    [[nodiscard]] uint32_t last(uint32_t a, uint32_t b) const;


//    friend std::ostream& operator<<(std::ostream& os, const Triangle& tri)
//    {
//        os << '[' << tri.m_I0 << ' ' << tri.m_I1 << ' ' << tri.m_I2 << ']';
//        return os;
//    }

    uint32_t I0;
    uint32_t I1;
    uint32_t I2;
};

class Triangle3D {
public:

    Triangle3D(const std::vector<glm::dvec3>& vertices, const TriIndex& indices);
    Triangle3D(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c);

    [[nodiscard]] glm::dvec3 normal() const;

    static glm::dvec3 normal(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c);

//    static double signedArea(const QVector2D& a, const QVector2D& b, const QVector2D& c);
//
//    static double area(const QVector2D& a, const QVector2D& b, const QVector2D& c);
//
//    static double area(const QVector3D& a, const QVector3D& b, const QVector3D& c);

    static double area(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c);


//    static bool isCCW(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c);
//    static bool isCCW(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c);

    static QVector3D normal(const QVector3D& a, const QVector3D& b, const QVector3D& c)
    {
        return QVector3D::crossProduct(b - a, c - a).normalized();
    }

    static double cross(const QVector2D &pivot, const QVector2D &a, const QVector2D &b);
    static double cross(const QVector2D &v1, const QVector2D &v2);


    static glm::dvec3 barycentric(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, const glm::dvec3& p);
    static glm::dvec3 clampedBarycentric(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, const glm::dvec3& p);

    static bool encloses(const QVector2D& a, const QVector2D& b, const QVector2D& c, const QVector2D& p);

    glm::dvec3 a;
    glm::dvec3 b;
    glm::dvec3 c;
};

class Triangle2D {
public:

    static double area(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c);
    static double signedArea(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c);

};

#endif //AUTOCARVER_TRIANGLE_H
