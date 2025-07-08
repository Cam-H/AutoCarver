//
// Created by cameronh on 26/04/24.
//

#ifndef AUTOCARVER_TRIANGLE_H
#define AUTOCARVER_TRIANGLE_H

#include <QVector2D>
#include <QVector3D>

#include <glm/glm.hpp>

class Triangle {
public:

    Triangle(uint32_t I0, uint32_t I1, uint32_t I2);

    bool operator==(const Triangle& other) const;

    uint32_t operator[](uint32_t i) const;
    uint32_t& operator[](uint32_t i);

    bool has(uint32_t i) const;

    uint32_t last(uint32_t a, uint32_t b) const;

//    friend std::ostream& operator<<(std::ostream& os, const Triangle& tri)
//    {
//        os << '[' << tri.m_I0 << ' ' << tri.m_I1 << ' ' << tri.m_I2 << ']';
//        return os;
//    }

    static float signedArea(const QVector2D& a, const QVector2D& b, const QVector2D& c);

    static float area(const QVector2D& a, const QVector2D& b, const QVector2D& c);

    static float area(const QVector3D& a, const QVector3D& b, const QVector3D& c);

    static float area(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c);
    static float area(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);


    static QVector3D normal(const QVector3D& a, const QVector3D& b, const QVector3D& c)
    {
        return QVector3D::crossProduct(b - a, c - a).normalized();
    }

    static float cross(const QVector2D &pivot, const QVector2D &a, const QVector2D &b);
    static float cross(const QVector2D &v1, const QVector2D &v2);

    static glm::vec3 normal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

    static glm::vec3 barycentric(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& p);
    static glm::vec3 clampedBarycentric(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& p);

    static bool encloses(const QVector2D& a, const QVector2D& b, const QVector2D& c, const QVector2D& p);

    uint32_t I0;
    uint32_t I1;
    uint32_t I2;
};


#endif //AUTOCARVER_TRIANGLE_H
