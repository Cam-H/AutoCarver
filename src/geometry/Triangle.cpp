//
// Created by cameronh on 26/04/24.
//

#include "Triangle.h"

#include <iostream>

Triangle::Triangle(uint32_t I0, uint32_t I1, uint32_t I2)
    : I0(I0)
    , I1(I1)
    , I2(I2)
{
}

uint32_t Triangle::operator[](uint32_t i) const
{
    switch(i){
        case 2:
            return I2;
        case 1:
            return I1;
        case 0:
        default:
            return I0;
    }
}

uint32_t& Triangle::operator[](uint32_t i)
{
    switch(i){
        case 2:
            return I2;
        case 1:
            return I1;
        case 0:
        default:
            return I0;
    }
}

uint32_t Triangle::last(uint32_t a, uint32_t b)
{
    if (I0 != a && I0 != b) return I0;
    if (I1 != a && I1 != b) return I1;
    return I2;
}

float Triangle::signedArea(const QVector2D& a, const QVector2D& b, const QVector2D& c)
{
    return (a.x() - c.x()) * (b.y() - c.y()) - (b.x() - c.x()) * (a.y() - c.y());
}

float Triangle::area(const QVector2D& a, const QVector2D& b, const QVector2D& c)
{
    return 0.5f * (a.x() * (b.y() - c.y()) + b.x() * (c.y() - a.y()) + c.x() * (a.y() - b.y()));
}

float Triangle::area(const QVector3D& a, const QVector3D& b, const QVector3D& c) {
    QVector3D ab = b - a;
    QVector3D ac = c - a;

    return 0.5f * (float)sqrt(pow(ab.y() * ac.z() - ab.z() * ac.y(), 2)
    + pow(ab.z() * ac.x() - ab.x ()* ac.z(), 2)
    + pow(ab.x() * ac.y() - ab.y() * ac.x(), 2));
}

float Triangle::cross(const QVector2D &pivot, const QVector2D &a, const QVector2D &b)
{
    return cross(a - pivot, b - pivot);
}

float Triangle::cross(const QVector2D &v1, const QVector2D &v2)
{
    return v1.x() * v2.y() - v2.x() * v1.y();
}

glm::vec3 Triangle::barycentric(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& p)
{
    glm::vec3 ab = b - a, ac = c - a, ap = p - a;
    float d00 = glm::dot(ab, ab), d01 = glm::dot(ab, ac);
    float d11 = glm::dot(ac, ac);
    float d20 = glm::dot(ap, ab), d21 = glm::dot(ap, ac);
    float den = 1 / (d00 * d11 - d01 * d01);

    float v = (d11 * d20 - d01 * d21) * den;
    float w = (d00 * d21 - d01 * d20) * den;

    return {
            v,
            w,
            1.0f - v - w
    };
}

bool Triangle::encloses(const QVector2D& a, const QVector2D& b, const QVector2D& c, const QVector2D& p)
{
    if (cross(a, p, b) < 0) return false;
    if (cross(b, p, c) < 0) return false;
    if (cross(c, p, a) < 0) return false;

    return true;
}
