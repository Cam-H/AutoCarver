//
// Created by cameronh on 26/04/24.
//

#include "Triangle.h"

Triangle::Triangle(uint32_t I0, uint32_t I1, uint32_t I2)
    : m_I0(I0)
    , m_I1(I1)
    , m_I2(I2)
{
}

uint32_t Triangle::operator[](uint32_t i) const
{
    switch(i){
        case 2:
            return m_I2;
        case 1:
            return m_I1;
        case 0:
        default:
            return m_I0;
    }
}

uint32_t& Triangle::operator[](uint32_t i)
{
    switch(i){
        case 2:
            return m_I2;
        case 1:
            return m_I1;
        case 0:
        default:
            return m_I0;
    }
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