//
// Created by cameronh on 26/04/24.
//

#ifndef AUTOCARVER_TRIANGLE_H
#define AUTOCARVER_TRIANGLE_H

#include <QVector2D>
#include <QVector3D>

class Triangle {
public:

    Triangle(uint32_t I0, uint32_t I1, uint32_t I2);

    uint32_t operator[](uint32_t i) const;
    uint32_t& operator[](uint32_t i);

//    friend std::ostream& operator<<(std::ostream& os, const Triangle& tri)
//    {
//        os << '[' << tri.m_I0 << ' ' << tri.m_I1 << ' ' << tri.m_I2 << ']';
//        return os;
//    }

    static float signedArea(const QVector2D& a, const QVector2D& b, const QVector2D& c);

    static float area(const QVector2D& a, const QVector2D& b, const QVector2D& c);

    static float area(const QVector3D& a, const QVector3D& b, const QVector3D& c);

    static QVector3D normal(const QVector3D& a, const QVector3D& b, const QVector3D& c)
    {
        return QVector3D::crossProduct(b - a, c - a).normalized();
    }

    uint32_t m_I0;
    uint32_t m_I1;
    uint32_t m_I2;
};


#endif //AUTOCARVER_TRIANGLE_H
