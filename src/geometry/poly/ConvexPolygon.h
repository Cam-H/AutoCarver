//
// Created by Cam on 2024-10-10.
//

#ifndef AUTOCARVER_CONVEXPOLYGON_H
#define AUTOCARVER_CONVEXPOLYGON_H

#include "Polygon.h"

class ConvexPolygon : public Polygon {

    explicit ConvexPolygon(const std::vector<QVector2D> &vertices);
    explicit ConvexPolygon(const std::vector<QVector3D> &vertices, const QVector3D &normal);

    virtual void boolean(const Polygon &poly, BooleanOperation operation);

};


#endif //AUTOCARVER_CONVEXPOLYGON_H
