//
// Created by Cam on 2024-09-16.
//

#ifndef AUTOCARVER_SURFACE_H
#define AUTOCARVER_SURFACE_H

#include <QVector3D>

#include <vector>

#include "Tesselation.h"

class Surface {
public:

    // TODO add support for other types of surfaces (non-critical)
    enum class SurfaceType {
        PLANAR// , CYLINDRICAL, TOROIDAL, NURBS
    };

    explicit Surface(const std::vector<QVector3D> &loop);
    explicit Surface(const std::vector<std::vector<QVector3D>> &loops);
//    explicit Surface(SurfaceType type, );

    void triangulation(std::vector<QVector3D> &vertices, std::vector<Triangle> &triangles) const;
    Tesselation tesselation();


private:


private:

    SurfaceType m_type;

    std::vector<std::vector<QVector3D>> m_vertices;
    QVector3D m_normal;
};


#endif //AUTOCARVER_SURFACE_H
