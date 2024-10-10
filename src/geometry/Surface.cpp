//
// Created by Cam on 2024-09-16.
//

#include "Surface.h"

#include <iostream>

#include "Polygon.h"
#include "../core/Timer.h"

Surface::Surface(const std::vector<QVector3D> &loop) : m_type(SurfaceType::PLANAR), m_vertices({loop})
{
    if (loop.size() > 2) {
        m_normal = -QVector3D::crossProduct(loop[1] - loop[0], loop[2] - loop[0]).normalized();
    }
}

Surface::Surface(const std::vector<std::vector<QVector3D>> &loops) : m_type(SurfaceType::PLANAR), m_vertices(loops)
{
    if (!loops.empty() && loops[0].size() > 2) {
        m_normal = -QVector3D::crossProduct(loops[0][1] - loops[0][0], loops[0][2] - loops[0][0]).normalized();
    }
}

void Surface::triangulation(std::vector<QVector3D> &vertices, std::vector<Triangle> &triangles) const
{
    switch (m_type) {
        case SurfaceType::PLANAR:
        {
            Polygon poly(m_vertices, m_normal);
            std::vector<Triangle> triangulation = poly.triangulation();

            uint32_t base = vertices.size();

            for (const std::vector<QVector3D> &loop : m_vertices) {
                vertices.insert(vertices.end(), loop.begin(), loop.end());
            }

            for (const Triangle &tri : triangulation) {
                triangles.emplace_back(base + tri.m_I0, base + tri.m_I1, base + tri.m_I2);
            }

        }
            break;
    }
}

Tesselation Surface::tesselation()
{

    std::vector<QVector3D> vertices;
    std::vector<Triangle> triangles;

    triangulation(vertices, triangles);

    Tesselation tessel;
    tessel.append(vertices, triangles);

    return tessel;
}