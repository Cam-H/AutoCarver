//
// Created by Cam on 2024-09-20.
//

#include "Body.h"

#include <utility>

#include <unordered_map>

#include "../core/Timer.h"

Body::Body(const Tesselation& tesselation) :
      m_tesselation(tesselation)
    , m_isManifold(false)
    , m_area(0)
    , m_volume(0)
    , m_tesselationOK(false)
    , m_isManifoldOK(false)
    , m_areaOK(false)
    , m_volumeOK(false)
    , m_surfaces(tesselation.surface())
{

}

Body::Body(std::vector<Surface> surfaces) : m_surfaces(std::move(surfaces))
    , m_tesselation()
    , m_isManifold(false)
    , m_area(0)
    , m_volume(0)
    , m_tesselationOK(false)
    , m_isManifoldOK(false)
    , m_areaOK(false)
    , m_volumeOK(false)
{

}

const Tesselation &Body::tesselation()
{
    if (!m_tesselationOK) tesselate();
    return m_tesselation;
}

bool Body::isManifold()
{
    if (!m_isManifoldOK) evaluateManifold();
    return m_isManifold;
}

float Body::area()
{
    if (!m_areaOK) calculateArea();
    return m_area;
}

float Body::volume()
{
    if (!m_volumeOK) calculateVolume();
    return m_volume;
}

void Body::tesselate()
{
    ScopedTimer timer("Body Tesselation");

    std::vector<QVector3D> vertices;
    std::vector<Triangle> triangles;

    for (const Surface &surface : m_surfaces) {
        surface.triangulation(vertices, triangles);
    }

    m_tesselation.clear();
    m_tesselation.append(vertices, triangles);

    m_tesselationOK = true;
}

void Body::evaluateManifold()
{
    //TODO

    m_isManifoldOK = true;
}
void Body::calculateArea()
{
    //TODO

    m_areaOK = true;
}
void Body::calculateVolume()
{
    //TODO

    m_volumeOK = true;
};