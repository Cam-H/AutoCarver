//
// Created by Cam on 2024-09-20.
//

#ifndef AUTOCARVER_BODY_H
#define AUTOCARVER_BODY_H

#include <vector>

#include "Surface.h"

class Body {
public:

    explicit Body(const Tesselation& tesselation);
    explicit Body(std::vector<Surface> surfaces);

    const Tesselation &tesselation();

    bool isManifold();
    float area();
    float volume();

private:

    void tesselate();

    void evaluateManifold();
    void calculateArea();
    void calculateVolume();

private:

    std::vector<Surface> m_surfaces;

    Tesselation m_tesselation;
    bool m_isManifold;
    float m_area;
    float m_volume;

    bool m_tesselationOK;
    bool m_isManifoldOK;
    bool m_areaOK;
    bool m_volumeOK;

};


#endif //AUTOCARVER_BODY_H
