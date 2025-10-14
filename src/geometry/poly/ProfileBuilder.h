//
// Created by cjhat on 2025-10-11.
//

#ifndef AUTOCARVER_PROFILEBUILDER_H
#define AUTOCARVER_PROFILEBUILDER_H

#include <memory>

#include <glm.hpp>

class Mesh;
class Profile;

#include "geometry/poly/Polygon.h"
#include "geometry/Axis3D.h"

// TODO extend beyond just the convex hull
class ProfileBuilder {
public:

    explicit ProfileBuilder(std::shared_ptr<Mesh> target);

    void build(const Axis3D& system);

    [[nodiscard]] const glm::dvec3& normal() const;
    [[nodiscard]] Polygon hull() const;

private:

    std::shared_ptr<Mesh> m_target;

    Axis3D m_system; // Axes used for the most recent build

    std::vector<glm::dvec2> m_projection;
    std::vector<uint32_t> m_hull; // Indices of the projected hull

};


#endif //AUTOCARVER_PROFILEBUILDER_H
