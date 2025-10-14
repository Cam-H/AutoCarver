//
// Created by cjhat on 2025-10-11.
//

#include "ProfileBuilder.h"

#include <utility>

#include "geometry/Mesh.h"
#include "geometry/poly/Profile.h"

#include "geometry/Axis3D.h"

ProfileBuilder::ProfileBuilder(std::shared_ptr<Mesh> target)
    : m_target(std::move(target))
{

}

void ProfileBuilder::build(const Axis3D& system)
{
    if (m_target == nullptr) return;
    m_system = system;

    m_projection = std::vector<glm::dvec2>();
    m_projection.reserve(m_target->vertexCount());

    for (const glm::dvec3& vertex : m_target->vertices().vertices()) {
        m_projection.emplace_back(glm::dot(m_system.xAxis, vertex), glm::dot(m_system.yAxis, vertex));
    }

    m_hull = Polygon::hull(m_projection);
}

const glm::dvec3& ProfileBuilder::normal() const
{
    return m_system.zAxis;
}

Polygon ProfileBuilder::hull() const
{
    std::vector<glm::dvec2> border;
    border.reserve(m_hull.size());

    for (uint32_t idx : m_hull) {
        border.emplace_back(m_projection[idx]);
    }

    return Polygon(border);
}