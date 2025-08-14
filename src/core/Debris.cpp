//
// Created by cjhat on 2025-08-13.
//

#include "Debris.h"

#include "geometry/collision/Collision.h"



Debris::Debris(const ConvexHull& hull)
    : CompositeBody({ hull })
    , m_connectionCounts(1, 0)
{

}

Debris::Debris(const std::vector<ConvexHull>& hulls)
    : CompositeBody(hulls)
    , m_connectionCounts(hulls.size(), 0)
{

}

// System - xAxis = direction of cut
void Debris::prepareCut(const Pose& system, double thickness)
{
    auto cutPlane = Plane(system.position + thickness * system.axes.yAxis, system.axes.yAxis);
    const uint32_t initialCount = hulls().size();

    std::vector<uint32_t> sources;

    // Only try splitting original hulls (Ignores new ones from previous cut commands since they will have no effect)
    for (uint32_t i = 0; i < m_connectionCounts.size(); i++) {
        if (split(cutPlane, i)) {
            m_connectionCounts[i]++; // Track number of cuts required to release fragment
            sources.emplace_back(i);
        }
    }

    std::cout << "PC " << initialCount << " " << hulls().size() << "\n";

    if (initialCount == hulls().size()) throw std::runtime_error("[Debris] Failed to prepare cut");

    glm::dvec3 origin;

    // Evaluate extents and find origin for cutting
    std::vector<std::pair<double, double>> extents;
    double min = std::numeric_limits<double>::max(), max = std::numeric_limits<double>::lowest();

    for (uint32_t i = initialCount; i < hulls().size(); i++) {
        uint32_t minIndex, maxIndex;
        VertexArray::extremes(hulls()[i].vertices(), system.axes.xAxis, minIndex, maxIndex);
        extents.emplace_back(
                glm::dot(system.axes.xAxis, hulls()[i].vertices()[minIndex]),
                glm::dot(system.axes.xAxis, hulls()[i].vertices()[maxIndex])
                );

        if (extents.back().first < min) {
            origin = hulls()[i].vertices()[minIndex];
            min = extents.back().first;
        }

        if (extents.back().second > max) max = extents.back().second;

        std::cout << extents.back().first << " " << extents.back().second << " EXTENTS\n";
    }

    m_cuts.emplace_back(origin, system.axes.xAxis);

    // Prepare sections
    double dist = max - min;
    std::cout << min << " " << max << " " << dist << "WTF\n";
    for (uint32_t i = initialCount; i < hulls().size(); i++) {
        m_cuts.back().sections.emplace_back(
                sources[i - initialCount],
                i,
                (extents[i - initialCount].first - min) / dist,
                (extents[i - initialCount].second - min) / dist
                );

        std::cout << "PP: " << m_cuts.back().sections.back().srcIndex << " " << m_cuts.back().sections.back().cutIndex << " " << m_cuts.back().sections.back().ts << " " << m_cuts.back().sections.back().tf << "\n";
    }


//    while (hulls().size() > count) {
//        remove(hulls().size() - 1);
//    }
}

// Shaves material from the next section at ratio [0-1]. Releases fragments as connections are removed
std::vector<std::shared_ptr<RigidBody>> Debris::removeMaterial(double ratio)
{
    std::vector<std::shared_ptr<RigidBody>> fragments;

    if (!m_cuts.empty()) {
        if (ratio >= 1) {
            fragments = removeCut();
        } else {
            Plane cutPlane(m_cuts[0].origin + m_cuts[0].axis * ratio, m_cuts[0].axis);

            for (uint32_t i = 0; i < m_cuts[0].sections.size(); i++) {
                if (m_cuts[0].sections[i].ts < ratio) {
                    if (m_cuts[0].sections[i].tf < ratio) { // To delete section
                        std::shared_ptr<RigidBody> fragment = removeSection(i);
                        if (fragment != nullptr) fragments.push_back(fragment);
                    } else {
                        auto remainder = Collision::fragment(hulls()[m_cuts[0].sections[i].cutIndex], cutPlane);
                        if (remainder.isValid()) replace(remainder, m_cuts[0].sections[i].cutIndex);
                    }
                }
            }
        }

        remesh();

        return fragments;
    }

    return {};
}

std::vector<std::shared_ptr<RigidBody>> Debris::removeCut()
{
    std::vector<std::shared_ptr<RigidBody>> fragments;

    for (uint32_t i = 0; i < m_cuts[0].sections.size(); i++) {
        std::shared_ptr<RigidBody> fragment = removeSection(i);
        if (fragment != nullptr) fragments.push_back(fragment);
    }

    m_cuts.pop_front();
    return fragments;
}

std::shared_ptr<RigidBody> Debris::removeSection(uint32_t index)
{
    uint32_t last = hulls().size() - 1, srcIndex = m_cuts[0].sections[index].srcIndex, cutIndex = m_cuts[0].sections[index].cutIndex;

    replaceIndex(last, cutIndex);

    remove(cutIndex);
    m_cuts[0].sections.erase(m_cuts[0].sections.begin() + index);

    if (m_connectionCounts[srcIndex] == 0) throw std::runtime_error("[Debris] Unhandled connection");
    m_connectionCounts[srcIndex]--;

    if (m_connectionCounts[srcIndex] == 0) { // All connections are removed -> release hull as a fragment

        replaceIndex(hulls().size() - 1, srcIndex);

        auto fragment = std::make_shared<RigidBody>(hulls()[srcIndex]);
        fragment->setTransform(m_transform);
        fragment->setType(RigidBody::Type::DYNAMIC);
//        fragment->zero();
//        fragment->disableCollisions();

        remove(srcIndex);
        return fragment;
    }

    return nullptr;
}

// Update section (cut) indices if it is not the last hull
void Debris::replaceIndex(uint32_t oldIndex, uint32_t newIndex)
{
    if (newIndex < oldIndex) {
        for (Cut& cut : m_cuts) {
            for (Section& sec : cut.sections) {
                if (sec.cutIndex == oldIndex) sec.cutIndex = newIndex; // CompositeBody swaps index & last hull prior to removal - Adjust index given that fact
            }
        }
    }
}