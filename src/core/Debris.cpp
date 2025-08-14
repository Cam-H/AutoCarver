//
// Created by cjhat on 2025-08-13.
//

#include "Debris.h"

#include "geometry/collision/Collision.h"



Debris::Debris(const ConvexHull& hull)
    : CompositeBody({ hull })
    , m_connectionCounts(1, 0)
{
    applyCompositeColors(false);
}

Debris::Debris(const std::vector<ConvexHull>& hulls)
    : CompositeBody(hulls)
    , m_connectionCounts(hulls.size(), 0)
{
    applyCompositeColors(false);
}

// System [Local] - xAxis = direction of cut
void Debris::prepareCut(const Pose& system, double thickness)
{
    auto cutPlane = Plane(system.position + thickness * system.axes.yAxis, system.axes.yAxis);
    auto axis = system.axes.xAxis;

    const uint32_t initialCount = hulls().size();

    std::vector<uint32_t> sources;

    // Only try splitting original hulls (Ignores new ones from previous cut commands since they will have no effect)
    for (uint32_t i = 0; i < m_connectionCounts.size(); i++) {
        if (split(cutPlane, i)) {
            m_connectionCounts[i]++; // Track number of cuts required to release fragment
            sources.emplace_back(i);
        }
    }

    if (initialCount == hulls().size()) throw std::runtime_error("[Debris] Failed to prepare cut");

    glm::dvec3 origin;

    // Evaluate extents and find origin for cutting
    std::vector<std::pair<double, double>> extents;
    double min = std::numeric_limits<double>::max();

    for (uint32_t i = initialCount; i < hulls().size(); i++) {
        auto [minIndex, maxIndex] = hulls()[i].extremes(axis);
        extents.emplace_back(
                glm::dot(axis, hulls()[i].vertices()[minIndex]),
                glm::dot(axis, hulls()[i].vertices()[maxIndex])
                );

        if (extents.back().first < min) {
            origin = hulls()[i].vertices()[minIndex];
            min = extents.back().first;
        }
    }

    m_cuts.emplace_back(origin, axis);

    // Prepare sections
    for (uint32_t i = initialCount; i < hulls().size(); i++) {
        m_cuts.back().sections.emplace_back(
                sources[i - initialCount],
                i,
                extents[i - initialCount].first - min,
                extents[i - initialCount].second - min,
                hulls()[sources[i - initialCount]].far(axis) - min
                );
    }
}

// Shaves material from the next section (depth from the origin). Releases fragments as connections are removed
std::vector<std::shared_ptr<RigidBody>> Debris::removeMaterial(double depth)
{
    std::vector<std::shared_ptr<RigidBody>> fragments;

    if (!m_cuts.empty()) {
        Plane cutPlane(m_cuts[0].origin + m_cuts[0].axis * depth, m_cuts[0].axis);

        for (uint32_t i = 0; i < m_cuts[0].sections.size(); i++) {
            if (m_cuts[0].sections[i].ts < depth) {
                if (m_cuts[0].sections[i].depth <= depth + 1e-12) {
                    std::shared_ptr<RigidBody> fragment = tryFragmentRelease(m_cuts[0].sections[i]);
                    if (fragment != nullptr) fragments.push_back(fragment);
                }

                if (m_cuts[0].sections[i].tf <= depth + 1e-12) { // To delete section
                    removeSection(i); // Fragment must already have been released if reached
                } else {
                    auto remainder = Collision::fragment(hulls()[m_cuts[0].sections[i].cutIndex], cutPlane);
                    if (remainder.isValid()) replace(remainder, m_cuts[0].sections[i].cutIndex);
                }
            }
        }

        if (m_cuts[0].sections.empty()) m_cuts.pop_front();

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
    auto fragment = tryFragmentRelease(m_cuts[0].sections[index]);

    uint32_t last = hulls().size() - 1, cutIndex = m_cuts[0].sections[index].cutIndex;

    replaceIndex(last, cutIndex);

    remove(cutIndex);
    m_cuts[0].sections.erase(m_cuts[0].sections.begin() + index);

    return fragment;
}

std::shared_ptr<RigidBody> Debris::tryFragmentRelease(Section& section)
{
    // Confirm section is connected to a fragment (Can be released a bit earlier than the section is finished)
    if (section.srcIndex != std::numeric_limits<uint32_t>::max()) {
        uint32_t index = section.srcIndex;

        if (m_connectionCounts[index] == 0) throw std::runtime_error("[Debris] Unhandled connection");
        m_connectionCounts[index]--;

        section.srcIndex = std::numeric_limits<uint32_t>::max();

        if (m_connectionCounts[index] == 0) { // All connections are removed -> release hull as a fragment

            replaceIndex(hulls().size() - 1, index);

            auto fragment = std::make_shared<RigidBody>(hulls()[index]);
            fragment->setTransform(m_transform);
            fragment->setType(RigidBody::Type::DYNAMIC);
//        fragment->zero();
//        fragment->disableCollisions();

            remove(index);

            return fragment;
        }
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