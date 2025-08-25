//
// Created by cjhat on 2025-08-13.
//

#include "Debris.h"

#include "geometry/collision/Collision.h"



Debris::Debris(const ConvexHull& hull)
    : CompositeBody({ hull })
    , m_inProcess(false)
{

}

Debris::Debris(const std::vector<ConvexHull>& hulls)
    : CompositeBody(hulls)
    , m_inProcess(false)
{

}

// Deferred in case hulls are to be individually inserted
void Debris::initialize()
{

    prepareHulls();
    prepareTree();

    remesh();
}

// System [Local] - xAxis = direction of cut
void Debris::queueCut(const glm::dvec3& origin, const glm::dvec3& normal, const glm::dvec3& axis, double thickness)
{
    m_cuts.emplace_back(origin, normal, axis, thickness);
}

void Debris::beginCut()
{
    if (m_cuts.empty() && !m_inProcess) return;

    CutOperation& cut = m_cuts[0];
    std::cout << "Cut: " << hulls().size() << " " << m_cuts.size() << " " << cut.axis.x << " " << cut.axis.y << " " << cut.axis.z << " | " << cut.normal.x << " " << cut.normal.y << " " << cut.normal.z << " " << "\n";

    double ht = 0.5 * cut.thickness;

    auto fwdPlane = Plane(cut.origin + ht * cut.normal, cut.normal);
    auto revPlane = Plane(cut.origin - ht * cut.normal, -cut.normal);

    std::vector<uint32_t> kerfs;

    // Split any hulls that pass through the cutting planes to limit effect to kerfs
    const uint32_t initialCount = hulls().size();
    for (uint32_t i = 0; i < initialCount; i++) {
        auto [min, max] = hulls()[i].extremes(cut.normal);
        double near = glm::dot(cut.normal, hulls()[i].vertices()[min] - cut.origin);
        double far = glm::dot(cut.normal, hulls()[i].vertices()[max] - cut.origin);
        std::cout << "H" << i << ": " << glm::dot(-cut.normal, hulls()[i].vertices()[min] - cut.origin) << " " << glm::dot(cut.normal, hulls()[i].vertices()[max] - cut.origin) << "\n";

        bool nearPass = near < -ht - 1e-12, farPass = far > ht + 1e-12;

        uint32_t idx = i, count = hulls().size();

        // Try splitting against the near cut plane
        if (nearPass) {
            if (split(revPlane, idx)) idx = hulls().size() - 1;
        }

        // Try splitting against the far cut plane
        if (farPass) {
            if (split(fwdPlane, idx)) idx = hulls().size() - 1;
        }

        // If splits were made (Normal kerf) OR thin-hull case (Entire hull fits inside kerf)
        if (count < hulls().size() || !(nearPass || farPass)) {
            kerfs.emplace_back(idx);
        }
    }

    if (kerfs.empty()) throw std::runtime_error("[Debris] Failed to prepare cut");

    // Prepare sections
    for (uint32_t kerf : kerfs) {
        auto [minIndex, maxIndex] = hulls()[kerf].extremes(cut.axis);

        cut.sections.emplace_back(
                std::numeric_limits<uint32_t>::max(),
                kerf,
                glm::dot(cut.axis, hulls()[kerf].vertices()[minIndex] - cut.origin),
                glm::dot(cut.axis, hulls()[kerf].vertices()[maxIndex] - cut.origin),
                10.0 // hulls()[sources[i - initialCount]].far(cut.axis) - min
                );

        std::cout << "CS " << cut.sections.back().ts << " " << cut.sections.back().tf << " " << cut.sections.back().depth << " " << cut.sections.back().srcIndex << " " << cut.sections.back().cutIndex << "\n";
    }


    m_inProcess = true;
}

void Debris::completeCut()
{
    if (!m_cuts.empty()) m_cuts.pop_front();
    m_inProcess = false;
}

bool Debris::inProcess() const
{
    return m_inProcess;
}

// Shaves material from the next section (depth from the origin). Releases fragments as connections are removed
std::vector<std::shared_ptr<RigidBody>> Debris::removeMaterial(const glm::dvec3& normal, double depth)
{
    std::vector<std::shared_ptr<RigidBody>> fragments;
//    std::cout << "RM " << depth << "\n";

    if (!m_cuts.empty()) {
        Plane cutPlane(m_cuts[0].origin + m_cuts[0].axis * depth, normal); // m_cuts[0].axis

        for (uint32_t i = 0; i < m_cuts[0].sections.size(); i++) {
            if (m_cuts[0].sections[i].ts < depth) {
//                if (m_cuts[0].sections[i].depth <= depth + 1e-12) {
//                    std::shared_ptr<RigidBody> fragment = tryFragmentRelease(m_cuts[0].sections[i]);
//                    if (fragment != nullptr) fragments.push_back(fragment);
//                }

//                std::cout << "S" << i << ": " << m_cuts[0].sections[i].ts << " " << m_cuts[0].sections[i].tf << " " << (m_cuts[0].sections[i].tf <= depth + 1e-12) << "\n";
//                if (m_cuts[0].sections[i].tf <= depth + 1e-12) { // To delete section
//                    removeKerf(i); // Fragment must already have been released if reached
//                } else {
                    auto remainder = Collision::fragment(hulls()[m_cuts[0].sections[i].cutIndex], cutPlane);
                    if (remainder.isValid()) replace(remainder, m_cuts[0].sections[i].cutIndex);
                    else {
                        removeKerf(i);
                        i--;
                    }
//                }
            }
        }

        remesh();

        return fragments;
    }

    return {};
}

//std::vector<std::shared_ptr<RigidBody>> Debris::removeCut()
//{
//    std::vector<std::shared_ptr<RigidBody>> fragments;
//
//    for (uint32_t i = 0; i < m_cuts[0].sections.size(); i++) {
//        std::shared_ptr<RigidBody> fragment = removeKerf(i);
//        if (fragment != nullptr) fragments.push_back(fragment);
//    }
//
//    m_cuts.pop_front();
//    return fragments;
//}

std::shared_ptr<RigidBody> Debris::removeKerf(uint32_t index)
{
    auto fragment = tryFragmentRelease(m_cuts[0].sections[index]);

    uint32_t last = hulls().size() - 1, cutIndex = m_cuts[0].sections[index].cutIndex;

    replaceIndex(last, cutIndex);

    remove(cutIndex);
    m_cuts[0].sections.erase(m_cuts[0].sections.begin() + index);

    return fragment;
}

std::shared_ptr<RigidBody> Debris::tryFragmentRelease(Kerf& section)
{
    // Confirm section is connected to a fragment (Can be released a bit earlier than the section is finished)
    if (section.srcIndex != std::numeric_limits<uint32_t>::max()) {
        uint32_t index = section.srcIndex;

//        if (m_connectionCounts[index] == 0) throw std::runtime_error("[Debris] Unhandled connection");
//        m_connectionCounts[index]--;

//        section.srcIndex = std::numeric_limits<uint32_t>::max();

//        if (m_connectionCounts[index] == 0) { // All connections are removed -> release hull as a fragment
//
//            replaceIndex(hulls().size() - 1, index);
//
//            auto fragment = std::make_shared<RigidBody>(hulls()[index]);
//            fragment->setTransform(m_transform);
//            fragment->setType(RigidBody::Type::DYNAMIC);
////        fragment->zero();
////        fragment->disableCollisions();
//
//            remove(index);
//
//            return fragment;
//        }
    }

    return nullptr;
}

// Update section (cut) indices if it is not the last hull
void Debris::replaceIndex(uint32_t oldIndex, uint32_t newIndex)
{
    if (newIndex < oldIndex) {
        for (CutOperation& cut : m_cuts) {
            for (Kerf& sec : cut.sections) {
                if (sec.cutIndex == oldIndex) sec.cutIndex = newIndex; // CompositeBody swaps index & last hull prior to removal - Adjust index given that fact
            }
        }
    }
}