//
// Created by Cam on 2025-07-06.
//

#include "CompositeBody.h"

#include "geometry/MeshBuilder.h"
#include "geometry/primitives/AABB.h"
#include "geometry/Octree.h"

#include "renderer/Colors.h"

#include <iostream>

CompositeBody::CompositeBody(const std::shared_ptr<Mesh>& mesh)
    : RigidBody(mesh)
    , m_baseColor(0.7f, 0.7f, 0.7f)
    , m_applyCompositeColor(true)
{

}

CompositeBody::CompositeBody(const std::vector<ConvexHull>& hulls)
    : RigidBody(MeshBuilder::composite(hulls))
    , m_hulls(hulls)
    , m_tree(nullptr)
    , m_baseColor(0.7f, 0.7f, 0.7f)
    , m_applyCompositeColor(true)
{
    prepareTree();

    for (ConvexHull& hull : m_hulls) hull.evaluate();
}

void CompositeBody::prepareTree()
{
    auto bounds = AABB(m_hull);

    m_tree = std::make_shared<Octree>(6, 1.01f * bounds.maxLength());
    m_tree->translate(bounds.center());

    m_mergeTests = std::vector<uint32_t>(m_hulls.size());
    std::iota(m_mergeTests.begin(), m_mergeTests.end(), 0);

    locate();
}

void CompositeBody::restore()
{
    m_hulls = { m_hull };

    m_mergeTests = { 0 };

    locate();
}

// Identifies the parents of every hull in relation to the underlying Octree
void CompositeBody::locate()
{
    if (m_tree == nullptr) {
        m_locations = std::vector<uint32_t>(m_hulls.size(), std::numeric_limits<uint32_t>::max());
        return;
    }

    m_locations = std::vector<uint32_t>(m_hulls.size());
    for (uint32_t i = 0; i < m_hulls.size(); i++) {
        m_locations[i] = m_tree->locateParent(m_hulls[i]);
    }
}

void CompositeBody::sort()
{
    m_map.clear();
    for (uint32_t i = 0; i < m_locations.size(); i++) addLink(i);
}

void CompositeBody::addLink(uint32_t childIndex)
{
    auto it = m_map.find(m_locations[childIndex]);
    if (it == m_map.end()) m_map[m_locations[childIndex]] = { childIndex };
    else it->second.emplace_back(childIndex);
}

void CompositeBody::removeLink(uint32_t childIndex)
{
    auto it = m_map.find(m_locations[childIndex]);
    if (it != m_map.end()) {
        auto vit = std::find(it->second.begin(), it->second.end(), childIndex);
        if (vit != it->second.end()) it->second.erase(vit);
    }
}

// Try to merge adjacent convex hulls together to simplify the body without changing the overall geometry
// TODO further development so it also works with loose Octrees
// Returns true if at least two convex hulls were merged
bool CompositeBody::tryMerge()
{
    if (m_mergeTests.empty()) return false; // Skip if further merging is known to be impossible

    // Keep track of hulls that have already been merged
    std::vector<bool> originals(m_locations.size(), true);
    const auto size = originals.size();

    sort();

    std::cout << "Conducting " << m_mergeTests.size() << " merge attempts...\n";

    // Consider every hull yet to be tested for merging
    while (!m_mergeTests.empty()) {
        uint32_t i = m_mergeTests[m_mergeTests.size() - 1];
        m_mergeTests.pop_back();

//        std::cout << "R " << m_mergeTests.size() << "->" << i << " | " << originals.size() << " " << m_locations.size() << " " << m_hulls.size() << "\n";

        if (originals[i]) {
            uint32_t index = m_locations[i];

            // Consider merges within the parent octant (of the hull) and all the octant's parents
            while (index != std::numeric_limits<uint32_t>::max()) {
                if (tryMerge(index, i, originals)) break;
                index = m_tree->parent(index);
            }
        }
    }

    if (size != m_hulls.size()) {

        if (!m_mergeTests.empty()) throw std::runtime_error("[CompositeBody] Impossible check reached");

        // Purge hulls that have been merged from the list
        uint32_t index = 0;
        for (uint32_t i = 0; i < originals.size(); i++) {
            if (originals[i]) {
                m_locations[index] = m_locations[i];
                m_hulls[index] = m_hulls[i];
                index++;
            }
        }

        m_locations.erase(m_locations.begin() + index, m_locations.end());
        m_hulls.erase(m_hulls.begin() + index, m_hulls.end());

        remesh();

        std::cout << "Hull # reduction: " << size << " --> " << m_hulls.size() << "\n";

        return true;
    }

    return false;
}

// Try to merge the specified hull (by index) with another hull in the specified octant (by index)
// Merges with the first appropriate match discovered. Returns true if an appropriate merge was found
bool CompositeBody::tryMerge(uint32_t octantIndex, uint32_t hullIndex, std::vector<bool>& originals)
{
    auto it = m_map.find(octantIndex);
    if (it != m_map.end()) { // Octant has children (hulls) to consider
        for (uint32_t& j : it->second) {
            if (hullIndex == j) continue;

            auto [good, result] = ConvexHull::tryMerge(m_hulls[hullIndex], m_hulls[j]);
            if (good) {

                // Eliminate references to old hulls
                originals[hullIndex] = originals[j] = false;
                originals.emplace_back(true);

                removeLink(hullIndex);
                removeLink(j);
                skipTest(j);

                // Prepare the new hull
                uint32_t newIndex = m_hulls.size();
                add(result);

                addLink(newIndex);

                return true;
            }
        }
    }

    return false;
}

void CompositeBody::add(const ConvexHull& hull)
{
    if (!hull.isValid()) throw std::runtime_error("[CompositeBody] Can not add invalid hull!");
    queueTest(m_hulls.size());

    m_locations.emplace_back(m_tree->locateParent(hull));
//    addLink(m_hulls.size());

    m_hulls.push_back(hull);

    m_hulls[m_hulls.size() - 1].evaluate();
}

void CompositeBody::replace(const ConvexHull& hull, uint32_t index)
{
    if (!hull.isValid()) throw std::runtime_error("[CompositeBody] Can not substitute invalid hull!");

    if (index >= m_hulls.size()) add(hull);
    else {

        queueTest(index);

//        removeLink(index);
        m_locations[index] = m_tree->locateParent(hull);
//        addLink(index);

        m_hulls[index] = hull;
        m_hulls[index].evaluate();

    }
}

void CompositeBody::remove(uint32_t index)
{
    if (index >= m_hulls.size()) return;

    if (index < m_hulls.size() - 1) {
        std::swap(m_hulls[index], m_hulls[m_hulls.size() - 1]);
        std::swap(m_locations[index], m_locations[m_hulls.size() - 1]);
    }

    m_hulls.pop_back();

    replaceTest(m_hulls.size(), index);

//    removeLink(m_hulls.size());
    m_locations.pop_back();
}

void CompositeBody::queueTest(uint32_t hullIndex)
{
    auto it = std::find(m_mergeTests.begin(), m_mergeTests.end(), hullIndex);
    if (it == m_mergeTests.end()) m_mergeTests.emplace_back(hullIndex);
}
void CompositeBody::replaceTest(uint32_t originalIndex, uint32_t newIndex)
{
    auto it = std::find(m_mergeTests.begin(), m_mergeTests.end(), originalIndex);
    if (it != m_mergeTests.end()) { // Preserve queued tests by adjusting to the new index
        auto it2 = std::find(m_mergeTests.begin(), m_mergeTests.end(), newIndex);
        if (it2 != m_mergeTests.end()) { // Prevent double testing
            std::swap(*it2, m_mergeTests[m_mergeTests.size() - 1]);
            m_mergeTests.pop_back();
        }

        *it = newIndex;
    } else skipTest(newIndex); // Block the test for the now replaced hull
}
void CompositeBody::skipTest(uint32_t hullIndex)
{
    auto it = std::find(m_mergeTests.begin(), m_mergeTests.end(), hullIndex);
    if (it != m_mergeTests.end()) {
        std::swap(*it, m_mergeTests[m_mergeTests.size() - 1]);
        m_mergeTests.pop_back();
    }
}

void CompositeBody::applyCompositeColors(bool enable)
{
    if (m_applyCompositeColor != enable && m_mesh != nullptr) {
        if (enable) colorHulls();
        else m_mesh->setFaceColor(m_baseColor);
    }

    m_applyCompositeColor = enable;
}

void CompositeBody::colorHulls()
{
    if (!m_hulls.empty()) {
        uint32_t faceIdx = 0, hullIdx = 0;
        for (const ConvexHull& hull : m_hulls) {
            const glm::dvec3& color = BRIGHT_SET[hullIdx];
            hullIdx = (hullIdx + 1) % BRIGHT_SET.size();

            for (uint32_t i = 0; i < hull.faces().faceCount(); i++) m_mesh->setFaceColor(faceIdx++, color);
        }
    }
}

void CompositeBody::remesh()
{
    m_mesh = MeshBuilder::composite(m_hulls);

    // Prepare styling for the hulls
    m_mesh->setFaceColor(m_baseColor);
    if (m_applyCompositeColor) colorHulls();
}

const std::vector<ConvexHull>& CompositeBody::hulls() const
{
    return m_hulls;
}

const glm::dvec3& CompositeBody::baseColor() const
{
    return m_baseColor;
}

void CompositeBody::printMap() const
{
    std::cout << "====================\nMap (" << m_map.size() << ")\n";
    for (const auto& i : m_map) {
        std::cout  << " [" << i.first << "] ";
        for (uint32_t j : i.second) std::cout << j << " ";
        std::cout << "\n";
    }
}