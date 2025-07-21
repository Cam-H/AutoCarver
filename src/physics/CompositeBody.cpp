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
}

void CompositeBody::prepareTree()
{
    auto bounds = AABB(m_hull);

    m_tree = std::make_shared<Octree>(6, 1.01f * bounds.maxLength());
    m_tree->translate(bounds.center());

    locate();
}

void CompositeBody::restore()
{
    m_hulls = { m_hull };

    locate();
}

// Identifies the parents of every hull in relation to the underlying Octree
void CompositeBody::locate()
{
    m_locations = std::vector<uint32_t>(m_hulls.size());
    for (uint32_t i = 0; i < m_hulls.size(); i++) {
        m_locations[i] = m_tree->locateParent(m_hulls[i]);
    }
}

void CompositeBody::sort()
{
    std::cout << "Clear\n";
    m_map.clear();

    for (uint32_t i = 0; i < m_locations.size(); i++) addChild(i);
}

void CompositeBody::addChild(uint32_t childIndex)
{
    auto it = m_map.find(m_locations[childIndex]);
    if (it == m_map.end()) m_map[m_locations[childIndex]] = { childIndex };
    else  it->second.emplace_back(childIndex);
}

// Try to merge adjacent convex hulls together to simplify the body without changing the overall geometry
// TODO further development so it also works with loose Octrees
// TODO Add ability to try merging only subsets (For cases where only a few hulls are known to have changed)
// Returns true if at least two convex hulls were merged
bool CompositeBody::tryMerge()
{

    // Keep track of hulls that have already been merged
    std::vector<bool> originals(m_locations.size(), true);
    auto size = originals.size();

    locate();
    sort();

//    std::cout << m_map.size() << " | " << m_locations.size() << " " << m_hulls.size() << " Locations: [";
//    for (uint32_t i : m_locations) std::cout << i << " ";
//    std::cout << "]\n";
//    printMap();
//
//    for (const auto& hull : m_hulls) {
//        std::cout << "HS: " << hull.vertexCount() << " " << hull.isValid() << "\n";
//    }
//
//    for (uint32_t& i : m_locations) {
//        if (i == std::numeric_limits<uint32_t>::max()) {
//            std::cout << &i << " " << m_locations.data() << " " << (&i - m_locations.data()) << " " << (&i - &m_locations[0]) << "\n";
//            std::cout << "Res: " << m_tree->locateParent(m_hulls[&i - m_locations.data()]) << "\n";
//        }
//    }

    // Consider every hull, even newly merged hulls (For additional merges)
    for (uint32_t i = 0; i < m_hulls.size(); i++) {
        if (originals[i]) {
            uint32_t index = m_locations[i];

            // Consider merges within the parent octant (of the hull) and all the octant's parents
            while (index != std::numeric_limits<uint32_t>::max()) {
                if (tryMerge(index, i, originals)) {
//                    printMap();
                    break;
                }
                index = m_tree->parent(index);
            }
        }
    }

    if (size != m_hulls.size()) {

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

//            std::cout << "Check: " << hullIndex << "-" << j << " " << m_hulls.size() << "\n";
            auto [good, result] = ConvexHull::tryMerge(m_hulls[hullIndex], m_hulls[j]);
            if (good) {
//                std::cout << hullIndex << " " << j << " Merging\n";
                originals[hullIndex] = originals[j] = false;
                originals.emplace_back(true);

                uint32_t newIndex = m_hulls.size();
                add(result);

                // Replace old indexing with the newly merged hull
                if (m_locations[newIndex] == m_locations[j]) j = newIndex;
                else { // Handle case where the new location is different (For loose Octrees)
                    std::cout << "Erasing " << (&j - it->second.data()) << "\n";
                    it->second.erase(it->second.begin() + (&j - it->second.data()));
                    addChild(newIndex);
                }
                auto& children = m_map[m_locations[hullIndex]];
                children.erase(std::find(children.begin(), children.end(), hullIndex));


                return true;
            }
        }
    }

//    std::cout << "Failed to merge " << hullIndex << " with Octant " << octantIndex << "'s children\n";

    return false;
}

void CompositeBody::add(const ConvexHull& hull)
{
    if (!hull.isValid()) throw std::runtime_error("[CompositeBody] Can not add invalid hull!");
    m_locations.emplace_back(m_tree->locateParent(hull));
    m_hulls.push_back(hull);
}

void CompositeBody::replace(const ConvexHull& hull, uint32_t index)
{
    if (!hull.isValid()) throw std::runtime_error("[CompositeBody] Can not substitute invalid hull!");

    if (index >= m_hulls.size()) add(hull);
    else {
        m_locations[index] = m_tree->locateParent(hull);
        m_hulls[index] = hull;
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
    m_locations.pop_back();
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
            const glm::vec3& color = BRIGHT_SET[hullIdx];
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

const glm::vec3& CompositeBody::baseColor() const
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