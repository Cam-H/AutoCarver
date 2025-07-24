//
// Created by cjhat on 2025-07-09.
//

#include "Octree.h"

#include <iostream>
#include <sstream>
#include <iomanip>

#include "geometry/primitives/Sphere.h"
#include "geometry/primitives/AABB.h"
#include "geometry/primitives/ConvexHull.h"
#include "geometry/collision/Collision.h"

Octree::Octant::Octant(uint32_t parent, const glm::dvec3& top, uint8_t depth)
    : parent(parent)
    , index(std::numeric_limits<uint32_t>::max())
    , status(Status::TERMINUS)
    , top(top)
    , depth(depth)
{

}

Octree::Octree(uint8_t maximumDepth, double length)
    : m_maxDepth(std::max(maximumDepth, (uint8_t)1))
    , m_lengths({ length })
    , m_offset(0)
{
    calculateLengths();
    reset();
}

void Octree::reset()
{
    if (m_maxDepth < 1) m_maxDepth = 1;
    else if (m_maxDepth > MAX_DEPTH) throw std::runtime_error("[Octree] Requested depth is too large. Try further subdividing with multiple Octrees");

    m_octants = { Octant(std::numeric_limits<uint32_t>::max(), top(), 0) };

    m_octants.reserve(maximumOctantCount(m_maxDepth));
}

void Octree::translate(const glm::dvec3& translation)
{
    for (Octant& octant : m_octants) octant.top += translation;
    m_offset += translation;
}

void Octree::setLength(double length)
{
    m_lengths = { length };
    calculateLengths();
}

void Octree::setMaximumDepth(uint8_t depth)
{
    if (m_maxDepth < depth) {
        m_maxDepth = depth;
        calculateLengths();
    } else {
        m_maxDepth = depth;
        // TODO Prune octree instead
        reset();
    }
}

uint32_t Octree::parent(uint32_t childIndex) const
{
    if (childIndex >= m_octants.size()) throw std::runtime_error("[Octree] Invalid child index");
    return m_octants[childIndex].parent;
}

bool Octree::isParent(uint32_t parentIndex, uint32_t childIndex) const
{
    if (childIndex >= m_octants.size()) throw std::runtime_error("[Octree] Invalid child index");

    while (childIndex != std::numeric_limits<uint32_t>::max()) {
        childIndex = m_octants[childIndex].parent;
        if (childIndex == parentIndex) return true;
    }

    return false;
}

glm::dvec3 Octree::top() const
{
    return -m_lengths[0] * 0.5f * glm::dvec3{ 1, 1, 1 } + m_offset;
}

uint32_t Octree::octantCount(uint8_t status) const
{
    if (status == Octant::Status::DEAD) return 0; //TODO

    uint32_t sum = 0;
    for (const Octant& octant : m_octants) {
        sum += octant.status == status;
    }

    return sum;
}

void Octree::calculateLengths()
{
    m_lengths.resize(m_maxDepth + 1);
    for (uint8_t i = 1; i < m_maxDepth + 1; i++) m_lengths[i] = 0.5f * m_lengths[i - 1];
}

bool Octree::tryExpansion(Octant& parent)
{

    // Verify max depth has not been reached & the parent does not yet have children
    if (parent.terminates() && parent.depth + 1 <= m_maxDepth) {
        parent.status = Octant::Status::LIVE_CHILDREN;
        parent.index = m_octants.size();

        AABB body(parent.top, m_lengths[parent.depth + 1]);
        for (uint8_t i = 0; i < 8; i++)
            m_octants.emplace_back(&parent - m_octants.data(), body.vertex(i), parent.depth + 1);

        return true;
    }

    return false;
}

size_t Octree::maximumOctantCount() const
{
    return maximumOctantCount(m_maxDepth);
}

size_t Octree::maximumOctantCount(uint8_t depth)
{
    size_t count = 1;
    for (uint8_t i = 0; i < depth; i++) count *= 8;
    return count + (depth > 0);
}

double Octree::octantLength(const Octant& octant) const
{
    return m_lengths[octant.depth];
}

//const Octant* Octree::root() const
//{
//    return m_root;
//}

double Octree::length() const
{
    return m_lengths[0];
}

uint8_t Octree::maximumDepth() const
{
    return m_maxDepth;
}

size_t Octree::size() const
{
    return m_octants.size();
}

std::string Octree::memoryFootprint() const
{
    double activeMem = (double)m_octants.size() * sizeof(Octant);
    double totalMem = (double)m_octants.capacity() * sizeof(Octant);

    auto activeUnit = unit(activeMem);
    auto totalUnit = unit(totalMem);

//    std::cout << "ST: " << sizeof(Octant::Status) << "\n";
    return "Octants: " + std::to_string(m_octants.size()) + " / " + std::to_string(m_octants.capacity())
        + " ( " + std::to_string(sizeof(Octant)) + " ) "
        + " [ " + toString(activeMem, 1) + activeUnit + " / " + toString(totalMem, 1) + totalUnit + " ]";
}

std::string Octree::unit(double& value)
{
    if (value > 1e12) {
        value *= 1e-12;
        return "TB";
    } else if (value > 1e9) {
        value *= 1e-9;
        return "GB";
    } else if (value > 1e6) {
        value *= 1e-6;
        return "MB";
    } else if (value > 1e3) {
        value *= 1e-3;
        return "KB";
    }

    return "B";
}

std::string Octree::toString(double value, int precision)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

Octree::Iterator Octree::begin(uint32_t startIndex)
{
    return { m_octants, startIndex };
}

Octree::ConstIterator Octree::begin(uint32_t startIndex) const
{
    return { m_octants, startIndex };
}

Octree::Iterator Octree::end()
{
    return { m_octants, m_octants.data() + m_octants.size() };
}

Octree::ConstIterator Octree::end() const
{
    return { m_octants, m_octants.data() + m_octants.size() };
}
