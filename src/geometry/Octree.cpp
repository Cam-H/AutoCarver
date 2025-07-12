//
// Created by cjhat on 2025-07-09.
//

#include "Octree.h"

#include <iostream>
#include <sstream>
#include <iomanip>

#include "Sphere.h"
#include "ConvexHull.h"

Octree::Octant::Octant(const glm::vec3& top, uint8_t depth)
    : index(std::numeric_limits<uint32_t>::max())
    , status(Status::TERMINUS)
    , top(top)
    , depth(depth)
{

}

template <bool IsConst>
void Octree::OctreeIterator<IsConst>::skip()
{
    if (m_stack.size() > 1) { // Move to the next branch if available
        m_stack.pop();
        m_ptr = &octants[m_stack.top()];
    } else { // Otherwise terminate iterator
        m_ptr = octants.data() + octants.size();
        m_stack = {};
    }
}

template <bool IsConst>
Octree::OctreeIterator<IsConst>& Octree::OctreeIterator<IsConst>::operator++()
{
    if (m_ptr->index != std::numeric_limits<uint32_t>::max()) { //  Octant has children
        m_stack.pop();
        for (uint8_t i = 0; i < 8; i++)
            if (octants[m_ptr->index + i].status != Octant::Status::DEAD) m_stack.push(m_ptr->index + i);
        m_ptr = &octants[m_stack.top()];
    } else skip();


//    std::cout << m_stack.size() << " " << (!m_stack.empty() ? std::to_string(m_stack.top()) : "--") << " " << m_ptr << "<\n";

    return *this;
}

//Octree::Iterator Octree::Iterator::operator++(int)
//{
//    Iterator tmp = *this;
//    ++(*this);
//    return tmp;
//}

//Octant::~Octant()
//{
//    std::cout << "Erasure\n";
//    clear();
//}

//void Octant::clear()
//{
//    for (Octant* child : children) delete child;
//}

//uint32_t Octant::octantCount() const
//{
//    uint32_t sum = status == 0;
//    for (Octant* child : children) {
//        if (child != nullptr) sum += child->octantCount();
//    }
//
//    return sum;
//}
//
//bool Octant::terminates() const
//{
//    for (Octant* child : children) if (child != nullptr) return false;
//    return true;
//}

Octree::Octree(float length)
    : m_lengths({ length })
    , m_maxDepth(6)
{
    calculateLengths();
    reset();
}

void Octree::reset()
{
    if (m_maxDepth > 10) throw std::runtime_error("[Octree] Requested depth is too large. Try further subdividing with multiple Octrees");

    m_octants = { Octant(top(), 0) };

    // Reserve memory to prepare for development of the Octree
    // Reserve less than maximum to reduce memory usage (Full development is rare)
    m_octants.reserve(maximumOctantCount(m_maxDepth - 1));
}

bool Octree::unite(const Sphere& sphere)
{
//    m_last = Iterator::end(m_octants);
    return false;
}
bool Octree::subtract(const Sphere& sphere)
{
    auto it = begin();

    bool update = false;

    while (it != end()) {
        if (collides(sphere, it->top, m_lengths[it->depth])) {
            if (encloses(sphere, it->top, m_lengths[it->depth])) { // Sphere entirely encloses the octant
                it->status = Octant::Status::DEAD;
                update = true;
                it.skip();
            } else {
                update = update || it->status != Octant::Status::LIVE_CHILDREN;
                it->status = Octant::Status::LIVE_CHILDREN;

                if (it->depth + 1 < m_maxDepth && it->index == std::numeric_limits<uint32_t>::max()) { // Introducing children
                    it->index = m_octants.size();
                    for (uint8_t i = 0; i < 8; i++)  m_octants.emplace_back(
                                it->top + octantOffset(i, m_lengths[it->depth + 1]),
                                it->depth + 1
                        );
                }

                ++it; // Move to the next octant (Could be child, sibling or ?ancestor)
            }

        } else it.skip(); // Ignore child octants when there is no collision with the parent
    }

    return update;
}
bool Octree::intersect(const Sphere& sphere)
{
    return false;
}

void Octree::setLength(float length)
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

bool Octree::collides(const Sphere& sphere) const
{
    auto it = begin();
    while (it != end()) {
        if (it->status != Octant::Status::DEAD && collides(sphere, it->top, m_lengths[it->depth])) {
            if (it->status == Octant::Status::TERMINUS) return true;
            ++it;
        } else it.skip();
    }

    return false;
}

bool Octree::collides(const Sphere& sphere, const glm::vec3& top, float length)
{
    glm::vec3 delta = sphere.center - nearest(sphere.center, top, length);
    return glm::dot(delta, delta) < sphere.radius * sphere.radius;
}

glm::vec3 Octree::nearest(const glm::vec3& point, const glm::vec3& top, float length)
{
    return {
        std::clamp(point.x, top.x, top.x + length),
        std::clamp(point.y, top.y, top.y + length),
        std::clamp(point.z, top.z, top.z + length)
    };
}

bool Octree::encloses(const Sphere& sphere, const glm::vec3& top, float length)
{
    glm::vec3 delta = sphere.center - farthest(sphere.center, top, length);
    return glm::dot(delta, delta) < sphere.radius * sphere.radius;
}
glm::vec3 Octree::farthest(const glm::vec3& point, const glm::vec3& top, float length)
{
    float halfLength = 0.5f * length;
    return {
        top.x + (top.x + halfLength > point.x ? length : 0),
        top.y + (top.y + halfLength > point.y ? length : 0),
        top.z + (top.z + halfLength > point.z ? length : 0)
    };
}

glm::vec3 Octree::top() const
{
    return -m_lengths[0] * 0.5f * glm::vec3{ 1, 1, 1 };
}

size_t Octree::octantCount(uint8_t status) const
{
    if (status == Octant::Status::DEAD) return 0; //TODO

    size_t sum = 0;
    for (const Octant& octant : *this) {
        sum += octant.status == status;
    }

//    uint32_t sum = 0;
//        sum += octant.status == status;
//    }
    return sum;
}

void Octree::calculateLengths()
{
    m_lengths.resize(m_maxDepth);
    for (uint8_t i = 1; i < m_maxDepth; i++) m_lengths[i] = 0.5f * m_lengths[i - 1];
}

size_t Octree::maximumOctantCount() const
{
    return maximumOctantCount(m_maxDepth);
}

size_t Octree::maximumOctantCount(uint8_t depth)
{
    size_t count = 1;
    for (uint8_t i = 0; i < depth; i++) count *= 8;
    return count;
}

glm::vec3 Octree::octantOffset(uint8_t index, float halfLength)
{
    return halfLength * octantOffset(index);
}
glm::vec3 Octree::octantOffset(uint8_t index)
{
    switch (index) {
        case 0: return {};
        case 1: return { 1, 0, 0 };
        case 2: return { 1, 0, 1 };
        case 3: return { 0, 0, 1 };
        case 4: return { 0, 1, 0 };
        case 5: return { 1, 1, 0 };
        case 6: return { 1, 1, 1 };
        case 7: return { 0, 1, 1 };
        default: throw std::runtime_error("[Octree] Invalid array access!");
    }
}

float Octree::octantLength(const Octant& octant) const
{
    return m_lengths[octant.depth];
}

//const Octant* Octree::root() const
//{
//    return m_root;
//}

float Octree::length() const
{
    return m_lengths[0];
}

uint32_t Octree::maximumDepth() const
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

Octree::Iterator Octree::begin()
{
    return { m_octants };
}

Octree::ConstIterator Octree::begin() const
{
    return { m_octants };
}

Octree::Iterator Octree::end()
{
    return { m_octants, m_octants.data() + m_octants.size() };
}

Octree::ConstIterator Octree::end() const
{
    return { m_octants, m_octants.data() + m_octants.size() };
}