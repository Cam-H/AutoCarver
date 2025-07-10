//
// Created by cjhat on 2025-07-09.
//

#include "Octree.h"

#include <iostream>

#include "Sphere.h"
#include "ConvexHull.h"

Octant::Octant()
    : children({ nullptr })
    , status(0)
{

}

Octant::~Octant()
{
    std::cout << "Erasure\n";
    clear();
}

void Octant::clear()
{
    for (Octant* child : children) delete child;
}

uint32_t Octant::octantCount() const
{
    uint32_t sum = status == 0;
    for (Octant* child : children) {
        if (child != nullptr) sum += child->octantCount();
    }

    return sum;
}

bool Octant::terminates() const
{
    for (Octant* child : children) if (child != nullptr) return false;
    return true;
}

Octree::Octree(float length)
    : m_root(new Octant)
    , m_length(length)
    , m_maximumDivisions(6)
{

}

void Octree::reset()
{
    delete m_root;

    m_root = new Octant;
}

void Octree::applyUnion(const Sphere& sphere)
{

}
void Octree::applyDifference(const Sphere& sphere)
{
    std::vector<Item> items = { { m_root, top(), m_length, 0 } };

    while (!items.empty()) {
        Item item = items[items.size() - 1];
        items.pop_back();

        if (item.octant->status != 2 && collides(sphere, item.offset, item.length)) {
            item.octant->status = 1;

            if (encloses(sphere, item.offset, item.length)) {
//                item.octant->clear();
                item.octant->status = 2;
            } else if (item.depth < m_maximumDivisions){

                float length = 0.5f * item.length;
                for (uint8_t i = 0; i < 8; i++) {
                    glm::vec3 offset = octantOffset(i, length) + item.offset;
                    if (item.octant->children[i] == nullptr) item.octant->children[i] = new Octant();
                    items.push_back({ item.octant->children[i], offset, length, item.depth + 1 });
                }
            }
        }
    }
}
void Octree::applyIntersection(const Sphere& sphere)
{

}

void Octree::setLength(float length)
{
    m_length = length;
}

void Octree::setMaximumDivisions(uint32_t num)
{
    m_maximumDivisions = num;
}

bool Octree::collides(const Sphere& sphere) const
{
    if (!collides(sphere, top(), m_length)) return false;

//    if (collides(sphere, octant, top)) ;

    return true;
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
    return -m_length * 0.5f * glm::vec3(1.0f);
}

uint32_t Octree::octantCount() const
{
    return m_root->octantCount();
}

uint32_t Octree::maximumOctantCount() const
{
    uint32_t count = 1;
    for (uint32_t i = 0; i < m_maximumDivisions; i++) count *= 8;
    return count;
}

glm::vec3 Octree::octantOffset(uint32_t index, float halfLength)
{
    return halfLength * octantOffset(index);
}
glm::vec3 Octree::octantOffset(uint32_t index)
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

const Octant* Octree::root() const
{
    return m_root;
}

float Octree::length() const
{
    return m_length;
}

uint32_t Octree::maximumDivisions() const
{
    return m_maximumDivisions;
}