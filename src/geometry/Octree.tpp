#include "Octree.h"

#include "Collision.h"
#include "primitives/AABB.h"

template <bool IsConst>
void Octree::OctreeIterator<IsConst>::skip()
{
    if (m_stack.size() > 1) { // Move to the next branch if available
        m_stack.pop();
        m_ptr = &octants[m_stack.top()];

        if (m_ptr->depth <= m_limit) { // Prevent ascending beyond the specified limit (Allows subsection selection)
            m_ptr = octants.data() + octants.size();
            m_stack = {};
        }

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
            if (octants[m_ptr->index + i].status != Octant::Status::DEAD)
                m_stack.push(m_ptr->index + i);

        m_ptr = &octants[m_stack.top()];
    } else skip();

    return *this;
}

//Octree::OctreeIterator<IsConst>& Octree::OctreeIterator<IsConst>::operator++(int)
//{
//    Iterator tmp = *this;
//    ++(*this);
//    return tmp;
//}

template<class T>
bool Octree::unite(const T& body)
{
//    auto it = begin();
//    it.
    return false;
}

template<class T>
bool Octree::subtract(const T& body)
{
    auto it = begin();

    bool update = false;

    while (it != end()) {
        AABB octant(it->top, m_lengths[it->depth]);
        if (Collision::test(body, octant)) {

            if (Collision::encloses(body, octant)) {
                it->status = Octant::Status::DEAD;
                update = true;
                it.skip();
            } else {
                if (it->terminates()) {
                    if (!tryExpansion(*it)) it->status = Octant::Status::DEAD;
                    update = true;
                }

                ++it; // Move to the next octant (Could be child, sibling or ?ancestor)
            }

        } else it.skip(); // Ignore child octants when there is no collision with the parent
    }

    return update;
}

template<class T>
bool Octree::collides(const T& body) const
{
    auto it = begin();
    while (it != end()) {
        if (it->status != Octant::Status::DEAD && Collision::test(body, AABB(it->top, m_lengths[it->depth]))) {
            if (it->status == Octant::Status::TERMINUS) return true;
            ++it;
        } else it.skip();
    }

    return false;
}

template<class T>
uint32_t Octree::locateParent(const T& body)
{
    uint32_t index = std::numeric_limits<uint32_t>::max();

    auto it = begin();
    while (it != end()) {
        if (Collision::encloses(AABB(it->top, m_lengths[it->depth]), body)) {
            it.limit(it->depth); // Block consideration outside current branch (Other branches can NOT enclose)

            index = it.pos();

            // Add children if absent
            tryExpansion(*it);
            ++it;
        } else it.skip();
    }

    return index;
}