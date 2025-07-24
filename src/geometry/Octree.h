//
// Created by cjhat on 2025-07-09.
//

#ifndef AUTOCARVER_OCTREE_H
#define AUTOCARVER_OCTREE_H

#include <vector>
#include <stack>
#include <string>
#include <cstdint>
#include <glm.hpp>

class Sphere;
class AABB;
class ConvexHull;

class Octree {
public:

    class Octant {
    public:

        typedef enum Status {
            DEAD = 0,          // The octant and all its children are 'off'
            LIVE_CHILDREN = 1, // The octant has active children
            TERMINUS = 2       // The octant has no children
        } Status;

        Octant(uint32_t parent, const glm::dvec3& top, uint8_t depth);

        [[nodiscard]] inline bool terminates() const { return index == std::numeric_limits<uint32_t>::max(); }

        uint32_t parent;
        uint32_t index;
        uint8_t status;
        glm::dvec3 top;
        uint8_t depth;
    };

    template<bool IsConst>
    class OctreeIterator {
        using iterator_category = std::forward_iterator_tag;
        using difference_type   = std::ptrdiff_t;
        using value_type        = Octant;
        using reference = std::conditional_t<IsConst, const Octant&, Octant&>;
        using pointer = std::conditional_t<IsConst, const Octant*, Octant*>;
        using list = std::conditional_t<IsConst, const std::vector<Octant>&, std::vector<Octant>&>;

    public:
        template<bool OtherIsConst,
                typename = std::enable_if_t<IsConst && !OtherIsConst>>
        OctreeIterator(const OctreeIterator<OtherIsConst>& other)
            : octants(other.octants)
            , m_ptr(other.m_ptr)
            , m_limit(other.m_limit)
            , m_ignoreDeadStatus(other.m_ignoreDeadStatus) {}

        OctreeIterator(list octants, uint32_t rootIndex)
            : octants(octants)
            , m_ptr(octants.data() + rootIndex)
            , m_limit(m_ptr->depth)
            , m_ignoreDeadStatus(false) { m_stack.push(rootIndex); }
        OctreeIterator(list octants, pointer ptr)
            : octants(octants)
            , m_ptr(ptr)
            , m_limit(0)
            , m_ignoreDeadStatus(false) {}

        void skip();
        void limit(uint8_t ceiling) { m_limit = ceiling; }

        [[nodiscard]] uint32_t pos() const { return m_ptr - octants.data(); }

        OctreeIterator& operator++();
//        Iterator operator++(int);

        reference operator*() const { return *m_ptr; }
        pointer operator->() { return m_ptr; }

        friend bool operator== (const OctreeIterator& a, const OctreeIterator& b) { return a.m_ptr == b.m_ptr; };
        friend bool operator!= (const OctreeIterator& a, const OctreeIterator& b) { return a.m_ptr != b.m_ptr; };

    private:
        list octants;
        pointer m_ptr;
        uint8_t m_limit;
        bool m_ignoreDeadStatus;

        std::stack<uint32_t> m_stack;
    };

    using Iterator = OctreeIterator<false>;
    using ConstIterator = OctreeIterator<true>;

    Octree(uint8_t maximumDepth = 6, double length = 1.0f);

    void reset();

    void translate(const glm::dvec3& translation);

    // Apply boolean operation to the octree
    template<class T>
    bool unite(const T& body);

    template<class T>
    bool subtract(const T& body);

//    bool intersect(const Sphere& sphere);

    void setLength(double length);
    void setMaximumDepth(uint8_t depth);

    template<class T>
    bool collides(const T& body) const;

    // Returns the index of the smallest octant that fully contains the provided body
    // Will expand the Octree when appropriate so long as maximum depth is not reached
    template<class T>
    uint32_t locateParent(const T& body);

    uint32_t parent(uint32_t childIndex) const;
    bool isParent(uint32_t parentIndex, uint32_t childIndex) const;

    glm::dvec3 top() const;

    uint32_t octantCount(uint8_t status = 1) const;
    size_t maximumOctantCount() const;
    static size_t maximumOctantCount(uint8_t depth);

    double octantLength(const Octant& octant) const;

//    const Octant* root() const;
    double length() const;
    uint8_t maximumDepth() const;

    size_t size() const;
    std::string memoryFootprint() const;

    Iterator begin(uint32_t startIndex = 0);
    ConstIterator begin(uint32_t startIndex = 0) const;

    Iterator end();
    ConstIterator end() const;

private:

    void calculateLengths();

    bool tryExpansion(Octant& parent);

    static std::string unit(double& value);
    static std::string toString(double value, int precision);

public:

    static const uint8_t MAX_DEPTH = 10; // Max tree size that can be handled with uint32_t indexing

private:

    std::vector<Octant> m_octants;

    uint8_t m_maxDepth;
    std::vector<double> m_lengths;

    glm::dvec3 m_offset;

};

#include "Octree.tpp"

#endif //AUTOCARVER_OCTREE_H
