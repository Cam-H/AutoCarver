//
// Created by cjhat on 2025-07-09.
//

#ifndef AUTOCARVER_OCTREE_H
#define AUTOCARVER_OCTREE_H

#include <vector>
#include <stack>
#include <string>
#include <cstdint>
#include <glm/glm.hpp>

class Sphere;
class ConvexHull;

//class Octant {
//public:
//
//    Octant();
//    ~Octant();
//
//    void clear();
//
//    uint32_t octantCount() const;
//    bool terminates() const;
//
//public:
//    std::array<Octant*, 8> children;
//    uint8_t status; // 0 - No further divisions
//};

class Octree {
public:

    class Octant {
    public:

        typedef enum Status {
            DEAD = 0,          // The octant and all its children are 'off'
            LIVE_CHILDREN = 1, // The octant has active children
            TERMINUS = 2       // The octant has not children
        } Status;

        Octant(const glm::vec3& top, uint8_t depth);

        uint32_t index;
        uint8_t status;
        glm::vec3 top;
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
        OctreeIterator(const OctreeIterator<OtherIsConst>& other) : m_ptr(other.m_ptr) {}

        OctreeIterator(list octants) : octants(octants), m_ptr(octants.data()) { m_stack.push(0); }
        OctreeIterator(list octants, pointer ptr) : octants(octants), m_ptr(ptr) {};

//        OctreeIterator(std::vector<Octant>& octants, uint32_t startIdx);

        void skip();

        OctreeIterator& operator++();
//        Iterator operator++(int);

        reference operator*() const { return *m_ptr; }
        pointer operator->() { return m_ptr; }

        friend bool operator== (const OctreeIterator& a, const OctreeIterator& b) { return a.m_ptr == b.m_ptr; };
        friend bool operator!= (const OctreeIterator& a, const OctreeIterator& b) { return a.m_ptr != b.m_ptr; };

//        static uint32_t end(const std::vector<Octant>& octants);

    private:
        list octants;
        pointer m_ptr;

        std::stack<uint32_t> m_stack;
    };

    using Iterator = OctreeIterator<false>;
    using ConstIterator = OctreeIterator<true>;

    Octree(float length = 1.0f);

    void reset();

    // Apply boolean operation to the octree
    bool unite(const Sphere& sphere);
    bool subtract(const Sphere& sphere);
    bool intersect(const Sphere& sphere);

    void setLength(float length);
    void setMaximumDepth(uint32_t depth);

    bool collides(const Sphere& sphere) const;

    glm::vec3 top() const;

    uint32_t octantCount(uint8_t status = 1) const;
    uint32_t maximumOctantCount() const;
    static uint32_t maximumOctantCount(uint32_t depth);

    static glm::vec3 octantOffset(uint32_t index, float halfLength);
    static glm::vec3 octantOffset(uint32_t index);

    float octantLength(const Octant& octant) const;

//    const Octant* root() const;
    float length() const;
    uint32_t maximumDepth() const;

    size_t size() const;
    std::string memoryFootprint() const;

    Iterator begin();
    ConstIterator begin() const;

    Iterator end();
    ConstIterator end() const;

private:

    static inline bool collides(const Sphere& sphere, const glm::vec3& top, float length);
    static inline glm::vec3 nearest(const glm::vec3& point, const glm::vec3& top, float length);

    static inline bool encloses(const Sphere& sphere, const glm::vec3& top, float length);
    static inline glm::vec3 farthest(const glm::vec3& point, const glm::vec3& top, float length);

    void calculateLengths();

    static std::string unit(double& value);
    static std::string toString(double value, int precision);

private:

    std::vector<Octant> m_octants;

    std::vector<float> m_lengths;

    uint32_t m_maxDepth;
};


#endif //AUTOCARVER_OCTREE_H
