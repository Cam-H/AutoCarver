//
// Created by cjhat on 2025-07-09.
//

#ifndef AUTOCARVER_OCTREE_H
#define AUTOCARVER_OCTREE_H

#include <array>
#include <cstdint>
#include <glm/glm.hpp>

class Sphere;
class ConvexHull;

class Octant {
public:

    Octant();
    ~Octant();

    void clear();

    uint32_t octantCount() const;
    bool terminates() const;

public:
    std::array<Octant*, 8> children;
    uint8_t status; // 0 - No further divisions
};

class Octree {
public:

    struct Iterator {
        using iterator_category = std::forward_iterator_tag;
        using difference_type   = std::ptrdiff_t;
        using value_type        = int;
        using pointer           = int*;
        using reference         = int&;

        Iterator(pointer ptr) : m_ptr(ptr) {}

        reference operator*() const { return *m_ptr; }
        pointer operator->() { return m_ptr; }
        Iterator& operator++() { m_ptr++; return *this; }
        Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }
        friend bool operator== (const Iterator& a, const Iterator& b) { return a.m_ptr == b.m_ptr; };
        friend bool operator!= (const Iterator& a, const Iterator& b) { return a.m_ptr != b.m_ptr; };

    private:
        pointer m_ptr;
    };

    Iterator begin() { return Iterator(&m_data[0]); }
    Iterator end()   { return Iterator(&m_data[200]); }

    Octree(float length = 1.0f);

    void reset();

    void applyUnion(const Sphere& sphere);
    void applyDifference(const Sphere& sphere);
    void applyIntersection(const Sphere& sphere);

    void setLength(float length);
    void setMaximumDivisions(uint32_t num);

    bool collides(const Sphere& sphere) const;

    glm::vec3 top() const;

    uint32_t octantCount() const;
    uint32_t maximumOctantCount() const;

    static glm::vec3 octantOffset(uint32_t index, float halfLength);
    static glm::vec3 octantOffset(uint32_t index);

    const Octant* root() const;
    float length() const;
    uint32_t maximumDivisions() const;


private:

    struct Item {
        Octant* octant;
        const glm::vec3 offset;
        const float length;
        const uint32_t depth;
    };

    static inline bool collides(const Sphere& sphere, const glm::vec3& top, float length);
    static inline glm::vec3 nearest(const glm::vec3& point, const glm::vec3& top, float length);

    static inline bool encloses(const Sphere& sphere, const glm::vec3& top, float length);
    static inline glm::vec3 farthest(const glm::vec3& point, const glm::vec3& top, float length);

private:

    Octant* m_root;

    float m_length;
    uint32_t m_maximumDivisions;
    int m_data[200];
};


#endif //AUTOCARVER_OCTREE_H
