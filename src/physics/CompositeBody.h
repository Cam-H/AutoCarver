//
// Created by Cam on 2025-07-06.
//

#ifndef AUTOCARVER_COMPOSITEBODY_H
#define AUTOCARVER_COMPOSITEBODY_H

#include "RigidBody.h"

#include <unordered_map>

class Mesh;
class Octree;

class CompositeBody : public RigidBody {
public:

    explicit CompositeBody(const std::shared_ptr<Mesh>& mesh);
    explicit CompositeBody(const std::vector<ConvexHull>& hulls);

    virtual void restore();

    void locate();
    void sort();
    bool tryMerge();

    void add(const ConvexHull& hull);
    void replace(const ConvexHull& hull, uint32_t index);
    void remove(uint32_t index);

    void applyCompositeColors(bool enable);

    virtual void remesh();

    const std::vector<ConvexHull>& hulls() const;

    const glm::dvec3& baseColor() const;

protected:
    void prepareTree();

private:

    void addLink(uint32_t childIndex);
    void removeLink(uint32_t childIndex);

    void queueTest(uint32_t hullIndex);
    void replaceTest(uint32_t originalIndex, uint32_t newIndex);
    void skipTest(uint32_t hullIndex);

    bool tryMerge(uint32_t octantIndex, uint32_t hullIndex, std::vector<bool>& originals);

    void colorHulls();

    void printMap() const;

private:

    std::vector<ConvexHull> m_hulls;

    // Locating hulls relative to one another
    std::shared_ptr<Octree> m_tree;
    std::vector<uint32_t> m_locations;
    std::unordered_map<uint32_t, std::vector<uint32_t>> m_map;
    std::vector<uint32_t> m_mergeTests; // Track hulls that could (maybe) still be merged

    // Styling
    glm::dvec3 m_baseColor;
    bool m_applyCompositeColor;
};


#endif //AUTOCARVER_COMPOSITEBODY_H
