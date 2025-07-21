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

    const glm::vec3& baseColor() const;

protected:
    void prepareTree();

private:

    void addChild(uint32_t childIndex);

    bool tryMerge(uint32_t octantIndex, uint32_t hullIndex, std::vector<bool>& originals);

    void colorHulls();

    void printMap() const;

private:

    std::vector<ConvexHull> m_hulls;

    // Locating hulls relative to one another
    std::shared_ptr<Octree> m_tree;
    std::vector<uint32_t> m_locations;
    std::unordered_map<uint32_t, std::vector<uint32_t>> m_map;

    // Styling
    glm::vec3 m_baseColor;
    bool m_applyCompositeColor;
};


#endif //AUTOCARVER_COMPOSITEBODY_H
