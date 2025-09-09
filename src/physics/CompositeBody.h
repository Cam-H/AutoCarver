//
// Created by Cam on 2025-07-06.
//

#ifndef AUTOCARVER_COMPOSITEBODY_H
#define AUTOCARVER_COMPOSITEBODY_H

#include "RigidBody.h"

#include <unordered_map>

#include "geometry/primitives/Plane.h"

class Mesh;
class Octree;

// Extension to RigidBody to handle spatial partitioning of components
class CompositeBody : public RigidBody {
public:

    explicit CompositeBody(const std::shared_ptr<Mesh>& mesh);
    explicit CompositeBody(const std::vector<ConvexHull>& hulls);

    bool serialize(std::ofstream& file) const override;
    bool deserialize(std::ifstream& file) override;

    virtual void restore();

    void recenter(const glm::dvec3& offset) override;

    void locate();
    void sort();
    bool tryMerge();

    void add(const ConvexHull& hull);
    void replace(const ConvexHull& hull, uint32_t index);
    void remove(uint32_t index);

    std::vector<uint32_t> split(const Plane& plane);

    void applyCompositeColors(bool enable);

    virtual void remesh();

    const glm::dvec3& baseColor() const;

    [[nodiscard]] const std::vector<ConvexHull>& components() const;
    [[nodiscard]] ConvexHull container() const;

//    bool test(const std::shared_ptr<RigidBody>& body) override;

protected:

    CompositeBody();

    void prepareComponents();
    void prepareTree();

    bool split(const Plane& plane, uint32_t componentIndex);

    bool precheck(uint32_t hullID0, uint32_t hullID1, const glm::dmat4& relative) const override;

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

    std::vector<ConvexHull> m_components;

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
