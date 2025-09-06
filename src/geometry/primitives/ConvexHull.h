//
// Created by cameronh on 24/08/23.
//

#ifndef AUTOCARVER_CONVEXHULL_H
#define AUTOCARVER_CONVEXHULL_H

#include <string>
#include <memory>
#include <vector>

#include "geometry/VertexArray.h"
#include "geometry/FaceArray.h"
#include "geometry/primitives/Triangle.h"
#include "geometry/primitives/Sphere.h"

#include "fileIO/Serializable.h"

class Mesh;
class Plane;

class HullBuilder;

class ConvexHull : public Serializable {
public:

    ConvexHull();

    explicit ConvexHull(std::ifstream& file);

    explicit ConvexHull(const VertexArray& cloud);
    explicit ConvexHull(const std::shared_ptr<Mesh>& mesh);

    explicit ConvexHull(const std::vector<glm::dvec3>& cloud);

    ConvexHull(const ConvexHull& rhs) = default;

    bool serialize(std::ofstream& file) const override;
    bool deserialize(std::ifstream& file) override;

    void evaluate();

    void setWalkStart(uint32_t startIndex);
    [[nodiscard]] bool getWalkStart() const;

    [[nodiscard]] bool isValid() const;

    [[nodiscard]] uint32_t vertexCount() const;
    [[nodiscard]] const std::vector<glm::dvec3>& vertices() const;

    [[nodiscard]] uint32_t facetCount() const;
    [[nodiscard]] const FaceArray& faces() const;

    [[nodiscard]] glm::dvec3 center() const;
    [[nodiscard]] Plane facePlane(uint32_t idx) const;

    [[nodiscard]] const glm::dvec3& start() const;

    [[nodiscard]] uint32_t supportIndex(const glm::dvec3& axis) const;
    [[nodiscard]] std::tuple<uint32_t, glm::dvec3> extreme(const glm::dvec3& axis) const;

    [[nodiscard]] uint32_t supportIndex(const glm::dvec3& axis, uint32_t startIndex) const;
    [[nodiscard]] std::tuple<uint32_t, glm::dvec3> extreme(const glm::dvec3& axis, uint32_t startIndex) const;

    [[nodiscard]] bool empty() const;

    [[nodiscard]] uint32_t walk(const glm::dvec3& axis) const;
    [[nodiscard]] uint32_t walk(const glm::dvec3& axis, uint32_t index) const;

    std::tuple<uint32_t, uint32_t> extremes(const glm::dvec3& axis) const;
    void extents(const glm::dvec3& axis, double& near, double& far) const;

    [[nodiscard]] double span(const glm::dvec3& axis) const;
    [[nodiscard]] double far(const glm::dvec3& axis) const;

    [[nodiscard]] const std::vector<uint32_t>& neighbors(uint32_t index) const;
    [[nodiscard]] std::vector<glm::dvec3> border(const glm::dvec3& faceNormal) const;

    [[nodiscard]] std::vector<uint32_t> horizon(const glm::dvec3& axis) const;
    [[nodiscard]] std::vector<uint32_t> horizon(const glm::dvec3& axis, const glm::dvec3& support) const;

    [[nodiscard]] double volume() const;

    static ConvexHull unite(const ConvexHull& hullA, const ConvexHull& hullB);
    static std::tuple<bool, ConvexHull> tryMerge(const ConvexHull& hullA, const ConvexHull& hullB);

    void print() const;

    friend class HullBuilder;

private:

    void build(HullBuilder& builder);
    void initialize();

    [[nodiscard]] uint32_t step(const glm::dvec3& normal, const glm::dvec3& axis, uint32_t index) const;

private:

    bool m_initialized;

    std::vector<glm::dvec3> m_vertices;

    FaceArray m_faces;

    glm::dvec3 m_center;
    std::vector<std::vector<uint32_t>> m_walks;
    uint32_t m_walkStart;
    bool m_orphans;

    double m_volume;

    const static uint8_t STRIDE = 3;

};

class HullBuilder {
public:

    explicit HullBuilder(const std::vector<glm::dvec3>& cloud);

    void clean();

    void initialize();

    void step();
    void solve();

    [[nodiscard]] uint32_t getIteration() const;
    [[nodiscard]] bool finished() const;

    [[nodiscard]] FaceArray getFaces() const;
    [[nodiscard]] std::vector<glm::dvec3> getVertices() const;

    [[nodiscard]] ConvexHull getHull() const;


private:

    struct Facet {
        TriIndex triangle;
        glm::dvec3 normal;
        std::vector<uint32_t> outside;
        std::vector<uint32_t> neighbors;
        bool onHull;
    };

    inline void iterate();

    std::vector<TriIndex> initialApproximation();

    void prepareFacets(const std::vector<TriIndex>& triangles);
    void prepareFacets(const std::vector<uint32_t>& horizon, std::vector<uint32_t>& set);
    void sortCloud(std::vector<uint32_t>& free, Facet& facet);
    void calculateHorizon(const glm::dvec3& apex, int64_t last, uint32_t current, std::vector<uint32_t>& horizon, std::vector<uint32_t>& set);

//    void purgeOrphans(std::vector<std::vector<uint32_t>>& faces);

private:

    std::vector<glm::dvec3> m_cloud;
    std::vector<Facet> m_facets;

    bool m_initialized;
    uint32_t m_iteration;

    std::vector<glm::dvec3> m_vertices;

};

#endif //PATHFINDER_CONVEXHULL_H