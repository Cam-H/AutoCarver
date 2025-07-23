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

class Mesh;
class Plane;

class ConvexHull {
public:

    ConvexHull();

    explicit ConvexHull(const VertexArray& cloud);
    explicit ConvexHull(const std::shared_ptr<Mesh>& mesh);

    explicit ConvexHull(const std::vector<glm::vec3>& cloud);


    ConvexHull(const ConvexHull& rhs) = default;

    void evaluate();

    void setWalkStart(uint32_t startIndex);
    [[nodiscard]] bool getWalkStart() const;

    [[nodiscard]] bool isValid() const;

    [[nodiscard]] uint32_t vertexCount() const;
    [[nodiscard]] const std::vector<glm::vec3>& vertices() const;

    [[nodiscard]] uint32_t facetCount() const;
    [[nodiscard]] const FaceArray& faces() const;

    [[nodiscard]] glm::vec3 center() const;

    [[nodiscard]] const glm::vec3& start() const;

    [[nodiscard]] uint32_t supportIndex(const glm::vec3& axis) const;
    [[nodiscard]] std::tuple<uint32_t, glm::vec3> extreme(const glm::vec3& axis) const;

    [[nodiscard]] uint32_t supportIndex(const glm::vec3& axis, uint32_t startIndex) const;
    [[nodiscard]] std::tuple<uint32_t, glm::vec3> extreme(const glm::vec3& axis, uint32_t startIndex) const;

    [[nodiscard]] bool empty() const;

    [[nodiscard]] uint32_t walk(const glm::vec3& axis) const;
    [[nodiscard]] uint32_t walk(const glm::vec3& axis, uint32_t index) const;

    void extents(const glm::vec3& axis, float& near, float& far) const;

    [[nodiscard]] const std::vector<uint32_t>& neighbors(uint32_t index) const;
    [[nodiscard]] std::vector<glm::vec3> border(const glm::vec3& faceNormal) const;

    [[nodiscard]] std::vector<uint32_t> horizon(const glm::vec3& axis) const;
    [[nodiscard]] std::vector<uint32_t> horizon(const glm::vec3& axis, const glm::vec3& support) const;

    [[nodiscard]] float volume() const;

    static ConvexHull unite(const ConvexHull& hullA, const ConvexHull& hullB);
    static std::tuple<bool, ConvexHull> tryMerge(const ConvexHull& hullA, const ConvexHull& hullB);

    void print() const;

private:

    struct Facet {
        Triangle triangle;
        glm::vec3 normal;
        std::vector<uint32_t> outside;
        std::vector<uint32_t> neighbors;
        bool onHull;

//        ~Facet() { delete[] normal; }
    };

    void initialize();

    std::vector<Triangle> initialApproximation();
    glm::vec3 wNormal(const Triangle& triangle);

    [[nodiscard]] uint32_t step(const glm::vec3& normal, const glm::vec3& axis, uint32_t index) const;

    void prepareFacets(const std::vector<Triangle>& triangles);
    void prepareFacets(const std::vector<uint32_t>& horizon, std::vector<uint32_t>& set);
    void sortCloud(std::vector<uint32_t>& free, Facet& facet);
    void calculateHorizon(const glm::vec3& apex, int64_t last, uint32_t current, std::vector<uint32_t>& horizon, std::vector<uint32_t>& set);

    void prepareFaces();
    void purgeOrphans(std::vector<std::vector<uint32_t>>& faces);

private:

    std::vector<glm::vec3> m_cloud;

    std::vector<glm::vec3> m_vertices;

    FaceArray m_faces;

//    std::vector<glm::vec3> w_vertices;
    std::vector<Facet> facets;


    glm::vec3 m_center;
    std::vector<std::vector<uint32_t>> m_walks;
    uint32_t m_walkStart;

    float m_volume;

    const static uint8_t STRIDE = 3;

};

#endif //PATHFINDER_CONVEXHULL_H