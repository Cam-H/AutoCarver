//
// Created by Cam on 2025-06-19.
//

#ifndef AUTOCARVER_PROFILE_H
#define AUTOCARVER_PROFILE_H

#include "Polygon.h"

#include <deque>

#include "geometry/Axis3D.h"

#include "SectionOperation.h"

class RRTPlanner;

class Profile : public Polygon {
public:

    enum class RefinementMethod {
        DIRECT = 0, DELAUNEY, TEST
    };

    Profile();
    Profile(const std::vector<glm::dvec2>& contour, const glm::dvec3& normal, const glm::dvec3& xAxis, const glm::dvec3& yAxis);

    explicit Profile(const std::string& filename);
    explicit Profile(std::ifstream& file);

    bool serialize(std::ofstream& file) const override;
    bool deserialize(std::ifstream& file) override;

    void setRefinementMethod(RefinementMethod method);
    void setMimimumArea(double area);

//    void translate(const glm::dvec2& translation);
    void translate(const glm::dvec3& translation);

    void positionVertex(uint32_t index, const glm::dvec2& position) override;

    void rotateAbout(const glm::dvec3& axis, double theta);

    void inverseWinding() override;

    void setToolDimensions(double width, double thickness);
    [[nodiscard]] std::tuple<double, double> toolDimensions() const;

    void skip();
    void refine();

    void prepare(const Polygon& wrapper);
    void prepare();

    bool planReturn();
    bool planNext();

    [[nodiscard]] TriIndex nextTriangle() const;
    [[nodiscard]] glm::dvec2 nextExternal() const;
    [[nodiscard]] glm::dvec3 nextNormal() const;
    [[nodiscard]] std::pair<glm::dvec3, glm::dvec3> nextEdge() const;

    [[nodiscard]] SectionOperation nextOperation() const;
    [[nodiscard]] std::vector<glm::dvec3> nextPath() const;

    [[nodiscard]] const Triangle2D& nextBounds() const;
    [[nodiscard]] bool nextSequenceAccessible() const;
    [[nodiscard]] bool nextSequenceReady() const;
    [[nodiscard]] std::vector<SectionOperation::Set> nextSequence() const;
    [[nodiscard]] std::vector<glm::dvec3> returnPath() const;

    [[nodiscard]] uint32_t remainingSections() const;
    [[nodiscard]] bool complete() const;

    [[nodiscard]] bool isVertexExternal(uint32_t index) const;
    [[nodiscard]] bool isNextExternal() const;

    [[nodiscard]] uint32_t nextDepth() const;

    [[nodiscard]] double area() const;

    [[nodiscard]] double floor() const;

    [[nodiscard]] std::pair<double, double> angles() const;
    [[nodiscard]] std::pair<double, double> clearance() const;

    [[nodiscard]] std::pair<double, double> angles(const TriIndex& triangle) const;
    [[nodiscard]] std::pair<double, double> clearance(const TriIndex& triangle) const;

//    [[nodiscard]] std::tuple<bool, glm::

    [[nodiscard]] bool validate(const SectionOperation::Relief& relief) const;

    [[nodiscard]] const Axis3D& system() const;
    [[nodiscard]] const glm::dvec3& normal() const;

    [[nodiscard]] glm::dvec2 project(const glm::dvec3& vertex) const;

    [[nodiscard]] inline glm::dvec3 projected3D(const glm::dvec2& vertex) const { return Polygon::projected3D(vertex, m_system.xAxis, m_system.yAxis, {}); }

    [[nodiscard]] std::vector<glm::dvec3> projected3D() const;
    [[nodiscard]] std::vector<glm::dvec3> projected3D(const TriIndex& triangle, const glm::dvec3& offset = {}) const;
    [[nodiscard]] std::vector<glm::dvec3> projected3D(const std::vector<uint32_t>& indices, const glm::dvec3& offset = {}) const;

    [[nodiscard]] uint32_t prevVertex(uint32_t vertexIndex) const override;
    [[nodiscard]] uint32_t nextVertex(uint32_t vertexIndex) const override;

    [[nodiscard]] std::vector<Polygon> decompose() const override;
    [[nodiscard]] std::vector<Polygon> frontier() const;

    [[nodiscard]] const std::vector<Polygon>& remainder() const;
    [[nodiscard]] std::vector<Polygon> nextSkips();

    [[nodiscard]] const RRTPlanner* planner() const;
    [[nodiscard]] const Polygon& wrapper() const;

    [[nodiscard]] std::vector<std::pair<glm::dvec2, glm::dvec2>> debugEdges() const override;

    void print() const override;

private:

    struct Section {
        Section(TriIndex triangle, uint32_t index, uint32_t depth, uint32_t children = 0, bool external = false)
            : triangle(triangle)
            , index(index)
            , depth(depth)
            , children(children)
            , external(external)
            , skipped(false) {}

        TriIndex triangle; // Indices to the vertices forming the section
        uint32_t index; // Index of this section (as created)

        uint32_t depth; // Number of sections that this one is dependent on
        uint32_t children; // Number of sections dependent on this one

        bool external; // Whether the section has no dependencies (depth == 0 to start)
        bool skipped; // Track if section has previously been skipped
    };

    struct Sequence {

        Sequence() : valid(false), bounds({}, {}, {}), leftFirst(false), cutWrap(false) {}

        Sequence(const Triangle2D& bounds, const std::vector<SectionOperation::Set>& left, const std::vector<SectionOperation::Set>& right)
            : valid(false)
            , bounds(bounds)
            , leftFirst(true)
            , left(left)
            , right(right)
            , cutWrap(false)
            , cutPosition()
            , cutDirection()
            , cutDepth(0.0)
        {
            // Make sure there is extra space to add return set if needed
            this->left.reserve(left.size() + 1);
            this->right.reserve(right.size() + 1);
        }

        [[nodiscard]] std::pair<glm::dvec3, glm::dvec3> cut() const;

        bool valid;
        Triangle2D bounds;

        std::vector<glm::dvec3> approach;
        std::vector<glm::dvec3> transition;

        bool leftFirst;
        std::vector<SectionOperation::Set> left;
        std::vector<SectionOperation::Set> right;

        bool cutWrap; // Whether m_wrapper needs to be trimmed before performing the sequence
        std::vector<glm::dvec3> cutApproach; // Approach to the cut, if required
        glm::dvec2 cutPosition;
        glm::dvec2 cutDirection;
        double cutDepth;
    };

    void initialize();

    void sortOperations();

    [[nodiscard]] inline uint32_t terminus(uint32_t step) const;
    [[nodiscard]] inline uint32_t offsetIndex(uint32_t idx, uint32_t offset = 1) const;

    void emplaceOperation(uint32_t start, uint32_t count);
    uint32_t subdivide(const glm::dvec2& normal, uint32_t start, uint32_t count);
    glm::dvec2 edgeNormal(uint32_t start, uint32_t end);

    void addTriangle(uint32_t startIndex);
    void directRefinement();
    void delauneyRefinement();
    void testRefinement();

    void findFloor();

    bool planSlice(const SectionOperation& op);
    bool planSequence(std::vector<Polygon>& decomposition, const SectionOperation& op);
    bool planTransition(const Triangle2D& bounds, const std::vector<Polygon>& env, std::vector<SectionOperation::Set>& sets);

    std::vector<glm::dvec3> planClear(const std::vector<Polygon>& decomposition, const std::vector<Polygon>& bounds, uint32_t iterations, bool& success);

    bool developApproach(const std::vector<Polygon>& decomposition, uint32_t iterations = 5000);

    [[nodiscard]] double area(const TriIndex& triangle) const;

    inline static uint32_t difference(uint32_t a, uint32_t b, uint32_t max);

    [[nodiscard]] inline std::vector<uint32_t> sectionIndices(const std::pair<uint32_t, uint32_t>& limits) const;
    [[nodiscard]] inline std::vector<glm::dvec2> sectionVertices(const std::pair<uint32_t, uint32_t>& limits) const;
    [[nodiscard]] inline std::vector<glm::dvec2> sectionVertices(const std::vector<uint32_t>& indices) const;

    static std::vector<Section> prepareSections(std::vector<TriIndex>& triangles, const std::vector<uint32_t>& indexMap);
    static std::tuple<bool, TriIndex> nextTriangle(std::vector<TriIndex>& triangles, uint32_t first, uint32_t last);

    [[nodiscard]] double clearance(const glm::dvec2& axis, uint32_t vertexIndex) const;

    bool intersects(const glm::dvec2& a, const glm::dvec2& b, uint32_t start, uint32_t end) const;
    void intersection(const glm::dvec2& a, const glm::dvec2& b, uint32_t start, uint32_t end, double& t) const;

    [[nodiscard]] std::pair<uint32_t, uint32_t> nextHullVertices(uint32_t vertexIndex) const;

    static double angle(const glm::dvec2& a, const glm::dvec2& b);

    void commitSections(const std::vector<Section>& sections);

private:

    Axis3D m_system;

    RefinementMethod m_method;

    std::vector<uint32_t> m_hull; // Indices of vertices that form the convex hull (Stored in ascending order)
    std::vector<std::pair<uint32_t, uint32_t>> m_operations; // first (index), second (number of subsequent vertices)

    std::deque<Section> m_sections;
    std::vector<Sequence> m_sequence;
    std::vector<glm::dvec3> m_return;

    std::vector<Polygon> m_remainder;

    double m_thickness;
    double m_width;

    double m_minimumArea;

    uint32_t m_floorIndex;

    RRTPlanner* m_planner;
    Polygon m_wrapper;
};


#endif //AUTOCARVER_PROFILE_H
