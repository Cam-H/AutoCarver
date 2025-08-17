//
// Created by Cam on 2025-06-19.
//

#ifndef AUTOCARVER_PROFILE_H
#define AUTOCARVER_PROFILE_H

#include "Polygon.h"
#include "fileIO/Serializable.h"

#include <deque>

class Profile : public Polygon, public Serializable {
public:

    enum class RefinementMethod {
        DIRECT = 0, DELAUNEY, TEST
    };

    Profile();
    Profile(const std::vector<glm::dvec2>& contour, const glm::dvec3& normal, const glm::dvec3& xAxis, const glm::dvec3& yAxis);

    explicit Profile(const std::string& filename);
    explicit Profile(std::ifstream& file);

    bool serialize(const std::string& filename) override;
    bool serialize(std::ofstream& file) override;

    bool deserialize(const std::string& filename) override;
    bool deserialize(std::ifstream& file) override;

    void setRefinementMethod(RefinementMethod method);
    void setMimimumArea(double area);

//    void translate(const glm::dvec2& translation);
    void translate(const glm::dvec3& translation);

    void rotateAbout(const glm::dvec3& axis, double theta);

    void inverseWinding() override;

    void skip();
    TriIndex refine();

    [[nodiscard]] uint32_t remainingSections() const;
    [[nodiscard]] bool complete() const;

    [[nodiscard]] bool isVertexExternal(uint32_t index) const;
    [[nodiscard]] bool isNextExternal() const;

    [[nodiscard]] std::pair<double, double> angles() const;
    [[nodiscard]] std::pair<double, double> clearance() const;

    [[nodiscard]] std::pair<double, double> angles(const TriIndex& triangle) const;
    [[nodiscard]] std::pair<double, double> clearance(const TriIndex& triangle) const;

    [[nodiscard]] const glm::dvec3& normal() const;


    [[nodiscard]] std::vector<glm::dvec3> projected3D(const glm::dvec3& offset = {});
    [[nodiscard]] std::vector<glm::dvec3> projected3D(const std::vector<uint32_t>& indices, const glm::dvec3& offset = {});

    [[nodiscard]] std::vector<std::pair<glm::dvec2, glm::dvec2>> debugEdges() const override;

private:

    struct Section {
        Section(TriIndex triangle, uint32_t children) : triangle(triangle), children(children) {}

        TriIndex triangle; // Indices to the vertices forming the section
        uint32_t children; // Number of sections dependent on this one
    };

    void initialize();

    [[nodiscard]] inline uint32_t terminus(uint32_t step) const;
    [[nodiscard]] inline uint32_t offsetIndex(uint32_t idx, uint32_t offset = 1) const;

    void emplaceRemainder(uint32_t start, uint32_t count);
    void insertRemainder(uint32_t index, uint32_t start, uint32_t count);
    uint32_t subdivide(const glm::dvec2& normal, uint32_t start, uint32_t count);
    glm::dvec2 edgeNormal(uint32_t start, uint32_t end);

    void addTriangle(uint32_t startIndex);
    void directRefinement();
    void delauneyRefinement();
    void testRefinement();

    [[nodiscard]] double area(const TriIndex& triangle) const;

    inline static uint32_t difference(uint32_t a, uint32_t b, uint32_t max);

    inline std::vector<uint32_t> sectionIndices(const std::pair<uint32_t, uint32_t>& limits) const;
    inline std::vector<glm::dvec2> sectionVertices(const std::pair<uint32_t, uint32_t>& limits) const;
    inline std::vector<glm::dvec2> sectionVertices(const std::vector<uint32_t>& indices) const;

    static std::vector<Section> prepareSections(std::vector<TriIndex>& triangles, const std::vector<uint32_t>& indexMap);
    static std::tuple<bool, TriIndex> nextTriangle(std::vector<TriIndex>& triangles, uint32_t first, uint32_t last);

    [[nodiscard]] double clearance(const glm::dvec2& axis, uint32_t vertexIndex) const;

    static std::tuple<bool, double> intersection(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c, const glm::dvec2& d);

    [[nodiscard]] std::pair<uint32_t, uint32_t> nextHullVertices(uint32_t vertexIndex) const;

    [[nodiscard]] uint32_t prevVertex(uint32_t vertexIndex) const;
    [[nodiscard]] uint32_t nextVertex(uint32_t vertexIndex) const;

    static double angle(const glm::dvec2& a, const glm::dvec2& b);


    void commitSections(const std::vector<Section>& sections);

private:

    glm::dvec3 m_normal;
    glm::dvec3 m_xAxis;
    glm::dvec3 m_yAxis;

    RefinementMethod m_method;

    std::vector<uint32_t> m_hull; // Indices of vertices that form the convex hull (Stored in ascending order)
    std::vector<std::pair<uint32_t, uint32_t>> m_remainder; // first (index), second (number of subsequent vertices)

    std::deque<Section> m_sections;

    double m_minimumArea;
};


#endif //AUTOCARVER_PROFILE_H
