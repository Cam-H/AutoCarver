//
// Created by Cam on 2025-06-19.
//

#ifndef AUTOCARVER_PROFILE_H
#define AUTOCARVER_PROFILE_H

#include "Polygon.h"
#include "fileIO/Serializable.h"

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

    void translate(const glm::dvec2& translation);
    void translate(const glm::dvec3& translation);

    void rotateAbout(const glm::dvec3& axis, double theta);

    void inverseWinding() override;

    std::vector<uint32_t> refine();
    bool complete() const;

    bool isNextExternal() const;

    const glm::dvec3& normal() const;


    [[nodiscard]] std::vector<glm::dvec3> projected3D(const glm::dvec3& offset = {});
    [[nodiscard]] std::vector<glm::dvec3> projected3D(const std::vector<uint32_t>& indices, const glm::dvec3& offset = {});

    [[nodiscard]] std::vector<std::pair<glm::dvec2, glm::dvec2>> debugEdges() const override;

private:

    void initialize();

    inline uint32_t offsetIndex(uint32_t idx, uint32_t offset = 1) const;

    void emplaceRemainder(uint32_t start, uint32_t count);
    void insertRemainder(uint32_t index, uint32_t start, uint32_t count);
    uint32_t subdivide(const glm::dvec2& normal, uint32_t start, uint32_t count);
    glm::dvec2 edgeNormal(uint32_t start, uint32_t end);

    std::vector<uint32_t> triangleRefinement();
    std::vector<uint32_t> directRefinement();
    std::vector<uint32_t> delauneyRefinement();
    std::vector<uint32_t> testRefinement();

    bool isValidRefinement(const std::vector<uint32_t>& indices) const;

    double area(const std::vector<uint32_t>& indices) const;
    inline static uint32_t difference(uint32_t a, uint32_t b, uint32_t max);

    inline std::vector<uint32_t> sectionIndices(const std::pair<uint32_t, uint32_t>& limits) const;
    inline std::vector<glm::dvec2> sectionVertices(const std::pair<uint32_t, uint32_t>& limits) const;
    inline std::vector<glm::dvec2> sectionVertices(const std::vector<uint32_t>& indices) const;

private:

    glm::dvec3 m_normal;
    glm::dvec3 m_xAxis;
    glm::dvec3 m_yAxis;

    RefinementMethod m_method;

    std::vector<uint32_t> m_hull; // Indices of vertices that form the convex hull
    std::vector<std::pair<uint32_t, uint32_t>> m_remainder; // first (index), second (number of subsequent vertices)
    uint32_t m_next;

    double m_minimumArea;
};


#endif //AUTOCARVER_PROFILE_H
