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
    Profile(const std::vector<glm::vec2>& contour, const glm::vec3& normal, const glm::vec3& xAxis, const glm::vec3& yAxis);

    explicit Profile(const std::string& filename);
    explicit Profile(std::ifstream& file);

    bool serialize(const std::string& filename) override;
    bool serialize(std::ofstream& file) override;

    bool deserialize(const std::string& filename) override;
    bool deserialize(std::ifstream& file) override;

    void setRefinementMethod(RefinementMethod method);
    void setMimimumArea(float area);

    void translate(const glm::vec3& translation);

    void rotateAbout(const glm::vec3& axis, float theta);

    void inverseWinding() override;

    std::vector<uint32_t> refine();
    bool complete() const;

    const glm::vec3& normal() const;


    [[nodiscard]] std::vector<glm::vec3> projected3D(const glm::vec3& offset = {});
    [[nodiscard]] std::vector<glm::vec3> projected3D(const std::vector<uint32_t>& indices, const glm::vec3& offset = {});

    [[nodiscard]] std::vector<std::pair<glm::vec2, glm::vec2>> debugEdges() const override;

private:

    void initialize();

    std::vector<uint32_t> triangleRefinement();
    std::vector<uint32_t> directRefinement();
    std::vector<uint32_t> delauneyRefinement();
    std::vector<uint32_t> testRefinement();

    bool isValidRefinement(const std::vector<uint32_t>& indices) const;

    float area(const std::vector<uint32_t>& indices) const;
    inline static uint32_t difference(uint32_t a, uint32_t b, uint32_t max);

    inline std::vector<uint32_t> sectionIndices(const std::pair<uint32_t, uint32_t>& limits) const;
    inline std::vector<glm::vec2> sectionVertices(const std::pair<uint32_t, uint32_t>& limits) const;
    inline std::vector<glm::vec2> sectionVertices(const std::vector<uint32_t>& indices) const;

private:

    glm::vec3 m_normal;
    glm::vec3 m_xAxis;
    glm::vec3 m_yAxis;

    RefinementMethod m_method;

    std::vector<std::pair<uint32_t, uint32_t>> m_remainder; // first (index), second (number of subsequent vertices)
    uint32_t m_next;

    float m_minimumArea;
};


#endif //AUTOCARVER_PROFILE_H
