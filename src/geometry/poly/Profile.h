//
// Created by Cam on 2025-06-19.
//

#ifndef AUTOCARVER_PROFILE_H
#define AUTOCARVER_PROFILE_H

#include "Polygon.h"
#include "fileIO/Serializable.h"

class Profile : public Polygon, public Serializable {
public:
    Profile();
    Profile(const std::vector<glm::vec2>& contour, const glm::vec3& normal, const glm::vec3& xAxis, const glm::vec3& yAxis);

    explicit Profile(const std::string& filename);
    explicit Profile(std::ifstream& file);

    bool serialize(const std::string& filename) override;
    bool serialize(std::ofstream& file) override;

    bool deserialize(const std::string& filename) override;
    bool deserialize(std::ifstream& file) override;


    std::tuple<uint32_t, uint32_t, uint32_t> refine(float margin = 0);

    [[nodiscard]] std::vector<glm::vec3> projected3D(const glm::vec3& offset = {});

    [[nodiscard]] std::vector<std::pair<glm::vec2, glm::vec2>> debugEdges() const override;

private:

    void initialize();

private:

    glm::vec3 m_normal;
    glm::vec3 m_xAxis;
    glm::vec3 m_yAxis;

    std::vector<std::pair<uint32_t, uint32_t>> m_remainder;
    uint32_t m_next;
};


#endif //AUTOCARVER_PROFILE_H
