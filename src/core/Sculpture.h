//
// Created by Cam on 2024-11-10.
//

#ifndef AUTOCARVER_SCULPTURE_H
#define AUTOCARVER_SCULPTURE_H

#include <vector>

#include "geometry/Mesh.h"
#include "physics/CompositeBody.h"

class Sculpture : public CompositeBody {
public:

    Sculpture(const std::shared_ptr<Mesh>& model, float width = 2.0f, float height = 6.0f);

    void scaleToFit(const std::shared_ptr<Mesh>& model, float width, float maxHeight);

    void update();

    void moved() override;

    void restore();
    void restoreAsHull();

    void recordSection(const glm::vec3& origin, const glm::vec3& normal);
    void recordSection(const std::vector<glm::vec3>& border, const glm::vec3& normal);

    void section();
    void section(const glm::vec3& origin, const glm::vec3& normal);
    void section(const std::vector<std::pair<glm::vec3, glm::vec3>>& surfaces);

    void remesh() override;

    const std::shared_ptr<Mesh>& sculpture();

    const std::shared_ptr<RigidBody>& model();

    [[nodiscard]] float width() const;
    [[nodiscard]] float height() const;
    [[nodiscard]] float rotation() const;

    [[nodiscard]] float scalar() const;

    [[nodiscard]] float initialVolume() const;
    [[nodiscard]] float currentVolume() const;
    [[nodiscard]] float finalVolume() const;

    [[nodiscard]] float bulkUsageRatio() const;
    [[nodiscard]] float remainderRatio() const;

private:

    void prepareBox();
    void prepareFragment(const ConvexHull& hull);

    void triangleSection(const std::pair<glm::vec3, glm::vec3>& planeA, const std::pair<glm::vec3, glm::vec3>& planeB);


    struct SectionOperation {
        std::vector<std::pair<glm::vec3, glm::vec3>> surfaces;

        SectionOperation()
            : surfaces() {}

        SectionOperation(const glm::vec3& origin, const glm::vec3& normal)
            : surfaces({ {origin, normal} }) {}

//        SectionOperation(const std::vector<std::pair<glm::vec3, glm::vec3>>& surfaces)
//            : surfaces(surfaces) {}
    };

private:

    std::shared_ptr<RigidBody> modelBody;

    std::vector<RigidBody> m_fragments;

    float m_width;
    float m_height;
    float m_rotation;

    float m_scalar;

    bool m_preserveDebris;

    uint32_t m_step;
    std::vector<SectionOperation> m_operations;

    // Styling
    glm::vec3 m_baseColor;
    glm::vec3 m_highlightColor;

};


#endif //AUTOCARVER_SCULPTURE_H
