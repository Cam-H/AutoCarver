//
// Created by Cam on 2024-11-10.
//

#ifndef AUTOCARVER_SCULPTURE_H
#define AUTOCARVER_SCULPTURE_H

#include <vector>

#include "geometry/Mesh.h"
#include "physics/CompositeBody.h"

#include "geometry/shape/Plane.h"

class Sculpture : public CompositeBody {
public:

    Sculpture(const std::shared_ptr<Mesh>& model, float width = 2.0f, float height = 6.0f);

    void scaleToFit(const std::shared_ptr<Mesh>& model, float width, float maxHeight);

    void update();

    void moved() override;

    void restore();
    void restoreAsHull();

    void queueSection(const glm::vec3& origin, const glm::vec3& normal);
    void queueSection(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& normal, bool external = false);
//    void queueSection(const std::vector<glm::vec3>& border, const glm::vec3& normal);

    bool applySection();

    bool form();

    void remesh() override;

    void applyCompositeColors(bool enable);

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

    void colorHulls();

    void prepareBox();
    void prepareFragment(const ConvexHull& hull);

    bool planarSection(const Plane& plane);
    bool triangleSection(const Plane& planeA, const Plane& planeB, const std::vector<Plane>& limits);

    inline static bool inLimit(const ConvexHull& hull, const Plane& limit);
    inline static bool inLimit(const ConvexHull& hull, const std::vector<Plane>& limits);

    struct SectionOperation {
        std::vector<Plane> surfaces;
        std::vector<Plane> limits;

        SectionOperation()
            : surfaces() {}

        SectionOperation(const glm::vec3& origin, const glm::vec3& normal)
            : surfaces({ {origin, normal} }) {}

        SectionOperation(const Plane& plane)
                : surfaces({ plane }) {}

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

    uint32_t m_formStep;

    // Styling
    glm::vec3 m_baseColor;
    glm::vec3 m_highlightColor;
    bool m_applyCompositeColor;

};


#endif //AUTOCARVER_SCULPTURE_H
