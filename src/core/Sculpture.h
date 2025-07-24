//
// Created by Cam on 2024-11-10.
//

#ifndef AUTOCARVER_SCULPTURE_H
#define AUTOCARVER_SCULPTURE_H

#include <vector>

#include "geometry/Mesh.h"
#include "physics/CompositeBody.h"

#include "geometry/primitives/Plane.h"

class Sculpture : public CompositeBody {
public:

    Sculpture(const std::shared_ptr<Mesh>& model, double width = 2.0f, double height = 6.0f);

    void scaleToFit(const std::shared_ptr<Mesh>& model, double width, double maxHeight);

    void update();

    void moved() override;

    void restore() override;
    void restoreAsHull();

    void queueSection(const glm::dvec3& origin, const glm::dvec3& normal);
    void queueSection(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, const glm::dvec3& normal, bool external = false);
//    void queueSection(const std::vector<glm::dvec3>& border, const glm::dvec3& normal);

    bool applySection();

    bool form();

    void remesh() override;

    const std::shared_ptr<Mesh>& sculpture();

    const std::shared_ptr<RigidBody>& model();

    [[nodiscard]] double width() const;
    [[nodiscard]] double height() const;
    [[nodiscard]] double rotation() const;

    [[nodiscard]] double scalar() const;

    [[nodiscard]] double initialVolume() const;
    [[nodiscard]] double currentVolume() const;
    [[nodiscard]] double finalVolume() const;

    [[nodiscard]] double bulkUsageRatio() const;
    [[nodiscard]] double remainderRatio() const;

private:

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

        SectionOperation(const glm::dvec3& origin, const glm::dvec3& normal)
            : surfaces({ {origin, normal} }) {}

        SectionOperation(const Plane& plane)
                : surfaces({ plane }) {}

//        SectionOperation(const std::vector<std::pair<glm::dvec3, glm::dvec3>>& surfaces)
//            : surfaces(surfaces) {}
    };

private:

    std::shared_ptr<RigidBody> modelBody;

    std::vector<RigidBody> m_fragments;

    double m_width;
    double m_height;
    double m_rotation;

    double m_scalar;

    bool m_preserveDebris;

    uint32_t m_step;
    std::vector<SectionOperation> m_operations;

    uint32_t m_formStep;

    // Styling
    glm::dvec3 m_highlightColor;

};


#endif //AUTOCARVER_SCULPTURE_H
