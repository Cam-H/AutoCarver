//
// Created by cjhat on 2025-08-13.
//

#ifndef AUTOCARVER_DEBRIS_H
#define AUTOCARVER_DEBRIS_H

#include <deque>

#include "physics/CompositeBody.h"

#include "geometry/primitives/Plane.h"
#include "robot/Pose.h"

// Provides means to track the progress of cuts enacted during the sculpting process, simulating changing geometry & releasing fragments as required
class Debris : public CompositeBody {
public:

    explicit Debris(const ConvexHull& hull);
    explicit Debris(const std::vector<ConvexHull>& hulls);

    void initialize();

    void queueCut(const Pose& system, double thickness);
    void beginCut();
    void completeCut();

    [[nodiscard]] bool inProcess() const;

    std::vector<std::shared_ptr<RigidBody>> removeMaterial(double depth);

private:

    struct Connection {
        std::vector<glm::dvec3> border;
    };

    struct Kerf {
        Kerf(uint32_t src, uint32_t cut, double ts, double tf, double depth) : srcIndex(src), cutIndex(cut), ts(ts), tf(tf), depth(std::min(tf, depth)) {}

        uint32_t srcIndex;
        uint32_t cutIndex;

        double ts; // Min distance from origin
        double tf; // Max distance from origin

        double depth; // Distance from origin to release fragment (Bit less than tf based on thickness)
    };

    struct CutOperation {
        CutOperation(const glm::dvec3& origin, const glm::dvec3& normal, const glm::dvec3& axis, double thickness) : origin(origin), normal(normal), axis(axis), thickness(thickness) {}

        glm::dvec3 origin;
        glm::dvec3 normal;

        glm::dvec3 axis;
        double thickness;

        std::vector<Kerf> sections;
    };

    std::vector<std::shared_ptr<RigidBody>> removeCut();
    std::shared_ptr<RigidBody> removeKerf(uint32_t index);

    std::shared_ptr<RigidBody> tryFragmentRelease(Kerf& section);

    void replaceIndex(uint32_t oldIndex, uint32_t newIndex);

private:

    std::vector<Plane> m_fixedPlanes; // Retains any hulls in contact (even indirect) with any plane
    std::deque<CutOperation> m_cuts; // Record of remaining operations to be conducted
    bool m_inProcess;

};


#endif //AUTOCARVER_DEBRIS_H
