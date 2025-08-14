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

    void prepareCut(const Pose& system, double thickness);

    std::vector<std::shared_ptr<RigidBody>> removeMaterial(double depth);

private:

    struct Section {
        Section(uint32_t src, uint32_t cut, double ts, double tf, double depth) : srcIndex(src), cutIndex(cut), ts(ts), tf(tf), depth(std::min(tf, depth)) {}

        uint32_t srcIndex;
        uint32_t cutIndex;

        double ts; // Min distance from origin
        double tf; // Max distance from origin

        double depth; // Distance from origin to release fragment (Bit less than tf based on thickness)
    };

    struct Cut {
        Cut(const glm::dvec3& origin, const glm::dvec3& axis) : origin(origin), axis(axis) {}

        glm::dvec3 origin;
        glm::dvec3 axis;

        std::vector<Section> sections;
    };

    std::vector<std::shared_ptr<RigidBody>> removeCut();
    std::shared_ptr<RigidBody> removeSection(uint32_t index);

    std::shared_ptr<RigidBody> tryFragmentRelease(Section& section);

    void replaceIndex(uint32_t oldIndex, uint32_t newIndex);

private:

    std::vector<uint32_t> m_connectionCounts;
    std::deque<Cut> m_cuts;

};


#endif //AUTOCARVER_DEBRIS_H
