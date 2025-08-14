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

    void prepareCut(const Pose& worldSystem, double thickness);

    std::vector<std::shared_ptr<RigidBody>> removeMaterial(double ratio);

private:

    std::vector<std::shared_ptr<RigidBody>> removeCut();
    std::shared_ptr<RigidBody> removeSection(uint32_t index);

    void replaceIndex(uint32_t oldIndex, uint32_t newIndex);

    struct Section {
        Section(uint32_t src, uint32_t cut, double ts, double tf) : srcIndex(src), cutIndex(cut), ts(ts), tf(tf) {}

        uint32_t srcIndex;
        uint32_t cutIndex;

        double ts;
        double tf;
    };

    struct Cut {
        Cut(const glm::dvec3& origin, const glm::dvec3& axis) : origin(origin), axis(axis) {}

        glm::dvec3 origin;
        glm::dvec3 axis;

        std::vector<Section> sections;
    };

private:

    std::vector<uint32_t> m_connectionCounts;
    std::deque<Cut> m_cuts;

};


#endif //AUTOCARVER_DEBRIS_H
