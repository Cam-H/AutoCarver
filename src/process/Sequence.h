//
// Created by cjhat on 2025-09-07.
//

#ifndef AUTOCARVER_SEQUENCE_H
#define AUTOCARVER_SEQUENCE_H

#include "geometry/Axis3D.h"
#include "robot/Pose.h"

#include "geometry/poly/Profile.h"
#include "geometry/poly/SectionOperation.h"

// A series of position/orientation instructions in world space
class Sequence {
public:

    Sequence(const Profile* profile, const std::vector<SectionOperation::Set>& cuts);

    void transform(const glm::dquat& rotation, const glm::dvec3& translation);

    [[nodiscard]] const glm::dvec3& start() const;
    [[nodiscard]] Pose startPose(const glm::dvec3& forward) const;

    [[nodiscard]] const glm::dvec3& end() const;

    struct Set {

        Set(const glm::dvec3& axis, const glm::dvec3& normal, const glm::dvec3& travel)
                : axis(axis)
                , normal(normal)
                , travel(travel) {}

        Axis3D axes(const glm::dvec3& forward) const;

        [[nodiscard]] bool isBlind() const;
        [[nodiscard]] bool isMill() const;

        std::vector<std::pair<glm::dvec3, double>> motions; // First = start position, second = final distance from start

        glm::dvec3 axis;
        glm::dvec3 normal;
        glm::dvec3 travel;
    };



public:
    std::vector<Set> sets;
    glm::dvec3 normal;
    glm::dvec3 exit;
};


#endif //AUTOCARVER_SEQUENCE_H
