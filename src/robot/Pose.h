//
// Created by cjhat on 2025-07-28.
//

#ifndef AUTOCARVER_POSE_H
#define AUTOCARVER_POSE_H

#include <glm.hpp>
#include <memory>

#include "geometry/Axis3D.h"

class Pose {
public:

    Pose(const glm::dvec3& position, const Axis3D& axes);

    friend bool operator==(const Pose& lhs, const Pose& rhs);
    friend bool operator!=(const Pose& lhs, const Pose& rhs);

    friend Pose operator*(const glm::dmat4& lhs, const Pose& rhs);
//    friend Pose operator*(const Pose& lhs, const glm::dmat4& rhs);

    // Returns true if the pose is oriented in the same direction
    [[nodiscard]] bool oriented(const Pose& pose, double tolerance = 1e-12) const;
    [[nodiscard]] bool oriented(const Axis3D& system, double tolerance = 1e-12) const;

    [[nodiscard]] Pose translated(const glm::dvec3& translation) const;

public:
    glm::dvec3 position;
    Axis3D axes;
};


#endif //AUTOCARVER_POSE_H
