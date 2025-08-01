//
// Created by cjhat on 2025-08-01.
//

#ifndef AUTOCARVER_CARTESIANTRAJECTORY_H
#define AUTOCARVER_CARTESIANTRAJECTORY_H

#include "TOPPTrajectory.h"

#include <glm.hpp>
#include <memory>

#include "robot/Pose.h"
#include "geometry/primitives/Ray.h"

class Robot;

class CartesianTrajectory : public TOPPTrajectory {
public:
    CartesianTrajectory(const std::shared_ptr<Robot>& robot, const Pose& initialPose, const glm::dvec3& translation);

    [[nodiscard]] bool validate(const std::shared_ptr<Robot>& robot, double dt) const override;

private:

    static std::vector<Waypoint> cartesianWaypoints(const std::shared_ptr<Robot>& robot,
                                                    const Pose& initialPose, const glm::dvec3& translation);


private:

    Ray m_curve;
    Axis3D m_axes;

};


#endif //AUTOCARVER_CARTESIANTRAJECTORY_H
