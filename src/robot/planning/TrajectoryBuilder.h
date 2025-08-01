//
// Created by cjhat on 2025-07-27.
//

#ifndef AUTOCARVER_TRAJECTORYBUILDER_H
#define AUTOCARVER_TRAJECTORYBUILDER_H

#include <glm.hpp>
#include <memory>

#include "Interpolator.h"
#include "robot/Pose.h"


class Robot;

class Axis3D;
class Waypoint;

class SimpleTrajectory;
class TOPPTrajectory;

// static interface to generate more complicated trajectories
class TrajectoryBuilder {
public:

    // Prepare a trajectory to travel from the initialPose [translation] in the local reference frame. Space is partitioned
    // between waypoints to restrict divergence of position and orientation below a set tolerance during travel
    static std::shared_ptr<TOPPTrajectory> cartesianMotion(const std::shared_ptr<Robot>& robot,
                                                           const Pose& initialPose, const glm::dvec3& translation);

    // Prepare a trajectory to travel from the initialPose [translation] in the local reference frame. Space is partitioned
    // between waypoints to restrict divergence of position and orientation below a set tolerance during travel
    static std::shared_ptr<TOPPTrajectory> cartesianMotion(const std::shared_ptr<Robot>& robot,
                                                           const Pose& initialPose, const glm::dvec3& translation,
                                                           const std::vector<double>& vLims, const std::vector<double>& aLims);

private:

    static std::vector<Waypoint> cartesianWaypoints(const std::shared_ptr<Robot>& robot,
                                                    const Pose& initialPose, const glm::dvec3& translation);

};


#endif //AUTOCARVER_TRAJECTORYBUILDER_H
