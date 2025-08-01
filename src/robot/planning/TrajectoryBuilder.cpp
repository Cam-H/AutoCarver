//
// Created by cjhat on 2025-07-27.
//

#include "TrajectoryBuilder.h"

#include "robot/Robot.h"

#include "Waypoint.h"
#include "SimpleTrajectory.h"
#include "TOPPTrajectory.h"

#include "geometry/primitives/Ray.h"

std::shared_ptr<TOPPTrajectory> TrajectoryBuilder::cartesianMotion(const std::shared_ptr<Robot>& robot,
                                                                   const Pose& initialPose, const glm::dvec3& translation)
{
    const std::vector<Waypoint>& waypoints = cartesianWaypoints(robot, initialPose, translation);
    if (!waypoints.empty()) return std::make_shared<TOPPTrajectory>(waypoints);
    else return nullptr;
}

std::shared_ptr<TOPPTrajectory> TrajectoryBuilder::cartesianMotion(const std::shared_ptr<Robot>& robot,
                                                                   const Pose& initialPose, const glm::dvec3& translation,
                                                                   const std::vector<double>& vLims, const std::vector<double>& aLims)
{
    const std::vector<Waypoint>& waypoints = cartesianWaypoints(robot, initialPose, translation);
    if (!waypoints.empty()) return std::make_shared<TOPPTrajectory>(waypoints, vLims, aLims);
    else return nullptr;
}

std::vector<Waypoint> TrajectoryBuilder::cartesianWaypoints(const std::shared_ptr<Robot>& robot,
                                                            const Pose& initialPose, const glm::dvec3& translation)
{
    if (glm::dot(translation, translation) < 1e-6) return {};

    Pose finalPose = initialPose;
    finalPose.localTranslate(translation);

    std::vector<Waypoint> waypoints = { robot->inverse(initialPose).toDg() };
    std::vector<Waypoint> remainder = { robot->inverse(finalPose).toDg() };

    if (waypoints[0].isValid() && remainder[0].isValid()) { // Don't try if endpoints are impossible to reach

        glm::dvec3 axis = finalPose.position - initialPose.position;
        double length = glm::length(axis);
        Ray ray(initialPose.position, axis / length);

        std::vector<double> steps = { 0.0, 1.0 }; // Track position along axis ([0] = max, -> steps)

        while (!remainder.empty()) {
            Waypoint wp = Waypoint::midpoint(waypoints.back(), remainder.back());
            auto pose = robot->getPose(wp);

            // Compare position/orientation divergence against tolerance OR exceeded step partition
            if (steps[0] + 1e-6 > steps.back() || (ray.squareDistance(pose.position) < 1e-3 && initialPose.oriented(pose, 1e-3))) {
                waypoints.emplace_back(remainder.back());
                remainder.pop_back();
                steps[0] = steps.back();
                steps.pop_back();
            } else { // Outside allowable tolerance for linear trajectory
                double ds = 0.5 * (steps.back() - steps[0]);
                steps.emplace_back(steps[0] + ds);

                pose = initialPose;
                pose.globalTranslate(axis * steps.back());
                remainder.emplace_back(robot->inverse(pose).toDg());
            }
        }

        return waypoints;
    }

    return {};
}