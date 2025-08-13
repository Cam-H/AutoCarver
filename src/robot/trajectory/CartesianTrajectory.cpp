//
// Created by cjhat on 2025-08-01.
//

#include "CartesianTrajectory.h"


// translation - in units local to the axes defined in initialPose
CartesianTrajectory::CartesianTrajectory(const std::shared_ptr<Robot>& robot, const Pose& initialPose, const glm::dvec3& translation)
    : TOPPTrajectory(cartesianWaypoints(robot, initialPose, translation))
{
    auto finalPose = initialPose;
    finalPose.localTranslate(translation);

    m_curve = Ray(initialPose.position, glm::normalize(finalPose.position - initialPose.position));
    m_axes = initialPose.axes;
}

std::vector<Waypoint> CartesianTrajectory::cartesianWaypoints(const std::shared_ptr<Robot>& robot,
                                                              const Pose& initialPose, const glm::dvec3& translation)
{
    assert(robot != nullptr);

    if (glm::dot(translation, translation) < 1e-6) {
        return {
            robot->inverse(initialPose)
        };
    }

    Pose finalPose = initialPose;
    finalPose.localTranslate(translation);

    std::vector<Waypoint> waypoints = { robot->inverse(initialPose) };
    std::vector<Waypoint> remainder = { robot->inverse(finalPose) };

    if (waypoints[0].isValid() && remainder[0].isValid()) { // Don't try if endpoints are impossible to reach

        uint32_t dof = waypoints[0].values.size();

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
                remainder.emplace_back(robot->inverse(pose));
                if (dof != remainder.back().values.size()) return { robot->getWaypoint() }; // Stop if some point along the path is not reachable
            }
        }

        return waypoints;
    }


    // Default to current robot position in case of failure (Must have a waypoint
    return { robot->getWaypoint() };
//    return {};
}

std::shared_ptr<CartesianTrajectory> CartesianTrajectory::reversed(const std::shared_ptr<Robot>& robot) const
{
    auto initialPose = robot->getPose(end());
    auto finalPose = robot->getPose(start());

    glm::dvec3 delta = finalPose.position - initialPose.position;

    return std::make_shared<CartesianTrajectory>(robot, initialPose, initialPose.axes.localize(delta));
}

bool CartesianTrajectory::validate(const std::shared_ptr<Robot>& robot, double dt) const
{
    if (TOPPTrajectory::validate(robot, dt)) {
        double t = dt;
        while (t < 1.0) {
            auto pose = robot->getPose(TOPPTrajectory::evaluate(t));
            if (m_curve.squareDistance(pose.position) > 1e-3 || !pose.oriented(m_axes, 1e-3)) return false;
            t += dt;
        }

        return true;
    }

    return false;
}