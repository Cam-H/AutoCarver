//
// Created by cjhat on 2025-08-01.
//

#include "CartesianTrajectory.h"


// translation - in units local to the axes defined in initialPose
CartesianTrajectory::CartesianTrajectory(const std::shared_ptr<Robot>& robot, const Pose& initialPose, const glm::dvec3& translation, uint32_t steps)
    : TOPPTrajectory({robot == nullptr ? std::vector<Waypoint>() : std::vector<Waypoint>{ robot->inverse(initialPose) }})
    , m_robot(robot)
    , m_initialPose(initialPose)
{
    auto finalPose = m_initialPose;
    finalPose.localTranslate(translation);

    m_curve = Ray(initialPose.position, finalPose.position - initialPose.position);
    m_distance = glm::length(m_curve.axis);
    m_curve.axis /= m_distance;

    resolve(steps);
}

void CartesianTrajectory::resolve(uint32_t steps)
{
    Pose pose = m_initialPose;
    std::vector<Waypoint> waypoints;
    waypoints.reserve(steps);

    glm::dvec3 dt = (m_distance / (steps - 1)) * m_curve.axis;
    for (uint32_t i = 0; i < steps; i++) {
        waypoints.emplace_back(m_robot->inverse(pose));
        if (!waypoints.back().isValid()) {
            std::cout << "\033[93mFAILED to resolve cartesian trajectory: " << i << " " << steps << "\n\033[0m";
            waypoints.clear();
            break;
        }

        pose.position += dt;
    }

    m_path = PiecewisePolyPath(waypoints);

    if (TOPPTrajectory::testValidity()) update();
//    testValidity();
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

    return std::make_shared<CartesianTrajectory>(robot, initialPose, initialPose.axes.localize(delta), m_path.segments());
}

// Given ratio (% of path completed) returns a corresponding t
// Applies a pseudo-binary search to find an approximate t
//double CartesianTrajectory::locate(double ratio) const
//{
////    if (ratio < 0.0) return 0.0;
////    if (ratio > 1.0) return 1.0;
//
//    auto target = m_initialPose.position + m_curve.axis * m_distance * ratio;
//    glm::dvec3 delta;
//
//    double t = 0.5, dt = 0.5 * t;
//    do {
//        delta = m_robot->getPose(TOPPTrajectory::evaluate(t)).position - target;
//        if (glm::dot(delta, m_curve.axis) > 0) { // Target is behind (less than t)
//            t -= dt;
//        } else { // Target is ahead (greater than t)
//            t += dt;
//        }
//
//        dt *= 0.5;
//
//        if (dt < 1e-12) {
//            std::cout << ratio << " " << dt << " " << t << " " << (glm::dot(delta, m_curve.axis) > 0) << " " << glm::dot(delta, delta) << "\n";
//            throw std::runtime_error("[CartesianTrajectory] Failed to locate a reasonable t");
//        }
//    } while (glm::dot(delta, delta) > 1e-3 * m_distance);
//
//    std::cout << "DELTA: " << glm::dot(delta, delta ) << " " << t << " " << dt << " " << delta.x << " " << delta.y << " " << delta.z << "\n";
//
//    return t;
//}

bool CartesianTrajectory::testValidity()
{
    if (TOPPTrajectory::testValidity()) {
        double dt = 0.01, t = dt;
        while (t < 1.0) {
            auto pose = m_robot->getPose(TOPPTrajectory::evaluate(t));
            if (m_curve.squareDistance(pose.position) > 1e-3 || !pose.oriented(m_initialPose.axes, 1e-3)) return m_valid = false;
            t += dt;
        }
    }

    return m_valid;
}