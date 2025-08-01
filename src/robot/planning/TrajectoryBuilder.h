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

// Builder class to generate more complicated trajectories
class TrajectoryBuilder {
public:

    TrajectoryBuilder(const std::shared_ptr<Robot>& robot);

    void setPose(const Pose& pose);
    void setSolver(Interpolator::SolverType solver);

    void setVelocityLimit(double velocity);
    void setAccelerationLimit(double acceleration);

    void clear();

    // Simple waypoint-based updates
//    void addWaypoint(const Waypoint& waypoint);
//    void removeWaypoint(uint32_t index);
//
//    void insertWaypoint(uint32_t index, const Waypoint& waypoint);
//    void replaceWaypoint(uint32_t index, const Waypoint& waypoint);

    // Involved trajectories

    // Move the specified delta (local to the initial pose) from the current waypoint, maintaining the same orientation the entire way
    std::shared_ptr<TOPPTrajectory> cartesianMotion(const glm::dvec3& delta);

private:

    std::shared_ptr<Robot> robot;
    Pose m_pose;

    Interpolator::SolverType m_solver;

    double m_velocityLimit;
    double m_accelerationLimit;

};


#endif //AUTOCARVER_TRAJECTORYBUILDER_H
