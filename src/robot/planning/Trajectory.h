//
// Created by Cam on 2025-04-16.
//

#ifndef AUTOCARVER_TRAJECTORY_H
#define AUTOCARVER_TRAJECTORY_H

#include <vector>
#include <cstdint>

struct Waypoint {
    std::vector<float> values;
    bool collides;
};

// Defines how trajectories should be interpolated based on endpoints (constraints)
// LINEAR - Only position endpoints, linear interpolation
// CUBIC - Addition of velocity endpoints
// QUINTIC - Addition of acceleration endpoints
enum class TrajectorySolverType {
    LINEAR = 0, CUBIC, QUINTIC
};

class JointTrajectory {
public:

    JointTrajectory(float start, float end);
    JointTrajectory(const std::vector<float>& waypoints);

//    float interpolate(float t) const;
    [[nodiscard]] float interpolate(uint32_t step, float t) const;

    void setVelocityEndpoints(float vo, float vf);
    void setAccelerationEndpoints(float ao, float af);



private:

    std::vector<float> m_waypoints;

};

class Trajectory {
public:


    Trajectory(const Waypoint& start, const Waypoint& end, TrajectorySolverType solverType);
    Trajectory(const std::vector<Waypoint>& waypoints, TrajectorySolverType solverType);

    void setEndpoints(const std::vector<float> endpoints);
//
    bool complete();


    [[nodiscard]] Waypoint next();
    [[nodiscard]] Waypoint evaluate(float t) const;


private:
    std::vector<Waypoint> m_waypoints;

    TrajectorySolverType m_solver;
//    std::vector<float> m_endpoints;

    float m_t;
    float m_tStep;

    std::vector<JointTrajectory> m_jointTrajectories;
};


#endif //AUTOCARVER_TRAJECTORY_H
