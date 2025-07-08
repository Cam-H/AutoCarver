//
// Created by Cam on 2025-04-16.
//

#ifndef AUTOCARVER_TRAJECTORY_H
#define AUTOCARVER_TRAJECTORY_H

#include <vector>
#include <iostream>
#include <cstdint>
#include <functional>

struct Waypoint {
    std::vector<float> values;
    float toRad;
    bool collides;
};

std::ostream& operator<<(std::ostream& stream, const Waypoint& waypoint);

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
    JointTrajectory(const std::vector<float>& waypoints, TrajectorySolverType solver = TrajectorySolverType::LINEAR);

    void updateCoefficients();

//    void setVelocityEndpoints(float vo, float vf);
//    void setAccelerationEndpoints(float ao, float af);

    [[nodiscard]] float position(uint32_t step, float t) const;
    [[nodiscard]] float velocity(uint32_t step, float t) const;
    [[nodiscard]] float acceleration(uint32_t step, float t) const;

    [[nodiscard]] float maxVelocity() const;
    [[nodiscard]] float maxAcceleration() const;

    [[nodiscard]] float maxVelocity(uint32_t step) const;
    [[nodiscard]] float maxAcceleration(uint32_t step) const;

    [[nodiscard]] std::vector<float> pTrajectory(float tStep = 0.05f) const;
    [[nodiscard]] std::vector<float> vTrajectory(float tStep = 0.05f) const;
    [[nodiscard]] std::vector<float> aTrajectory(float tStep = 0.05f) const;

    [[nodiscard]] std::vector<float> t(float tStep = 0.05f) const;

private:

    void prepareLinearCoefficients();
    void prepareCubicCoefficients();
    void prepareQuinticCoefficients();

    [[nodiscard]] std::vector<float> trajectory(const std::function<float (uint32_t, float)>& func, float tStep = 0.05f) const;

private:

    std::vector<float> m_waypoints;
    std::vector<float> m_coeffs;
    TrajectorySolverType m_solver;

};

class Trajectory {
public:

    Trajectory(const Waypoint& start, const Waypoint& end, TrajectorySolverType solverType);
    Trajectory(const std::vector<Waypoint>& waypoints, TrajectorySolverType solverType);

//    void setEndpoints(const std::vector<float> endpoints);

    void insertWaypoint(uint32_t idx, const Waypoint& waypoint);

    void setMaxVelocity(float velocity);
//    void setMaxAcceleration(float acceleration);

    [[nodiscard]] uint32_t waypointCount() const;
    [[nodiscard]] uint32_t dimensions() const;

    [[nodiscard]] float tStep() const;
    [[nodiscard]] float t() const;

    [[nodiscard]] bool complete() const;

    [[nodiscard]] Waypoint start();
    [[nodiscard]] Waypoint end();

    [[nodiscard]] Waypoint next();
    [[nodiscard]] Waypoint timestep(float delta);
    [[nodiscard]] Waypoint evaluate(float t) const;

    [[nodiscard]] const JointTrajectory& jointTrajectory(uint32_t idx);

private:
    void initialize();

    void calculateDuration();

private:
    std::vector<Waypoint> m_waypoints;

    TrajectorySolverType m_solver;
    uint32_t m_jointCount;
//    std::vector<float> m_endpoints;

    float m_t;
    float m_tStep;

    float m_maxVelocity;
    float m_maxAcceleration;

    float m_duration;

    std::vector<JointTrajectory> m_jointTrajectories;
};

const static JointTrajectory NULL_TRAJECTORY = JointTrajectory(0, 0);

#endif //AUTOCARVER_TRAJECTORY_H
