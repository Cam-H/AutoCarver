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
    std::vector<double> values;
    double toRad;
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

    JointTrajectory(double start, double end);
    JointTrajectory(const std::vector<double>& waypoints, TrajectorySolverType solver = TrajectorySolverType::LINEAR);

    void updateCoefficients();

//    void setVelocityEndpoints(double vo, double vf);
//    void setAccelerationEndpoints(double ao, double af);

    [[nodiscard]] double position(uint32_t step, double t) const;
    [[nodiscard]] double velocity(uint32_t step, double t) const;
    [[nodiscard]] double acceleration(uint32_t step, double t) const;

    [[nodiscard]] double maxVelocity() const;
    [[nodiscard]] double maxAcceleration() const;

    [[nodiscard]] double maxVelocity(uint32_t step) const;
    [[nodiscard]] double maxAcceleration(uint32_t step) const;

    [[nodiscard]] std::vector<double> pTrajectory(double tStep = 0.05f) const;
    [[nodiscard]] std::vector<double> vTrajectory(double tStep = 0.05f) const;
    [[nodiscard]] std::vector<double> aTrajectory(double tStep = 0.05f) const;

    [[nodiscard]] std::vector<double> t(double tStep = 0.05f) const;

private:

    void prepareLinearCoefficients();
    void prepareCubicCoefficients();
    void prepareQuinticCoefficients();

    [[nodiscard]] std::vector<double> trajectory(const std::function<double (uint32_t, double)>& func, double tStep = 0.05f) const;

private:

    std::vector<double> m_waypoints;
    std::vector<double> m_coeffs;
    TrajectorySolverType m_solver;

};

class Trajectory {
public:

    Trajectory(const Waypoint& start, const Waypoint& end, TrajectorySolverType solverType);
    Trajectory(const std::vector<Waypoint>& waypoints, TrajectorySolverType solverType);

//    void setEndpoints(const std::vector<double> endpoints);

    void insertWaypoint(uint32_t idx, const Waypoint& waypoint);

    void setMaxVelocity(double velocity);
//    void setMaxAcceleration(double acceleration);

    [[nodiscard]] uint32_t waypointCount() const;
    [[nodiscard]] uint32_t dimensions() const;

    [[nodiscard]] double tStep() const;
    [[nodiscard]] double t() const;

    [[nodiscard]] bool complete() const;

    [[nodiscard]] Waypoint start();
    [[nodiscard]] Waypoint end();

    [[nodiscard]] Waypoint next();
    [[nodiscard]] Waypoint timestep(double delta);
    [[nodiscard]] Waypoint evaluate(double t) const;

    [[nodiscard]] const JointTrajectory& jointTrajectory(uint32_t idx);

private:
    void initialize();

    void calculateDuration();

private:
    std::vector<Waypoint> m_waypoints;

    TrajectorySolverType m_solver;
    uint32_t m_jointCount;
//    std::vector<double> m_endpoints;

    double m_t;
    double m_tStep;

    double m_maxVelocity;
    double m_maxAcceleration;

    double m_duration;

    std::vector<JointTrajectory> m_jointTrajectories;
};

const static JointTrajectory NULL_TRAJECTORY = JointTrajectory(0, 0);

#endif //AUTOCARVER_TRAJECTORY_H
