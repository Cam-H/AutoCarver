//
// Created by Cam on 2025-04-16.
//

#include "Trajectory.h"

#include <limits>

Trajectory::Trajectory()
    : m_t(0.0)
    , m_tStep(0.05)
    , m_duration(0.0)
    , m_minDuration(0.0)
    , m_velocityLimit(std::numeric_limits<double>::max())
    , m_accelerationLimit(std::numeric_limits<double>::max())
    , m_maxVelocity(0.0)
    , m_maxAcceleration(0.0)
    , m_inDg(false)
{

}

void Trajectory::restart()
{
    m_t = 0;
}

void Trajectory::setDuration(double duration)
{
    if (duration < m_minDuration) {
        std::cout << "[Trajectory] Requested duration is less than allowable to meet constraints. Request not applied";
        return;
    }

    // Adjust velocity / acceleration for the new interval
    double scalar = duration / m_duration;
    m_maxVelocity *= scalar;
    m_maxAcceleration *= scalar;

    m_duration = duration;
}

// Assign velocity limit to the trajectory
void Trajectory::setVelocityLimit(double velocity)
{
    m_velocityLimit = std::abs(velocity);
    if (m_velocityLimit < 1e-12) throw std::runtime_error("[Trajectory] Velocity limit is unreasonable");
    updateDuration();
}

// Assign acceleration limit to the trajectory
void Trajectory::setAccelerationLimit(double acceleration)
{
    m_accelerationLimit = std::abs(acceleration);
    if (m_accelerationLimit < 1e-12) throw std::runtime_error("[Trajectory] Acceleration limit is unreasonable");
    updateDuration();
}

// Apply new velocity limit to the trajectory if it is stricter than the current limit
void Trajectory::limitVelocity(double velocity)
{
    if (std::abs(velocity) < m_velocityLimit) setVelocityLimit(velocity);
}

// Apply new acceleration limit to the trajectory if it is stricter than the current limit
void Trajectory::limitAcceleration(double acceleration)
{
    if (std::abs(acceleration) < m_accelerationLimit) setAccelerationLimit(acceleration);
}

void Trajectory::setStep(double tStep)
{
    m_tStep = tStep;
}

double Trajectory::velocityLimit() const
{
    return m_velocityLimit;
}
double Trajectory::accelerationLimit() const
{
    return m_accelerationLimit;
}

double Trajectory::maximumVelocity() const
{
    return m_maxVelocity;
}
double Trajectory::maximumAcceleration() const
{
    return m_maxAcceleration;
}

double Trajectory::duration() const
{
    return m_duration;
}
double Trajectory::minimumDuration() const
{
    return m_minDuration;
}

double Trajectory::t() const
{
    return m_t;
}
double Trajectory::tStep() const
{
    return m_tStep;
}

bool Trajectory::complete() const
{
    return m_t - 1 > m_tStep;
}

std::vector<double> Trajectory::velocity() const
{
    return velocity(m_t);
}

std::vector<double> Trajectory::acceleration() const
{
    return acceleration(m_t);
}

Waypoint Trajectory::next()
{
    auto wp =  evaluate(m_t);
    m_t += m_tStep;

    return wp;
}

Waypoint Trajectory::timestep(double delta)
{
    if (m_duration == 0.0) return evaluate(m_t = 1);
    else return evaluate(m_t += delta / m_duration);
}

