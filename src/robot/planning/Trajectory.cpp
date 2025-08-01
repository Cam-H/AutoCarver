//
// Created by Cam on 2025-04-16.
//

#include "Trajectory.h"

#include <cassert>

Trajectory::Trajectory(uint32_t dof)
    : m_dof(dof)
    , m_t(0.0)
    , m_tStep(0.05)
    , m_duration(0.0)
    , m_minDuration(0.0)
    , m_velocityLimits(dof, std::numeric_limits<double>::max())
    , m_accelerationLimits(dof, std::numeric_limits<double>::max())
    , m_maxVelocity(0.0)
    , m_maxAcceleration(0.0)
    , m_inDg(false)
{
    assert(m_dof > 0);
}

void Trajectory::restart()
{
    m_t = 0;
}

void Trajectory::finish()
{
    m_t = 1;
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

// Set velocity limits
void Trajectory::setVelocityLimits(const std::vector<double>& vLims)
{
    assignLimits(m_velocityLimits, vLims);
    update();
}

// Set acceleration limit
void Trajectory::setAccelerationLimits(const std::vector<double>& aLims)
{
    assignLimits(m_accelerationLimits, aLims);
    update();
}

// Set velocity and acceleration limits
void Trajectory::setLimits(const std::vector<double>& vLims, const std::vector<double>& aLims)
{
    assignLimits(m_velocityLimits, vLims);
    assignLimits(m_accelerationLimits, aLims);
    update();
}

// Apply joint velocity limits, if they more strict
void Trajectory::limitVelocity(const std::vector<double>& vLims)
{
    bool modified = applyLimits(m_velocityLimits, vLims);
    if (modified) update();
}

// Apply joint acceleration limits, if they more strict
void Trajectory::limitAcceleration(const std::vector<double>& vLims)
{
    bool modified = applyLimits(m_accelerationLimits, vLims);
    if (modified) update();
}

// Apply joint velocity and acceleration limits, if they more strict
void Trajectory::limit(const std::vector<double>& vLims, const std::vector<double>& aLims)
{
    bool modified = applyLimits(m_velocityLimits, vLims);
    modified = applyLimits(m_accelerationLimits, aLims) || modified;
    if (modified) update();
}

void Trajectory::assignLimits(std::vector<double>& limits, const std::vector<double>& newLimits)
{
    assert(limits.size() == newLimits.size());
    limits = newLimits;

    // Ensure that all limits are positive
    for (double& lim : limits) lim = std::abs(lim);
}
bool Trajectory::applyLimits(std::vector<double>& limits, const std::vector<double>& additionalLimits)
{
    assert(limits.size() == additionalLimits.size());

    bool modified = false;
    for (uint32_t i = 0; i < limits.size(); i++) {
        if (std::abs(additionalLimits[i]) < limits[i]) {
            limits[i] = std::abs(additionalLimits[i]);
            modified = true;
        }
    }

    return modified;
}

void Trajectory::setStep(double tStep)
{
    m_tStep = tStep;
}

std::vector<double> Trajectory::velocityLimits() const
{
    return m_velocityLimits;
}
std::vector<double> Trajectory::accelerationLimits() const
{
    return m_accelerationLimits;
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

