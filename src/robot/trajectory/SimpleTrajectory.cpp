//
// Created by cjhat on 2025-07-26.
//

#include "SimpleTrajectory.h"

SimpleTrajectory::SimpleTrajectory(const Waypoint& start, const Waypoint& end, Interpolator::SolverType solverType)
        : Trajectory(start.values.size())
{
    if (m_dof != end.values.size())  throw std::runtime_error("[SimpleTrajectory] Invalid waypoints. Dimensions do not match");
    else if (start.inDg != end.inDg) throw std::runtime_error("[SimpleTrajectory] Invalid waypoints. Must be in the same units");

    m_inDg = start.inDg;

    m_jointTrajectories.reserve(m_dof);
    for (uint32_t i = 0; i < m_dof; i++) {
        m_jointTrajectories.emplace_back(start.values[i], end.values[i], solverType);
    }

    SimpleTrajectory::update();
}

void SimpleTrajectory::update()
{
    m_minDuration = 0;

    m_maxVelocity = 0, m_maxAcceleration = 0;
    for (uint32_t i = 0; i < m_dof; i++) {
        double vMax = m_jointTrajectories[i].maxVelocity(), aMax = m_jointTrajectories[i].maxAcceleration();

        m_minDuration = std::max(m_minDuration, vMax / m_velocityLimits[i]);
        m_minDuration = std::max(m_minDuration, aMax / m_accelerationLimits[i]);

        m_maxVelocity = std::max(m_maxVelocity, std::min(vMax, m_velocityLimits[i]));
        m_maxAcceleration = std::max(m_maxAcceleration, std::min(aMax, m_accelerationLimits[i]));
    }

    m_duration = std::max(m_duration, m_minDuration);

    // Convert to window t = [0, duration]
    m_maxVelocity /= m_duration;
    m_maxAcceleration /= m_duration;
}

Waypoint SimpleTrajectory::start() const
{
    std::vector<double> values(m_dof);
    for (uint32_t i = 0; i < m_dof; i++) values[i] = m_jointTrajectories[i].initialPosition();
    return Waypoint(values, m_inDg);
}

Waypoint SimpleTrajectory::end() const
{
    std::vector<double> values(m_dof);
    for (uint32_t i = 0; i < m_dof; i++) values[i] = m_jointTrajectories[i].finalPosition();
    return Waypoint(values, m_inDg);
}

std::vector<double> SimpleTrajectory::velocity(double t) const
{
    double step = 1 / m_duration;
    std::vector<double> values(m_dof);
    for (uint32_t i = 0; i < m_dof; i++) values[i] = step * m_jointTrajectories[i].velocity(t);
    return values;
}
std::vector<double> SimpleTrajectory::acceleration(double t) const
{
    double step = 1 / m_duration;
    std::vector<double> values(m_dof);
    for (uint32_t i = 0; i < m_dof; i++) values[i] = step * m_jointTrajectories[i].acceleration(t);
    return values;
}

double SimpleTrajectory::maximumDelta() const
{
    double max = 0;
    for (const Interpolator& jt : m_jointTrajectories) max = std::max(max, std::abs(jt.delta()));
    return max;
}

Waypoint SimpleTrajectory::evaluate(double t) const
{
    if (t >= 1) return end();
    else if (t < 0) return start();

    std::vector<double> values;
    values.reserve(m_dof);

    for (const Interpolator& jt : m_jointTrajectories) values.emplace_back(jt.position(t));

    return Waypoint(values, m_inDg);
}