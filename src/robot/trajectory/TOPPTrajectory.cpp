//
// Created by cjhat on 2025-07-26.
//

#include "TOPPTrajectory.h"

#include "core/Functions.h"
#include "TOPP.h"


TOPPTrajectory::TOPPTrajectory(const std::vector<Waypoint>& waypoints)
    : Trajectory(0)
    , m_path(waypoints)
    , m_ds(m_path.sEnd())
{
    if (!waypoints.empty()) {
        m_dof = waypoints[0].values.size();
        resetLimits();

        m_inDg = waypoints[0].inDg;
    }
}

TOPPTrajectory::TOPPTrajectory(const std::vector<Waypoint>& waypoints, const std::vector<double>& vLims, const std::vector<double>& aLims)
    : TOPPTrajectory(waypoints)
{
    assert(m_path.order() == vLims.size());
    assert(m_path.order() == aLims.size());

    m_velocityLimits = vLims;
    m_accelerationLimits = aLims;

    TOPPTrajectory::update();
}

void TOPPTrajectory::update()
{
    if (m_path.empty()) return;

    m_ds = m_path.sEnd() / (500 - 1);
    TOPP solver(m_path, 500);
    solver.compute(m_velocityLimits, m_accelerationLimits);

    m_t = solver.timestep();

    m_minDuration = m_t.back();
    m_duration = std::max(m_duration, m_minDuration);

    double inv = 1 / m_duration;
    for (double& t : m_t) t *= inv;

    //TODO calculate max vel/accel
}

Waypoint TOPPTrajectory::start() const
{
    if (m_path.empty()) throw std::runtime_error("[TOPPTrajectory] There are no waypoints. Can not return start point");
    return Waypoint(m_path.evaluate(0), m_inDg);
}
Waypoint TOPPTrajectory::end() const
{
    if (m_path.empty()) throw std::runtime_error("[TOPPTrajectory] There are no waypoints. Can not return end point");
    return Waypoint(m_path.evaluate(m_path.sEnd()), m_inDg);
}

std::vector<double> TOPPTrajectory::velocity(double t) const
{
    auto [s, dt] = tToS(t);
    auto velocities = m_path.evaluateFirstDerivative(s);
    for (double& vel : velocities) vel *= dt;
    return velocities;
}

std::vector<double> TOPPTrajectory::acceleration(double t) const
{
    auto [s, dt] = tToS(t);
    auto accelerations = m_path.evaluateSecondDerivative(s);
    for (double& accel : accelerations) accel *= dt;
    return accelerations;
}

double TOPPTrajectory::maximumDelta() const
{
    return 0;//todo
}

Waypoint TOPPTrajectory::evaluate(double t) const
{
    auto [s, dt] = tToS(t);
    return Waypoint(m_path.evaluate(s), m_inDg);
}

bool TOPPTrajectory::validate(const std::shared_ptr<Robot>& robot, double dt) const
{
    return !m_path.empty() && Trajectory::validate(robot, dt);
}

// Identifies the region based on t, returning s and dt, the time interval for the segment
std::tuple<double, double> TOPPTrajectory::tToS(double t) const
{
    if (m_t.empty()) throw std::runtime_error("[TOPPTrajectory] Can not evaluate empty trajectory");

    auto it = std::lower_bound(m_t.begin(), m_t.end(), t);
    if (it == m_t.begin()) return { 0, 0 };
    if (it == m_t.end()) return { m_path.sEnd(), 0 };

    return { std::distance(m_t.begin(), it) * m_ds, *(it + 1) - *it };
}