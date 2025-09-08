//
// Created by cjhat on 2025-07-29.
//

#include "CompositeTrajectory.h"

#include <iostream>

#include "SimpleTrajectory.h"

CompositeTrajectory::CompositeTrajectory(uint32_t dof)
    : Trajectory(dof)
    , m_error(false)
{

}

// Attaches the provided trajectory, result is disjoint if the end and start points do not match
void CompositeTrajectory::addTrajectory(const std::shared_ptr<Trajectory>& trajectory)
{
    if (trajectory != nullptr) {
        m_trajectories.push_back(trajectory);

        // TODO ensure all same units [dg or rad]
    } else {
        std::cout << "Failed to add trajectory. The trajectory provided is invalid\n";
        m_error = true;
    }
}

// Attaches the provided trajectory, developing an intermediary trajectory if required to maintain continuity
void CompositeTrajectory::connectTrajectory(const std::shared_ptr<Trajectory>& trajectory)
{
    // Check for disjoint trajectory
    if (!(m_trajectories.empty() || Waypoint::compare(m_trajectories.back()->end(), trajectory->start()))) {
        addTrajectory(std::make_shared<SimpleTrajectory>(m_trajectories.back()->end(), trajectory->start(), Interpolator::SolverType::QUINTIC));
    }

    addTrajectory(trajectory);
}

void CompositeTrajectory::clear()
{
    m_trajectories.clear();
    m_error = false;
}

void CompositeTrajectory::update()
{
    testValidity();

    // Make sure parent limits are passed to children, if assigned
    if (!m_velocityLimits.empty() || !m_accelerationLimits.empty()) {
        for (const std::shared_ptr<Trajectory>& traj : m_trajectories) {
            traj->limit(m_velocityLimits, m_accelerationLimits);
        }
    }

    // Calculate duration and other trajectory characteristics
    m_minDuration = 0, m_maxVelocity = 0, m_maxAcceleration = 0;
    double duration = 0;

    for (const std::shared_ptr<Trajectory>& traj : m_trajectories) {
        m_minDuration += traj->minimumDuration();
        m_maxVelocity = std::max(m_maxVelocity, traj->maximumVelocity());
        m_maxAcceleration = std::max(m_maxAcceleration, traj->maximumAcceleration());
        duration += traj->duration();
    }

    m_duration = std::max(m_duration, duration);
    m_duration = std::max(m_duration, m_minDuration);
}

bool CompositeTrajectory::continuous() const
{
    for (uint32_t i = 0; i < m_trajectories.size() - 1; i++) {
        if (Waypoint::compare(m_trajectories[i]->end(), m_trajectories[i + 1]->start())) return false;
    }

    return true;
}

uint32_t CompositeTrajectory::segmentCount() const
{
    return m_trajectories.size();
}
std::tuple<double, double> CompositeTrajectory::tLimits(uint32_t subIndex) const
{
    if (subIndex >= m_trajectories.size()) return { 1.0, 1.0 };

    double sum = 0;
    for (uint32_t i = 0; i < subIndex; i++) {
        sum += m_trajectories[i]->duration();
    }

    double t = sum / m_duration;
    return { t, t + m_trajectories[subIndex]->duration() / m_duration };
}

Waypoint CompositeTrajectory::start() const
{
    if (m_trajectories.empty()) throw std::runtime_error("[CompositeTrajectory] There are no trajectories. Can not return start point");
    return m_trajectories[0]->start();
}
Waypoint CompositeTrajectory::end() const
{
    if (m_trajectories.empty()) throw std::runtime_error("[CompositeTrajectory] There are no trajectories. Can not return end point");
    return m_trajectories.back()->end();
}

std::vector<double> CompositeTrajectory::velocity(double t) const
{
    auto [index, subT] = transform(t);
    return m_trajectories[index]->velocity(subT);
}
std::vector<double> CompositeTrajectory::acceleration(double t) const
{
    auto [index, subT] = transform(t);
    return m_trajectories[index]->acceleration(subT);
}

double CompositeTrajectory::maximumDelta() const
{
    double max = 0;
    for (const std::shared_ptr<Trajectory>& traj : m_trajectories) max = std::max(max, traj->maximumDelta());
    return max;
}

Waypoint CompositeTrajectory::evaluate(double t) const
{
    auto [index, subT] = transform(t);
    return m_trajectories[index]->evaluate(subT);
}

bool CompositeTrajectory::testValidity()
{
    if (!m_error && Trajectory::testValidity()) {
        for (const std::shared_ptr<Trajectory>& traj : m_trajectories) {
            if (!traj->isValid()) return m_valid = false;
        }
    }

    return m_valid && !m_error;
}

// Identify the appropriate section based on t, and transform to match
std::tuple<uint32_t, double> CompositeTrajectory::transform(double t) const
{
    if (m_trajectories.empty()) throw std::runtime_error("[CompositeTrajectory] Can not evaluate empty trajectory");
    else if (t >= 1 || m_duration == 0) return { m_trajectories.size() - 1, 1.0 };
    else if (t < 0) return { 0, 0.0 };

    double sum = 0, temp;
    for (const std::shared_ptr<Trajectory>& traj : m_trajectories) {
        temp = traj->duration() / m_duration;

        if (sum + temp < t) sum += temp;
        else return { &traj - &m_trajectories[0], (t - sum) / temp };
    }

    return { m_trajectories.size() - 1, 1.0 };
}