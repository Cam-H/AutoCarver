//
// Created by cjhat on 2025-07-26.
//

#include "CompoundTrajectory.h"

#include "Waypoint.h"
#include "SimpleTrajectory.h"

CompoundTrajectory::CompoundTrajectory()
    : Trajectory()
    , m_allowDiscontinuities(false)
    , m_solver(Interpolator::SolverType::QUINTIC)
{

}

CompoundTrajectory::CompoundTrajectory(const std::vector<Waypoint>& waypoints)
    : CompoundTrajectory()
{
    for (const Waypoint& waypoint : waypoints) addWaypoint(waypoint);
}

void CompoundTrajectory::calculateMaximums()
{
    m_maxVelocity = 0, m_maxAcceleration = 0;
    for (const std::shared_ptr<Trajectory>& traj : m_trajectories) {
        m_maxVelocity = std::max(m_maxVelocity, traj->maximumVelocity());
        m_maxAcceleration = std::max(m_maxAcceleration, traj->maximumVelocity());
    }
}

void CompoundTrajectory::calculateDuration()
{
    m_minDuration = 0, m_duration = 0;
    for (const std::shared_ptr<Trajectory>& traj : m_trajectories) {
        if (m_velocityLimit != std::numeric_limits<double>::max()) traj->limitVelocity(m_velocityLimit);
        if (m_accelerationLimit != std::numeric_limits<double>::max()) traj->limitAcceleration(m_accelerationLimit);

        m_minDuration += traj->minimumDuration();
        m_duration += traj->duration();
    }
}

// Add contribution of new trajectories to  duration & update maximums if needed
void CompoundTrajectory::addContribution(const std::shared_ptr<Trajectory>& trajectory)
{
    trajectory->limitVelocity(m_velocityLimit);
    trajectory->limitAcceleration(m_accelerationLimit);
    m_maxVelocity = std::max(m_maxVelocity, trajectory->maximumVelocity());
    m_maxAcceleration = std::max(m_maxAcceleration, trajectory->maximumVelocity());
    m_minDuration += trajectory->minimumDuration();
    m_duration += trajectory->duration();
}

// TODO
// Calculates appropriate velocity/acceleration constraints to reduce stops between steps
// Repetitively solves the system to converge towards the best constraints for speed
void CompoundTrajectory::smooth(uint8_t iterations)
{

}

void CompoundTrajectory::enableJumps(bool enable)
{
    if (m_allowDiscontinuities != enable) {
        m_allowDiscontinuities = enable;

        if (!m_allowDiscontinuities) resolveDiscontinuities();
    }
}

void CompoundTrajectory::resolveDiscontinuities()
{
    for (uint32_t i = 1; i < m_trajectories.size(); i++) resolveDiscontinuities(i);
}
void CompoundTrajectory::resolveDiscontinuities(uint32_t index)
{
    if (index > 0) {
        auto start = m_trajectories[index - 1]->end();
        auto end = m_trajectories[index]->start();

        if (start != end) {
            auto traj = createTrajectory(start, end);
            m_trajectories.insert(m_trajectories.begin() + index + 1, traj);
        }
    }
}

std::shared_ptr<Trajectory> CompoundTrajectory::createTrajectory(const Waypoint& start, const Waypoint& end){
    if (m_inDg) {
        auto traj = std::make_shared<SimpleTrajectory>(
                start.inDg ? start : start.toDg(),
                end.inDg ? end : end.toDg(),
                m_solver
        );

        addContribution(traj);
        return traj;
    } else {
        auto traj = std::make_shared<SimpleTrajectory>(
                start.inDg ? start.toRad() : start,
                end.inDg ? end.toRad() : end,
                m_solver
        );

        addContribution(traj);
        return traj;
    }
}


void CompoundTrajectory::addWaypoint(const Waypoint& waypoint)
{
    if (m_trajectories.empty()) {
        if (m_freeWaypoints.empty()) {
            m_freeWaypoints.emplace_back(waypoint);
            m_inDg = waypoint.inDg;
        } else {
            m_trajectories.emplace_back(createTrajectory(m_freeWaypoints[0], waypoint));
            m_freeWaypoints.clear();
        }
    } else {
        m_trajectories.emplace_back(createTrajectory(m_trajectories.back()->end(), waypoint));
    }
}

void CompoundTrajectory::insertWaypoint(uint32_t index, const Waypoint& waypoint)
{

//    idx = (idx >= m_waypoints.size() ? m_waypoints.size() - 1 : idx); // Safe because m_waypoints minimum size is 1
//    m_waypoints.insert(m_waypoints.begin() + idx, waypoint);
//
//    m_jointTrajectories.clear();
//    initialize();
}

void CompoundTrajectory::replaceWaypoint(uint32_t index, const Waypoint& waypoint)
{
//    if (idx < m_waypoints.size()) {
//        m_waypoints[idx] = waypoint;
//        m_jointTrajectories.clear();
//        initialize();
//    } else throw std::runtime_error("[Trajectory] Index out of bounds. Can not replace waypoint");
}

void CompoundTrajectory::removeWaypoint(uint32_t index)
{
//    if (idx < m_waypoints.size()) {
//        m_waypoints.erase(m_waypoints.begin() + idx);
//        m_jointTrajectories.clear();
//        initialize();
//    } else throw std::runtime_error("[Trajectory] Index out of bounds. Can not remove waypoint");
}

Waypoint CompoundTrajectory::start() const
{
    if (m_trajectories.empty()) throw std::runtime_error("[CompoundTrajectory] There are no trajectories. Can not return start point");
    return m_trajectories[0]->start();
}
Waypoint CompoundTrajectory::end() const
{
    if (m_trajectories.empty()) throw std::runtime_error("[CompoundTrajectory] There are no trajectories. Can not return end point");
    return m_trajectories.back()->end();
}

std::vector<double> CompoundTrajectory::velocity(double t) const
{
    auto [index, subT] = transform(t);
    return m_trajectories[index]->velocity(subT);
}
std::vector<double> CompoundTrajectory::acceleration(double t) const
{
    auto [index, subT] = transform(t);
    return m_trajectories[index]->acceleration(subT);
}

double CompoundTrajectory::maximumDelta() const
{
    double max = 0;
    for (const std::shared_ptr<Trajectory>& traj : m_trajectories) max = std::max(max, traj->maximumDelta());
    return max;
}

Waypoint CompoundTrajectory::evaluate(double t) const
{
    auto [index, subT] = transform(t);
    return m_trajectories[index]->evaluate(subT);
}

// Identify the appropriate section based on t, and transform to match
std::tuple<uint32_t, double> CompoundTrajectory::transform(double t) const
{
    if (t >= 1 || m_duration == 0) return { m_trajectories.size() - 1, 1.0 };
    else if (t < 0) return { 0, 0.0 };

    double sum = 0, temp;
    for (const std::shared_ptr<Trajectory>& traj : m_trajectories) {
        temp = traj->duration() / m_duration;

//        std::cout << sum << " " << temp << " " << traj->duration() << " " << m_duration << " " << t << " | " << m_trajectories.size() << "\n";
        if (sum + temp < t) sum += temp;
        else return { &traj - &m_trajectories[0], (t - sum) / temp };
    }

    return { m_trajectories.size() - 1, 1.0 };
}