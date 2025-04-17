//
// Created by Cam on 2025-04-16.
//

#include "Trajectory.h"

#include "../Robot.h"

JointTrajectory::JointTrajectory(float start, float end)
    : JointTrajectory(std::vector<float>{ start, end })
{

}

JointTrajectory::JointTrajectory(const std::vector<float>& waypoints)
    : m_waypoints(waypoints)
{

}

//float JointTrajectory::interpolate(float t) const
//{
//    return interpolate((uint32_t)std::floor(t), t);
//}
float JointTrajectory::interpolate(uint32_t step, float t) const
{
    return m_waypoints[step] + (m_waypoints[step + 1] - m_waypoints[step]) * t;
}

Trajectory::Trajectory(const Waypoint& start, const Waypoint& end, TrajectorySolverType solverType)
    : Trajectory({ start, end }, solverType)
{

}

Trajectory::Trajectory(const std::vector<Waypoint>& waypoints, TrajectorySolverType solverType)
    : m_waypoints(waypoints)
    , m_solver(solverType)
    , m_t(0.0f)
    , m_tStep(0.05f)
{
    switch (solverType) {
        case TrajectorySolverType::LINEAR:
            break;
        case TrajectorySolverType::CUBIC:
            break;
        case TrajectorySolverType::QUINTIC:
            break;
    }
}

bool Trajectory::complete()
{
    return m_t >= (float)m_waypoints.size();
}

Waypoint Trajectory::next()
{
    m_t += m_tStep;
    return evaluate(m_t);
}

Waypoint Trajectory::evaluate(float t) const
{
    if (t >= (float)m_waypoints.size()) return m_waypoints[m_waypoints.size() - 1];
    else if (t < 0) return m_waypoints[0];

    uint32_t idx = std::floor(t);

    std::vector<float> values;
    values.reserve(m_waypoints[0].values.size());

    switch (m_solver) {
        case TrajectorySolverType::LINEAR:
            for (const JointTrajectory& jt : m_jointTrajectories) values.emplace_back(jt.interpolate(idx, t));
            break;
        case TrajectorySolverType::CUBIC:
            break;
        case TrajectorySolverType::QUINTIC:
            break;
    }

    return { values };
}