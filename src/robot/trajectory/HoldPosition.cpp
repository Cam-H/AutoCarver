//
// Created by cjhat on 2025-08-13.
//

#include "HoldPosition.h"

#include "core/Scene.h"

HoldPosition::HoldPosition(const Waypoint& position, double duration)
        : Trajectory(position.values.size())
        , m_position(position)
{

    m_minDuration = duration;
    m_duration = duration;
}

// Intentionally does nothing
void HoldPosition::update() {}

Waypoint HoldPosition::start() const
{
    return m_position;
}

Waypoint HoldPosition::end() const
{
    return m_position;
}

std::vector<double> HoldPosition::velocity(double t) const
{
    return std::vector<double>(m_dof, 0);
}
std::vector<double> HoldPosition::acceleration(double t) const
{
    return std::vector<double>(m_dof, 0);
}

double HoldPosition::maximumDelta() const
{
    return 0;
}

Waypoint HoldPosition::evaluate(double t) const
{
    return m_position;
}

// Collision only needs to be tested once unlike other trajectory types
bool HoldPosition::test(const Scene* scene, const std::shared_ptr<Robot>& robot, double dt) const
{
    if (scene == nullptr || robot == nullptr) throw std::runtime_error("[HoldPosition] nullptr can not evaluate collision");

    const Waypoint initialWP = robot->getWaypoint();
    robot->moveTo(m_position);

    bool collision = scene->test(robot);

    robot->moveTo(initialWP); // Restore initial position

    return collision;
}