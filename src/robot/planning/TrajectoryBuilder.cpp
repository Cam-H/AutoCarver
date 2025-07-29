//
// Created by cjhat on 2025-07-27.
//

#include "TrajectoryBuilder.h"

#include "robot/Robot.h"

#include "robot/Pose.h"
#include "Waypoint.h"
#include "SimpleTrajectory.h"
#include "CompoundTrajectory.h"

TrajectoryBuilder::TrajectoryBuilder(const std::shared_ptr<Robot>& robot)
    : robot(robot)
    , m_solver(Interpolator::SolverType::QUINTIC)
    , m_velocityLimit(std::numeric_limits<double>::max())
    , m_accelerationLimit(std::numeric_limits<double>::max())
{
    if (robot == nullptr || !robot->isValid()) throw std::runtime_error("[TrajectoryBuilder] Can not handle invalid robots");
}

void TrajectoryBuilder::clear()
{

}

//void TrajectoryBuilder::addWaypoint(const Waypoint& waypoint)
//{
//    if (m_waypoints.empty()) m_inDg = waypoint.inDg; // Record units of the first waypoint (Used henceforth)
//
//    // Correct units of the new waypoint if they do not match
//    if (m_inDg != waypoint.inDg) m_waypoints.emplace_back(m_inDg ? waypoint.toDg() : waypoint.toRad());
//    else m_waypoints.emplace_back(waypoint);
//}
//
//void TrajectoryBuilder::insertWaypoint(uint32_t index, const Waypoint& waypoint)
//{
//    if (index >= m_waypoints.size()) {
//        addWaypoint(waypoint);
//        return;
//    }
//
//    if (m_waypoints[index] != waypoint) {
//        m_waypoints.insert(m_waypoints.begin() + index, waypoint);
//    }
//}
//
//void TrajectoryBuilder::replaceWaypoint(uint32_t index, const Waypoint& waypoint)
//{
//    if (index < m_waypoints.size()) {
//        m_waypoints[index] = waypoint;
//    } else throw std::runtime_error("[TrajectoryBuilder] Index out of bounds. Can not replace waypoint");
//}
//
//void TrajectoryBuilder::removeWaypoint(uint32_t index)
//{
//    if (index < m_waypoints.size()) {
//        m_waypoints.erase(m_waypoints.begin() + index);
//    } else throw std::runtime_error("[TrajectoryBuilder] Index out of bounds. Can not Remove waypoint");
//}

void TrajectoryBuilder::moveConstrained(const glm::dvec3& delta)
{
    if (m_waypoints.empty()) throw std::runtime_error("[TrajectoryBuilder] Start point for constrained motion undefined");


    Pose start = robot->getPose(m_waypoints.back());
    Waypoint finalWP = robot->inverse(start.translated(delta));

    if (!finalWP.isValid()) { // Verify that end position is reachable
        m_valid = false;
        return;
    }

    uint32_t last = m_waypoints.size() - 1;

    double distance = glm::length(delta);
    glm::dvec3 axis = delta / distance;

    distance /= 10;
    for (uint32_t i = 1; i < 10; i++) {
        Waypoint wp = robot->inverse(start.translated(i * distance * axis));
        if (wp.isValid()) m_waypoints.push_back(m_inDg ? wp.toDg() : wp);
        else std::cout << "F " << i << "\n";
    }

//    std::vector<Waypoint> remainder = { robot->inverse(start.translated(delta)) };
//
//    while (!remainder.empty()) {
//
//    }

}

// Determines what type of solver will be used to resolve trajectories. Only applies to features added after this call
void TrajectoryBuilder::setSolver(Interpolator::SolverType solver)
{
    m_solver = solver;
}

void TrajectoryBuilder::setVelocityLimit(double velocity)
{
    m_velocityLimit = std::abs(velocity);
    if (m_target != nullptr) m_target->setVelocityLimit(m_velocityLimit);
}
void TrajectoryBuilder::setAccelerationLimit(double acceleration)
{
    m_accelerationLimit = std::abs(acceleration);
    if (m_target != nullptr) m_target->setAccelerationLimit(m_accelerationLimit);
}

// Applies the prepared waypoints to create the trajectory
void TrajectoryBuilder::generate()
{
    if (m_waypoints.empty()) throw std::runtime_error("[TrajectoryBuilder] Can not generate trajectory. No waypoints");
//    m_target = std::make_shared<CompoundTrajectory>(m_waypoints, m_solver);
//    m_target->setVelocityLimit(m_velocityLimit);
//    m_target->setAccelerationLimit(m_accelerationLimit);
}

// TODO
// Calculates appropriate velocity/acceleration constraints to reduce stops between steps
// Repetitively solves the system to converge towards the best constraints for speed
//void TrajectoryBuilder::smooth(uint8_t iterations)
//{
//    if (m_target == nullptr || m_target->m_trajectories.empty()) return;
//
//    for (uint32_t i = 0; i < iterations; i++) {
//        std::cout << "Iteration: " << i << "~~~~~~~~~~~~~~~~~~~\n";
//        std::cout << "Initial: " << m_target->m_duration << " " << m_target->m_maxVelocity << " " << m_target->m_maxAcceleration << "\n";
//        iterateSmoothing();
//    }
//
//
//}
//
//void TrajectoryBuilder::iterateSmoothing()
//{
//    for (uint32_t i = 0; i < m_target->m_trajectories.size() - 1; i++) {
//        SimpleTrajectory& left = m_target->m_trajectories[i];
//        SimpleTrajectory& right = m_target->m_trajectories[i + 1];
//
//        // Get duration ratios - Shorter durations are prioritized
//        double dt = left.m_duration + right.m_duration, dr = left.m_duration / dt, dl = right.m_duration / dt;
//
//        std::cout << i << ": " << dt << " " << dl << " " << dr << " ===========================\n";
//        for (uint32_t j = 0; j < left.m_jointTrajectories.size(); j++) {
////            double velocity = lvr * left.m_jointTrajectories[j].delta()
//            double lv = m_velocityLimit * (1 - 2 * (left.m_jointTrajectories[j].delta() < 0)) * left.m_maxVelocity / left.m_jointTrajectories[j].maxVelocity() ;
//            double rv = m_velocityLimit * (1 - 2 * (right.m_jointTrajectories[j].delta() < 0)) * right.m_maxVelocity / right.m_jointTrajectories[j].maxVelocity();
//
//            double v = lv * dl + rv * dr;
//
//            std::cout << left.m_jointTrajectories[j].delta() << " " << right.m_jointTrajectories[j].delta() << " --> " << lv << " " << rv << " " << v << " | " << left.m_jointTrajectories[j].maxVelocity() << " " << right.m_jointTrajectories[j].maxVelocity() << "\n";
//
////            v += left.m_jointTrajectories[j].finalVelocity();
//            left.m_jointTrajectories[j].setFinalVelocity(v);
//            right.m_jointTrajectories[j].setInitialVelocity(v);
//
//            break;
//        }
//    }
//
//    m_target->updateDuration();
//}

bool TrajectoryBuilder::isValid() const
{
    return m_valid && m_target != nullptr;
}

const std::shared_ptr<CompoundTrajectory>& TrajectoryBuilder::finalize()
{
    if (m_target == nullptr) throw std::runtime_error("[TrajectoryBuilder] ");
    m_target->updateDuration();
    return m_target;
}

