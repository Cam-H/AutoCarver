//
// Created by cjhat on 2025-07-27.
//

#include "TrajectoryBuilder.h"

#include "robot/Robot.h"

#include "Waypoint.h"
#include "SimpleTrajectory.h"
#include "TOPPTrajectory.h"

#include "geometry/primitives/Ray.h"

TrajectoryBuilder::TrajectoryBuilder(const std::shared_ptr<Robot>& robot)
    : robot(robot)
    , m_pose(robot != nullptr ? robot->getPose() : Pose(glm::dvec3(), Axis3D()))
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

std::shared_ptr<TOPPTrajectory> TrajectoryBuilder::cartesianMotion(const glm::dvec3& delta)
{

    std::cout << "CM " << delta.x << " " << delta.y << " " << delta.z << "\n";


    std::vector<Waypoint> waypoints = { robot->inverse(m_pose).toDg() };
    std::cout << "Original: " << robot->getWaypoint().toDg() << "\nTO: " << waypoints[0].toDg() << "\n";
    // Identify the final waypoint, and verify that it is reachable
    std::vector<Pose> poses = { m_pose };
    poses.back().localTranslate(delta);

    std::vector<Waypoint> remainder = { robot->inverse(poses.back()).toDg() };

    m_pose.print();
    poses.back().print();

    if (waypoints[0].isValid() && remainder[0].isValid()) {

        double distance = glm::length(delta), step = 0.5 * distance;
        Ray ray(m_pose.position, delta / distance);

        int limit = 0;

        std::cout << "START: " << waypoints[0].toDg() << "\nEND: " << remainder[0].toDg() << "\n";

//        auto traj = std::make_shared<SimpleTrajectory>(waypoints[0], remainder[0], Interpolator::SolverType::QUINTIC);
//        traj->setVelocityLimit(m_velocityLimit);
//        traj->setAccelerationLimit(m_accelerationLimit);
//        return traj;

//        for (uint32_t i = 0; i < 10; i++) {
//            m_pose.localTranslate(ray.axis * (distance / 10));
//            waypoints.emplace_back(robot->inverse(m_pose).toDg());
//            if (!waypoints.back().isValid()) waypoints.pop_back();
//        }

        waypoints.emplace_back(remainder[0]);
        std::cout << "============= SIZE: " << waypoints.size() << "\n";

//        return std::make_shared<TOPPTrajectory>(std::vector<Waypoint>{ waypoints[0].toDg(), remainder[0].toDg() }, m_velocityLimit, m_accelerationLimit);
        return std::make_shared<TOPPTrajectory>(std::vector<Waypoint>{waypoints }, m_velocityLimit, m_accelerationLimit);

        while (!remainder.empty()) {
            Waypoint wp = Waypoint::midpoint(waypoints.back(), remainder.back());
            Pose pose = robot->getPose(wp);

            std::cout << "===> " << waypoints.size() << " " << remainder.size() << " | "<< ray.squareDistance(pose.position) << " " << glm::dot(ray.axis, pose.position - m_pose.position)
                      << " [" << glm::dot(pose.axes.xAxis, m_pose.axes.xAxis) << " " << glm::dot(pose.axes.yAxis, m_pose.axes.yAxis) << " " << glm::dot(pose.axes.zAxis, m_pose.axes.zAxis) << "]\n";

            std::cout << "CUR: " << wp << "\n";

            if (ray.squareDistance(pose.position) < 1e-3 && m_pose.oriented(pose, 1e-2)) {
                std::cout << "X\n";
                waypoints.emplace_back(remainder.back());
                poses.pop_back();
                remainder.pop_back();
                step *= 2.0;
            } else {
                poses.emplace_back(poses.back());
                poses.back().globalTranslate(-ray.axis * step);
                std::cout << "O " << step << " " << glm::dot(ray.axis, poses.back().position - m_pose.position) << "\n";
                remainder.emplace_back(robot->inverse(poses.back()));
                step *= 0.5;
            }

            if (limit++ > 100) break;
        }

    }


    if (remainder.empty()) return std::make_shared<TOPPTrajectory>(waypoints, m_velocityLimit, m_accelerationLimit);
    else return nullptr;
}

void TrajectoryBuilder::setPose(const Pose& pose)
{
    m_pose = pose;
}

// Determines what type of solver will be used to resolve trajectories. Only applies to features added after this call
void TrajectoryBuilder::setSolver(Interpolator::SolverType solver)
{
    m_solver = solver;
}

void TrajectoryBuilder::setVelocityLimit(double velocity)
{
    m_velocityLimit = std::abs(velocity);
}
void TrajectoryBuilder::setAccelerationLimit(double acceleration)
{
    m_accelerationLimit = std::abs(acceleration);
}