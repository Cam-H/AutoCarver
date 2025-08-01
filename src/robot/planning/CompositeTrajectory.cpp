////
//// Created by cjhat on 2025-07-29.
////
//
//#include "CompositeTrajectory.h"
//
//CompositeTrajectory::CompositeTrajectory()
//        : Trajectory()
//{
//
//}
//
//void CompositeTrajectory::updateMaximums()
//{
//    for (const std::shared_ptr<Trajectory>& traj : m_trajectories) {
//        traj->limitVelocity(m_velocityLimit);
//        traj->limitAcceleration(m_accelerationLimit);
//    }
//
//    m_maxVelocity = 0, m_maxAcceleration = 0;
//    for (const std::shared_ptr<Trajectory>& traj : m_trajectories) {
//        m_maxVelocity = std::max(m_maxVelocity, traj->maximumVelocity());
//        m_maxAcceleration = std::max(m_maxAcceleration, traj->maximumAcceleration());
//    }
//}
//
//void CompositeTrajectory::updateDuration()
//{
//    updateMaximums();
//
//    m_minDuration = 0, m_duration = 0;
//    for (const std::shared_ptr<Trajectory>& traj : m_trajectories) {
//        m_minDuration += traj->minimumDuration();
//        m_duration += traj->duration();
//    }
//}
//
//Waypoint CompositeTrajectory::start() const
//{
//    if (m_trajectories.empty()) throw std::runtime_error("[CompositeTrajectory] There are no trajectories. Can not return start point");
//    return m_trajectories[0]->start();
//}
//Waypoint CompositeTrajectory::end() const
//{
//    if (m_trajectories.empty()) throw std::runtime_error("[CompositeTrajectory] There are no trajectories. Can not return end point");
//    return m_trajectories.back()->end();
//}
//
//std::vector<double> CompositeTrajectory::velocity(double t) const
//{
//    auto [index, subT] = transform(t);
//    return m_trajectories[index]->velocity(subT);
//}
//std::vector<double> CompositeTrajectory::acceleration(double t) const
//{
//    auto [index, subT] = transform(t);
//    return m_trajectories[index]->acceleration(subT);
//}
//
//double CompositeTrajectory::maximumDelta() const
//{
//    double max = 0;
//    for (const std::shared_ptr<Trajectory>& traj : m_trajectories) max = std::max(max, traj->maximumDelta());
//    return max;
//}
//
//Waypoint CompositeTrajectory::evaluate(double t) const
//{
//    auto [index, subT] = transform(t);
//    return m_trajectories[index]->evaluate(subT);
//}
//
//// Identify the appropriate section based on t, and transform to match
//std::tuple<uint32_t, double> CompositeTrajectory::transform(double t) const
//{
//    if (m_trajectories.empty()) throw std::runtime_error("[CompositeTrajectory] Can not evaluate empty trajectory");
//    else if (t >= 1 || m_duration == 0) return { m_trajectories.size() - 1, 1.0 };
//    else if (t < 0) return { 0, 0.0 };
//
//    double sum = 0, temp;
//    for (const std::shared_ptr<Trajectory>& traj : m_trajectories) {
//        temp = traj->duration() / m_duration;
//
//        if (sum + temp < t) sum += temp;
//        else return { &traj - &m_trajectories[0], (t - sum) / temp };
//    }
//
//    return { m_trajectories.size() - 1, 1.0 };
//}