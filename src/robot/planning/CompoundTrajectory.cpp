//
// Created by cjhat on 2025-07-26.
//

#include "CompoundTrajectory.h"

#include "Waypoint.h"
#include "SimpleTrajectory.h"

CompoundTrajectory::CompoundTrajectory(const std::vector<Waypoint>& waypoints, double velocityLimit, double accelerationLimit)
    : Trajectory()
    , m_waypoints(waypoints)
    , m_order(0)
    , m_delta(0)
{
    if (m_waypoints.empty()) return;

    m_velocityLimit = velocityLimit;
    m_accelerationLimit = accelerationLimit;

    // Ensure all waypoints are in the same units, and are of the same size
    m_inDg = waypoints[0].inDg;
    m_order = waypoints[0].values.size();

    for (Waypoint& waypoint : m_waypoints) {
        if (m_order != waypoint.values.size()) throw std::runtime_error("[CompoundTrajectory] Invalid waypoints. Sizes do not match");

        if (m_inDg != waypoint.inDg) {
            if (m_inDg) waypoint = waypoint.toDg();
            else waypoint = waypoint.toRad();
        }
    }

    CompoundTrajectory::updateDuration();
}

CompoundTrajectory::Step::Step(const Waypoint& waypoint, uint32_t order)
    : waypoint(waypoint)
    , duration(0)
    , vo(order)
    , dv(order)
    , dt(order)
{

}

void CompoundTrajectory::Step::synchronize()
{
    for (double t : dt) duration = std::max(duration, t);

    for (double& t : dt) t /= duration;
}

void CompoundTrajectory::Step::print() const
{
    std::cout << "Step (duration: " << duration << "s)\n" << waypoint.toString();

    std::cout << "\nInitial Joint Velocities: ";
    for (double val : vo) std::cout << val << " ";

    std::cout << "\nJoint Velocity Change: ";
    for (double val : dv) std::cout << val << " ";

    std::cout << "\nDuration: ";
    for (double val : dt) std::cout << val << " ";
    std::cout << "\n";

    std::cout << "End point: " << evaluate(1.0).toString() << "\n";

    std::cout << "=======================\n";
}


void CompoundTrajectory::topp()
{
    m_steps.clear();

    std::vector<std::vector<double>> deltas; // Differences between adjacent waypoints
    std::vector<std::vector<double>> profiles; // Maximum feasible velocity profile as evaluated with TOPP

    for (uint32_t i = 0; i < m_order; i++) {
        deltas.emplace_back(jointDelta(m_waypoints, i));
        profiles.emplace_back(idealVelocityProfile(deltas[i]));
    }

    for (uint32_t i = 0; i < m_waypoints.size() - 1; i++) {
        m_steps.emplace_back(m_waypoints[i], m_order);

        for (uint32_t j = 0; j < m_order; j++) {
            m_steps.back().vo[j] = profiles[j][i];
            m_steps.back().dv[j] = profiles[j][i + 1] - m_steps.back().vo[j];

            double vt = m_steps.back().vo[j] + profiles[j][i + 1];
            m_steps.back().dt[j] = vt == 0 ? 0 : 2 * deltas[j][i] / vt;
        }

        m_steps.back().synchronize();
    }

//    for (const Step& step : m_steps) step.print();

    // Record greatest delta for reference while convenient
    m_delta = 0;
    for (const auto& set : deltas)
        for (double val : set) m_delta = std::max(m_delta, std::abs(val));
}

std::vector<double> CompoundTrajectory::jointDelta(const std::vector<Waypoint>& waypoints, uint32_t idx) const
{
    std::vector<double> deltas(waypoints.size() - 1);
    for (uint32_t i = 0; i < waypoints.size() - 1; i++) {
        deltas[i] = waypoints[i + 1].values[idx] - waypoints[i].values[idx];
    }

    return deltas;
}

std::vector<double> CompoundTrajectory::idealVelocityProfile(const std::vector<double>& deltas) const
{
    std::vector<double> fwdMax(deltas.size() + 1), revMax(deltas.size() + 1);

    fwdMax[0] = 0;
    revMax.back() = 0;

    for (uint32_t i = 0; i < deltas.size(); i++) {
        fwdMax[i + 1] = feasibleMaxVelocity(fwdMax[i], deltas[i]);

        uint32_t idx = deltas.size() - i - 1;
        revMax[idx] = feasibleMaxVelocity(revMax[idx + 1], -deltas[idx]);
    }

    for (uint32_t i = 0; i < fwdMax.size(); i++) {
        if (std::abs(fwdMax[i]) > std::abs(revMax[i])) fwdMax[i] = revMax[i];
    }

    return fwdMax;
}

double CompoundTrajectory::feasibleMaxVelocity(double prevVelocity, double delta) const
{
    double accel = (1 - 2 * (delta < 0)) * m_accelerationLimit; // TODO functionalize acceleration to address discontinuities
    double val = prevVelocity * prevVelocity + 2 * delta * accel;
    if (val < 0) return std::clamp(-sqrt(-val), -m_velocityLimit, m_velocityLimit);
    else         return std::clamp( sqrt( val), -m_velocityLimit, m_velocityLimit);
}

double CompoundTrajectory::Step::maximumVelocity() const
{
    double max = 0;
    for (uint32_t i = 0; i < dt.size(); i++) {
        double val = dt[i] * (vo[i] + dv[i]);
        max = std::max(max, std::abs(val));
    }

    return max;
}
double CompoundTrajectory::Step::maximumAcceleration() const
{
    double max = 0;
    for (uint32_t i = 0; i < dt.size(); i++) {
        double val = dt[i] * (dv[i] / duration);
        max = std::max(max, std::abs(val));
    }

    return max;
}

void CompoundTrajectory::updateMaximums()
{
    topp();

    m_maxVelocity = m_maxAcceleration = 0;
    for (const Step& step : m_steps) {
        m_maxVelocity = std::max(m_maxVelocity, step.maximumVelocity());
        m_maxAcceleration = std::max(m_maxAcceleration, step.maximumAcceleration());
    }
}

void CompoundTrajectory::updateDuration()
{
    updateMaximums();

    m_minDuration = 0;
    for (const Step& step : m_steps) {
        m_minDuration += step.duration;
    }

    if (m_duration < m_minDuration) m_duration = m_minDuration;
}

Waypoint CompoundTrajectory::start() const
{
    if (m_waypoints.empty()) throw std::runtime_error("[CompoundTrajectory] There are no waypoints. Can not return start point");
    return m_waypoints[0];
}
Waypoint CompoundTrajectory::end() const
{
    if (m_waypoints.empty()) throw std::runtime_error("[CompoundTrajectory] There are no waypoints. Can not return end point");
    return m_waypoints.back();
}

std::vector<double> CompoundTrajectory::velocity(double t) const
{
    auto [index, subT] = stepKey(t);
    return m_steps[index].velocity(subT);
}

std::vector<double> CompoundTrajectory::Step::velocity(double t) const
{
    std::vector<double> velocities(vo.size());
    for (uint32_t i = 0; i < velocities.size(); i++) velocities[i] = vo[i] + dv[i] * t;
    return velocities;
}

std::vector<double> CompoundTrajectory::acceleration(double t) const
{
    auto [index, subT] = stepKey(t);
    return m_steps[index].acceleration(subT);
}

std::vector<double> CompoundTrajectory::Step::acceleration(double t) const
{
    std::vector<double> accelerations(dv.size());
    for (uint32_t i = 0; i < accelerations.size(); i++) accelerations[i] = dv[i] / duration;
    return accelerations;
}

double CompoundTrajectory::maximumDelta() const
{
    return m_delta;
}

Waypoint CompoundTrajectory::evaluate(double t) const
{
    auto [index, subT] = stepKey(t);
    return m_steps[index].evaluate(subT);
}

Waypoint CompoundTrajectory::Step::evaluate(double t) const
{
    Waypoint wp = waypoint;
    for (uint32_t i = 0; i < wp.values.size(); i++) {
        double rt = t * dt[i];
        wp.values[i] += (vo[i] + 0.5 * dv[i] * rt) * rt * duration;
    }

    return wp;
}

// Identify the appropriate section based on t, and transform to match
std::tuple<uint32_t, double> CompoundTrajectory::stepKey(double t) const
{
    if (m_steps.empty()) throw std::runtime_error("[CompoundTrajectory] Can not evaluate empty trajectory");
    else if (t >= 1 || m_duration == 0) return { m_steps.size() - 1, 1.0 };
    else if (t < 0) return { 0, 0.0 };

    double sum = 0, temp;
    for (const Step& step : m_steps) {
        temp = step.duration / m_duration;

        if (sum + temp < t) sum += temp;
        else return { &step - &m_steps[0], (t - sum) / temp };
    }

    return { m_waypoints.size() - 1, 1.0 };
}