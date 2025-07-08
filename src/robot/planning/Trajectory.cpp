//
// Created by Cam on 2025-04-16.
//

#include "Trajectory.h"

#include "../Robot.h"

std::ostream& operator<<(std::ostream& stream, const Waypoint& waypoint)
{
    stream << "{ ";
    for (float value : waypoint.values) {
    stream << value << ", ";
    }

    stream << "[" << waypoint.toRad << "] }";
    return stream;
}

JointTrajectory::JointTrajectory(float start, float end)
    : JointTrajectory(std::vector<float>{ start, end })
{

}

JointTrajectory::JointTrajectory(const std::vector<float>& waypoints, TrajectorySolverType solver)
    : m_solver(solver)
    , m_waypoints(waypoints)
{
    updateCoefficients();
}

void JointTrajectory::updateCoefficients()
{
    switch (m_solver) {
        case TrajectorySolverType::LINEAR:
            prepareLinearCoefficients();
            break;
        case TrajectorySolverType::CUBIC:
            prepareCubicCoefficients();
            break;
        case TrajectorySolverType::QUINTIC:
            prepareQuinticCoefficients();
            break;
    }
}

void JointTrajectory::prepareLinearCoefficients()
{
    m_coeffs = std::vector<float>();
    m_coeffs.reserve(2 * m_waypoints.size());

    for (uint32_t i = 0; i < m_waypoints.size() - 1; i++) {
        glm::vec2 coeffs = glm::inverse(glm::mat2x2{
                1, 0,
                i, 1
        }) * glm::vec2{ m_waypoints[i], m_waypoints[i + 1]};

        m_coeffs.emplace_back(coeffs.x);
        m_coeffs.emplace_back(coeffs.y);
    }
}
void JointTrajectory::prepareCubicCoefficients()
{
    m_coeffs = std::vector<float>();
    m_coeffs.reserve(4 * m_waypoints.size());

    for (uint32_t i = 0; i < m_waypoints.size() - 1; i++) {
        float t0 = (float)i, tf = t0 + 1;
        glm::vec4 coeffs = glm::inverse(glm::mat4x4{
                1, 0, 1, 0,
                t0, 1, tf, 1,
                t0*t0, 2*t0, tf*tf, 2*tf,
                t0*t0*t0, 3*t0*t0, tf*tf*tf, 3*tf*tf

        }) * glm::vec4(m_waypoints[i], 0, m_waypoints[i + 1], 0); // TODO smooth velocity endpoints

        m_coeffs.emplace_back(coeffs.x);
        m_coeffs.emplace_back(coeffs.y);
        m_coeffs.emplace_back(coeffs.z);
        m_coeffs.emplace_back(coeffs.w);
    }
}
void JointTrajectory::prepareQuinticCoefficients()
{
    m_coeffs = std::vector<float>();
    m_coeffs.reserve(6 * m_waypoints.size());

    for (uint32_t i = 0; i < m_waypoints.size() - 1; i++) {
//        float t0 = (float)i, tf = t0 + 1;

        // TODO quintic coefficient calculation (Difficult because glm does not support 6x6 matrices
        for (int j = 0; j < 6; j++) m_coeffs.emplace_back(0);
    }

    std::cerr << "Quintic trajectory generation incomplete!\n";
}

float JointTrajectory::position(uint32_t step, float t) const
{
    switch (m_solver) {
        case TrajectorySolverType::LINEAR:
            return m_coeffs[2 * step] + t * m_coeffs[2 * step + 1];
        case TrajectorySolverType::CUBIC:
            return m_coeffs[4 * step] + t * m_coeffs[4 * step + 1] + t*t * m_coeffs[4 * step + 2] + t*t*t * m_coeffs[4 * step + 3];
        case TrajectorySolverType::QUINTIC:
            return m_coeffs[6 * step] + t * m_coeffs[6 * step + 1] + t*t * m_coeffs[6 * step + 2]
            + t*t*t * m_coeffs[6 * step + 3] + t*t*t*t * m_coeffs[6 * step + 4] + t*t*t*t*t * m_coeffs[6 * step + 5];
    }

    return 0;
}

float JointTrajectory::velocity(uint32_t step, float t) const
{
    switch (m_solver) {
        case TrajectorySolverType::LINEAR:
            return m_waypoints[step + 1] - m_waypoints[step];
        case TrajectorySolverType::CUBIC:
            return m_coeffs[4 * step + 1] + 2*t * m_coeffs[4 * step + 2] + 3*t*t * m_coeffs[4 * step + 3];
        case TrajectorySolverType::QUINTIC:
            return m_coeffs[6 * step + 1] + 2*t * m_coeffs[6 * step + 2]
            + 3*t*t * m_coeffs[6 * step + 3] + 4*t*t*t * m_coeffs[6 * step + 4] + 5*t*t*t*t * m_coeffs[6 * step + 5];
    }

    return 0;
}
float JointTrajectory::acceleration(uint32_t step, float t) const
{
    switch (m_solver) {
        case TrajectorySolverType::LINEAR:
            return 0;
        case TrajectorySolverType::CUBIC:
            return 1000.0f * (velocity(step, t + 0.001f) - velocity(step, t));
        case TrajectorySolverType::QUINTIC:
            return 2 * m_coeffs[6 * step + 2]
            + 6*t * m_coeffs[6 * step + 3] + 12*t*t * m_coeffs[6 * step + 4] + 20*t*t*t * m_coeffs[6 * step + 5];
    }

    return 0;
}

float JointTrajectory::maxVelocity() const
{
    float max = 0;
    for (uint32_t i = 0; i < m_waypoints.size() - 1; i++) {
        float test = maxVelocity(i);
        if (std::abs(max) < std::abs(test)) max = test;
    }

    return max;
}
float JointTrajectory::maxAcceleration() const
{
    float max = 0;
    for (uint32_t i = 0; i < m_waypoints.size() - 1; i++) {
        float test = maxAcceleration(i);
        if (std::abs(max) < std::abs(test)) max = test;
    }

    return max;
}

float JointTrajectory::maxVelocity(uint32_t step) const
{
    return velocity(step, (float)step + 0.5f);
}
float JointTrajectory::maxAcceleration(uint32_t step) const
{
    return 0; // TODO
}

std::vector<float> JointTrajectory::pTrajectory(float tStep) const
{
    return trajectory(
            std::bind(&JointTrajectory::position, this, std::placeholders::_1, std::placeholders::_2), tStep);
}
std::vector<float> JointTrajectory::vTrajectory(float tStep) const
{
    return trajectory(
            std::bind(&JointTrajectory::velocity, this, std::placeholders::_1, std::placeholders::_2), tStep);
}
std::vector<float> JointTrajectory::aTrajectory(float tStep) const
{
    return trajectory(
            std::bind(&JointTrajectory::acceleration, this, std::placeholders::_1, std::placeholders::_2), tStep);
}

std::vector<float> JointTrajectory::trajectory(const std::function<float (uint32_t, float)>& func, float tStep) const
{
    std::vector<float> values;

    int per = (int)(1 / tStep) + 1, count = (m_waypoints.size() - 1) * per;
    values.reserve(count);

    for (uint32_t i = 0; i < m_waypoints.size() - 1; i++) {
        float t = i;

        for (uint32_t j = 0; j < per; j++) {
            values.emplace_back(func(i, t));
            t += tStep;
        }

        values.emplace_back(func(i, i + 1));
    }

    return values;
}

std::vector<float> JointTrajectory::t(float tStep) const
{
    std::vector<float> values;
    int per = (int)(1 / tStep) + 1, count = (m_waypoints.size() - 1) * per;
    values.reserve(count);

    for (uint32_t i = 0; i < m_waypoints.size() - 1; i++) {
        float val = i;

        for (uint32_t j = 0; j < per; j++) {
            values.emplace_back(val);
            val += tStep;
        }

        values.emplace_back(i + 1);
    }

    return values;
}

Trajectory::Trajectory(const Waypoint& start, const Waypoint& end, TrajectorySolverType solverType)
    : Trajectory({ start, end }, solverType)
{

}

Trajectory::Trajectory(const std::vector<Waypoint>& waypoints, TrajectorySolverType solverType)
    : m_waypoints(waypoints)
    , m_solver(solverType)
    , m_jointCount(!waypoints.empty() ? waypoints[0].values.size() : 0)
    , m_t(0.0f)
    , m_tStep(0.05f)
    , m_maxVelocity(0)
    , m_maxAcceleration(0)
    , m_duration(1.0f)
{
    if (m_waypoints.size() > 1) initialize();
}

void Trajectory::initialize()
{
    for (uint32_t i = 0; i < m_jointCount; i++) {
        std::vector<float> jwp;
        jwp.reserve(m_waypoints.size());

        for (const Waypoint& waypoint : m_waypoints) jwp.emplace_back(waypoint.values[i]);

        m_jointTrajectories.emplace_back(jwp, m_solver);
    }

    if (m_maxVelocity != 0.0f) calculateDuration();
}

void Trajectory::insertWaypoint(uint32_t idx, const Waypoint& waypoint)
{
    idx = (idx >= m_waypoints.size() ? m_waypoints.size() - 1 : idx); // Safe because m_waypoints minimum size is 1
    m_waypoints.insert(m_waypoints.begin() + idx, waypoint);

    m_jointTrajectories.clear();
    initialize();
}

void Trajectory::setMaxVelocity(float velocity)
{
    m_maxVelocity = std::abs(velocity);
    calculateDuration();
}

void Trajectory::calculateDuration()
{
    if (m_jointTrajectories.empty()) return; // Skip calculation if called before trajectories are generated

    float maxVelocity = 0;
    for (const auto& jt : m_jointTrajectories) {
        float velocity = jt.maxVelocity();
        if (maxVelocity < velocity) maxVelocity = velocity;
    }

    m_duration = maxVelocity / m_maxVelocity; // Duration in seconds

    std::cout << "Duration: " << m_duration << " | " << maxVelocity << " " << m_maxVelocity << "\n";
    m_duration = 1.0f / m_duration; // Inverse to use as a multiplier
//    m_duration =
//    switch (m_solver) {
//        case TrajectorySolverType::LINEAR:
//
//            break;
//        case TrajectorySolverType::CUBIC:
//            break;
//        case TrajectorySolverType::QUINTIC:
//            break;
//    }
}

uint32_t Trajectory::waypointCount() const
{
    return m_waypoints.size();
}

uint32_t Trajectory::dimensions() const
{
    return m_jointCount;
}

float Trajectory::tStep() const
{
    return m_tStep;
}
float Trajectory::t() const
{
    return m_t;
}

bool Trajectory::complete() const
{
    return m_t - (float)m_waypoints.size() + 1 > m_tStep;
}

Waypoint Trajectory::start()
{
    return m_waypoints[0];
}
Waypoint Trajectory::end()
{
    return m_waypoints[m_waypoints.size() - 1];
}

Waypoint Trajectory::next()
{
    auto wp =  evaluate(m_t);
    m_t += m_tStep;

    return wp;
}

Waypoint Trajectory::timestep(float delta)
{
    // Default to next when no max velocity has been set
    if (m_maxVelocity == 0.0f) return next();

    return evaluate(m_t += delta * m_duration);
}

Waypoint Trajectory::evaluate(float t) const
{
    if (t >= (float)m_waypoints.size() - 1) return m_waypoints[m_waypoints.size() - 1];
    else if (t < 0) return m_waypoints[0];

    uint32_t idx = std::floor(t);
    if (std::abs(t - (float)idx) < 1e-6) return m_waypoints[idx];

    std::vector<float> values;
    values.reserve(m_waypoints[0].values.size());

    for (const JointTrajectory& jt : m_jointTrajectories) values.emplace_back(jt.position(idx, t));

    return { values, m_waypoints[0].toRad };
}

const JointTrajectory& Trajectory::jointTrajectory(uint32_t idx)
{
    if (idx < m_jointTrajectories.size()) return m_jointTrajectories[idx];
    return NULL_TRAJECTORY;
}