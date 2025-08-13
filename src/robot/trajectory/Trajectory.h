//
// Created by Cam on 2025-04-16.
//

#ifndef AUTOCARVER_TRAJECTORY_H
#define AUTOCARVER_TRAJECTORY_H

#include <vector>
#include <memory>

#include "Waypoint.h"

#include "robot/Robot.h"

class Scene;

class Trajectory {
public:

    explicit Trajectory(uint32_t dof);

    void restart();
    void finish();

    virtual void update() = 0;

    void setDuration(double duration);

    void resetLimits();

    void setVelocityLimits(const std::vector<double>& vLims);
    void setAccelerationLimits(const std::vector<double>& aLims);
    void setLimits(const std::vector<double>& vLims, const std::vector<double>& aLims);

    void limitVelocity(const std::vector<double>& vLims);
    void limitAcceleration(const std::vector<double>& vLims);
    void limit(const std::vector<double>& vLims, const std::vector<double>& aLims);

    void setStep(double tStep);

    [[nodiscard]] std::vector<double> velocityLimits() const;
    [[nodiscard]] std::vector<double> accelerationLimits() const;

    [[nodiscard]] double maximumVelocity() const;
    [[nodiscard]] double maximumAcceleration() const;

    [[nodiscard]] virtual double maximumDelta() const = 0;

    [[nodiscard]] double duration() const;
    [[nodiscard]] double minimumDuration() const;

    [[nodiscard]] double t() const;
    [[nodiscard]] double tStep() const;

    [[nodiscard]] bool complete() const;

    [[nodiscard]] virtual Waypoint start() const = 0;
    [[nodiscard]] virtual Waypoint end() const = 0;

    [[nodiscard]] virtual std::vector<double> velocity() const;
    [[nodiscard]] virtual std::vector<double> velocity(double t) const = 0;

    [[nodiscard]] virtual std::vector<double> acceleration() const;
    [[nodiscard]] virtual std::vector<double> acceleration(double t) const = 0;

    [[nodiscard]] Waypoint next();
    [[nodiscard]] Waypoint timestep(double delta);
    [[nodiscard]] virtual Waypoint evaluate(double t) const = 0;

    // Verify that every position of the trajectory is reachable by the specified robot. Tests 1/dt equally spaced positions
    [[nodiscard]] virtual bool validate(const std::shared_ptr<Robot>& robot, double dt) const;
    [[nodiscard]] bool test(const Scene* scene, const std::shared_ptr<Robot>& robot, double dt) const;// TODO Also: scene collisions

private:

    static void assignLimits(std::vector<double>& limits, const std::vector<double>& newLimits);
    static bool applyLimits(std::vector<double>& limits, const std::vector<double>& additionalLimits);

protected:

    uint32_t m_dof;

    double m_t;
    double m_tStep;

    double m_duration;
    double m_minDuration;

    // Limits to be compared against maximums in the interval to calculate required duration
    std::vector<double> m_velocityLimits, m_accelerationLimits;

    double m_maxVelocity, m_maxAcceleration;

    bool m_inDg;

};

#endif //AUTOCARVER_TRAJECTORY_H
