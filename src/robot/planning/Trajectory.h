//
// Created by Cam on 2025-04-16.
//

#ifndef AUTOCARVER_TRAJECTORY_H
#define AUTOCARVER_TRAJECTORY_H

#include <vector>

#include "Waypoint.h"


class Trajectory {
public:

    Trajectory();

    void restart();

    virtual void update() = 0;

    void setDuration(double duration);

    void setVelocityLimits(const std::vector<double>& vLims);
    void setAccelerationLimits(const std::vector<double>& aLims);

    inline void setVelocityLimit(double velocity) { setVelocityLimits(std::vector<double>(m_velocityLimits.size(), velocity)); }
    inline void setAccelerationLimit(double acceleration) { setAccelerationLimits(std::vector<double>(m_accelerationLimits.size(), acceleration)); }

    void limitVelocity(double velocity);
    void limitAcceleration(double acceleration);

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


protected:

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
