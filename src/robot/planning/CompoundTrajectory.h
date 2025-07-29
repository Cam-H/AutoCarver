//
// Created by cjhat on 2025-07-26.
//

#ifndef AUTOCARVER_COMPOUNDTRAJECTORY_H
#define AUTOCARVER_COMPOUNDTRAJECTORY_H

#include "Trajectory.h"

#include <vector>
#include <memory>

class Waypoint;

class CompoundTrajectory : public Trajectory {
public:

    explicit CompoundTrajectory(const std::vector<Waypoint>& waypoints, double velocityLimit, double accelerationLimit);

    void updateDuration() override;

    [[nodiscard]] Waypoint start() const override;
    [[nodiscard]] Waypoint end() const override;

    [[nodiscard]] std::vector<double> velocity(double t) const override;
    [[nodiscard]] std::vector<double> acceleration(double t) const override;

    [[nodiscard]] double maximumDelta() const override;

    [[nodiscard]] Waypoint evaluate(double t) const override;

protected:

    void updateMaximums() override;

private:

    class Step {
    public:

        Step(const Waypoint& waypoint, uint32_t order);

        void synchronize();
        void print() const;

        [[nodiscard]] std::vector<double> velocity(double t) const;
        [[nodiscard]] std::vector<double> acceleration(double t) const;

        [[nodiscard]] Waypoint evaluate(double t) const;

        [[nodiscard]] double maximumVelocity() const;
        [[nodiscard]] double maximumAcceleration() const;

        const Waypoint& waypoint;

        double duration;
        std::vector<double> vo, dv, dt;
    };

    void topp();

    [[nodiscard]] std::vector<double> jointDelta(const std::vector<Waypoint>& waypoints, uint32_t idx) const;
    [[nodiscard]] std::vector<double> idealVelocityProfile(const std::vector<double>& deltas) const;
    [[nodiscard]] double feasibleMaxVelocity(double prevVelocity, double delta) const;

    [[nodiscard]] std::tuple<uint32_t, double> stepKey(double t) const;

private:

    std::vector<Waypoint> m_waypoints;

    uint32_t m_order;
    double m_delta;

    std::vector<Step> m_steps;

};


#endif //AUTOCARVER_COMPOUNDTRAJECTORY_H
