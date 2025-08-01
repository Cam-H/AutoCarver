//
// Created by cjhat on 2025-07-26.
//

#ifndef AUTOCARVER_TOPPTRAJECTORY_H
#define AUTOCARVER_TOPPTRAJECTORY_H

#include "Trajectory.h"

#include <vector>
#include <memory>

#include "Waypoint.h"
#include "PiecewisePolyPath.h"

class TOPPTrajectory : public Trajectory {
public:

    TOPPTrajectory(const std::vector<Waypoint>& waypoints, double velocityLimit, double accelerationLimit);
    TOPPTrajectory(const std::vector<Waypoint>& waypoints, const std::vector<double>& vLims, const std::vector<double>& aLims);

    void update() override;

    [[nodiscard]] Waypoint start() const override;
    [[nodiscard]] Waypoint end() const override;

    [[nodiscard]] std::vector<double> velocity(double t) const override;
    [[nodiscard]] std::vector<double> acceleration(double t) const override;

    [[nodiscard]] double maximumDelta() const override;

    [[nodiscard]] Waypoint evaluate(double t) const override;

private:

    [[nodiscard]] std::tuple<double, double> tToS(double t) const;

private:

    PiecewisePolyPath m_path;

    double m_ds;
    std::vector<double> m_t;
};


#endif //AUTOCARVER_TOPPTRAJECTORY_H
