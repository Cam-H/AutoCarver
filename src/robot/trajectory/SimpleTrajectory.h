//
// Created by cjhat on 2025-07-26.
//

#ifndef AUTOCARVER_SIMPLETRAJECTORY_H
#define AUTOCARVER_SIMPLETRAJECTORY_H

#include "Trajectory.h"

#include "geometry/curves/Interpolator.h"
#include "Waypoint.h"

class SimpleTrajectory : public Trajectory {
public:

    SimpleTrajectory(const Waypoint& start, const Waypoint& end, Interpolator::SolverType solverType);

    void update() override;

    [[nodiscard]] Waypoint start() const override;
    [[nodiscard]] Waypoint end() const override;

    [[nodiscard]] std::vector<double> velocity(double t) const override;
    [[nodiscard]] std::vector<double> acceleration(double t) const override;

    [[nodiscard]] double maximumDelta() const override;

    [[nodiscard]] Waypoint evaluate(double t) const override;

private:

    std::vector<Interpolator> m_jointTrajectories;

};


#endif //AUTOCARVER_SIMPLETRAJECTORY_H
