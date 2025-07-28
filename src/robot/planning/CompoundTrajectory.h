//
// Created by cjhat on 2025-07-26.
//

#ifndef AUTOCARVER_COMPOUNDTRAJECTORY_H
#define AUTOCARVER_COMPOUNDTRAJECTORY_H

#include "Trajectory.h"

#include <vector>
#include <memory>

#include "Interpolator.h"

class Waypoint;

class CompoundTrajectory : public Trajectory {
public:

    CompoundTrajectory();
    CompoundTrajectory(const std::vector<Waypoint>& waypoints);

    void smooth(uint8_t iterations = 8);

    void enableJumps(bool enable);

    void addWaypoint(const Waypoint& waypoint);
    void insertWaypoint(uint32_t index, const Waypoint& waypoint);
    void replaceWaypoint(uint32_t index, const Waypoint& waypoint);

    void removeWaypoint(uint32_t index);

    [[nodiscard]] Waypoint start() const override;
    [[nodiscard]] Waypoint end() const override;

    [[nodiscard]] std::vector<double> velocity(double t) const override;
    [[nodiscard]] std::vector<double> acceleration(double t) const override;

    [[nodiscard]] double maximumDelta() const override;

    [[nodiscard]] Waypoint evaluate(double t) const override;

protected:

    void calculateMaximums();
    void calculateDuration() override;

private:

    std::shared_ptr<Trajectory> createTrajectory(const Waypoint& start, const Waypoint& end);

    void addContribution(const std::shared_ptr<Trajectory>& trajectory);

    void resolveDiscontinuities();
    void resolveDiscontinuities(uint32_t index);

    std::tuple<uint32_t, double> transform(double t) const;

private:

    std::vector<std::shared_ptr<Trajectory>> m_trajectories;
    std::vector<Waypoint> m_freeWaypoints;

    bool m_allowDiscontinuities;

    Interpolator::SolverType m_solver;

};


#endif //AUTOCARVER_COMPOUNDTRAJECTORY_H
