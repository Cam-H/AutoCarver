//
// Created by cjhat on 2025-07-29.
//

#ifndef AUTOCARVER_COMPOSITETRAJECTORY_H
#define AUTOCARVER_COMPOSITETRAJECTORY_H

#include "Trajectory.h"

#include <memory>

class CompositeTrajectory : public Trajectory {
public:

    explicit CompositeTrajectory(uint32_t dof);

    void update() override;

    void addTrajectory(const std::shared_ptr<Trajectory>& trajectory);

    void clear();

    [[nodiscard]] Waypoint start() const override;
    [[nodiscard]] Waypoint end() const override;

    [[nodiscard]] std::vector<double> velocity(double t) const override;
    [[nodiscard]] std::vector<double> acceleration(double t) const override;

    [[nodiscard]] double maximumDelta() const override;

    [[nodiscard]] Waypoint evaluate(double t) const override;

    [[nodiscard]] bool validate(const std::shared_ptr<Robot>& robot, double dt) const override;

private:

    [[nodiscard]] std::tuple<uint32_t, double> transform(double t) const;

private:

    std::vector<std::shared_ptr<Trajectory>> m_trajectories;
    bool m_valid;
};


#endif //AUTOCARVER_COMPOSITETRAJECTORY_H
