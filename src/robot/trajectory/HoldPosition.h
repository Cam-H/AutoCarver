//
// Created by cjhat on 2025-08-13.
//

#ifndef AUTOCARVER_HOLDPOSITION_H
#define AUTOCARVER_HOLDPOSITION_H

#include "Trajectory.h"

class HoldPosition : public Trajectory {
public:

    HoldPosition(const Waypoint& position, double duration = 1.0);

    void update() override;

    [[nodiscard]] Waypoint start() const override;
    [[nodiscard]] Waypoint end() const override;

    [[nodiscard]] std::vector<double> velocity(double t) const override;
    [[nodiscard]] std::vector<double> acceleration(double t) const override;

    [[nodiscard]] double maximumDelta() const override;

    [[nodiscard]] Waypoint evaluate(double t) const override;

    [[nodiscard]] bool test(const Scene* scene, const std::shared_ptr<Robot>& robot, double dt) const override;


private:

    Waypoint m_position;

};


#endif //AUTOCARVER_HOLDPOSITION_H
