//
// Created by cjhat on 2025-08-01.
//

#ifndef AUTOCARVER_CARTESIANTRAJECTORY_H
#define AUTOCARVER_CARTESIANTRAJECTORY_H

#include "TOPPTrajectory.h"

#include <glm.hpp>
#include <memory>

#include "robot/Pose.h"
#include "geometry/primitives/Ray.h"

class Robot;

class CartesianTrajectory : public TOPPTrajectory {
public:
    CartesianTrajectory(const std::shared_ptr<Robot>& robot, const Pose& initialPose, const glm::dvec3& translation);

    [[nodiscard]] std::shared_ptr<CartesianTrajectory> reversed(const std::shared_ptr<Robot>& robot) const;

//    [[nodiscard]] double locate(double ratio) const;

    [[nodiscard]] bool isValid() const override;

private:

    static std::vector<Waypoint> cartesianWaypoints(const std::shared_ptr<Robot>& robot,
                                                    const Pose& initialPose, const glm::dvec3& translation);


private:

    std::shared_ptr<Robot> m_robot; // Robot for which this trajectory was prepared
    Pose m_initialPose;

    Ray m_curve;
    double m_distance;

};


#endif //AUTOCARVER_CARTESIANTRAJECTORY_H
