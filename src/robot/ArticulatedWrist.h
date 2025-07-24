//
// Created by Cam on 2025-03-15.
//

#ifndef AUTOCARVER_ARTICULATEDWRIST_H
#define AUTOCARVER_ARTICULATEDWRIST_H

#include "KinematicChain.h"

class ArticulatedWrist : public KinematicChain {
public:

    ArticulatedWrist(double d1, double l2, double l3, double d6);

    std::vector<double> invkin(const glm::dvec3& position, const glm::dquat& rotation) override;

private:
};


#endif //AUTOCARVER_ARTICULATEDWRIST_H
