//
// Created by Cam on 2025-03-15.
//

#ifndef AUTOCARVER_ARTICULATEDWRIST_H
#define AUTOCARVER_ARTICULATEDWRIST_H

#include "KinematicChain.h"

class ArticulatedWrist : public KinematicChain {
public:

    ArticulatedWrist(float d1, float l2, float l3, float d6);

    std::vector<float> invkin(const glm::vec3& position, const glm::quat& rotation) override;

private:
};


#endif //AUTOCARVER_ARTICULATEDWRIST_H
