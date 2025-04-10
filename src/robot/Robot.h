//
// Created by Cam on 2025-03-20.
//

#ifndef AUTOCARVER_ROBOT_H
#define AUTOCARVER_ROBOT_H

#include <vector>

#include "KinematicChain.h"
#include "geometry/Body.h"

class Robot {
public:

    Robot(KinematicChain* kinematics);

    void prepareLinks(rp3d::PhysicsCommon *phys, rp3d::PhysicsWorld *world);

    void update();

    void setJointValue(uint32_t idx, float value);
    void setJointValueDg(uint32_t idx, float value);

    void moveTo(const glm::vec3& position, const glm::vec3& euler = {0.0f, 1.0f, 0.0f});

    const std::vector<std::shared_ptr<Body>>& links();

    float getJointValue(uint32_t idx);
    float getJointValueDg(uint32_t idx);

    [[nodiscard]] glm::vec3 getEOATPosition() const;
    [[nodiscard]] glm::vec3 getEOATEuler() const;


protected:

    void updateTransforms();

private:

    KinematicChain* m_kinematics;
    glm::mat4 m_transform;

    // Maps to the joints of the kinematic chain
    std::vector<std::shared_ptr<Body>> m_links;
};


#endif //AUTOCARVER_ROBOT_H
