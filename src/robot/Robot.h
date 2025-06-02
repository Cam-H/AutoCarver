//
// Created by Cam on 2025-03-20.
//

#ifndef AUTOCARVER_ROBOT_H
#define AUTOCARVER_ROBOT_H

#include <vector>

#include "KinematicChain.h"
#include "geometry/RigidBody.h"

#include "geometry/Transformable.h"
#include "planning/Trajectory.h"

class Robot : public Transformable { // TODO make serializable
public:

    Robot(KinematicChain* kinematics);

    void prepareLinks();

    void step();
    void update();

    void setJointValue(uint32_t idx, float value);
    void setJointValueDg(uint32_t idx, float value);

    void moveTo(const glm::vec3& position, const glm::vec3& euler = {0.0f, 1.0f, 0.0f});
    void moveTo(const Waypoint& waypoint);

    void traverse(const std::shared_ptr<Trajectory>& trajectory);

    const std::vector<std::shared_ptr<RigidBody>>& links();

    float getJointValue(uint32_t idx);
    float getJointValueDg(uint32_t idx);

    [[nodiscard]] glm::vec3 getEOATPosition() const;
    [[nodiscard]] glm::vec3 getEOATEuler() const;

    bool inTransit();


protected:

    void updateTransforms();

private:

    KinematicChain* m_kinematics;

    // Maps to the joints of the kinematic chain
    std::vector<std::shared_ptr<RigidBody>> m_links;

    //
    std::shared_ptr<Trajectory> m_currentTrajectory;

};


#endif //AUTOCARVER_ROBOT_H
