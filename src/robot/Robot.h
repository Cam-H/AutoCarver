//
// Created by Cam on 2025-03-20.
//

#ifndef AUTOCARVER_ROBOT_H
#define AUTOCARVER_ROBOT_H

#include <vector>

#include "KinematicChain.h"
#include "physics/RigidBody.h"

#include "geometry/Transformable.h"
#include "planning/Trajectory.h"

class Robot : public Transformable { // TODO make serializable
public:

    Robot(const std::shared_ptr<KinematicChain>& kinematics, const std::shared_ptr<RigidBody>& eoat = nullptr);

    void prepareLinks();

    void step();
    void step(float delta);

    void update();

    void setEOAT(const std::shared_ptr<RigidBody>& eoat, bool preserveTransform = true);

    void setJointValue(uint32_t idx, float value);
    void setJointValueDg(uint32_t idx, float value);

    void moveTo(const glm::vec3& position, const glm::vec3& euler = {0.0f, 1.0f, 0.0f});
    void moveTo(const Waypoint& waypoint);

    void traverse(const std::shared_ptr<Trajectory>& trajectory);

    const std::vector<std::shared_ptr<RigidBody>>& links();

    float getJointValue(uint32_t idx);
    float getJointValueDg(uint32_t idx);

    Waypoint getWaypoint() const;

    [[nodiscard]] const glm::mat4x4& getEOATTransform() const;
    [[nodiscard]] glm::vec3 getEOATPosition() const;
    [[nodiscard]] glm::vec3 getEOATEuler() const;

    bool inTransit();

    [[nodiscard]] Waypoint inverse(const glm::vec3& position, const Axis3D& axes) const;
    [[nodiscard]] Waypoint inverse(const glm::vec3& position, const glm::vec3& euler) const;


protected:

    void updateTransforms();

private:

    std::shared_ptr<KinematicChain> m_kinematics;

    // Maps to the joints of the kinematic chain
    std::vector<std::shared_ptr<RigidBody>> m_links;

    std::shared_ptr<RigidBody> m_eoat;
    glm::mat4x4 m_eoatRelativeTransform;

    //
    std::shared_ptr<Trajectory> m_currentTrajectory;

};


#endif //AUTOCARVER_ROBOT_H
