//
// Created by Cam on 2025-03-15.
//

#ifndef AUTOCARVER_KINEMATICCHAIN_H
#define AUTOCARVER_KINEMATICCHAIN_H

#include <vector>
#include <cstdint>

#include "Joint.h"

static Joint NULL_JOINT = Joint(Joint::Type::NONE, {0, 0, 0, 0});

class KinematicChain {
public:

    KinematicChain();

    void addJoint(Joint joint);
    void setJointValues(const std::vector<float>& values);

    bool moveTo(const glm::vec3& position, const glm::vec3& euler);

    std::vector<float> invkin(const glm::mat4& transform);
    std::vector<float> invkin(const glm::vec3& position, const glm::vec3& euler);
    virtual std::vector<float> invkin(const glm::vec3& position, const glm::quat& rotation);

    uint32_t jointCount();
    Joint& getJoint(uint32_t idx);
    const std::vector<Joint>& getJoints();

    std::vector<glm::mat4> jointTransforms();

protected:
    std::vector<glm::mat4> jointHTMs();

    std::vector<glm::mat3> jointHRMs(const std::vector<float>& values);
    std::vector<glm::mat4> jointHTMs(const std::vector<float>& values);

    bool ikValidation(const std::vector<float>& values);
    bool ikValidation(const std::vector<float>& values, const glm::vec3& position, const glm::quat& rotation);

protected:
    std::vector<Joint> m_joints;

    // YZ Transformation to correct difference in coordinate systems
    glm::mat3 m_axisTransform3;
    glm::mat4 m_axisTransform4;

};


#endif //AUTOCARVER_KINEMATICCHAIN_H
