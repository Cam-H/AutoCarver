//
// Created by Cam on 2025-03-15.
//

#ifndef AUTOCARVER_KINEMATICCHAIN_H
#define AUTOCARVER_KINEMATICCHAIN_H

#include <vector>
#include <cstdint>

#include "Joint.h"

class Axis3D;

static Joint NULL_JOINT = Joint(Joint::Type::NONE, {0, 0, 0, 0});

class KinematicChain {
public:

    KinematicChain();

    void addJoint(Joint joint);
    void setJointValues(const std::vector<double>& values);

    bool moveTo(const glm::dvec3& position, const glm::dvec3& euler);

    std::vector<double> invkin(const glm::dmat4& transform);
    std::vector<double> invkin(const glm::dvec3& position, const Axis3D& axes);
    std::vector<double> invkin(const glm::dvec3& position, const glm::dvec3& euler);
    virtual std::vector<double> invkin(const glm::dvec3& position, const glm::dquat& rotation);

    uint32_t jointCount();
    Joint& getJoint(uint32_t idx);
    const std::vector<Joint>& getJoints();

    std::vector<glm::dmat4> jointTransforms();

protected:
    std::vector<glm::dmat4> jointHTMs();

    std::vector<glm::dmat3> jointHRMs(const std::vector<double>& values);
    std::vector<glm::dmat4> jointHTMs(const std::vector<double>& values);

    bool ikValidation(const std::vector<double>& values);
    bool ikValidation(const std::vector<double>& values, const glm::dvec3& position, const glm::dquat& rotation);

protected:
    std::vector<Joint> m_joints;

    // YZ Transformation to correct difference in coordinate systems
    glm::dmat3 m_axisTransform3;
    glm::dmat4 m_axisTransform4;

};


#endif //AUTOCARVER_KINEMATICCHAIN_H
