//
// Created by Cam on 2025-03-15.
//

#ifndef AUTOCARVER_KINEMATICCHAIN_H
#define AUTOCARVER_KINEMATICCHAIN_H

#include <vector>
#include <cstdint>

#include "Joint.h"

class Pose;
class Waypoint;

static Joint NULL_JOINT = Joint(Joint::Type::NONE, {0, 0, 0, 0});

class KinematicChain {
public:

    KinematicChain();

    void addJoint(Joint joint);
    void setJointValues(const std::vector<double>& values);

    bool moveTo(const Pose& pose);
    bool moveTo(const glm::dvec3& position, const glm::dvec3& euler);
    bool moveTo(const Waypoint& waypoint);

    std::vector<double> invkin(const glm::dmat4& transform);
    std::vector<double> invkin(const Pose& pose);
    std::vector<double> invkin(const glm::dvec3& position, const glm::dvec3& euler);

    [[nodiscard]] Waypoint getWaypoint() const;

    uint32_t jointCount();
    Joint& getJoint(uint32_t idx);
    const std::vector<Joint>& getJoints();

    [[nodiscard]] Pose getPose() const;
    [[nodiscard]] Pose getPose(const Waypoint& waypoint) const;

    [[nodiscard]] std::vector<glm::dmat4> jointTransforms() const;

    [[nodiscard]] const glm::dmat3& axisInversion() const;
    [[nodiscard]] const glm::dmat4& inversion() const;

protected:

    virtual std::vector<double> invkin(const glm::dvec3& position, const glm::dquat& rotation);

    std::vector<glm::dmat4> jointHTMs() const;

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
