//
// Created by Cam on 2025-03-15.
//

#include "KinematicChain.h"

#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/matrix_decompose.hpp>
#include <gtc/epsilon.hpp>

#include "Pose.h"
#include "planning/Waypoint.h"
#include "geometry/Transformable.h"

KinematicChain::KinematicChain()
    : m_axisTransform3(1, 0, 0,
                       0, 0, 1,
                       0, -1, 0)
    , m_axisTransform4(1, 0, 0, 0,
                       0, 0, -1, 0,
                       0, 1, 0, 0,
                       0, 0, 0, 1)
{

}

void KinematicChain::addJoint(Joint joint)
{
    m_joints.push_back(joint);
}

void KinematicChain::setJointValues(const std::vector<double>& values)
{
    size_t max = std::min(values.size(), m_joints.size());
    if (max != m_joints.size()) std::cout << "\033[31mWarning value set length is different from joint number\033[0m\n";

    for (uint32_t i = 0; i < max; i++) {
        m_joints[i].setValue(values[i]);
    }
}

bool KinematicChain::moveTo(const Pose& pose)
{
    const std::vector<double>& values = invkin(pose);
    setJointValues(values);

    return !values.empty();
}

bool KinematicChain::moveTo(const glm::dvec3& position, const glm::dvec3& euler)
{
    const std::vector<double>& values = invkin(position, euler);
    setJointValues(values);

    return !values.empty();
}

bool KinematicChain::moveTo(const Waypoint& waypoint)
{
    if (waypoint.inDg) setJointValues(waypoint.toRad().values);
    else setJointValues(waypoint.values);

    return !waypoint.values.empty();
}

// Calculates required joint angles such that the final link's transform is as specified
// Parameters must be expressed in the local coordinate system
std::vector<double> KinematicChain::invkin(const glm::dmat4& transform)
{

    glm::dquat rotation;
    glm::dvec3 translation;

    glm::dvec3 scale;
    glm::dvec3 skew;
    glm::dvec4 perspective;
    glm::decompose(m_axisTransform4 * transform, scale, rotation, translation, skew, perspective);

    return invkin(translation, rotation);
}

// Calculates required joint angles to reach the specified position and orientation
// Parameters must be expressed in the local coordinate system
std::vector<double> KinematicChain::invkin(const Pose& pose)
{
    return invkin(m_axisTransform3 * pose.position, glm::quat_cast(m_axisTransform3 * pose.axes.toTransform()));
}

// Calculates required joint angles to reach the specified position and orientation
// Parameters must be expressed in the local coordinate system
std::vector<double> KinematicChain::invkin(const glm::dvec3& position, const glm::dvec3& euler)
{
    return invkin(m_axisTransform3 * position, glm::quat_cast(m_axisTransform3 * glm::mat3_cast(glm::dquat(euler))));
}

// Internal method for actually solving the inverse kinematics problem
std::vector<double> KinematicChain::invkin(const glm::dvec3& position, const glm::dquat& rotation)
{
    std::cout << "\033[31mUnable to move to desired position. The generic kinematic chain solver has not been implemented\033[0m\n";
    return {};
}

Waypoint KinematicChain::getWaypoint() const
{
    std::vector<double> values(m_joints.size());
    for (uint32_t i = 0; i < m_joints.size(); i++) values[i] = m_joints[i].getValue();
    return Waypoint(values, false);
}

uint32_t KinematicChain::jointCount()
{
    return m_joints.size();
}
Joint& KinematicChain::getJoint(uint32_t idx)
{
    if (idx < m_joints.size()) return m_joints[idx];
    std::cout << "\033[31mInvalid joint index selected\033[0m\n";
    return NULL_JOINT;
}
const std::vector<Joint>& KinematicChain::getJoints()
{
    return m_joints;
}

Pose KinematicChain::getPose() const
{
    const auto& transforms = jointTransforms();
    return {
        Transformable::position(transforms.back()),
        Transformable::rotation(transforms.back())
    };
}
Pose KinematicChain::getPose(const Waypoint& waypoint) const
{
    KinematicChain clone = *this;
    clone.moveTo(waypoint);
    return clone.getPose();
}

std::vector<glm::dmat4> KinematicChain::jointHTMs() const
{
    std::vector<glm::dmat4> transforms { glm::dmat4(1.0f) };

    for (uint32_t i = 0; i < m_joints.size(); i++) {
        transforms.push_back(transforms[i] * m_joints[i].getHTM());
        transforms[i] = transforms[i] * m_joints[i].localRotationMatrix();
    }

    return transforms;
}

std::vector<glm::dmat3> KinematicChain::jointHRMs(const std::vector<double>& values)
{
    std::vector<glm::dmat3> matrices { glm::dmat3(1.0f) };
    uint32_t count = std::min(values.size(), m_joints.size());

    for (uint32_t i = 0; i < count; i++) {
        matrices.push_back(matrices[i] * m_joints[i].calculateHRM(values[i]));
    }

    return matrices;
}

std::vector<glm::dmat4> KinematicChain::jointHTMs(const std::vector<double>& values)
{
    std::vector<glm::dmat4> transforms { glm::dmat4(1.0f) };
    uint32_t count = std::min(values.size(), m_joints.size());

    for (uint32_t i = 0; i < count; i++) {
        transforms.push_back(transforms[i] * m_joints[i].calculateHTM(values[i]));
    }

    return transforms;
}

bool KinematicChain::ikValidation(const std::vector<double>& values)
{
    for (uint32_t i = 0; i < values.size(); i++) {
        if (std::isnan(values[i])) {
            std::cout << "\033[31mFailed to solve for the specified position! Out of range!\033[0m\n";
            return false;
        } else if (!m_joints[i].withinLimits(values[i])) {
            std::cout << "\033[31mFailed to solve for the specified position! Robot mobility is limited!\033[0m\n";
            return false;
        }
    }

    return !values.empty();
}

bool KinematicChain::ikValidation(const std::vector<double>& values, const glm::dvec3& position, const glm::dquat& rotation)
{
    if (!ikValidation(values)) return false; // Base joint value validity check

    // Get the transform after the final joint
    auto transform = jointHTMs(values)[values.size() - 1];

    // Verify the calculated iks result in the specified position
    glm::dvec3 delta = position - glm::dvec3{ transform[3][0], transform[3][1], transform[3][2] };
    if (glm::dot(delta, delta) < 1e-3) {
        std::cout << "\033[31mFailed to solve! Specified position does not match calculated result! Delta: "
            << "[" << delta.x << ", " << delta.y << ", " << delta.z << "]\033[0m\n";
        return false;
    }

    // Verify the calculated iks result in the specified rotation TODO
//    glm::dmat3 rDel = glm::dmat3_cast(rotation)
//            - glm::dmat3(
//                transform[0][0], transform[0][1], transform[0][2],
//                transform[1][0], transform[1][1], transform[1][2],
//                transform[2][0], transform[2][1], transform[2][2]
//            );
//
//    for (uint8_t i = 0; i < 3; i++) {
//        for (uint8_t j = 0; j < 3; j++) {
//            if (std::abs(rDel[i][j]) > 1e-12) {
//                std::cout << "\033[31mFailed to solve! Specified rotation does not match calculated result!\033[0m\n";
//                return false;
//            }
//        }
//    }

    return true;
}

std::vector<glm::dmat4> KinematicChain::jointTransforms() const
{
    std::vector<glm::dmat4> transforms = jointHTMs();
    for (glm::dmat4& transform : transforms) {
        transform = m_axisTransform4 * transform;
    }

    return transforms;
}