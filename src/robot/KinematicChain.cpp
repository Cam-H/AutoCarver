//
// Created by Cam on 2025-03-15.
//

#include "KinematicChain.h"

#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtc/epsilon.hpp>

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

void KinematicChain::setJointValues(const std::vector<float>& values)
{
    size_t max = std::min(values.size(), m_joints.size());
    if (max != m_joints.size()) std::cout << "\033[31mWarning value set length is different from joint number\033[0m\n";

    for (uint32_t i = 0; i < max; i++) {
        m_joints[i].setValue(values[i]);
    }
}

bool KinematicChain::moveTo(const glm::vec3& position, const glm::vec3& euler)
{
    const std::vector<float>& values = invkin(position, euler);
    setJointValues(values);

    return !values.empty();
}

std::vector<float> KinematicChain::invkin(const glm::mat4& transform)
{

    glm::quat rotation;
    glm::vec3 translation;

    glm::vec3 scale;
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::decompose(m_axisTransform4 * transform, scale, rotation, translation, skew,perspective);

    return invkin(translation, rotation);
}

std::vector<float> KinematicChain::invkin(const glm::vec3& position, const glm::vec3& euler)
{
    return invkin(m_axisTransform3 * position, glm::quat_cast(m_axisTransform3 * glm::mat3_cast(glm::quat(euler))));
}

std::vector<float> KinematicChain::invkin(const glm::vec3& position, const glm::quat& rotation)
{
    std::cout << "\033[31mUnable to move to desired position. The generic kinematic chain solver has not been implemented\033[0m\n";
    return {};
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

std::vector<glm::mat4> KinematicChain::jointHTMs()
{
    std::vector<glm::mat4> transforms { glm::mat4(1.0f) };

    for (uint32_t i = 0; i < m_joints.size(); i++) {
        transforms.push_back(transforms[i] * m_joints[i].getHTM());
        transforms[i] = transforms[i] * m_joints[i].localRotationMatrix();
    }

    return transforms;
}

std::vector<glm::mat3> KinematicChain::jointHRMs(const std::vector<float>& values)
{
    std::vector<glm::mat3> matrices { glm::mat3(1.0f) };
    uint32_t count = std::min(values.size(), m_joints.size());

    for (uint32_t i = 0; i < count; i++) {
        matrices.push_back(matrices[i] * m_joints[i].calculateHRM(values[i]));
    }

    return matrices;
}

std::vector<glm::mat4> KinematicChain::jointHTMs(const std::vector<float>& values)
{
    std::vector<glm::mat4> transforms { glm::mat4(1.0f) };
    uint32_t count = std::min(values.size(), m_joints.size());

    for (uint32_t i = 0; i < count; i++) {
        transforms.push_back(transforms[i] * m_joints[i].calculateHTM(values[i]));
    }

    return transforms;
}

bool KinematicChain::ikValidation(const std::vector<float>& values)
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

bool KinematicChain::ikValidation(const std::vector<float>& values, const glm::vec3& position, const glm::quat& rotation)
{
    if (!ikValidation(values)) return false; // Base joint value validity check

    // Get the transform after the final joint
    auto transform = jointHTMs(values)[values.size() - 1];

    // Verify the calculated iks result in the specified position
    glm::vec3 delta = position - glm::vec3{ transform[3][0], transform[3][1], transform[3][2] };
    if (glm::dot(delta, delta) < 1e-3) {
        std::cout << "\033[31mFailed to solve! Specified position does not match calculated result! Delta: "
            << "[" << delta.x << ", " << delta.y << ", " << delta.z << "]\033[0m\n";
        return false;
    }

    // Verify the calculated iks result in the specified rotation TODO
//    glm::mat3 rDel = glm::mat3_cast(rotation)
//            - glm::mat3(
//                transform[0][0], transform[0][1], transform[0][2],
//                transform[1][0], transform[1][1], transform[1][2],
//                transform[2][0], transform[2][1], transform[2][2]
//            );
//
//    for (uint8_t i = 0; i < 3; i++) {
//        for (uint8_t j = 0; j < 3; j++) {
//            if (std::abs(rDel[i][j]) > 1e-6) {
//                std::cout << "\033[31mFailed to solve! Specified rotation does not match calculated result!\033[0m\n";
//                return false;
//            }
//        }
//    }

    return true;
}

std::vector<glm::mat4> KinematicChain::jointTransforms()
{
    std::vector<glm::mat4> transforms = jointHTMs();
    for (glm::mat4& transform : transforms) {
        transform = m_axisTransform4 * transform;
    }

    return transforms;
}