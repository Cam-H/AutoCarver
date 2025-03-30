//
// Created by Cam on 2025-03-15.
//

#include "KinematicChain.h"

#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_decompose.hpp>

KinematicChain::KinematicChain()
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

//    const std::vector<float>& values = invkin(YZAxisRotationMatrix * position, euler);
    const std::vector<float>& values = invkin(position, euler);

    setJointValues(values);

    auto t = jointTransforms();
    auto R = t[t.size() - 1];
    std::cout << " GLM R2:\n"
              << R[0][0] << " " << R[0][1] << " " << R[0][2] << "\n"
              << R[1][0] << " " << R[1][1] << " " << R[1][2] << "\n"
              << R[2][0] << " " << R[2][1] << " " << R[2][2] << "\n";

    return !values.empty();
}

std::vector<float> KinematicChain::invkin(const glm::mat4& transform)
{

    glm::quat rotation;
    glm::vec3 translation;

    glm::vec3 scale;
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::decompose(transform, scale, rotation, translation, skew,perspective);

    return invkin(translation, rotation);
}

std::vector<float> KinematicChain::invkin(const glm::vec3& position, const glm::vec3& euler)
{
//    glm::quat(euler)
//    glm::mat3 rot = {
//            1, 0, 0,
//            0, 1, 0,
//            0, 0, 1
//    };

    auto m = glm::rotate(glm::mat4(1.0f), (float)M_PI / 4, {1, 0, 0});
    glm::mat3 rot = {
            m[0][0], m[0][1], m[0][2],
            m[1][0], m[1][1], m[1][2],
            m[2][0], m[2][1], m[2][2]
    };

//    glm::mat3 rot = {
//            m[0][0], m[1][0], m[2][0],
//            m[0][1], m[1][1], m[2][1],
//            m[0][2], m[1][2], m[2][2]
//    };
//    glm::mat3 rot = {
//            -1, 0, 0,
//            0, 1, 0,
//            0, 0, 1
//    };
//    glm::mat3 rot = {
//            0, 0, 1,
//            0, 1, 0,
//            -1, 0, 0
//    };

    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";

    std::cout << " GLM ROT:\n"
              << rot[0][0] << " " << rot[0][1] << " " << rot[0][2] << "\n"
              << rot[1][0] << " " << rot[1][1] << " " << rot[1][2] << "\n"
              << rot[2][0] << " " << rot[2][1] << " " << rot[2][2] << "\n";
//    return invkin(position, glm::quat_cast(rot));

    auto YZAxisRotationMatrix = glm::mat3x3(
            1, 0, 0,
            0, 0, -1,
            0, 1, 0
    );
//    const std::vector<float>& values = invkin(YZAxisRotationMatrix * position, euler);
    return invkin(position * YZAxisRotationMatrix, glm::quat_cast(glm::mat3_cast(glm::quat(euler)) * YZAxisRotationMatrix));

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
        transforms.push_back(m_joints[i].getHTM() * transforms[i]);
        transforms[i] = m_joints[i].localRotationMatrix() * transforms[i];
    }

    return transforms;
}

std::vector<glm::mat4> KinematicChain::jointHTMs(const std::vector<float>& values)
{
    std::vector<glm::mat4> transforms { glm::mat4(1.0f) };
    uint32_t count = std::min(values.size(), m_joints.size());

    for (uint32_t i = 0; i < count; i++) {
        transforms.push_back(m_joints[i].calculateHTM(values[i]) * transforms[i]);
    }

    return transforms;
}

std::vector<glm::mat3> KinematicChain::jointHRMs(const std::vector<float>& values)
{
    std::vector<glm::mat3> matrices { glm::mat3(1.0f) };
    uint32_t count = std::min(values.size(), m_joints.size());

//    auto YZAxisRotationMatrix = glm::mat3x3(
//            1, 0, 0,
//            0, 0, 1,
//            0, -1, 0
//    );
    auto YZAxisRotationMatrix = glm::mat3x3(1.0f);
    for (uint32_t i = 0; i < count; i++) {
        matrices.push_back(m_joints[i].calculateHRM(values[i]) * matrices[i]);
    }

    return matrices;
}

std::vector<glm::mat4> KinematicChain::jointTransforms()
{
    auto YZAxisRotationMatrix = glm::mat4x4(
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, -1, 0, 0,
            0, 0, 0, 1
            );

//    YZAxisRotationMatrix = glm::mat4(1.0f);
//    auto YZAxisRotationMatrix = glm::mat4x4(
//            0, 0, 1, 0,
//            0, 1, 0, 0,
//            -1, 0, 0, 0,
//            0, 0, 0, 1
//    );

//    auto YZAxisRotationMatrix = glm::mat4x4(
//            0, -1, 0, 0,
//            1, 0, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1
//    );

    std::vector<glm::mat4> transforms = jointHTMs();
    for (glm::mat4& transform : transforms) {
        transform = transform * YZAxisRotationMatrix;
    }

    return transforms;
}