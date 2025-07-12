//
// Created by Cam on 2025-03-15.
//

#include "ArticulatedWrist.h"

#include "Joint.h"

#include <glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/quaternion.hpp>

#include <iostream>

ArticulatedWrist::ArticulatedWrist(float d1, float l2, float l3, float d6)
{
    // Elbow joint
    addJoint(Joint(Joint::Type::REVOLUTE, {0, d1, M_PI / 2, 0}));
    addJoint(Joint(Joint::Type::REVOLUTE, {l2, 0, 0, 0}));
    addJoint(Joint(Joint::Type::REVOLUTE, {0, 0, M_PI / 2, 0}));

    // Wrist joint
    addJoint(Joint(Joint::Type::REVOLUTE, {0, l3, -M_PI / 2, 0}));
    addJoint(Joint(Joint::Type::REVOLUTE, {0, 0, M_PI / 2, 0}));
    addJoint(Joint(Joint::Type::REVOLUTE, {0, d6, 0, 0}));

    m_joints[0].setJointLimits(-2 * M_PI, 2 * M_PI);
    m_joints[1].setJointLimits(0, M_PI);
    m_joints[2].setJointLimits(-M_PI / 3, M_PI / 2);
    m_joints[3].setJointLimits(-M_PI, M_PI);
    m_joints[4].setJointLimits(-5 * M_PI / 6, 5 * M_PI / 6);
    m_joints[5].setJointLimits(-M_PI, M_PI);

}

std::vector<float> ArticulatedWrist::invkin(const glm::vec3& position, const glm::quat& rotation)
{

    std::vector<float> values(6, 0);
    float l2 = m_joints[1].getParameters().len;
    float l3 = m_joints[3].getParameters().dist;
    float d1 = m_joints[0].getParameters().dist;
    float d6 = m_joints[5].getParameters().dist;

    // Identify the center of the spherical wrist
    glm::vec3 Oc = position - rotation * glm::vec3{0, 0, d6};

    // Calculate corresponding joint angles for the first 3 joints
    values[0] = atan2f(Oc.y, Oc.x);

    // Third joint via cosine law
    float r = Oc.x * Oc.x + Oc.y * Oc.y, s = (Oc.z - d1) * (Oc.z - d1);
    float D = (r + s - l2 * l2 - l3 * l3) / (2 * l2 * l3);
    values[2] = atan2f(D, sqrtf(1 - D * D));

    // Second joint by substitution
    values[1] = atan2f(Oc.z - d1, sqrtf(r)) + atan2f(l3 * cosf(values[2]), l2 + l3 * sinf(values[2]));


    // Verify the solved position joints are valid
    if (!ikValidation({ values[0], values[1], values[2]})) return {};


    // Calculate rotation matrix of wrist to match desired rotation
    std::vector<glm::mat3> rotations = jointHRMs({ values[0], values[1], values[2] });
    auto R03 = rotations[rotations.size() - 1];
    auto R36 = glm::transpose(R03) * glm::toMat3(rotation);

    values[3] = atan2f(R36[2][1], R36[2][0]);
    values[4] = atan2f(sqrtf(R36[2][0] * R36[2][0] + R36[2][1] * R36[2][1]), R36[2][2]);
    values[5] = atan2f(R36[1][2], -R36[0][2]);

    // Verify the solved rotation joints are valid
    if (!ikValidation(values, position, rotation)) return {};

    return values;
}