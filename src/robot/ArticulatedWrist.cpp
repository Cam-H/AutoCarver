//
// Created by Cam on 2025-03-15.
//

#include "ArticulatedWrist.h"

#include "Joint.h"

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

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

//    m_joints[0].setJointLimits(-2 * M_PI, 2 * M_PI);
//    m_joints[1].setJointLimits(0, M_PI);
//    m_joints[2].setJointLimits(-M_PI / 3, M_PI / 2);
//    m_joints[3].setJointLimits(-M_PI, M_PI);
////    m_joints[4].setJointLimits(-2 * M_PI / 3, 2 * M_PI / 3);
//    m_joints[4].setJointLimits(-5 * M_PI / 6, 5 * M_PI / 6);
//    m_joints[5].setJointLimits(-2 * M_PI, 2 * M_PI);

}

std::vector<float> ArticulatedWrist::invkin(const glm::vec3& position, const glm::quat& rotation)
{

    std::vector<float> values(6, 0);
    float l2 = m_joints[1].getParameters().len;
    float l3 = m_joints[3].getParameters().dist;
    float d1 = m_joints[0].getParameters().dist;
    float d6 = m_joints[5].getParameters().dist;

    auto YZAxisRotationMatrix = glm::mat3x3(
            1, 0, 0,
            0, 0, -1,
            0, 1, 0
    );
    std::cout << "Goal: " << position.x << " " << position.y << " " << position.z << "\n";

//    auto vec =  YZAxisRotationMatrix * glm::vec3{1, 2, 5};
//    std::cout << "||" << vec.x << " " << vec.y << " " << vec.z << "\n";

    // Identify the center of the spherical wrist
    glm::vec3 Oc = position - glm::vec3{0, 0, d6} * rotation;
//    glm::vec3 Oc = position - rotation * glm::vec3{0, 0, d6} * rotation;
    std::cout << "-> " << (Oc.x - position.x) << " " << (Oc.y - position.y) << " " << (Oc.z - position.z) << "\n";


    // Calculate corresponding joint angles for the first 3 joints
    values[0] = atan2f(Oc.y, Oc.x);

    // Third joint via cosine law
    float r = Oc.x * Oc.x + Oc.y * Oc.y, s = (Oc.z - d1) * (Oc.z - d1);
    float D = (r + s - l2 * l2 - l3 * l3) / (2 * l2 * l3);
    values[2] = atan2f(D, sqrtf(1 - D * D));

    std::cout << r << " " << s << " " << D << " " << sqrtf(1 - D * D) << " " << l2 << " " << l3 << " " << d1 << " " << d6 << "\n";

    // Second joint by substitution
    values[1] = atan2f(Oc.z - d1, sqrtf(r)) + atan2f(l3 * cosf(values[2]), l2 + l3 * sinf(values[2]));

    // Verify the solved position joints are valid
    for (uint32_t i = 0; i < 3; i++) {
        if (std::isnan(values[i])) {
            std::cout << "\033[31mFailed to solve for the specified position! Out of range!\033[0m\n";
            return {};
        } else if (!m_joints[i].withinLimits(values[i])) {
            std::cout << "\033[31mFailed to solve for the specified position! Robot motion is limited!\033[0m\n";
            return {};
        }
    }

    // Calculate rotation matrix of wrist to match desired rotation
    std::vector<glm::mat3> rotations = jointHRMs({ values[0], values[1], values[2] });
    auto R03 = rotations[rotations.size() - 1];
    for (uint32_t i = 1; i < rotations.size(); i++) {
        std::cout << "R0" << i << ":\n"
                  << rotations[i][0][0] << " " << rotations[i][0][1] << " " << rotations[i][0][2] << "\n"
                  << rotations[i][1][0] << " " << rotations[i][1][1] << " " << rotations[i][1][2] << "\n"
                  << rotations[i][2][0] << " " << rotations[i][2][1] << " " << rotations[i][2][2] << "\n";
    }

    std::cout << "XXXXXXXX: " << rotations.size() << " \n";
    auto R36 = glm::toMat3(rotation) * glm::transpose(R03);
//    R36 = YZAxisRotationMatrix * R36;
            std::cout << " GLM R36:\n"
                  << R36[0][0] << " " << R36[0][1] << " " << R36[0][2] << "\n"
                  << R36[1][0] << " " << R36[1][1] << " " << R36[1][2] << "\n"
                  << R36[2][0] << " " << R36[2][1] << " " << R36[2][2] << "\n";

    values[3] = atan2f(R36[1][2], R36[0][2]);
    values[4] = atan2f(sqrtf(R36[0][2] * R36[0][2] + R36[1][2] * R36[1][2]), R36[2][2]);
    values[5] = atan2f(R36[2][1], -R36[2][0]);

//    R36 = m_joints[3].calculateHRM(values[3]) * m_joints[4].calculateHRM(values[4]) * m_joints[5].calculateHRM(values[5]);
    R36 = m_joints[5].calculateHRM(values[5]) * m_joints[4].calculateHRM(values[4]) * m_joints[3].calculateHRM(values[3]);
    R36 = R03 * R36;

    std::cout << " GLM RES:\n"
              << R36[0][0] << " " << R36[0][1] << " " << R36[0][2] << "\n"
              << R36[1][0] << " " << R36[1][1] << " " << R36[1][2] << "\n"
              << R36[2][0] << " " << R36[2][1] << " " << R36[2][2] << "\n";
//    values[3] = atan2f(R36[2][1], R36[2][0]);
//    values[4] = atan2f(sqrtf(R36[2][0] * R36[2][0] + R36[2][1] * R36[2][1]), R36[2][2]);
//    values[5] = atan2f(R36[1][2], -R36[0][2]);

//    % Identify desired rotation matrix for final 3 joints
//    R03 = fwdkin(rad2deg(q(1:3)));
//    R03 = R03(1:3, 1:3);
//    R36 = transpose(R03) * R06;
//    % Calculate appropriate angles for the spherical wrist to match R36
//    q(4) = atan2(R36(2,3), R36(1,3));
//    q(5) = atan2(sqrt(R36(1,3) ^ 2 + R36(2,3) ^ 2), R36(3,3));
//    q(6) = atan2(R36(3,2), -R36(3,1));
//    % Convert results to degrees for consistency
//            q = rad2deg(q);

    // Error checking
    std::cout << "IK Values: ";
    for (uint32_t i = 0; i < values.size(); i++) std::cout << values[i] << "|" << m_joints[i].getValue() << " ";
    std::cout << "\n";

    // Verify the solved rotation joints are valid
    for (uint32_t i = 3; i < 6; i++) {
        if (std::isnan(values[i])) {
            std::cout << "\033[31mFailed to solve for the specified position! Out of range!\033[0m\n";
            return {};
        } else if (!m_joints[i].withinLimits(values[i])) {
            std::cout << "\033[31mFailed to solve for the specified position! Robot motion is limited!\033[0m\n";
            return {};
        }
    }

    auto R = glm::mat3_cast(rotation);
    std::cout << " GLM RTarget:\n"
              << R[0][0] << " " << R[0][1] << " " << R[0][2] << "\n"
              << R[1][0] << " " << R[1][1] << " " << R[1][2] << "\n"
              << R[2][0] << " " << R[2][1] << " " << R[2][2] << "\n";

    return values;
}