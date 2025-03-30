//
// Created by Cam on 2025-03-15.
//

#include "Joint.h"

#include <iostream>

Joint::Joint(Joint::Type type, const DHParameter& parameters, float initialValue)
    : m_jointType(type)
    , m_parameters(parameters)
    , m_lowerLimit(std::numeric_limits<float>::lowest())
    , m_upperLimit(std::numeric_limits<float>::max())
    , m_value(initialValue)
    , m_htm(calculateHTM())
{

}

void Joint::recalculate()
{
    m_htm = calculateHTM();
}

void Joint::setJointLimits(float lower, float upper)
{
    m_lowerLimit = lower;
    m_upperLimit = upper;
}

void Joint::setValue(float value)
{
    m_value = std::clamp(value, m_lowerLimit, m_upperLimit);
    recalculate();
}

const DHParameter& Joint::getParameters() const
{
    return m_parameters;
}

float Joint::getLowerLimit() const
{
    return m_lowerLimit;
}
float Joint::getUpperLimit() const
{
    return m_upperLimit;
}

float Joint::getValue() const
{
    return m_value;
}

bool Joint::withinLimits(float value) const
{
    return value > m_lowerLimit && value < m_upperLimit;
}

glm::mat4x4 Joint::calculateHTM() const
{
    return calculateHTM(m_value);
}

glm::mat4x4 Joint::calculateHTM(float value) const
{
    float dist = distance(value), theta = angle(value);

    float ct = cosf(theta), st = sinf(theta);
    float ca = cosf(m_parameters.alpha), sa = sinf(m_parameters.alpha);

//    glm::mat4x4 rot = glm::mat4x4(
//            1, 0, 0, 0,
//            0, 0, 1, 0,
//            0, -1, 0, 0,
//            0, 0, 0, 1
//    );

    auto htm = glm::mat4x4(
            ct, -st * ca, st * sa, m_parameters.len * ct,
            st, ct * ca, -ct * sa, m_parameters.len * st,
            0, sa, ca, dist,
            0, 0, 0, 1
    );

//    glm::mat4x4 htm = glm::mat4x4(
//            ct, -st * ca, st * sa, m_parameters.len * ct,
//            0, sa, ca, dist,
//            st, ct * ca, -ct * sa, m_parameters.len * st,
//            0, 0, 0, 1
//    );

//    htm = glm::mat4x4(
//            1, 0, 0, 0,
//            0, 0, 1, 0,
//            0, 1, 0, 0,
//            0, 0, 0, 1
//            ) * htm;

//    m_htm = glm::mat4x4(
//            ct, -st, 0, m_parameters.len,
//            ca * st, ca * ct, -sa, dist * sa,
//            sa * st, sa * ct, ca, dist * ca,
//            0, 0, 0, 1
//    );

//    return htm * rot;
    return htm;
}

glm::mat3x3 Joint::calculateHRM() const
{
    return calculateHRM(m_value);
}
glm::mat3x3 Joint::calculateHRM(float value) const
{
    float theta = angle(value);

    std::cout << "HRMC: " << value << " -> " << theta << "\n";

    float ct = cosf(theta), st = sinf(theta);
    float ca = cosf(m_parameters.alpha), sa = sinf(m_parameters.alpha);

    auto hrm = glm::mat3x3(
            ct, -st * ca, st * sa,
            st, ct * ca, -ct * sa,
            0, sa, ca
    );

    return hrm;
}

const glm::mat4x4& Joint::getHTM() const
{
    return m_htm;
}

glm::mat4x4 Joint::localRotationMatrix() const
{
    if (m_jointType != Joint::Type::REVOLUTE) return {1.0f};

    float ct = cosf(m_value), st = sinf(m_value);
    return {
            ct, -st, 0, 0,
            st, ct, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
    };
//    return {
//            1, 0, 0, 0,
//            0, ct,  -st, 0,
//            0, st, ct, 0,
//            0, 0, 0, 1
//    };
//    return {
//            ct, 0, st, 0,
//            0, 1,  0, 0,
//            -st, 0, ct, 0,
//            0, 0, 0, 1
//    };
}

float Joint::distance(float value) const
{
    return m_parameters.dist + (m_jointType == Joint::Type::PRISMATIC ? value : 0);
}
float Joint::angle(float value) const
{
    return m_parameters.theta + (m_jointType == Joint::Type::REVOLUTE ? value : 0);
}