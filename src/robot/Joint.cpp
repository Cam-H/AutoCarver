//
// Created by Cam on 2025-03-15.
//

#include "Joint.h"

#include <iostream>

Joint::Joint(Joint::Type type, const DHParameter& parameters, double initialValue)
    : m_jointType(type)
    , m_parameters(parameters)
    , m_lowerLimit(std::numeric_limits<double>::lowest())
    , m_upperLimit(std::numeric_limits<double>::max())
    , m_value(initialValue)
    , m_htm(calculateHTM())
{

}

void Joint::recalculate()
{
    m_htm = calculateHTM();
}

void Joint::setJointLimits(double lower, double upper)
{
    m_lowerLimit = lower;
    m_upperLimit = upper;
}

void Joint::setValue(double value)
{
    m_value = std::clamp(value, m_lowerLimit, m_upperLimit);
    recalculate();
}

const DHParameter& Joint::getParameters() const
{
    return m_parameters;
}

double Joint::getLowerLimit() const
{
    return m_lowerLimit;
}
double Joint::getUpperLimit() const
{
    return m_upperLimit;
}

double Joint::getRange() const
{
    return m_upperLimit - m_lowerLimit;
}

double Joint::getValue() const
{
    return m_value;
}

bool Joint::withinLimits(double value) const
{
    return value > m_lowerLimit && value < m_upperLimit;
}

// Return the distance between the value and the nearest limit
double Joint::remainingLimit(double value) const
{
    double lLim = value - m_lowerLimit, uLim = m_upperLimit - value;
    if (lLim <= 0 || uLim <= 0) return 0;
    return std::min(lLim, uLim);
}

glm::dmat4 Joint::calculateHTM() const
{
    return calculateHTM(m_value);
}

glm::dmat4 Joint::calculateHTM(double value) const
{
    double dist = distance(value), theta = angle(value);

    double ct = cos(theta), st = sin(theta);
    double ca = cos(m_parameters.alpha), sa = sin(m_parameters.alpha);

    auto htm = glm::dmat4x4(
            ct, st, 0, 0,
            -st * ca, ct * ca, sa, 0,
            st * sa, -ct * sa, ca, 0,
            m_parameters.len * ct, m_parameters.len * st, dist, 1
    );

    return htm;
}

glm::dmat3 Joint::calculateHRM() const
{
    return calculateHRM(m_value);
}
glm::dmat3 Joint::calculateHRM(double value) const
{
    double theta = angle(value);

    double ct = cos(theta), st = sin(theta);
    double ca = cos(m_parameters.alpha), sa = sin(m_parameters.alpha);

    return {
            ct, st, 0,
            -st * ca, ct * ca, sa,
            st * sa, -ct * sa, ca
    };
}

const glm::dmat4& Joint::getHTM() const
{
    return m_htm;
}

glm::dmat4 Joint::localRotationMatrix() const
{
    if (m_jointType != Joint::Type::REVOLUTE) return {1.0f};

    double ct = cos(m_value), st = sin(m_value);

    return {
            ct, st, 0, 0,
            -st, ct, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
    };
}

double Joint::distance(double value) const
{
    return m_parameters.dist + (m_jointType == Joint::Type::PRISMATIC ? value : 0);
}
double Joint::angle(double value) const
{
    return m_parameters.theta + (m_jointType == Joint::Type::REVOLUTE ? value : 0);
}