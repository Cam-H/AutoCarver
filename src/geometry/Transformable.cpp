//
// Created by Cam on 2025-04-17.
//

#include "Transformable.h"

#include <iostream>

#include <gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/matrix_decompose.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/string_cast.hpp>

Transformable::Transformable()
    : m_transform(1.0f)
    , m_moved(true)
{

}

void Transformable::setPosition(const glm::dvec3& position)
{
    m_transform[3][0] = position.x;
    m_transform[3][1] = position.y;
    m_transform[3][2] = position.z;
    moved();
}

void Transformable::setRotation(const glm::dvec3& euler)
{
    setRotation(glm::dquat(euler));
}

void Transformable::setRotation(const glm::dquat& quat)
{
    auto rotation = glm::mat3_cast(quat);

    m_transform[0][0] = rotation[0][0];
    m_transform[1][0] = rotation[1][0];
    m_transform[2][0] = rotation[2][0];

    m_transform[0][1] = rotation[0][1];
    m_transform[1][1] = rotation[1][1];
    m_transform[2][1] = rotation[2][1];

    m_transform[0][2] = rotation[0][2];
    m_transform[1][2] = rotation[1][2];
    m_transform[2][2] = rotation[2][2];
    moved();
}

void Transformable::translate(const glm::dvec3& translation)
{
    m_transform = glm::translate(m_transform, translation);
    moved();
}
void Transformable::rotate(const glm::dvec3& axis, double theta)
{
    m_transform = glm::rotate(m_transform, theta, axis);
    moved();
}

void Transformable::rotate(const glm::dvec3& w)
{
//    if (glm::dot(w, w) <= 1e-12) return;

    m_transform = m_transform * glm::eulerAngleXYZ(w.x, w.y, w.z);

//    m_transform = m_transform * glm::eulerAngleXYX(w.x, w.y, w.z);
//    m_transform = m_transform * glm::eulerAngleXZX(w.x, w.y, w.z);
//    m_transform = m_transform * glm::eulerAngleXZY(w.x, w.y, w.z);
    moved();
}

void Transformable::globalTranslate(const glm::dvec3& translation)
{
    m_transform = glm::translate(glm::dmat4(1.0f), translation) * m_transform;
    moved();
}
void Transformable::globalRotate(const glm::dvec3& axis, double theta)
{
    m_transform = glm::rotate(glm::dmat4(1.0f), theta, axis) * m_transform;
    moved();
}

void Transformable::scale(double scalar)
{
    m_transform = glm::scale(m_transform, glm::dvec3(scalar));
}

void Transformable::transform(const glm::dmat4x4& transform)
{
    m_transform = m_transform * transform;
    moved();
}

void Transformable::setTransform(glm::dmat4x4 transform)
{
    m_transform = transform;
    moved();
}

const glm::dmat4& Transformable::getTransform()
{
    return m_transform;
}

glm::dmat3 Transformable::getRotation()
{
    return rotation(m_transform);
}

void Transformable::moved()
{
    m_moved = true;
}

bool Transformable::checkMoveState() const
{
    return m_moved;
}

void Transformable::clearMoveState()
{
    m_moved = false;
}

glm::dvec3 Transformable::position() const
{
    return position(m_transform);
}

glm::dvec3 Transformable::up() const
{
    auto temp = m_transform * glm::vec4{ 0, 1, 0, 0 };
    return { temp.x, temp.y, temp.z };
}