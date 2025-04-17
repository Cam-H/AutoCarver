//
// Created by Cam on 2025-04-17.
//

#include "Transformable.h"

#include <glm/gtc/matrix_transform.hpp>

Transformable::Transformable()
    : m_transform(1.0f)
{

}

void Transformable::setPosition(const glm::vec3& position)
{
    m_transform[3][0] = position.x;
    m_transform[3][1] = position.y;
    m_transform[3][2] = position.z;
}

void Transformable::translate(const glm::vec3& translation)
{
    m_transform = glm::translate(m_transform, translation);

}
void Transformable::rotate(const glm::vec3& axis, float theta)
{
    m_transform = glm::rotate(m_transform, theta, axis);
}

void Transformable::globalTranslate(const glm::vec3& translation)
{
    m_transform = glm::translate(glm::mat4(1.0f), translation) * m_transform;
}
void Transformable::globalRotate(const glm::vec3& axis, float theta)
{
    m_transform = glm::rotate(glm::mat4(1.0f), theta, axis) * m_transform;
}

void Transformable::transform(const glm::mat4x4& transform)
{
    m_transform = m_transform * transform;
}

void Transformable::setTransform(glm::mat4x4 transform)
{
    m_transform = transform;
}

const glm::mat4x4& Transformable::getTransform()
{
    return m_transform;
}