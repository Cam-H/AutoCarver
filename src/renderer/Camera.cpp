//
// Created by Cam on 2025-06-03.
//

#include "Camera.h"

#include <QQuaternion>

Camera::Camera(Type type)
    : m_type(type)
    , m_fov(60.0)
    , m_aspect(1.0)
    , m_left(-1)
    , m_right(1)
    , m_bot(-1)
    , m_top(1)
    , m_zNear(1.0)
    , m_zFar(100.0)
    , m_yaw(45.0f)
    , m_pitch(45.0f)
    , m_radius(5.0f)
    , m_center(QVector3D(0, 0, 0))
    , m_eye(m_center + cameraRotated(QVector3D(m_radius, 0, 0)))

{
    calculateViewProjectionMatrix();
}

void Camera::setType(Type type)
{
    m_type = type;

    calculateViewProjectionMatrix();
}

void Camera::setFOV(float fov)
{
    m_fov = fov;

    calculateViewProjectionMatrix();
}

void Camera::setAspectRatio(float aspect)
{
    m_aspect = aspect;

    calculateViewProjectionMatrix();
}

void Camera::setRect(float left, float right, float bot, float top)
{
    m_left = left;
    m_right = right;
    m_bot = bot;
    m_top = top;

    calculateViewProjectionMatrix();
}

void Camera::setPosition(const QVector3D& position)
{
    setOrientation(m_center - position);
    calculateViewProjectionMatrix();
}

void Camera::setFocus(const QVector3D& position)
{
    setOrientation(position - m_eye);
    m_center = position;

    calculateViewProjectionMatrix();
}

void Camera::setOrientation(QVector3D axis)
{
    m_radius = axis.length();
    axis /= m_radius;

    m_yaw = 180.0f / (float)M_PI * atan2f(axis.z(), -axis.x());
    m_pitch = 180.0f / (float)M_PI * acosf(QVector3D::dotProduct(UP_VECTOR, axis)) - 90.0f;
}

void Camera::setRadius(float radius)
{
    m_radius = radius;

    calculateViewProjectionMatrix();
}

void Camera::setViewingAngle(float yaw, float pitch)
{
    m_yaw = yaw;
    m_pitch = pitch;

    calculateViewProjectionMatrix();
}

void Camera::offset(const QVector3D& offset)
{
    m_center += offset;

    calculateViewProjectionMatrix();
}

void Camera::rotate(float theta)
{
    m_yaw += theta;

    calculateViewProjectionMatrix();
}

void Camera::calculateViewProjectionMatrix()
{

    m_viewProjection.setToIdentity();

    if (m_type == Type::PERSPECTIVE) {
        m_viewProjection.perspective(m_fov, m_aspect, m_zNear, m_zFar);
    } else if (m_type == Type::ORTHOGRAPHIC) {
        m_viewProjection.ortho(m_left, m_right, m_bot, m_top, m_zNear, m_zFar);
    }

    // Apply the view matrix to the projection
    m_eye = m_center + cameraRotated(QVector3D(m_radius, 0, 0));
    m_viewProjection.lookAt(m_eye, m_center, UP_VECTOR);
}

QVector3D Camera::cameraRotated(QVector3D base) const
{
    base = QQuaternion::fromAxisAndAngle(0, 0, 1, m_pitch) * base; // Apply pitch to eye
    return QQuaternion::fromAxisAndAngle(UP_VECTOR, m_yaw) * base; // Apply yaw to eye
}

Camera::Type Camera::getType() const
{
    return m_type;
}

float Camera::getRadius() const
{
    return m_radius;
}

float Camera::getYaw() const
{
    return m_yaw;
}
float Camera::getPitch() const
{
    return m_pitch;
}

QVector3D Camera::horizontal() const
{
    return cameraRotated(QVector3D(0, 0, 1)).normalized();
}
QVector3D Camera::vertical() const
{
    return cameraRotated(UP_VECTOR).normalized();
}

QMatrix4x4 Camera::getViewProjection() const
{
    return m_viewProjection;
}