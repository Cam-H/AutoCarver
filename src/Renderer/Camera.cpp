//
// Created by cameronh on 23/04/24.
//

#include "Camera.h"

Camera::Camera(const QVector3D &pos)
        : m_forward(0.0f, 0.0f, -1.0f),
          m_right(1.0f, 0.0f, 0.0f),
          m_up(0.0f, 1.0f, 0.0f),
          m_pos(pos),
          m_yaw(0.0f),
          m_pitch(0.0f),
          m_fov(45.0f),
          m_aspectRatio(1.0f),
          m_zNear(1.0f),
          m_zFar(50.0f)
{
}

void Camera::setPosition(const QVector3D &pos)
{
    if (m_pos == pos) return;

    m_pos = pos;

    invalidate();
}

void Camera::translate(const QVector3D &delta)
{
    if (delta.lengthSquared() == 0) return;

    m_pos += delta;

    invalidate();
}

void Camera::walk(float amount)
{
    if (amount == 0) return;

    m_pos[0] += amount * m_forward.x();
    m_pos[2] += amount * m_forward.z();

    invalidate();
}

void Camera::strafe(float amount)
{
    if (amount == 0) return;

    m_pos[0] += amount * m_right.x();
    m_pos[2] += amount * m_right.z();

    invalidate();
}

void Camera::climb(float amount)
{
    if (amount == 0) return;

    m_pos[1] += amount;

    invalidate();
}

void Camera::yaw(float degrees, float limit)
{
    if (degrees == 0) return;

    m_yaw += degrees;
    clamp360(&m_yaw);
    if (limit > 0) clamp(&m_yaw, limit);
    m_yawMatrix.setToIdentity();
    m_yawMatrix.rotate(m_yaw, 0, 1, 0);

    recalculateAxes();
    invalidate();
}

void Camera::pitch(float degrees, float limit)
{
    if (degrees == 0) return;

    m_pitch += degrees;
    clamp360(&m_pitch);
    if (limit > 0) clamp(&m_pitch, limit);
    m_pitchMatrix.setToIdentity();
    m_pitchMatrix.rotate(m_pitch, 1, 0, 0);

    recalculateAxes();
    invalidate();
}

void Camera::setFOV(float fov)
{
    if (fov == m_fov) return;

    m_fov = fov;

    invalidate();
}

void Camera::setAspectRatio(float aspectRatio)
{
    if (aspectRatio == m_aspectRatio) return;

    m_aspectRatio = aspectRatio;

    invalidate();
}

void Camera::recalculateAxes(){
    QMatrix4x4 rotMat = m_pitchMatrix * m_yawMatrix;

    m_forward = (QVector4D(0.0f, 0.0f, -1.0f, 0.0f) * rotMat).toVector3D();
    m_forward = QVector3D(m_forward.x(), 0, m_forward.z()).normalized();

    m_right = (QVector4D(1.0f, 0.0f, 0.0f, 0.0f) * rotMat).toVector3D();
    m_right = QVector3D(m_right.x(), 0, m_right.z()).normalized();

//    m_up = (QVector4D(0.0f, 1.0f, 0.0f, 0.0f) * rotMat).toVector3D();
}

inline void Camera::clamp(float *value, float limit)
{
    if (*value > limit) *value = limit;
    if (*value < -limit) *value = -limit;
}

inline void Camera::clamp360(float *value)
{
    if (*value > 360.0f)
        *value -= 360.0f;
    if (*value < -360.0f)
        *value += 360.0f;
}

void Camera::invalidate()
{
    emit poseChanged();
}

QMatrix4x4 Camera::viewMatrix() const
{
    QMatrix4x4 view = m_pitchMatrix * m_yawMatrix;
    view.translate(-m_pos);
    return view;
}

QMatrix4x4 Camera::projectionMatrix() const
{
    QMatrix4x4 proj;
    proj.perspective(m_fov, m_aspectRatio, m_zNear, m_zFar);
    return proj;
}

QMatrix4x4 Camera::viewProjectionMatrix() const
{
    return projectionMatrix() * viewMatrix();
}