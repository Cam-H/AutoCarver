//
// Created by cameronh on 23/04/24.
//

#ifndef CUBE_CAMERA_H
#define CUBE_CAMERA_H

#include <QObject>
#include <QVector3D>
#include <QMatrix4x4>

class Camera : public QObject
{
Q_OBJECT

public:
    Camera(const QVector3D &pos);

    void setPosition(const QVector3D &pos);
    void translate(const QVector3D &delta);

    void walk(float amount);
    void strafe(float amount);
    void climb(float amount);

    void yaw(float degrees, float limit = -1);
    void pitch(float degrees, float limit = -1);

    void setFOV(float fov);
    void setAspectRatio(float aspectRatio);

    QMatrix4x4 viewMatrix() const;
    QMatrix4x4 projectionMatrix() const;

    QMatrix4x4 viewProjectionMatrix() const;

signals:
    void poseChanged();
private:
    void recalculateAxes();

    static inline void clamp(float *value, float limit);
    static inline void clamp360(float *value);

    void invalidate();
private:

    // View Matrix Information
    QVector3D m_forward;
    QVector3D m_right;
    QVector3D m_up;
    QVector3D m_pos;
    float m_yaw;
    float m_pitch;
    QMatrix4x4 m_yawMatrix;
    QMatrix4x4 m_pitchMatrix;

    // Perspective Matrix Information
    float m_fov;
    float m_aspectRatio;
    float m_zNear;
    float m_zFar;
};


#endif //CUBE_CAMERA_H
