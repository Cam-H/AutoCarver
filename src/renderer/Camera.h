//
// Created by Cam on 2025-06-03.
//

#ifndef AUTOCARVER_CAMERA_H
#define AUTOCARVER_CAMERA_H

#include <QVector3D>
#include <QMatrix4x4>

class Camera {
public:

    enum class Type {
        PERSPECTIVE = 0, ORTHOGRAPHIC
    };

    Camera(Type type = Type::PERSPECTIVE);

    void setType(Type type);

    // Perspective camera controls
    void setFOV(float fov);
    void setAspectRatio(float aspect);

    // Orthographic camera controls
    void setRect(float left, float right, float bot, float top);

    // Camera positioning
    void setPosition(const QVector3D& position);
    void setFocus(const QVector3D& position);

    void setRadius(float radius);
    void setViewingAngle(float yaw, float pitch);

    void offset(const QVector3D& offset);
    void rotate(float theta);


    // Camera getters
    Type getType() const;

    float getRadius() const;

    float getYaw() const;
    float getPitch() const;

    QVector3D horizontal() const;
    QVector3D vertical() const;

    QMatrix4x4 getViewProjection() const;


    QVector3D cameraRotated(QVector3D base) const;

private:

    void setOrientation(QVector3D axis);

    void calculateViewProjectionMatrix();

private:

    Type m_type;

    // Camera projection
    float m_fov;
    float m_aspect;

    float m_left;
    float m_right;
    float m_bot;
    float m_top;

    float m_zNear;
    float m_zFar;

    // Camera position
    float m_radius;

    float m_yaw;
    float m_pitch;

    QVector3D m_center;
    QVector3D m_eye;


    QMatrix4x4 m_viewProjection;

    const QVector3D UP_VECTOR = QVector3D(0, 1, 0);

};


#endif //AUTOCARVER_CAMERA_H
