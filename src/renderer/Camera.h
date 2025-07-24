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
    void setFOV(double fov);
    void setAspectRatio(double aspect);

    // Orthographic camera controls
    void setRect(double left, double right, double bot, double top);

    // Camera positioning
    void setCenter(const QVector3D& center);
    void setPosition(const QVector3D& position);
    void setFocus(const QVector3D& position);

    void setRadius(double radius);
    void setViewingAngle(double yaw, double pitch);

    void offset(const QVector3D& offset);
    void rotate(double theta);


    // Camera getters
    Type getType() const;

    double getRadius() const;

    double getYaw() const;
    double getPitch() const;

    QVector3D getPosition() const;
    QVector3D getFocus() const;

    QVector3D forward() const;
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
    double m_fov;
    double m_aspect;

    double m_left;
    double m_right;
    double m_bot;
    double m_top;

    double m_zNear;
    double m_zFar;

    // Camera position
    double m_radius;

    double m_yaw;
    double m_pitch;

    QVector3D m_center;
    QVector3D m_eye;


    QMatrix4x4 m_viewProjection;

    const QVector3D UP_VECTOR = QVector3D(0, 1, 0);

};


#endif //AUTOCARVER_CAMERA_H
