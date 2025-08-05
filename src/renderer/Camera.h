//
// Created by Cam on 2025-06-03.
//

#ifndef AUTOCARVER_CAMERA_H
#define AUTOCARVER_CAMERA_H

#include <QVector3D>
#include <QMatrix4x4>

#include "geometry/primitives/Ray.h"

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
    [[nodiscard]] Type getType() const;

    [[nodiscard]] double getRadius() const;

    [[nodiscard]] double getYaw() const;
    [[nodiscard]] double getPitch() const;

    [[nodiscard]] QVector3D getPosition() const;
    [[nodiscard]] QVector3D getFocus() const;

    [[nodiscard]] QVector3D forward() const;
    [[nodiscard]] QVector3D horizontal() const;
    [[nodiscard]] QVector3D vertical() const;

    [[nodiscard]] QMatrix4x4 getViewProjection() const;


    [[nodiscard]] QVector3D cameraRotated(QVector3D base) const;

    // Returns a world-space ray projected from the camera, provided the normalized screen coordinates
    [[nodiscard]] Ray getRay(QPointF position) const;

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
