//
// Created by cameronh on 24/04/24.
//

#include "CameraController.h"

#include <iostream>

CameraController::CameraController(Camera *camera)
        : camera(camera)
{
    setFocusPolicy(Qt::FocusPolicy::StrongFocus);
    setMouseTracking(true);

    m_timer.start(1000 / 40, this);
}

CameraController::~CameraController()
{
}

void CameraController::setControlType(CameraControllerType type)
{
    m_controlType = type;

    switch(m_controlType){
        case CameraControllerType::FPS:
            setCursor(Qt::BlankCursor);
            QCursor::setPos(mapToGlobal(rect().center()));
//            grabKeyboard();
            break;
        case CameraControllerType::ORBITAL:
        case CameraControllerType::LOCKED:
            unsetCursor();
//            releaseKeyboard();
            break;
    }
}

void CameraController::mouseMoveEvent(QMouseEvent *e) {

    if (m_controlType == CameraControllerType::FPS) {
        QVector2D delta = QVector2D {
                (float)e->pos().x() - (float)rect().center().x(),
                (float)e->pos().y() - (float)rect().center().y()
        } * 0.3f;

        if(delta.lengthSquared() > 0){
            QCursor::setPos(mapToGlobal(rect().center()));

            camera->yaw(delta.x());
            camera->pitch(delta.y());
        }
    }
}

void CameraController::keyPressEvent(QKeyEvent *e)
{
//    const float amount = e->modifiers().testFlag(Qt::ShiftModifier) ? 1.0f : 0.1f;
    m_KeyMap[e->key()] = true;

    if (!e->isAutoRepeat() && e->key() == Qt::Key_Escape) {
        setControlType(m_controlType == CameraControllerType::FPS
            ? CameraControllerType::LOCKED : CameraControllerType::FPS);
    }
}

void CameraController::keyReleaseEvent(QKeyEvent *e)
{
    m_KeyMap[e->key()] = false;
}

void CameraController::timerEvent(QTimerEvent *)
{
    if (m_controlType == CameraControllerType::FPS) {
        QVector3D heading = {
                (float)(m_KeyMap[Qt::Key_W] - m_KeyMap[Qt::Key_S]),
                (float)(m_KeyMap[Qt::Key_D] - m_KeyMap[Qt::Key_A]),
                (float)(m_KeyMap[Qt::Key_Space] - m_KeyMap[Qt::Key_Shift])
        };

        if (heading.lengthSquared() > 0) {
            QVector3D delta = heading * 0.3;
            camera->walk(delta.x());
            camera->strafe(delta.y());
            camera->climb(delta.z());
        }
    }
}