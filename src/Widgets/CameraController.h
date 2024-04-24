//
// Created by cameronh on 24/04/24.
//

#ifndef AUTOCARVER_CAMERACONTROLLER_H
#define AUTOCARVER_CAMERACONTROLLER_H

#include <QWidget>
#include <QBasicTimer>
#include <QKeyEvent>

#include <unordered_map>

#include "../Renderer/Camera.h"

enum class CameraControllerType
{
    LOCKED, FPS, ORBITAL
};

class CameraController : public QWidget
{
    Q_OBJECT

public:
    using QWidget::QWidget;

    CameraController(Camera *camera);
    ~CameraController();

    void setControlType(CameraControllerType type);

protected:

    void mouseMoveEvent(QMouseEvent *e) override;
    void keyPressEvent(QKeyEvent *e) override;
    void keyReleaseEvent(QKeyEvent *e) override;
    void timerEvent(QTimerEvent *e) override;

private:
    QBasicTimer m_timer;

    Camera *camera;
    CameraControllerType m_controlType;

    std::unordered_map<int, bool> m_KeyMap;
};


#endif //AUTOCARVER_CAMERACONTROLLER_H
