//
// Created by cameronh on 24/04/24.
//

#ifndef AUTOCARVER_MONITORCAMERACONTROLLER_H
#define AUTOCARVER_MONITORCAMERACONTROLLER_H

#include <Qt3DCore/QEntity>

namespace Qt3DRender {
    class QCamera;
}

namespace Qt3DLogic {
    class QFrameAction;
}

namespace Qt3DInput {
    class QKeyboardDevice;
    class QMouseDevice;
    class QLogicalDevice;
    class QAction;
    class QActionInput;
    class QAxis;
    class QAnalogAxisInput;
    class QButtonAxisInput;
    class QAxisActionHandler;
    class QAbstractPhysicalDevice;
}

class MonitorCameraController : public Qt3DCore::QEntity
{
    Q_OBJECT

public:
    explicit MonitorCameraController(Qt3DCore::QNode *parent = nullptr);
    ~MonitorCameraController();

    void setCamera(Qt3DRender::QCamera *camera);
    void linkContainer(QWidget *container);
private:
    void prepareAxis(Qt3DInput::QAbstractPhysicalDevice *source, Qt3DInput::QAxis *axis,
                     Qt3DInput::QButtonAxisInput *pos, const QList<int>& posKeys,
                     Qt3DInput::QButtonAxisInput *neg, const QList<int>& negKeys);
    void applyInputAccelerations();

    void engage();
    void disengage();

    void restrainCursor();

    void moveCamera(float dt);

private:

    Qt3DRender::QCamera *m_camera;
    QWidget *m_container;
    bool m_active;

    Qt3DLogic::QFrameAction *m_frameAction;
    Qt3DInput::QLogicalDevice *m_logicalDevice;

    Qt3DInput::QActionInput *m_escapeButtonInput;
    Qt3DInput::QActionInput *m_shiftButtonInput;
    Qt3DInput::QAction *m_escapeButtonAction;
    Qt3DInput::QAction *m_shiftButtonAction;

    Qt3DInput::QMouseDevice *m_mouseDevice;
    Qt3DInput::QAnalogAxisInput *m_mouseRxInput;
    Qt3DInput::QAnalogAxisInput *m_mouseRyInput;
    Qt3DInput::QAxis *m_rxAxis;
    Qt3DInput::QAxis *m_ryAxis;

    Qt3DInput::QKeyboardDevice *m_keyboardDevice;
    Qt3DInput::QButtonAxisInput *m_keyboardTxPInput;
    Qt3DInput::QButtonAxisInput *m_keyboardTyPInput;
    Qt3DInput::QButtonAxisInput *m_keyboardTzPInput;
    Qt3DInput::QButtonAxisInput *m_keyboardTxNInput;
    Qt3DInput::QButtonAxisInput *m_keyboardTyNInput;
    Qt3DInput::QButtonAxisInput *m_keyboardTzNInput;
    Qt3DInput::QAxis *m_txAxis;
    Qt3DInput::QAxis *m_tyAxis;
    Qt3DInput::QAxis *m_tzAxis;

    float m_acceleration;
    float m_deceleration;

    float m_linear;
    float m_look;

    bool m_ignore;
};


#endif //AUTOCARVER_MONITORCAMERACONTROLLER_H
