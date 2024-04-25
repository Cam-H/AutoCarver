//
// Created by cameronh on 24/04/24.
//

#include "MonitorCameraController.h"

#include <Qt3DRender/QCamera>

#include <QFrameAction>

#include <QMouseDevice>
#include <QAnalogAxisInput>
#include <QAxis>

#include <QKeyboardDevice>
#include <QButtonAxisInput>
#include <QLogicalDevice>
#include <QActionInput>
#include <Qt3DInput/QAction>

#include <QCursor>
#include <QWidget>

#include <iostream>
#include <QGuiApplication>

MonitorCameraController::MonitorCameraController(Qt3DCore::QNode *parent)
        : Qt3DCore::QEntity(parent)
        , m_frameAction(new Qt3DLogic::QFrameAction())
        , m_logicalDevice(new Qt3DInput::QLogicalDevice())
        , m_escapeButtonInput(new Qt3DInput::QActionInput())
        , m_shiftButtonInput(new Qt3DInput::QActionInput())
        , m_escapeButtonAction(new Qt3DInput::QAction())
        , m_shiftButtonAction(new Qt3DInput::QAction())
        , m_mouseDevice(new Qt3DInput::QMouseDevice())
        , m_mouseRxInput(new Qt3DInput::QAnalogAxisInput())
        , m_mouseRyInput(new Qt3DInput::QAnalogAxisInput())
        , m_rxAxis(new Qt3DInput::QAxis())
        , m_ryAxis(new Qt3DInput::QAxis())
        , m_keyboardDevice(new Qt3DInput::QKeyboardDevice())
        , m_keyboardTxPInput(new Qt3DInput::QButtonAxisInput())
        , m_keyboardTyPInput(new Qt3DInput::QButtonAxisInput())
        , m_keyboardTzPInput(new Qt3DInput::QButtonAxisInput())
        , m_keyboardTxNInput(new Qt3DInput::QButtonAxisInput())
        , m_keyboardTyNInput(new Qt3DInput::QButtonAxisInput())
        , m_keyboardTzNInput(new Qt3DInput::QButtonAxisInput())
        , m_txAxis(new Qt3DInput::QAxis())
        , m_tyAxis(new Qt3DInput::QAxis())
        , m_tzAxis(new Qt3DInput::QAxis())
        , m_acceleration(-1.0f)
        , m_deceleration(-1.0f)
        , m_linear(10.0f)
        , m_look(120.0f)
        , m_camera(nullptr)
        , m_container(nullptr)
        , m_active(false)
        , m_ignore(false)

{
    // Escape Button Action
    m_escapeButtonInput->setButtons(QList<int> { Qt::Key_Escape });
    m_escapeButtonInput->setSourceDevice(m_keyboardDevice);
    m_escapeButtonAction->addInput(m_escapeButtonInput);

    // Shift Button Action
    m_shiftButtonInput->setButtons(QList<int> { Qt::Key_Shift });
    m_shiftButtonInput->setSourceDevice(m_keyboardDevice);
    m_shiftButtonAction->addInput(m_shiftButtonInput);

    // Mouse X
    m_mouseRxInput->setAxis(Qt3DInput::QMouseDevice::X);
    m_mouseRxInput->setSourceDevice(m_mouseDevice);
    m_rxAxis->addInput(m_mouseRxInput);

    // Mouse Y
    m_mouseRyInput->setAxis(Qt3DInput::QMouseDevice::Y);
    m_mouseRyInput->setSourceDevice(m_mouseDevice);
    m_ryAxis->addInput(m_mouseRyInput);

    prepareAxis(m_keyboardDevice, m_txAxis, m_keyboardTxPInput, QList<int> { Qt::Key_D }, m_keyboardTxNInput, QList<int> { Qt::Key_A });
    prepareAxis(m_keyboardDevice, m_tyAxis, m_keyboardTyPInput, QList<int> { Qt::Key_Space }, m_keyboardTyNInput, QList<int> { Qt::Key_Shift });
    prepareAxis(m_keyboardDevice, m_tzAxis, m_keyboardTzPInput, QList<int> { Qt::Key_W }, m_keyboardTzNInput, QList<int> { Qt::Key_S });

    m_logicalDevice->addAction(m_escapeButtonAction);
    m_logicalDevice->addAction(m_shiftButtonAction);
    m_logicalDevice->addAxis(m_rxAxis);
    m_logicalDevice->addAxis(m_ryAxis);
    m_logicalDevice->addAxis(m_txAxis);
    m_logicalDevice->addAxis(m_tyAxis);
    m_logicalDevice->addAxis(m_tzAxis);

    applyInputAccelerations();

    QObject::connect(m_escapeButtonAction, &Qt3DInput::QAction::activeChanged, this, [this](bool isActive) {
        if (m_camera && isActive) {
            m_active = !m_active;

            if (m_active) engage();
            else disengage();
        }
    });

    QObject::connect(m_frameAction, &Qt3DLogic::QFrameAction::triggered, this, [this] (float dt) {
        if (m_active) moveCamera(dt);
    });

    addComponent(m_frameAction);
    addComponent(m_logicalDevice);
}

MonitorCameraController::~MonitorCameraController()
{
}

void MonitorCameraController::prepareAxis(Qt3DInput::QAbstractPhysicalDevice *source, Qt3DInput::QAxis *axis,
                 Qt3DInput::QButtonAxisInput *pos, const QList<int>& posKeys,
                 Qt3DInput::QButtonAxisInput *neg, const QList<int>& negKeys)
{
    pos->setButtons(posKeys);
    pos->setScale(1.0f);
    pos->setSourceDevice(source);
    axis->addInput(pos);

    neg->setButtons(negKeys);
    neg->setScale(-1.0f);
    neg->setSourceDevice(source);
    axis->addInput(neg);
}

void MonitorCameraController::applyInputAccelerations()
{
    const auto inputs = {
            m_keyboardTxPInput,
            m_keyboardTyPInput,
            m_keyboardTzPInput,
            m_keyboardTxNInput,
            m_keyboardTyNInput,
            m_keyboardTzNInput
    };

    for (auto input : inputs) {
        input->setAcceleration(m_acceleration);
        input->setDeceleration(m_deceleration);
    }
}

void MonitorCameraController::setCamera(Qt3DRender::QCamera *camera)
{
    if (m_camera != camera) {
        if (camera && !camera->parent()) camera->setParent(this);

        m_camera = camera;
    }
}

void MonitorCameraController::linkContainer(QWidget *container)
{
    m_container = container;
}

void MonitorCameraController::engage()
{
    if (m_container) {
        m_mouseDevice->setUpdateAxesContinuously(true);
        QGuiApplication::setOverrideCursor(Qt::BlankCursor);
        restrainCursor();

    }
}

void MonitorCameraController::disengage()
{
    if (m_container) {
        m_mouseDevice->setUpdateAxesContinuously(false);
        QGuiApplication::restoreOverrideCursor();
        restrainCursor();
    }
}

void MonitorCameraController::restrainCursor()
{
    m_ignore = false;

    if (m_container) {
        QPoint center = m_container->mapToGlobal(m_container->rect().center());
        if ((center - QCursor::pos()).manhattanLength() > 300) {
            QCursor::setPos(center);
            m_ignore = true;
        }
    }
}

void MonitorCameraController::moveCamera(float dt)
{
    if (m_camera == nullptr) return;

    QVector3D delta = QVector3D{
        m_txAxis->value(),
        m_tyAxis->value(),
        m_tzAxis->value()
    } * m_linear * dt;

    m_camera->translate(delta);

    if (!m_ignore) {// TODO use a less jank method
        m_camera->pan(m_rxAxis->value() * m_look * dt, QVector3D(0, 1, 0));
        m_camera->tilt(m_ryAxis->value() * m_look * dt);
    }

    restrainCursor();
}