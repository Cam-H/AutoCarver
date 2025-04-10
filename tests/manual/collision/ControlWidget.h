//
// Created by Cam on 2025-04-02.
//

#ifndef AUTOCARVER_CONTROLWIDGET_H
#define AUTOCARVER_CONTROLWIDGET_H


#include "renderer/SceneWidget.h"

#include <QKeyEvent>

class ControlWidget : public SceneWidget
{
Q_OBJECT

public:
    using SceneWidget::SceneWidget;

protected:
    void keyPressEvent(QKeyEvent *e) override;

};


#endif //AUTOCARVER_CONTROLWIDGET_H
