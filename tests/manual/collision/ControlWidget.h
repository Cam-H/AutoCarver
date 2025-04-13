//
// Created by Cam on 2025-04-02.
//

#ifndef AUTOCARVER_CONTROLWIDGET_H
#define AUTOCARVER_CONTROLWIDGET_H


#include "renderer/SceneWidget.h"

#include <QKeyEvent>

class Scene;
class Body;

class ControlWidget : public SceneWidget
{
Q_OBJECT

public:
//    using SceneWidget::SceneWidget;
    ControlWidget(Scene* scene = nullptr, QWidget* parent = nullptr);


protected:
    void keyPressEvent(QKeyEvent *e) override;

private:

    std::vector<std::shared_ptr<Body>> m_points;

};


#endif //AUTOCARVER_CONTROLWIDGET_H
