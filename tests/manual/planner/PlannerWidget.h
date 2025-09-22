//
// Created by cjhat on 2025-09-17.
//

#ifndef AUTOCARVER_PLANNERWIDGET_H
#define AUTOCARVER_PLANNERWIDGET_H


#include <QWidget>

#include <QMouseEvent>
#include <QKeyEvent>

#include <vector>

#include <glm.hpp>

class Polygon;

#include "renderer/PolygonWidget.h"
#include "Configuration.h"
#include "RRTPlanner.h"

class PlannerWidget : public PolygonWidget {
Q_OBJECT

signals:
    void startMoved(const glm::dvec2& delta);
    void goalMoved(const glm::dvec2& delta);

public slots:
    void moved(uint32_t selection, const glm::dvec2& delta);

public:

    explicit PlannerWidget(QWidget *parent = nullptr);

    void setStart(const glm::dvec3& start, double width, double height);
    void setGoal(const glm::dvec3& goal, double width, double height);

    void updateStart(const glm::dvec3& start, double width, double height);
    void updateGoal(const glm::dvec3& goal, double width, double height);

    void offsetStart(const glm::dvec3& offset);

    void setPlanner(const RRTPlanner* planner);

    void enableBounds(bool enable);

protected:
//    void paintEvent(QPaintEvent *event) override;

    void drawForeground(QPainter& painter) override;
    void drawPolygon(uint32_t index, QPainter& painter, QColor color) override;

    [[nodiscard]] bool isStart(uint32_t index) const;
    [[nodiscard]] bool isGoal(uint32_t index) const;

private:

    const RRTPlanner* planner;

    bool m_boundsEnable;

};

#endif //AUTOCARVER_DEBUGWIDGET_H
