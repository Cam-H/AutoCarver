//
// Created by cjhat on 2025-09-17.
//

#include "PlannerWidget.h"

#include "geometry/primitives/Circle.h"
#include "geometry/poly/Polygon.h"

#include "geometry/collision/Collision2D.h"

#include <iostream>

PlannerWidget::PlannerWidget(QWidget *parent)
    : PolygonWidget(parent)
    , m_boundsEnable(true)
{
    QObject::connect(this, &PolygonWidget::moved, this, &PlannerWidget::moved);
}

void PlannerWidget::setStart(const glm::dvec3& start, double width, double height)
{
    addPolygon(new Polygon({ start.x, start.y }, width, height, start.z));
    m_polyControl.back().editable = false;
}
void PlannerWidget::setGoal(const glm::dvec3& goal, double width, double height)
{
    addPolygon(new Polygon({ goal.x, goal.y }, width, height, goal.z));
    m_polyControl.back().editable = false;
}

void PlannerWidget::updateStart(const glm::dvec3& start, double width, double height)
{
    if (m_polygons.size() < 2) return;

    delete m_polygons[m_polygons.size() - 2];
    m_polygons[m_polygons.size() - 2] = new Polygon({ start.x, start.y }, width, height, start.z);
}
void PlannerWidget::updateGoal(const glm::dvec3& goal, double width, double height)
{
    if (m_polygons.size() < 2) return;

    delete m_polygons[m_polygons.size() - 1];
    m_polygons[m_polygons.size() - 1] = new Polygon({ goal.x, goal.y }, width, height, goal.z);
}

void PlannerWidget::offsetStart(const glm::dvec3& offset)
{
    if (m_polygons.size() < 2) return;
    m_polygons[m_polygons.size() - 2]->translate({ offset.x, offset.y });
    m_polygons[m_polygons.size() - 2]->rotate(offset.z);
}

void PlannerWidget::setPlanner(const RRTPlanner* planner)
{
    this->planner = planner;
}

void PlannerWidget::enableBounds(bool enable)
{
    m_boundsEnable = enable;
}

//void PlannerWidget::paintEvent(QPaintEvent *event)
//{
//    PolygonWidget::paintEvent(event);
//}

void PlannerWidget::drawForeground(QPainter& painter)
{
    // Draw bounding circles around all polygons if enabled
    if (m_boundsEnable) {
        painter.setPen(Qt::gray);
        painter.setBrush(Qt::NoBrush);

        for (uint32_t i = 0; i < m_polygons.size(); i++) {
            if (m_polyControl[i].visible) {

                const Circle& bounds = m_polygons[i]->bounds();
                const glm::dvec2 center = transformed(bounds.center);

                painter.drawEllipse(QPointF(center.x, center.y), m_scalar * bounds.radius, m_scalar * bounds.radius);

            }
        }
    }

    // Draw planned paths if available
    if (planner != nullptr) {
        QColor color(0, 0, 0);
        painter.setPen(color);
        painter.setBrush(Qt::NoBrush);

        for (const Node& node : planner->nodes) {
            glm::dvec3 start = node.parent == std::numeric_limits<uint32_t>::max()
                               ? planner->start() : planner->nodes[node.parent].location;

            color.setRedF((node.location.z - start.z) / (2 * M_PI)); // Red  - Change in rotation
            color.setBlueF(start.z / (2 * M_PI));                    // Blue - Initial rotation
            painter.setPen(color);

            glm::dvec2 a = transformed({ start.x, start.y }), b = transformed({ node.location.x, node.location.y });
            painter.drawLine(a.x, a.y, b.x, b.y);
        }
    }

    PolygonWidget::drawForeground(painter);
}

void PlannerWidget::drawPolygon(uint32_t index, QPainter& painter, QColor color)
{
    if (isStart(index) || isGoal(index)) {
        for (uint32_t i = 0; i < m_polygons.size(); i++) {
            if (i != index && Collision2D::test(*m_polygons[index], *m_polygons[i])) {
                color = Qt::red;
                break;
            }
        }
    }

    PolygonWidget::drawPolygon(index, painter, color);
}

void PlannerWidget::moved(uint32_t selection, const glm::dvec2& delta)
{
    if (isStart(selection)) emit startMoved(delta);
    if (isGoal(selection)) emit goalMoved(delta);
}

bool PlannerWidget::isStart(uint32_t index) const
{
    return index == m_polygons.size() - 2;
}
bool PlannerWidget::isGoal(uint32_t index) const
{
    return index == m_polygons.size() - 1;
}