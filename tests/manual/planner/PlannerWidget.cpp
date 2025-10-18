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
    , planner(nullptr)
    , m_path({})
    , m_boundsEnable(true)
    , m_treeEnable(true)
    , m_pathEnable(true)
{
    QObject::connect(this, &PolygonWidget::moved, this, &PlannerWidget::moved);
}

void PlannerWidget::setStart(const glm::dvec3& start, double width, double height)
{
    addPolygon(new Polygon({ start.x, start.y }, width, height, start.z));
    m_polyControl.back().editable = false;

    m_path = PiecewisePolyPath({});
}
void PlannerWidget::setGoal(const glm::dvec3& goal, double width, double height)
{
    addPolygon(new Polygon({ goal.x, goal.y }, width, height, goal.z));
    m_polyControl.back().editable = false;

    m_path = PiecewisePolyPath({});
}

void PlannerWidget::updateStart(const glm::dvec3& start, double width, double height)
{
    if (m_polygons.size() < 2) return;

    delete m_polygons[m_polygons.size() - 2];
    m_polygons[m_polygons.size() - 2] = new Polygon({ start.x, start.y }, width, height, start.z);

    m_path = PiecewisePolyPath({});
}
void PlannerWidget::updateGoal(const glm::dvec3& goal, double width, double height)
{
    if (m_polygons.size() < 2) return;

    delete m_polygons[m_polygons.size() - 1];
    m_polygons[m_polygons.size() - 1] = new Polygon({ goal.x, goal.y }, width, height, goal.z);

    m_path = PiecewisePolyPath({});
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

void PlannerWidget::setPath(const PiecewisePolyPath& path)
{
    m_path = path;
}

void PlannerWidget::enableBounds(bool enable)
{
    m_boundsEnable = enable;
}

void PlannerWidget::enableTree(bool enable)
{
    m_treeEnable = enable;
}

void PlannerWidget::enablePath(bool enable)
{
    m_pathEnable = enable;
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

        painter.setPen(Qt::blue);
        switch (planner->getSampler()) {
            case RRTPlanner::SampleSpace::BOX:
            {
                auto [min, max] = planner->getBounds();
                min = transformed(min);
                max = transformed(max);

                painter.drawLine(min.x, min.y, max.x, min.y);
                painter.drawLine(max.x, min.y, max.x, max.y);
                painter.drawLine(max.x, max.y, min.x, max.y);
                painter.drawLine(min.x, max.y, min.x, min.y);
            }
                break;
            case RRTPlanner::SampleSpace::TRIANGLE:
            {
                Triangle2D tri = planner->getTriBounds();
                tri.a = transformed(tri.a);
                tri.b = transformed(tri.b);
                tri.c = transformed(tri.c);

                painter.drawLine(tri.a.x, tri.a.y, tri.b.x, tri.b.y);
                painter.drawLine(tri.b.x, tri.b.y, tri.c.x, tri.c.y);
                painter.drawLine(tri.c.x, tri.c.y, tri.a.x, tri.a.y);

            }
                break;
            case RRTPlanner::SampleSpace::CIRCLE:
                Circle circ = planner->getCircBounds();
                circ.center = transformed(circ.center);
                circ.radius *= m_scalar;

                painter.drawEllipse(QPointF(circ.center.x, circ.center.y), circ.radius, circ.radius);
                break;
        }
    }

    QColor color(0, 0, 0);
    painter.setPen(color);
    painter.setBrush(Qt::NoBrush);

    // Draw planned paths if available
    if (m_treeEnable && planner != nullptr) {
        const auto& nodes = planner->nodes();

        for (const Node& node : nodes) {
            glm::dvec3 start = node.parent == std::numeric_limits<uint32_t>::max()
                               ? planner->start() : nodes[node.parent].location;

            float cc = start.z / M_PI;
            color.setRedF(cc);
            color.setBlueF(cc);
            painter.setPen(color);

            glm::dvec2 a = transformed({ start.x, start.y }), b = transformed({ node.location.x, node.location.y });
            painter.drawLine(a.x, a.y, b.x, b.y);
        }
    }

    if (m_pathEnable && !m_path.empty()) {
        color.setRgb(0, 0, 0);
        painter.setPen(color);

        std::vector<double> wp = m_path.evaluate(0);
        glm::dvec2 last = transformed({ wp[0], wp[1] });

        double dt = m_path.sEnd() / 100, t = dt;
        while (t < m_path.sEnd()) {
            wp = m_path.evaluate(t);

            glm::dvec2 vertex = transformed({ wp[0], wp[1] });
            painter.drawLine(last.x, last.y, vertex.x, vertex.y);

            last = vertex;
            t += dt;
        }

        for (uint32_t i = 0; i < m_path.segments(); i++) {
            wp = m_path[i];

            glm::dvec2 vertex = transformed({ wp[0], wp[1] });

            color.setBlueF(wp[2] / (2 * M_PI));
            color.setBlueF(1.0 - wp[2] / (2 * M_PI));
            painter.setPen(color);
            painter.setBrush(color);

            painter.drawEllipse(QPointF(vertex.x, vertex.y), 2, 2);
        }
    }

    PolygonWidget::drawForeground(painter);

    if (planner != nullptr) {
        painter.setPen(Qt::green);

        glm::dvec2 start = { planner->start().x, planner->start().y }, goal = { planner->goal().x, planner->goal().y };

        auto [axis, r] = planner->profile(planner->start(), planner->goal());

        glm::dvec2 norm = { axis.y, -axis.x };
        r *= m_scalar;

        glm::dvec2 a = transformed(start), b = transformed(goal);
        painter.drawLine(a.x, a.y, b.x, b.y);

        glm::dvec2 vas = a + r * glm::dvec2(norm.x, -norm.y), vaf = a - r * glm::dvec2(norm.x, -norm.y);
        glm::dvec2 vbs = b + r * glm::dvec2(norm.x, -norm.y), vbf = b - r * glm::dvec2(norm.x, -norm.y);

        painter.drawLine(vas.x, vas.y, vaf.x, vaf.y);
        painter.drawLine(vas.x, vas.y, vbs.x, vbs.y);
        painter.drawLine(vaf.x, vaf.y, vbf.x, vbf.y);
        painter.drawLine(vbs.x, vbs.y, vbf.x, vbf.y);

        painter.setPen(Qt::black);

        std::string text = "Iterations: " + std::to_string(planner->iterations()) + ", nodes: " + std::to_string(planner->nodes().size());
        if (planner->solved()) {
            const auto& sol = planner->lastSolution();
            text += ", solution cost: " + std::to_string(sol.second);
        }

        painter.drawText(50, 20, text.c_str());
    }
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