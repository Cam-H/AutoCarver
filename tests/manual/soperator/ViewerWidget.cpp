//
// Created by cjhat on 2025-09-17.
//

#include "ViewerWidget.h"

#include "geometry/primitives/Circle.h"
#include "geometry/poly/Profile.h"

#include "geometry/collision/Collision2D.h"
#include "robot/planning/RRTPlanner.h"

#include <iostream>
#include <QPainterPath>

ViewerWidget::ViewerWidget(QWidget *parent)
    : PolygonWidget(parent)
    , profile(nullptr)
    , m_path({})
    , m_boundsEnable(true)
    , m_treeEnable(true)
    , m_pathEnable(true)
{
    QObject::connect(this, &PolygonWidget::moved, this, &ViewerWidget::moved);

}

void ViewerWidget::setProfile(const Profile* profile)
{
    this->profile = profile;
    m_selection = { -1, -1 };
    updatePolygons();
}

void ViewerWidget::updatePolygons()
{
    for (auto& m_polygon : m_polygons) delete m_polygon;
    m_polygons.clear();

    if (profile != nullptr) {
        const auto& polygons = profile->decompose();
        for (const Polygon& poly : polygons) {
            addPolygon(new Polygon(poly));
            m_polyControl.back().editable = false;
        }

        m_coreCount = polygons.size() - profile->remainingSections();

        if (profile->planner() != nullptr) {
            addPolygon(nullptr);
            m_polyControl.back().editable = false;
            updateTool();
        }

        center();
    }

    repaint();
}

void ViewerWidget::updateTool()
{
    if (profile != nullptr && profile->planner() != nullptr) updateTool(profile->planner()->start());
}

void ViewerWidget::updateTool(const glm::dvec3& location)
{
    if (profile != nullptr) {
        delete m_polygons.back();

        auto [w, t] = profile->toolDimensions();
        m_polygons.back() = new Polygon({ location.x, location.y }, w, t, location.z);
    }
}

void ViewerWidget::updatePath()
{
    if (profile != nullptr) m_path = profile->nextPath();
}

void ViewerWidget::enableBounds(bool enable)
{
    m_boundsEnable = enable;
}

void ViewerWidget::enableTree(bool enable)
{
    m_treeEnable = enable;
}

void ViewerWidget::enablePath(bool enable)
{
    m_pathEnable = enable;
}

const std::vector<glm::dvec3>& ViewerWidget::path() const
{
    return m_path;
}

//void PlannerWidget::paintEvent(QPaintEvent *event)
//{
//    PolygonWidget::paintEvent(event);
//}

void ViewerWidget::drawForeground(QPainter& painter)
{
    const RRTPlanner* planner = profile == nullptr ? nullptr : profile->planner();

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

        if (profile != nullptr) {
            const Polygon& wrapper = profile->wrapper();
            const std::vector<glm::dvec2>& border = wrapper.border();

            painter.setPen(Qt::darkCyan);
            for (uint32_t i = 0; i < border.size(); i++) {
                const glm::vec2& a = transformed(border[i]);
                const glm::vec2& b = transformed(border[(i + 1) % border.size()]);
                painter.drawLine(QPointF(a.x, a.y), QPointF(b.x, b.y));
            }

            if (!profile->complete()) {
                auto tri = profile->nextTriangle();
                Ray<2> ray(profile->border()[tri.I0], glm::normalize(profile->border()[tri.I2] - profile->border()[tri.I0]));

                auto [hit, tr, tf] = Collision2D::intersection(wrapper, ray);
//                std::cout << "P " << hit << " " << tr << " " << tf << "\n";
                if (hit) {
                    const glm::vec2& a = transformed(tr * ray);
                    const glm::vec2& b = transformed(tf * ray);
                    painter.drawLine(QPointF(a.x, a.y), QPointF(b.x, b.y));
                }
            }
        }

        if (planner != nullptr) {
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

        std::vector<Polygon> wps = { planner->bounds(planner->start()), planner->bounds(planner->goal()) };
        std::vector<QColor> colors = { Qt::darkGreen, Qt::green };
        for (const Polygon& poly : wps) {
            const std::vector<glm::dvec2>& border = poly.border();
            QPainterPath path;

            const glm::vec2& origin = transformed(border[0]);
            path.moveTo(QPointF(origin.x, origin.y));

            for (uint32_t i = 1; i < border.size(); i++) {
                const glm::vec2& vertex = transformed(border[i]);
                path.lineTo(QPointF(vertex.x, vertex.y));
            }

            painter.fillPath(path, colors[&poly - &wps[0]]);
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

        glm::dvec2 prev = transformed({ m_path[0].x, m_path[0].y });
        for (const glm::dvec3& vertex : m_path) {
            glm::dvec2 next = transformed({ vertex.x, vertex.y });
            painter.drawLine(prev.x, prev.y, next.x, next.y);
            prev = next;
        }
    }

    PolygonWidget::drawForeground(painter);

    if (planner != nullptr) {
        painter.setPen(Qt::blue);

        std::string text = "Iterations: " + std::to_string(planner->iterations()) + ", nodes: " + std::to_string(planner->nodes().size());
        if (planner->solved()) {
            const auto& sol = planner->lastSolution();
            text += ", solution cost: " + std::to_string(sol.second);
        }

        painter.drawText(50, 20, text.c_str());
    }
}

void ViewerWidget::drawPolygon(uint32_t index, QPainter& painter, QColor color)
{
    if (isStart(index)) {
        for (uint32_t i = 0; i < m_polygons.size(); i++) {
            if (i != index && Collision2D::test(*m_polygons[index], *m_polygons[i])) {
                color = Qt::red;
                break;
            }
        }
    } else if (index >= m_coreCount) color = color.lighter(50);

    PolygonWidget::drawPolygon(index, painter, color);
}

void ViewerWidget::moved(uint32_t selection, const glm::dvec2& delta)
{
    if (isStart(selection)) emit startMoved(delta);
}

bool ViewerWidget::isStart(uint32_t index) const
{
    return index == m_polygons.size() - 1;
}