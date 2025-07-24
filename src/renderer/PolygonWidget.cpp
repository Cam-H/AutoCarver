//
// Created by Cam on 2025-06-19.
//

#include "PolygonWidget.h"

#include "geometry/poly/Polygon.h"

#include <QPainter>

#include <iostream>
#include <algorithm>

PolygonWidget::PolygonWidget(QWidget *parent)
        : QWidget(parent)
        , m_poly(nullptr)
        , m_selection(-1)
        , m_polygon(true)
        , m_triangulation(false)
        , m_hull(false)
        , m_debugEdges(true)
        , m_centered(false)
        , m_scalar(1.0f)
        , m_offset()
{
    grabKeyboard();
}

void PolygonWidget::setPolygon(Polygon *polygon)
{
    if (m_poly == polygon) return;

    m_poly = polygon;
    m_selection = -1;
    std::cout << "CCW: " << polygon->isCCW() << "\n";
    if (m_centered) center();
    else repaint();
}

void PolygonWidget::uncenter()
{
    if (!m_centered) return;
    m_centered = false;
    repaint();
}

void PolygonWidget::center()
{
    if (m_poly == nullptr) return;

    const auto& border = m_poly->border();

    auto [xMin, xMax] = std::minmax_element(border.begin(), border.end(), [](const glm::vec2& lhs, const glm::vec2& rhs){
        return lhs.x < rhs.x;
    });

    auto [yMin, yMax] = std::minmax_element(border.begin(), border.end(), [](const glm::vec2& lhs, const glm::vec2& rhs){
        return lhs.y < rhs.y;
    });

    glm::vec2 size = { width(), height() };

    double margin = 0.1, im = 1.0 - margin;
    m_scalar = std::min(im * size.x / ((*xMax).x - (*xMin).x), im * size.y / ((*yMax).y - (*yMin).y));

    m_offset = 0.5 * (-m_scalar * glm::dvec2{
            (*xMax).x + (*xMin).x,
            -((*yMax).y + (*yMin).y)
    } + glm::dvec2{ size.x, size.y });

    m_centered = true;
    repaint();
}

void PolygonWidget::setCentered(bool centered)
{
    if (centered) center();
    else uncenter();
}

glm::vec2 PolygonWidget::transformed(const glm::vec2& vertex) const
{
    if (!m_centered) return { vertex.x, height() - vertex.y };

    return {
             m_scalar * vertex.x + m_offset.x,
            -m_scalar * vertex.y + m_offset.y
    };
}

glm::vec2 PolygonWidget::invTransformed(const glm::vec2& vertex) const
{
    if (!m_centered) return { vertex.x, height() - vertex.y };

    return glm::dvec2{
             vertex.x - m_offset.x,
            -vertex.y + m_offset.y
    } / m_scalar;
}

void PolygonWidget::enablePolygon(bool enable)
{
    m_polygon = enable;
    repaint();
}

void PolygonWidget::enableTriangulation(bool enable)
{
    m_triangulation = enable;
    repaint();
}

void PolygonWidget::enableHull(bool enable)
{
    m_hull = enable;
    repaint();
}

Polygon* PolygonWidget::getPolygon()
{
    return m_poly;
}

void PolygonWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    // Draw background
    painter.setBrush(Qt::white);
    painter.setPen(Qt::black);
    painter.drawRect(0, 0, width(), height());

    if (m_poly == nullptr) return;

    QPen borderPen(Qt::black);
    borderPen.setWidth(2.0);

    QPen partitionPen(Qt::blue);
    partitionPen.setWidth(2.0);
    QPen triPen(Qt::cyan);
    triPen.setWidth(1.0);

    std::vector<QColor> colorSet = {Qt::green, Qt::cyan, Qt::yellow, Qt::blue, Qt::red, Qt::darkBlue, Qt::magenta, Qt::gray, Qt::black, Qt::darkCyan};
    int idx = 0;

    // Draw tesselation results
    if (m_triangulation) {
        Polygon *poly = getPolygon();
        std::vector<Triangle> triangles = poly->triangulate();

        for (const Triangle &tri : triangles) {
            painter.setPen(colorSet[idx]);
            if (++idx >= colorSet.size()) idx = 0;

            glm::vec2 a = transformed(m_poly->border()[tri.I0]);
            glm::vec2 b = transformed(m_poly->border()[tri.I1]);
            glm::vec2 c = transformed(m_poly->border()[tri.I2]);

            QPoint ap = {(int)a.x, (int)a.y};
            QPoint bp = {(int)b.x, (int)b.y};
            QPoint cp = {(int)c.x, (int)c.y};

            painter.drawLine(ap, bp);
            painter.drawLine(bp, cp);
            painter.drawLine(cp, ap);
        }
    }

    // Draw partition results
    if (m_hull) {
        Polygon *poly = getPolygon();
        std::vector<uint32_t> hull = poly->hull();

        painter.setPen(colorSet[idx]);

        for(uint32_t i = 0; i < hull.size(); i++) {
            const glm::vec2& a = transformed(m_poly->border()[hull[i]]);
            const glm::vec2& b = transformed(m_poly->border()[hull[(i + 1) % hull.size()]]);
            painter.drawLine(QPointF(a.x, a.y), QPointF(b.x, b.y));
        }
    }

    // Draw edges generated for debugging purposes
    if (m_debugEdges) {
        painter.setPen(colorSet[idx]);
        const auto& edges = m_poly->debugEdges();
        for (const auto& edge : edges) {
            const glm::vec2 a = transformed(edge.first);
            const glm::vec2 b = transformed(edge.second);
            painter.drawLine(QPointF(a.x, a.y), QPointF(b.x, b.y));
        }
    }

    // Draw border
    if (m_polygon) {
        painter.setBrush(Qt::black);
        painter.setPen(borderPen);
        for (uint16_t i = 0; i < m_poly->vertexCount(); i++) {
            const glm::vec2& a = transformed(m_poly->border()[i]);
            const glm::vec2& b = transformed(m_poly->border()[(i + 1) % m_poly->vertexCount()]);
            painter.drawLine(QPointF(a.x, a.y), QPointF(b.x, b.y));
        }
    }

    // Draw vertices
    painter.setPen(Qt::black);
    for (const glm::vec2& vertex : m_poly->border()) {
        const glm::vec2 vec = transformed(vertex);
        painter.drawEllipse(QPoint((int)vec.x, (int)vec.y), 3, 3);
    }

    // Style selected vertex
    if (m_selection != -1) {
        painter.setBrush(Qt::blue);
        const glm::vec2& vertex = transformed(m_poly->border()[m_selection]);

        painter.drawEllipse(QPoint((int)vertex.x, (int)vertex.y), 5, 5);

        painter.drawText(20, 20, QString(std::to_string(m_selection).c_str()));
    }
}

void PolygonWidget::mousePressEvent(QMouseEvent *event)
{
    if (m_poly == nullptr) return;

    if (event->button() == Qt::LeftButton) {
        m_selection = -1;

        glm::vec2 mouse = invTransformed({ event->pos().x(), event->pos().y() });
        for (uint16_t i = 0; i < m_poly->vertexCount(); i++) {
            const glm::vec2& vertex = m_poly->border()[i];
            if (glm::length(vertex - mouse) < 8 / m_scalar) {
                m_selection = i;
                break;
            }
        }

        repaint();
    }
}

void PolygonWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (m_poly == nullptr) return;

    if ((event->buttons() & Qt::LeftButton) && m_selection != -1) {
        m_poly->positionVertex(m_selection, invTransformed({ event->pos().x(), event->pos().y() }));
        repaint();
    }
}

void PolygonWidget::mouseReleaseEvent(QMouseEvent *event) {}

void PolygonWidget::keyPressEvent(QKeyEvent *event)
{
    if (m_poly == nullptr) return;

    if (m_selection != -1) {
        switch (event->key()) {
            case Qt::Key_Delete:
                if (m_poly->vertexCount() > 3) {
                    m_poly->removeVertex(m_selection);
                    m_selection = -1;
                    repaint();
                }
                break;
            case Qt::Key_Control:
                m_poly->insertVertex(m_selection, m_poly->border()[m_selection]);
                m_selection++;
                repaint();
                break;
        }
    }
}