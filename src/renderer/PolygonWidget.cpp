//
// Created by Cam on 2025-06-19.
//

#include "PolygonWidget.h"

#include "geometry/poly/Polygon.h"

#include <iostream>
#include <algorithm>
#include <QPainterPath>

std::vector<QColor> PolygonWidget::COLOR_SET = {
        Qt::green,
        Qt::cyan,
        Qt::yellow,
        Qt::blue,
        Qt::red,
        Qt::darkBlue,
        Qt::magenta,
        Qt::gray,
        Qt::black,
        Qt::darkCyan
};

PolygonWidget::PolygonWidget(QWidget *parent)
        : QWidget(parent)
        , m_selection(-1, -1)
        , m_polygon(true)
        , m_triangulation(false)
        , m_hull(false)
        , m_debugEdges(true)
        , m_fill(false)
        , m_vertices(true)
        , m_centered(false)
        , m_scalar(1.0f)
        , m_offset()
        , m_down()
        , m_borderPen(Qt::black)
        , m_partitionPen(Qt::blue)
        , m_triPen(Qt::cyan)
{
    setFocusPolicy(Qt::FocusPolicy::ClickFocus);

    m_borderPen.setWidth(2.0);
    m_partitionPen.setWidth(2.0);
    m_triPen.setWidth(1.0);
}

void PolygonWidget::setPolygon(Polygon* poly)
{
    m_polygons = { poly };
    m_polyControl = { {} };
    m_selection = { -1, -1 };

    std::cout << "CCW: " << m_polygons[0]->isCCW() << "\n";
    if (m_centered) center();
    else repaint();
}

void PolygonWidget::clearPolygons()
{
    m_polygons.clear();
    m_selection = { -1, -1 };
    m_polyControl.clear();
}
void PolygonWidget::addPolygon(Polygon* poly)
{
    m_polygons.emplace_back(poly);
    m_polyControl.emplace_back();
}

void PolygonWidget::focusPolygon(uint32_t idx)
{
    m_polyControl = std::vector<PolyControl>(m_polygons.size(), false);
    if (idx < m_polyControl.size()) m_polyControl[idx].visible = true;
    m_selection = { -1, -1 };

    if (m_centered) center();
    else repaint();
}

void PolygonWidget::showAll()
{
    m_polyControl = std::vector<PolyControl>(m_polygons.size(), true);
}

void PolygonWidget::uncenter()
{
    if (!m_centered) return;
    m_centered = false;
    repaint();
}

void PolygonWidget::center()
{
    m_centered = true;

    if (m_polygons.empty()) return;

    auto [xMin, xMax, yMin, yMax] = bounds();
    glm::vec2 size = { width(), height() };

    double margin = 0.1, im = 1.0 - margin;
    m_scalar = std::min(im * size.x / (xMax - xMin), im * size.y / (yMax - yMin));

    m_offset = 0.5 * (-m_scalar * glm::dvec2{
            xMax + xMin,
            -(yMax + yMin)
    } + glm::dvec2{ size.x, size.y });

    repaint();
}

std::tuple<double, double, double, double> PolygonWidget::bounds() const
{
    double xMin = std::numeric_limits<double>::max(), xMax = std::numeric_limits<double>::lowest(), yMin = xMin, yMax = xMax;
    for (uint32_t i = 0; i < m_polygons.size(); i++) {
        if (m_polyControl[i].visible) {
            auto [pXMin, pXMax, pYMin, pYMax] = bounds(m_polygons[i]);
            if (pXMin < xMin) xMin = pXMin;
            if (pXMax > xMax) xMax = pXMax;
            if (pYMin < yMin) yMin = pYMin;
            if (pYMax > yMax) yMax = pYMax;
        }
    }

    return { xMin, xMax, yMin, yMax };
}
std::tuple<double, double, double, double> PolygonWidget::bounds(Polygon* poly)
{
    const auto& border = poly->border();

    auto [xMin, xMax] = std::minmax_element(border.begin(), border.end(), [](const glm::vec2& lhs, const glm::vec2& rhs){
        return lhs.x < rhs.x;
    });

    auto [yMin, yMax] = std::minmax_element(border.begin(), border.end(), [](const glm::vec2& lhs, const glm::vec2& rhs){
        return lhs.y < rhs.y;
    });

    return { (*xMin).x, (*xMax).x, (*yMin).y, (*yMax).y };
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

void PolygonWidget::enableFill(bool enable)
{
    m_fill = enable;
}
void PolygonWidget::enableVertices(bool enable)
{
    m_vertices = enable;
}

bool PolygonWidget::isCentered() const
{
    return m_centered;
}

uint32_t PolygonWidget::polygonCount() const
{
    return m_polygons.size();
}

const std::vector<Polygon*>& PolygonWidget::getPolygons() const
{
    return m_polygons;
}

Polygon* PolygonWidget::getPolygon(uint32_t idx)
{
    assert(idx < m_polygons.size());
    return m_polygons[idx];
}

int PolygonWidget::selection() const
{
    return m_selection.first;
}

void PolygonWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);

    // Draw background
    painter.setBrush(Qt::white);
    painter.setPen(Qt::black);
    painter.drawRect(0, 0, width(), height());

    drawForeground(painter);
}
void PolygonWidget::drawForeground(QPainter& painter)
{
    for (uint32_t i = 0; i < m_polygons.size(); i++) {
        if (m_polyControl[i].visible) paint(painter, i);
    }

    // Style selected vertex
    if (m_selection.first != -1 && m_selection.second != -1) {
        painter.setPen(Qt::NoPen);
        painter.setBrush(Qt::blue);
        const glm::vec2& vertex = transformed(m_polygons[m_selection.first]->border()[m_selection.second]);

        painter.drawEllipse(QPoint((int)vertex.x, (int)vertex.y), 5, 5);
    }

    if (m_selection.first != -1) {
        std::string text = "[" + std::to_string(m_selection.first) + ": " + std::to_string(m_selection.second) + "]";

        painter.setPen(Qt::darkCyan);
        painter.drawText(10, 20, text.c_str());
    }
}

void PolygonWidget::paint(QPainter& painter, uint32_t index)
{
    Polygon* poly = m_polygons[index];
    int idx = 0;

    const std::vector<glm::dvec2>& border = poly->border();

    // Draw tesselation results
    if (m_triangulation) {
        std::vector<TriIndex> triangles = poly->triangulate();

        for (const TriIndex &tri : triangles) {
            painter.setPen(COLOR_SET[idx]);
            if (++idx >= COLOR_SET.size()) idx = 0;

            glm::vec2 a = transformed(border[tri.I0]);
            glm::vec2 b = transformed(border[tri.I1]);
            glm::vec2 c = transformed(border[tri.I2]);

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
        std::vector<uint32_t> hull = poly->hull();

        painter.setPen(COLOR_SET[idx]);

        for(uint32_t i = 0; i < hull.size(); i++) {
            const glm::vec2& a = transformed(border[hull[i]]);
            const glm::vec2& b = transformed(border[hull[(i + 1) % hull.size()]]);
            painter.drawLine(QPointF(a.x, a.y), QPointF(b.x, b.y));
        }
    }

    // Draw edges generated for debugging purposes
    if (m_debugEdges) {
        painter.setPen(COLOR_SET[idx]);
        const auto& edges = poly->debugEdges();
        for (const auto& edge : edges) {
            const glm::vec2 a = transformed(edge.first);
            const glm::vec2 b = transformed(edge.second);
            painter.drawLine(QPointF(a.x, a.y), QPointF(b.x, b.y));
        }
    }

    // Draw border
    if (m_polygon) {
        auto color = m_selection.first == index && m_selection.second == -1 ? Qt::blue : Qt::black;
        drawPolygon(index, painter, color);
    }

    // Draw vertices
    painter.setBrush(Qt::black);
    if (m_vertices) {
        for (const glm::dvec2& vertex : border) {
            const glm::vec2 vec = transformed(vertex);
            painter.drawEllipse(QPoint((int)vec.x, (int)vec.y), 3, 3);
        }
    }
}

void PolygonWidget::drawPolygon(uint32_t index, QPainter& painter, QColor color)
{
    const std::vector<glm::dvec2>& border = m_polygons[index]->border();

    m_borderPen.setColor(color);
    painter.setPen(m_borderPen);

    if (m_fill) {
        QPainterPath path;

        const glm::vec2& origin = transformed(border[0]);
        path.moveTo(QPointF(origin.x, origin.y));

        for (uint32_t i = 1; i < border.size(); i++) {
            const glm::vec2& vertex = transformed(border[i]);
            path.lineTo(QPointF(vertex.x, vertex.y));
        }

        painter.fillPath(path, color);
    } else {
        for (uint32_t i = 0; i < border.size(); i++) {
            const glm::vec2& a = transformed(border[i]);
            const glm::vec2& b = transformed(border[(i + 1) % border.size()]);
            painter.drawLine(QPointF(a.x, a.y), QPointF(b.x, b.y));
        }
    }
}

void PolygonWidget::mousePressEvent(QMouseEvent *event)
{
    if (m_polygons.empty()) return;

    if (event->button() == Qt::LeftButton) {
        m_selection = { -1, -1 };

        m_down = invTransformed({ event->pos().x(), event->pos().y() });
        for (uint32_t i = 0; i < m_polygons.size(); i++) {
            if (m_polyControl[i].visible) {
                if (m_polyControl[i].editable) {
                    for (uint16_t j = 0; j < m_polygons[i]->vertexCount(); j++) {
                        const glm::dvec2& vertex = m_polygons[i]->border()[j];
                        if (glm::length(vertex - m_down) < 8 / m_scalar) {
                            m_selection = { i, j };
                            break;
                        }
                    }
                }

                // Test against body if vertex tests all failed
                if (m_selection.first == -1 && m_polygons[i]->test(m_down)) {
                    m_selection = { i, -1 };
                    break;
                }
            }
        }

        repaint();
    }
}

void PolygonWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (m_polygons.empty()) return;

    if ((event->buttons() & Qt::LeftButton) && m_selection.first != -1) {
        glm::dvec2 mouse = invTransformed({ event->pos().x(), event->pos().y() });

        if (m_selection.second == -1) {
            const glm::dvec2 delta = mouse - m_down;
            m_polygons[m_selection.first]->translate(delta);
            emit moved(m_selection.first, delta);
        } else {
            m_polygons[m_selection.first]->positionVertex(m_selection.second, mouse);
        }

        m_down = mouse;
        repaint();
    }
}

void PolygonWidget::mouseReleaseEvent(QMouseEvent *event) {}

void PolygonWidget::keyPressEvent(QKeyEvent *event)
{
    if (m_polygons.empty()) return;

    if (m_selection.first != -1 && m_polyControl[m_selection.first].editable) {
        if (m_selection.second != -1) {
            switch (event->key()) {
                case Qt::Key_Delete: // Delete vertex of polygon
                    if (m_polygons[m_selection.first]->vertexCount() > 3) {
                        m_polygons[m_selection.first]->removeVertex(m_selection.second);
                        m_selection = { -1, -1 };
                        repaint();
                    }
                    break;
                case Qt::Key_Control: // Insert vertex on polygon
                    m_polygons[m_selection.first]->insertVertex(m_selection.second, m_polygons[m_selection.first]->border()[m_selection.second]);
                    m_selection.second++;
                    repaint();
                    break;
            }
        } else if (event->key() == Qt::Key_Delete) { // Delete polygon itself
            emit requestDeletion(m_selection.first);
            m_polygons.erase(m_polygons.begin() + m_selection.first);
            m_polyControl.erase(m_polyControl.begin() + m_selection.first);
            m_selection = { -1, -1 };
            repaint();
        }

    }
}

void PolygonWidget::resizeEvent(QResizeEvent* event)
{
    if (m_centered) center();
}

void PolygonWidget::focusInEvent(QFocusEvent* event)
{
    grabKeyboard();
}
void PolygonWidget::focusOutEvent(QFocusEvent* event)
{
    releaseKeyboard();
}