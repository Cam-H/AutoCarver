//
// Created by cameronh on 02/06/24.
//

#include "window.h"

#include <QPainter>
#include <QPainterPath>

#include <QPolygonF>
#include <QList>

#include <iostream>

Window::Window(QWidget *parent) : QWidget(parent), m_selection(-1)
    , m_poly(nullptr)
    , m_polygon(true)
    , m_diagonals(false)
    , m_partition(false)
    , m_tesselation(false)
    , m_p(100, 100)
    , m_enclosing(false)
{
    grabKeyboard();
}

void Window::setPolygon(Polygon *polygon)
{
    m_poly = polygon;
    m_selection = -1;
    repaint();
}

void Window::enablePolygon(bool enable)
{
    m_polygon = enable;
    repaint();
}

void Window::enableDiagonals(bool enable)
{
    m_diagonals = enable;
    repaint();
}

void Window::enablePartition(bool enable)
{
    m_partition = enable;
    repaint();
}

void Window::enableTesselation(bool enable)
{
    m_tesselation = enable;
    repaint();
}

Polygon* Window::getPolygon()
{
    return m_poly;
}

void Window::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    // Draw background
    painter.setBrush(Qt::white);
    painter.setPen(Qt::black);
    painter.drawRect(0, 0, 500, 500);

    QPen borderPen(Qt::black);
    borderPen.setWidth(4.0);

    QPen partitionPen(Qt::blue);
    partitionPen.setWidth(3.0);
    QPen triPen(Qt::cyan);
    triPen.setWidth(2.0);

    std::vector<QColor> colorSet = {Qt::green, Qt::cyan, Qt::yellow, Qt::blue, Qt::red, Qt::darkBlue, Qt::magenta, Qt::gray, Qt::darkGray, Qt::darkCyan};
    int idx = 0;

    // Draw tesselation results
    if (m_tesselation) {
        Polygon *poly = getPolygon();
        std::vector<Triangle> triangles = poly->triangulation();

        for (const Triangle &tri : triangles) {
            painter.setPen(colorSet[idx]);
            if (++idx >= colorSet.size()) idx = 0;

            QVector2D a = m_poly->getVertex(tri.m_I0);
            QVector2D b = m_poly->getVertex(tri.m_I1);
            QVector2D c = m_poly->getVertex(tri.m_I2);

            QPointF ap = {a.x(), a.y()};
            QPointF bp = {b.x(), b.y()};
            QPointF cp = {c.x(), c.y()};

            // Additional check to confirm triangle winding is correct / consistent
            bool out = QVector3D::dotProduct({0, 0, 1}, QVector3D::crossProduct({b.x() - a.x(), b.y() - a.y(), 0}, {c.x() - a.x(), c.y() - a.y(), 0})) < 0;

            QPainterPath path = QPainterPath(ap);
            path.addPolygon(QPolygonF({ap, bp, cp}));
            painter.fillPath(path, out ? colorSet[idx] : Qt::black);

//            painter.drawLine(ap, bp);
//            painter.drawLine(bp, cp);
//            painter.drawLine(cp, ap);

            std::cout << "T: " << tri.m_I0 << " " << tri.m_I1 << " " << tri.m_I2 << "\n";
        }
    }

    if (m_diagonals) {
        Polygon *poly = getPolygon();
        std::vector<std::pair<int, int>> diagonals = poly->diagonals();
        for(const std::pair<int, int> &diag : diagonals) {
            painter.setPen(borderPen);

            QVector2D a = m_poly->getVertex(diag.first);
            QVector2D b = m_poly->getVertex(diag.second);

            painter.drawLine(QPoint(a.x(), a.y()), QPoint(b.x(), b.y()));
        }
    }

    // Draw partition results
    if (m_partition) {
        Polygon *poly = getPolygon();
        std::vector<Polygon::IndexedBorder> partitions = poly->partitions();

        for(const Polygon::IndexedBorder &partition : partitions) {
            painter.setPen(colorSet[idx]);
            if (++idx >= colorSet.size()) idx = 0;

            for (uint32_t i = 0; i < partition.vertices.size(); i++) {
                QVector2D a = m_poly->getVertex(partition.vertices[i]);
                QVector2D b = m_poly->getVertex(partition.vertices[(i + 1) % partition.vertices.size()]);

                painter.drawLine(QPoint(a.x(), a.y()), QPoint(b.x(), b.y()));
            }

        }
    }

    // Draw border / internal loops
    if (m_polygon) {
        painter.setBrush(Qt::black);
        painter.setPen(borderPen);
        for (uint32_t i = 0; i < m_poly->loopCount(); i++) {
            std::vector<uint32_t> loop = m_poly->getLoop(i);

            for (uint32_t j = 0; j < loop.size(); j++) {
                QVector2D a = m_poly->getVertex(loop[j]);
                QVector2D b = m_poly->getVertex(loop[(j + 1) % loop.size()]);
                painter.drawLine(QPoint(a.x(), a.y()), QPoint(b.x(), b.y()));
            }
        }
    }

    // Draw vertices
    painter.setPen(Qt::black);
    for (uint32_t i = 0; i < m_poly->vertexCount(); i++) {
        QVector2D vertex = m_poly->getVertex(i);
        painter.drawEllipse(QPoint(vertex.x(), vertex.y()), 3, 3);
    }

    if (m_selection != -1) {
        painter.setBrush(Qt::blue);
        QVector2D vertex = m_poly->getVertex(m_selection);

        painter.drawEllipse(QPoint(vertex.x(), vertex.y()), 5, 5);

        painter.drawText(20, 20, QString(std::to_string(m_selection).c_str()));
    }

    painter.setBrush(m_enclosing ? Qt::darkGreen : Qt::darkRed);
    painter.drawEllipse(QPoint(m_p.x(), m_p.y()), 4, 4);
}

void Window::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        m_selection = -1;

        for (uint32_t i = 0; i < m_poly->vertexCount(); i++) {
            QVector2D vertex = m_poly->getVertex(i);
            if ((event->pos() - QPoint(vertex.x(), vertex.y())).manhattanLength() < 8) {
                m_selection = i;
                break;
            }
        }

        repaint();
    }
}

void Window::mouseMoveEvent(QMouseEvent *event)
{
    if ((event->buttons() & Qt::LeftButton) && m_selection != -1) {
        m_poly->positionVertex(m_selection, QVector2D(event->pos().x(), event->pos().y()), true);
        repaint();
    }
}

void Window::mouseReleaseEvent(QMouseEvent *event) {}

void Window::keyPressEvent(QKeyEvent *event)
{
    if (m_selection != -1) {
        switch (event->key()) {
            case Qt::Key_Delete:
                if (m_poly->vertexCount() > 3) {
                    m_poly->removeVertex(m_selection, true);
                    m_selection = -1;
                    repaint();
                }
                break;
            case Qt::Key_Control:
                m_poly->insertVertex(m_selection, m_poly->getVertex(m_selection));
                m_selection++;
                repaint();
                break;
        }
    }

    float del = 5;
    QVector2D delta = {0, 0};
    switch (event->key()) {
        case Qt::Key_Up:
            delta = {0, -del};
            break;
        case Qt::Key_Down:
            delta = {0, del};
            break;
        case Qt::Key_Right:
            delta = {del, 0};
            break;
        case Qt::Key_Left:
            delta = {-del, 0};
            break;
    }

    m_poly->translate(delta);
    repaint();
//    m_p += delta;
//    check();

}

void Window::check()
{
    m_enclosing = m_poly->encloses(m_p);
    repaint();
}