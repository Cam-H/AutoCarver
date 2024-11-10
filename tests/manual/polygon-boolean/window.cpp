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

    float del = 5, theta = 0;
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
        case Qt::Key_PageUp:
            theta -= M_PI / 64;
            break;
        case Qt::Key_PageDown:
            theta += M_PI / 64;
            break;
    }

    if (event->modifiers() & Qt::ControlModifier) {
        if (delta.lengthSquared() > 0) m_poly->translate(delta);
        if (theta != 0) m_poly->rotate(m_p, theta);
        repaint();
    } else {
        m_p += delta;
        check();
    }
//    m_p += delta;
//    check();

}

void Window::check()
{
    m_enclosing = m_poly->encloses(m_p);
    repaint();
}