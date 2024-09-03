//
// Created by cameronh on 02/06/24.
//

#include "window.h"

#include <QPainter>

#include <iostream>

Window::Window(QWidget *parent) : QWidget(parent), m_vertices(), m_selection(-1)
    , m_latest(false)
    , m_partition(true)
    , m_tesselation(false)
{
    grabKeyboard();
}

void Window::setBorder(const std::vector<QPoint> &border)
{
    m_vertices = border;
    m_selection = -1;
    m_latest = false;
    repaint();
}

void Window::enablePartition(bool enable)
{
    m_partition = enable;
    repaint();
}

void Window::enableTesselation(bool enable)
{
    m_tesselation = m_partition && enable;
    repaint();
}

const std::vector<QPoint> &Window::getBorder()
{
    return m_vertices;
}

void Window::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    // Draw background
    painter.setBrush(Qt::white);
    painter.setPen(Qt::black);
    painter.drawRect(0, 0, 500, 500);

    QPen borderPen(Qt::black);
    borderPen.setWidth(3.0);

    QPen partitionPen(Qt::blue);
    partitionPen.setWidth(2.0);
    QPen triPen(Qt::cyan);
    triPen.setWidth(1.0);

    // Draw partition results
    if (m_partition) {
        if (!m_latest) {
            std::vector<QVector2D> border;
            for (const QPoint &vertex : m_vertices) {
                border.emplace_back(vertex.x(), vertex.y());
            }

            Polygon poly(border);
            m_partitions = poly.partition();
            m_latest = true;

            std::cout << "Partitions: \n";
            for(const Polygon::IndexedBorder &partition : m_partitions) {
                for (auto idx : partition.vertices) {
                    std::cout << idx << " ";
                }

                std::cout << "\n";
            }
        }


        painter.setPen(partitionPen);
        for (const Polygon::IndexedBorder &partition : m_partitions) {
            for (uint16_t i = 0; i < partition.vertices.size(); i++) {
                painter.drawLine(m_vertices[partition.vertices[i]], m_vertices[partition.vertices[(i + 1) % partition.vertices.size()]]);
            }
        }
    }

    if (m_tesselation) {

    }

    // Draw border & vertices
    painter.setBrush(Qt::black);
    painter.setPen(borderPen);
    for (uint16_t i = 0; i < m_vertices.size(); i++) {
        painter.drawLine(m_vertices[i], m_vertices[(i + 1) % m_vertices.size()]);
    }

    painter.setPen(Qt::black);
    for (const QPoint& vertex : m_vertices) {
        painter.drawEllipse(vertex, 3, 3);
    }

    if (m_selection != -1) {
       painter.setBrush(Qt::blue);
       painter.drawEllipse(m_vertices[m_selection], 5, 5);

       painter.drawText(20, 20, QString(std::to_string(m_selection).c_str()));
    }
}

void Window::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        m_selection = -1;

        for (uint16_t i = 0; i < m_vertices.size(); i++) {
            if ((event->pos() - m_vertices[i]).manhattanLength() < 8) {
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
        m_vertices[m_selection] = event->pos();
        m_latest = false;
        repaint();
    }
}

void Window::mouseReleaseEvent(QMouseEvent *event) {}

void Window::keyPressEvent(QKeyEvent *event)
{
    if (m_selection != -1) {
        switch (event->key()) {
            case Qt::Key_Delete:
                if (m_vertices.size() > 3) {
                    m_vertices.erase(m_vertices.begin() + m_selection);
                    m_selection = -1;
                    m_latest = false;
                    repaint();
                }
                break;
            case Qt::Key_Control:
                m_vertices.insert(m_vertices.begin() + m_selection + 1, m_vertices[m_selection]);
                m_selection++;
                m_latest = false;
                repaint();
                break;
        }
    }
}