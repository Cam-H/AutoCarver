//
// Created by cameronh on 02/06/24.
//

#include "window.h"

#include <QPainter>

#include <iostream>

Window::Window(QWidget *parent) : QWidget(parent), m_selection(-1)
    , m_poly(nullptr)
    , m_latest(false)
    , m_polygon(true)
    , m_partition(false)
    , m_tesselation(false)
{
    grabKeyboard();
}

void Window::setPolygon(Polygon *polygon)
{
    m_poly = polygon;
    m_selection = -1;
    m_latest = false;
    repaint();
}

void Window::enablePolygon(bool enable)
{
    m_polygon = enable;
    repaint();
}

void Window::enablePartition(bool enable)
{
    m_partition = enable;
    repaint();
}

void Window::enableTesselation(bool enable)
{
//    m_tesselation = m_partition && enable;
    m_tesselation = enable;
    repaint();
}

Polygon* Window::getPolygon()
{
    if (!m_latest) {
//        delete m_poly;
//
//        // Construct a polygon to process based on visual geometry
//        std::vector<QVector2D> border;
//        for (const QPoint &vertex : m_vertices) {
//            border.emplace_back(vertex.x(), vertex.y());
//        }
//
//        m_poly = new Polygon(border);
        m_latest = true;
    }

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

    std::vector<QVector2D> cut = {QVector2D(250, 150), QVector2D(150, 250), QVector2D(250, 350), QVector2D(350, 250)};
    std::vector<QColor> colorSet = {Qt::green, Qt::cyan, Qt::yellow, Qt::blue, Qt::red, Qt::darkBlue, Qt::magenta, Qt::gray, Qt::black, Qt::darkCyan};
    int idx = 0;

    // Draw tesselation results
    if (m_tesselation) {
        Polygon *poly = getPolygon();
        std::vector<Triangle> triangles = poly->tesselate();

        std::cout << "\nPolygon tesselation: " << triangles.size() << " triangles\n";

        for (const Triangle &tri : triangles) {
            painter.setPen(colorSet[idx]);
            if (++idx >= colorSet.size()) idx = 0;

            QVector2D a = m_poly->getVertex(tri.m_I0);
            QVector2D b = m_poly->getVertex(tri.m_I1);
            QVector2D c = m_poly->getVertex(tri.m_I2);

            QPoint ap = {(int)a.x(), (int)a.y()};
            QPoint bp = {(int)b.x(), (int)b.y()};
            QPoint cp = {(int)c.x(), (int)c.y()};

            painter.drawLine(ap, bp);
            painter.drawLine(bp, cp);
            painter.drawLine(cp, ap);
        }
    }

    // Draw partition results
    if (m_partition) {
        Polygon *poly = getPolygon();
        std::vector<Polygon::IndexedBorder> partitions = poly->partition();

        // Output partition results for user-check
        std::cout << "Partitions: \n";
        for(const Polygon::IndexedBorder &partition : partitions) {
            for (auto idx : partition.vertices) {
                std::cout << idx << " ";
            }

            std::cout << "\n";
        }

        for(const Polygon::IndexedBorder &partition : partitions) {
            painter.setPen(colorSet[idx]);
            if (++idx >= colorSet.size()) idx = 0;

            for (uint16_t i = 0; i < partition.vertices.size(); i++) {
                QVector2D a = partition.vertices[i] >= m_poly->vertexCount() ? cut[partition.vertices[i] - m_poly->vertexCount()] : m_poly->getVertex(partition.vertices[i]);
                QVector2D b = partition.vertices[(i + 1) % partition.vertices.size()] >= m_poly->vertexCount() ?
                        cut[partition.vertices[(i + 1) % partition.vertices.size()] - m_poly->vertexCount()] : m_poly->getVertex(partition.vertices[(i + 1) % partition.vertices.size()]);

                painter.drawLine(QPoint(a.x(), a.y()), QPoint(b.x(), b.y()));
            }

        }
    }

    // Draw border / internal loops
    if (m_polygon) {
        painter.setBrush(Qt::black);
        painter.setPen(borderPen);
        for (uint16_t i = 0; i < m_poly->loopCount(); i++) {
            std::vector<uint32_t> loop = m_poly->getLoop(i);

            for (uint16_t j = 0; j < loop.size(); j++) {
                std::cout << i << " " << j << " " << loop[j] << "\n";
                QVector2D a = m_poly->getVertex(loop[j]);
                QVector2D b = m_poly->getVertex(loop[(j + 1) % loop.size()]);
                painter.drawLine(QPoint(a.x(), a.y()), QPoint(b.x(), b.y()));
            }
        }


//        painter.drawLine(cut[0], cut[1]);
//        painter.drawLine(cut[1], cut[2]);
//        painter.drawLine(cut[2], cut[3]);
//        painter.drawLine(cut[3], cut[0]);
    }

    // Draw vertices
    painter.setPen(Qt::black);
    for (uint16_t i = 0; i < m_poly->vertexCount(); i++) {
        QVector2D vertex = m_poly->getVertex(i);
        painter.drawEllipse(QPoint(vertex.x(), vertex.y()), 3, 3);
    }

    if (m_selection != -1) {
       painter.setBrush(Qt::blue);
        QVector2D vertex = m_poly->getVertex(m_selection);

        painter.drawEllipse(QPoint(vertex.x(), vertex.y()), 5, 5);

       painter.drawText(20, 20, QString(std::to_string(m_selection).c_str()));
    }
}

void Window::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        m_selection = -1;

        for (uint16_t i = 0; i < m_poly->vertexCount(); i++) {
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
                if (m_poly->vertexCount() > 3) {
                    m_poly->removeVertex(m_selection, true);
                    m_selection = -1;
                    m_latest = false;
                    repaint();
                }
                break;
            case Qt::Key_Control:
                m_poly->insertVertex(m_selection, m_poly->getVertex(m_selection));
                m_selection++;
                m_latest = false;
                repaint();
                break;
        }
    }
}