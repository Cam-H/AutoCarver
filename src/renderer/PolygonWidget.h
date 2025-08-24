//
// Created by Cam on 2025-06-19.
//

#ifndef AUTOCARVER_POLYGONWIDGET_H
#define AUTOCARVER_POLYGONWIDGET_H


#include <QWidget>

#include <QMouseEvent>
#include <QKeyEvent>

#include <vector>

#include <glm.hpp>

class Polygon;

class PolygonWidget : public QWidget {
Q_OBJECT
public:
    explicit PolygonWidget(QWidget *parent = nullptr);

    void setPolygon(Polygon *polygon);

    void uncenter();
    void center();
    void setCentered(bool centered);

    void enablePolygon(bool enable);
    void enableTriangulation(bool enable);
    void enableHull(bool enable);

    Polygon* getPolygon();

protected:
    void paintEvent(QPaintEvent *);

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

    void keyPressEvent(QKeyEvent *event) override;

    void resizeEvent(QResizeEvent* event) override;

private:

    glm::vec2 transformed(const glm::vec2& vertex) const;
    glm::vec2 invTransformed(const glm::vec2& vertex) const;

private:

    Polygon *m_poly;

    // Selected vertex
    int m_selection;

    // Rendering settings
    bool m_polygon;
    bool m_triangulation;
    bool m_hull;
    bool m_debugEdges;

    // Polygon positioning
    bool m_centered;
    double m_scalar;
    glm::vec2 m_offset;

};


#endif //AUTOCARVER_POLYGONWIDGET_H
