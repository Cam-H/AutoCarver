//
// Created by cameronh on 02/06/24.
//

#ifndef AUTOCARVER_WINDOW_H
#define AUTOCARVER_WINDOW_H

#include <QWidget>

#include <QMouseEvent>
#include <QKeyEvent>

#include <vector>

#include "geometry/poly/CompositePolygon.h"


class Window : public QWidget {
Q_OBJECT
public:
    explicit Window(QWidget *parent = nullptr);

    void setPolygon(CompositePolygon *polygon);

    void enablePolygon(bool enable);
    void enableDiagonals(bool enable);
    void enablePartition(bool enable);
    void enableTesselation(bool enable);

    CompositePolygon* getPolygon();

protected:
    void paintEvent(QPaintEvent *);

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

    void keyPressEvent(QKeyEvent *event) override;

private:

    void check();

private:

    int m_selection;

    CompositePolygon *m_poly;

    bool m_polygon;
    bool m_diagonals;
    bool m_partition;
    bool m_tesselation;

    QVector2D m_p;
    bool m_enclosing;
};


#endif //AUTOCARVER_WINDOW_H
