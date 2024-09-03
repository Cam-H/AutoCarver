//
// Created by cameronh on 02/06/24.
//

#ifndef AUTOCARVER_WINDOW_H
#define AUTOCARVER_WINDOW_H

#include <QWidget>

#include <QMouseEvent>
#include <QKeyEvent>

#include <vector>

#include "../../../src/geometry/Polygon.h"


class Window : public QWidget {
Q_OBJECT
public:
    explicit Window(QWidget *parent = nullptr);

//    void addWidgets(const QWidget *from, const QWidget *to);
    void setBorder(const std::vector<QPoint> &border);
    const std::vector<QPoint> &getBorder();

    void enablePartition(bool enable);
    void enableTesselation(bool enable);

protected:
    void paintEvent(QPaintEvent *);

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

    void keyPressEvent(QKeyEvent *event) override;

private:

    std::vector<QPoint> m_vertices;
    int m_selection;

    bool m_latest;
    std::vector<Polygon::IndexedBorder> m_partitions;

    bool m_partition;
    bool m_tesselation;
};


#endif //AUTOCARVER_WINDOW_H
