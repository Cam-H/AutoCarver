//
// Created by cjhat on 2025-09-17.
//

#ifndef AUTOCARVER_PLANNERWIDGET_H
#define AUTOCARVER_PLANNERWIDGET_H


#include <QWidget>

#include <QMouseEvent>
#include <QKeyEvent>

#include <vector>

#include <glm.hpp>

class Polygon;
class Profile;

#include "renderer/PolygonWidget.h"

class ViewerWidget : public PolygonWidget {
Q_OBJECT

signals:
    void startMoved(const glm::dvec2& delta);
    void goalMoved(const glm::dvec2& delta);

public slots:
    void moved(uint32_t selection, const glm::dvec2& delta);

public:

    explicit ViewerWidget(QWidget *parent = nullptr);

    void setProfile(const Profile* planner);

    void updatePolygons();

    void updateTool();
    void updateTool(const glm::dvec3& location);

    void updatePath();

    void enableBounds(bool enable);
    void enableTree(bool enable);
    void enablePath(bool enable);

    [[nodiscard]] const std::vector<glm::dvec3>& path() const;

protected:
//    void paintEvent(QPaintEvent *event) override;

    void drawForeground(QPainter& painter) override;
    void drawPolygon(uint32_t index, QPainter& painter, QColor color) override;

    [[nodiscard]] bool isStart(uint32_t index) const;

private:

    const Profile* profile;
    uint32_t m_coreCount;

    std::vector<glm::dvec3> m_path;

    bool m_boundsEnable;
    bool m_treeEnable;
    bool m_pathEnable;

};

#endif //AUTOCARVER_DEBUGWIDGET_H
