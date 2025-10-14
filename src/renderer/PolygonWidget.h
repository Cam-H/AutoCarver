//
// Created by Cam on 2025-06-19.
//

#ifndef AUTOCARVER_POLYGONWIDGET_H
#define AUTOCARVER_POLYGONWIDGET_H


#include <QWidget>

#include <QMouseEvent>
#include <QKeyEvent>

#include <QPainter>

#include <vector>

#include <glm.hpp>

class Polygon;

class PolygonWidget : public QWidget {
Q_OBJECT

signals:
    void moved(uint32_t selection, const glm::dvec2& delta);

    void requestDeletion(uint32_t index);

public:
    explicit PolygonWidget(QWidget *parent = nullptr);

    void setPolygon(Polygon* poly);
    void clearPolygons();

    void addPolygon(Polygon* poly);

    void focusPolygon(uint32_t idx);
    void showAll();

    void uncenter();
    void center();
    void setCentered(bool centered);

    void enablePolygon(bool enable);
    void enableTriangulation(bool enable);
    void enableHull(bool enable);

    void enableFill(bool enable);
    void enableVertices(bool enable);

    [[nodiscard]] bool isCentered() const;

    [[nodiscard]] uint32_t polygonCount() const;
    [[nodiscard]] const std::vector<Polygon*>& getPolygons() const;

    Polygon* getPolygon(uint32_t idx);

    [[nodiscard]] int selection() const;

protected:

    void paintEvent(QPaintEvent *event) override;

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

    void keyPressEvent(QKeyEvent *event) override;

    void resizeEvent(QResizeEvent* event) override;

    void focusInEvent(QFocusEvent* event) override;
    void focusOutEvent(QFocusEvent* event) override;

    [[nodiscard]] glm::vec2 transformed(const glm::vec2& vertex) const;
    [[nodiscard]] glm::vec2 invTransformed(const glm::vec2& vertex) const;

    virtual void drawForeground(QPainter& painter);
    virtual void drawPolygon(uint32_t index, QPainter& painter, QColor color);

private:

    void paint(QPainter& painter, uint32_t index);

    [[nodiscard]] std::tuple<double, double, double, double> bounds() const;
    [[nodiscard]] static std::tuple<double, double, double, double> bounds(Polygon* poly) ;

protected:

    struct PolyControl {
        PolyControl(bool visible = true) : visible(visible), editable(true) {}

        bool visible;
        bool editable;
    };

    std::vector<Polygon*> m_polygons;
    std::vector<PolyControl> m_polyControl;

    // Selected vertex
    std::pair<int, int> m_selection;
    glm::dvec2 m_down;

    // Rendering settings
    bool m_polygon;
    bool m_triangulation;
    bool m_hull;
    bool m_debugEdges;

    bool m_fill;
    bool m_vertices;

    // Polygon positioning
    bool m_centered;
    double m_scalar;
    glm::vec2 m_offset;

    // Styling
    QPen m_borderPen;
    QPen m_partitionPen;
    QPen m_triPen;

    static std::vector<QColor> COLOR_SET;
};


#endif //AUTOCARVER_POLYGONWIDGET_H
