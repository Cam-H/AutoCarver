//
// Created by Cam on 2025-04-16.
//

#ifndef AUTOCARVER_LINECHARTWIDGET_H
#define AUTOCARVER_LINECHARTWIDGET_H

#include <vector>
#include <array>

#include <QWidget>
#include <QBrush>
#include <QPen>

#include <QMouseEvent>
#include <QResizeEvent>

struct Series {
public:

    Series(const QString& name, uint32_t size, const QPen& pen)
        : name(name)
        , x({})
        , y(size, 0)
        , offset(0)
        , pen(pen) {}

    Series(const QString& name, const std::vector<double>& y, const QPen& pen) : Series(name, x, y, pen) {}
    Series(const QString& name, const std::vector<double>& x, const std::vector<double>& y, const QPen& pen)
        : name(name)
        , x(x)
        , y(y)
        , offset(0)
        , pen(pen) {}

    void stream(double ny)
    {
        y[offset] = ny;
        next();
    }

    void stream(double nx, double ny)
    {
        if (!x.empty()) x[offset] = nx;
        y[offset] = ny;
        next();
    }

    void next() { if (++offset >= y.size()) offset = 0; }
    uint32_t lastIndex() const { return std::min((uint32_t)y.size(), offset); }

    QString name;

    std::vector<double> x;
    std::vector<double> y;
    uint32_t offset;

    QPen pen;

};

class LineChartWidget : public QWidget {
Q_OBJECT

public:

    explicit LineChartWidget(QWidget* parent = nullptr);

    ~LineChartWidget();

    void clear();
    void zero();

    void reset();

    void setX(const std::vector<double>& x);
    void setX(uint32_t size, double minimum, double maximum);
    void setX(uint32_t index, double x);

    void setT(double t);

    void plot(const std::vector<double>& y);
    void plot(const std::vector<double>& y, const QString& name);

    void plot(const std::vector<double>& y, QPen pen);
    void plot(const std::vector<double>& y, const QString& name, QPen pen);

    void create(uint32_t size, const QString& name);
    void create(uint32_t size, const QString& name, QPen pen);

    void stream(uint32_t plotIndex, double y);
    void stream(uint32_t plotIndex, double x, double y);

    void showLegend(bool visible);


    void xlim();
    void ylim();

    void xlim(double minimum, double maximum);
    void ylim(double minimum, double maximum);


protected:
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;

    void resizeEvent(QResizeEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
private:

    QPen nextPen();

    void drawXMinor(QPainter& painter);
    void drawYMinor(QPainter& painter);

    void drawLegend(QPainter& painter);

    void updateTransforms();

    [[nodiscard]] inline int xTransform(double x) const;
    [[nodiscard]] inline int yTransform(double y) const;

private:

    bool m_legendEnable;

    std::vector<double> m_x;
    std::vector<Series> m_series;
    double m_t;

    double m_xMin, m_xMax, m_xMag;
    double m_yMin, m_yMax, m_yMag;
    int m_xOff, m_yOff;

    double m_xMinor, m_yMinor;

    QPoint m_translation, m_mouseLast;

    uint32_t m_penIdx;
    const std::array<QPen, 12> s_pens = {
            QPen { Qt::red, 1.0f },
            { Qt::green, 1.0f },
            { Qt::blue, 1.0f },
            { Qt::yellow, 1.0f },
            { Qt::magenta, 1.0f },
            { Qt::cyan, 1.0f },
            { Qt::darkRed, 1.0f },
            { Qt::darkGreen, 1.0f },
            { Qt::darkBlue, 1.0f },
            { Qt::darkYellow, 1.0f },
            { Qt::darkMagenta, 1.0f },
            { Qt::darkCyan, 1.0f }
    };

};


#endif //AUTOCARVER_LINECHARTWIDGET_H
