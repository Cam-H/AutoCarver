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

struct Series {
    float* x;
    float* y;
    uint32_t count;

    QPen pen;

};

class LineChartWidget : public QWidget {
Q_OBJECT

public:

    explicit LineChartWidget(QWidget* parent = nullptr);

    ~LineChartWidget();

    void clear();

    void plot(const std::vector<float>& y);
    void plot(const std::vector<float>& y, QPen pen);

    void xlim();
    void ylim();

    void xlim(float minimum, float maximum);
    void ylim(float minimum, float maximum);


protected:
    void paintEvent(QPaintEvent *event) override;

private:

    [[nodiscard]] inline int xTransform(float x) const;
    [[nodiscard]] inline int yTransform(float y) const;

private:

    std::vector<Series> m_series;

    float m_yMin, m_yMax, m_yMag;
    int m_yOff;

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
