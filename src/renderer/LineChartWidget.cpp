//
// Created by Cam on 2025-04-16.
//

#include "LineChartWidget.h"

#include <QPainter>
#include <QPainterPath>

#include <iostream>

LineChartWidget::LineChartWidget(QWidget* parent)
        : QWidget(parent)
        , m_penIdx(0)
        , m_yMin(-1.0f)
        , m_yMax(-1.0f)
        , m_yMag(1.0f)
        , m_yOff(0.0f)
{

    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

}

LineChartWidget::~LineChartWidget()
{
    clear();
}


void LineChartWidget::clear()
{
    for (Series& series : m_series) {
//        delete[] series.x;
//        delete[] series.y;
    }
    m_series.clear();

    m_yMin = m_yMax = -1;
    m_penIdx = 0;
}


void LineChartWidget::plot(const std::vector<float>& y)
{
    plot(y, s_pens[m_penIdx]);
    if (++m_penIdx >= s_pens.size()) m_penIdx = 0;
}

void LineChartWidget::plot(const std::vector<float>& y, QPen pen)
{
    auto *points = new float[y.size()];
    std::copy(y.begin(), y.end(), points);

    m_series.push_back(Series{ nullptr, points, (uint32_t)y.size(), pen });

    if (m_yMin == -1 && m_yMax == -1) ylim();

    update();
}

void LineChartWidget::xlim()
{

}
void LineChartWidget::ylim()
{
    if (m_series.empty()) return;

    m_yMin = m_yMax = m_series[0].y[0];

    for (const Series& series : m_series) {
        for (uint32_t i = 0; i < series.count; i++) {
            if (m_yMin > series.y[i]) m_yMin = (float)series.y[i];
            else if (m_yMax < series.y[i]) m_yMax = (float)series.y[i];
        }
    }
}

void LineChartWidget::xlim(float minimum, float maximum)
{

}
void LineChartWidget::ylim(float minimum, float maximum)
{
    m_yMin = minimum;
    m_yMax = maximum;
}

inline int LineChartWidget::xTransform(float x) const
{
    return 0;
}
inline int LineChartWidget::yTransform(float y) const
{
    return -(int)(y * m_yMag) + m_yOff;
}

void LineChartWidget::paintEvent(QPaintEvent *)
{

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(Qt::NoPen);
    painter.save();

    painter.setBackground(Qt::white);
    painter.setBackgroundMode(Qt::OpaqueMode);

    // Clear background
    painter.fillRect(0, 0, this->width(), this->height(), Qt::white);


//    painter.translate(x, y);
//        painter.rotate(60.0);
//        painter.scale(0.6, 0.9);
//        painter.translate(-50, -50);

    m_yMag = ((float)this->height() - 2) / (m_yMax - m_yMin);
    m_yOff = this->height() - (int)((m_yMax + m_yMin) * m_yMag);

    std::cout << "LL: " << m_yMin << " " << m_yMax << " " << m_yMag << " " << m_yOff << "\n";
    for (const Series& series : m_series) {
        painter.setPen(series.pen);

        if (series.x == nullptr) {
            int y, ly = yTransform(series.y[0]);
            for (int x = 1; x < series.count; x++) {
                y = yTransform(series.y[x]);
                painter.drawLine((x - 1) * 2, ly, x * 2, y);
                ly = y;
            }
        }
    }

    // Restore painter transform
    painter.restore();



//    painter.setRenderHint(QPainter::Antialiasing, false);
//    painter.setPen(palette().dark().color());
//    painter.setBrush(Qt::NoBrush);
//    painter.drawRect(QRect(0, 0, width() - 1, height() - 1));
}
