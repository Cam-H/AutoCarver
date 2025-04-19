//
// Created by Cam on 2025-04-16.
//

#include "LineChartWidget.h"

#include <QPainter>
#include <QPainterPath>

#include <iostream>

LineChartWidget::LineChartWidget(QWidget* parent)
        : QWidget(parent)
        , m_legendEnable(true)
        , m_penIdx(0)
        , m_t(1.0f)
        , m_xMin(-1.0f)
        , m_xMax(-1.0f)
        , m_xMag(1.0f)
        , m_yMin(-1.0f)
        , m_yMax(-1.0f)
        , m_yMag(1.0f)
        , m_xOff(0.0f)
        , m_yOff(0.0f)
        , m_xMinor(1.0f)
        , m_yMinor(1.0f)
        , m_translation(0, 0)

{

    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

}

LineChartWidget::~LineChartWidget()
{
    clear();
}


void LineChartWidget::clear()
{
    m_penIdx = 0;

    for (Series& series : m_series) {
        delete[] series.x;
        delete[] series.y;
    }
    m_series.clear();
}

void LineChartWidget::zero()
{
    m_translation = {};
}

void LineChartWidget::reset()
{
    clear();
    zero();

    m_x.clear();

    m_t = 1.0f;
    m_xMin = m_xMax = m_yMin = m_yMax = -1.0f;
    m_xMag = m_yMag = 1.0f;
    m_xMinor = m_yMinor = 1.0f;
}

void LineChartWidget::mousePressEvent(QMouseEvent *e)
{
    m_mouseLast = e->pos();
}
void LineChartWidget::mouseMoveEvent(QMouseEvent *e)
{
    QPoint pos = e->pos();

    if (e->buttons() == Qt::MouseButton::LeftButton) {
        m_translation += pos - m_mouseLast;
        update();
    }

    m_mouseLast = pos;
}

void LineChartWidget::plot(const std::vector<float>& y)
{
    plot(y, s_pens[m_penIdx]);
    if (++m_penIdx >= s_pens.size()) m_penIdx = 0;
}

void LineChartWidget::plot(const std::vector<float>& y, const QString& name)
{
    plot(y, name, s_pens[m_penIdx]);
    if (++m_penIdx >= s_pens.size()) m_penIdx = 0;
}

void LineChartWidget::plot(const std::vector<float>& y, QPen pen)
{
    QString name = "Series";
    name.append(QString::number(m_series.size()));

    plot(y, name, pen);
}

void LineChartWidget::plot(const std::vector<float>& y, const QString& name, QPen pen)
{
    auto *points = new float[y.size()];
    std::copy(y.begin(), y.end(), points);

    m_series.push_back(Series{
            name,
            nullptr,
            points,
            (uint32_t)y.size(),
            pen
    });

    if (m_xMin == -1 && m_xMax == -1) xlim();
    if (m_yMin == -1 && m_yMax == -1) ylim();

    update();
}

void LineChartWidget::showLegend(bool visible)
{
    m_legendEnable = visible;
}

void LineChartWidget::setX(const std::vector<float>& x)
{
    m_x = x;

    xlim(x[0], x[x.size() - 1]);
}

void LineChartWidget::setT(float t)
{
    m_t = t;
}

void LineChartWidget::xlim()
{
    if (m_series.empty()) {
        xlim(0, m_t);
        return;
    }

    bool limited = false;

    float min = std::numeric_limits<float>::lowest();
    float max = std::numeric_limits<float>::max();

    for (const Series& series : m_series) {
        if (series.x == nullptr) continue;

        if (min > series.x[0]) min = series.x[0];
        if (max < series.x[series.count - 1]) max = series.x[series.count - 1];

        limited = true;
    }

    if (limited) xlim(min, max);
    else xlim(0, m_t);
}
void LineChartWidget::ylim()
{
    if (m_series.empty()) return;

    float min = m_series[0].y[0], max = m_series[0].y[0];

    for (const Series& series : m_series) {
        for (uint32_t i = 0; i < series.count; i++) {
            if (min > series.y[i]) min = (float)series.y[i];
            else if (max < series.y[i]) max = (float)series.y[i];
        }
    }

    ylim(min, max);
}

void LineChartWidget::xlim(float minimum, float maximum)
{
    m_xMin = minimum;
    m_xMax = maximum;

    m_xMinor = (m_xMax - m_xMin) / 4;

    updateTransforms();
}
void LineChartWidget::ylim(float minimum, float maximum)
{
    m_yMin = minimum;
    m_yMax = maximum;

    m_yMinor = (m_yMax - m_yMin) / 4;

    updateTransforms();
}

inline int LineChartWidget::xTransform(float x) const
{
    return (int)(x * m_xMag) + m_xOff;
}
inline int LineChartWidget::yTransform(float y) const
{
    return -(int)(y * m_yMag) + m_yOff;
}

void LineChartWidget::resizeEvent(QResizeEvent *)
{
    updateTransforms();
}

void LineChartWidget::updateTransforms()
{
    m_xMag = ((float)this->width() - 2) / (m_xMax - m_xMin);
    m_xOff = (int)(m_xMin * m_xMag);

    m_yMag = ((float)this->height() - 2) / (m_yMax - m_yMin);
    m_yOff = (int)(m_yMax * m_yMag);
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

    // Apply transformations to the plot from user manipulation
    painter.translate(m_translation);

    // Draw minor axes
    painter.setPen({ Qt::darkGray, 0.5f });
    drawXMinor(painter);
    drawYMinor(painter);

    // Draw series
    for (const Series& series : m_series) {
        painter.setPen(series.pen);

        float tStep = m_t / (float)series.count;
        int x, y, lx, ly;

        if (series.x != nullptr) { // Use series-specific x-coordinates

        } else if (m_x.size() == series.count) { // Use a special x-coordinate set for plotting (for non-linear)
            lx = xTransform(m_x[0]);
            ly = yTransform(series.y[0]);
            for (int i = 1; i < series.count; i++) {
                x = xTransform(m_x[i]);
                y = yTransform(series.y[i]);

                painter.drawLine(lx, ly, x, y);

                lx = x;
                ly = y;
            }
        } else { // Use the base t as the plotted x-coordinate
            lx = xTransform(0.0f);
            ly = yTransform(series.y[0]);
            for (int i = 1; i < series.count; i++) {
                x = xTransform(i * tStep);
                y = yTransform(series.y[i]);

                painter.drawLine(lx, ly, x, y);

                lx = x;
                ly = y;
            }
        }
    }

    // Draw major axes
    painter.setPen({ Qt::black, 2.0f });
    painter.drawLine(-m_translation.x(), m_yOff, this->width() - m_translation.x(), m_yOff); // Horizontal axis
    painter.drawLine(m_xOff, -m_translation.y(), m_xOff, this->height() - m_translation.y()); // Vertical axis


    // Restore painter transform
    painter.translate(-m_translation);

    if (m_legendEnable) drawLegend(painter);


    painter.restore();


}

void LineChartWidget::drawXMinor(QPainter& painter)
{
    auto fm = QFontMetrics(painter.font());
    int offset = (int)(m_xMinor * m_xMag), margin = 2;

    int x, y = std::clamp(m_yOff, margin - m_translation.y(), this->height() - margin - fm.height() - m_translation.y());
    int start = -(int)std::ceil((m_xOff + m_translation.x()) / offset);
    int end = start + this->width() / offset + 1;

    for (int i = start; i < end; i++) {
        x = m_xOff + i * offset;

        painter.drawLine(x, -m_translation.y(), x, this->height() - m_translation.y());

        // Draw coordinate marker
        auto text = QString::number(i * m_xMinor, 'g', 4);
        int textWidth = fm.horizontalAdvance(text);
        auto bounds = QRect(x - textWidth - margin, y, textWidth, fm.height());
        painter.drawText(bounds, Qt::AlignTop | Qt::AlignLeft, text);
    }
}
void LineChartWidget::drawYMinor(QPainter& painter)
{
    auto fm = QFontMetrics(painter.font());
    int offset = (int)(m_yMinor * m_yMag), margin = 2;

    int x = std::clamp(m_xOff, margin - m_translation.x(), this->width() - margin - m_translation.x()), y;
    int start = -(int)std::ceil((m_yOff + m_translation.y()) / offset) - 1;
    int end = start + this->height() / offset + 1;

    for (int i = start; i < end; i++) {
        y = m_yOff + i * offset;

        painter.drawLine(-m_translation.x(), y, this->width() - m_translation.x(), y);

        // Draw coordinate marker
        auto text = QString::number(-i * m_yMinor, 'g', 4);
        int textWidth = fm.horizontalAdvance(text), test;
        auto bounds = QRect(x - textWidth - margin, y + margin, textWidth, fm.height());
        if ((test = -2 * margin + m_translation.x() + x - bounds.width()) < 0) {
            bounds.translate(-test, 0);
            painter.drawText(bounds, Qt::AlignTop | Qt::AlignLeft, text);
        } else painter.drawText(bounds, Qt::AlignTop | Qt::AlignRight, text);
    }
}

void LineChartWidget::drawLegend(QPainter& painter)
{
    const static QString line = "■ "; // ---  ⎯⎯⎯  ───  ━━━
    auto fm = QFontMetrics(painter.font());
    int margin = 2, width = 0, offset = fm.horizontalAdvance(line);

    for (const Series& series : m_series) {
        auto advance = fm.horizontalAdvance(series.name);
        if (advance > width) width = advance;
    }

    int x = this->width() - width - margin, y = margin;


    for (const Series& series : m_series) {

        painter.setPen({ Qt::black, 1.0f });
        auto bounds = QRect(x, y, width, fm.height());
        painter.drawText(bounds, Qt::AlignLeft | Qt::AlignTop, series.name);

        painter.setPen(series.pen);
        bounds = QRect(x - offset, y, offset, fm.height());
        painter.drawText(bounds, Qt::AlignLeft | Qt::AlignTop, line);

        y += fm.height();
    }

}