//
// Created by Cam on 2025-06-04.
//

#include "EdgeDetect.h"

#include "geometry/Mesh.h"
#include "renderer/RenderCapture.h"

#include <iostream>

EdgeDetect::EdgeDetect(const std::shared_ptr<Mesh>& mesh)
    : mesh(mesh)
    , m_size(500)
    , m_epsilon(200.0f)
    , m_capture(new RenderCapture(nullptr, QSize(m_size, m_size)))
    , m_source()
    , m_sink()
    , m_data(nullptr)
    , m_contours()
{
    m_capture->addTarget(mesh, QColor(255, 255, 255));
}

EdgeDetect::~EdgeDetect()
{
    delete[] m_data;
}

void EdgeDetect::setSize(uint32_t size)
{
    m_capture->resize(size, size);
    m_size = size;
}

void EdgeDetect::setEpsilon(float epsilon)
{
    m_epsilon = epsilon;
}

void EdgeDetect::update()
{
    m_capture->capture();
    m_source = m_capture->grabFramebuffer();
    m_sink = m_source.convertToFormat(QImage::Format::Format_Grayscale8);

    delete[] m_data;
    m_data = new uchar[m_size * m_size];

    detectEdges(m_sink);

    m_contours = suzukiAbe(m_data, m_size);
    for (Contour& contour : m_contours) {
        douglasPeucker(contour, m_epsilon);
    }

    m_sink = QImage(m_data, m_size, m_size, QImage::Format::Format_Grayscale8);
}

void EdgeDetect::detectEdges(const QImage& image)
{
    uint32_t idx = image.width() + 1;
    for (uint32_t i = 1; i < image.height() - 1; i++){
        for (uint32_t j = 1; j < image.width() - 1; j++) { // Don't consider borders to avoid complicating the kernel
            bool edge = m_sink.bits()[idx] == 255 &&
                        (m_sink.bits()[idx] != m_sink.bits()[idx - image.width()]     // Above
                         || m_sink.bits()[idx] != m_sink.bits()[idx + image.width()]  // Below
                         || m_sink.bits()[idx] != m_sink.bits()[idx - 1]              // Left
                         || m_sink.bits()[idx] != m_sink.bits()[idx + 1]);            // Right

            m_data[idx] = 255 * edge;
            idx++;
        }

        idx += 2;
    }

    // Border cleaning
    for (uint32_t i = 0; i < image.width(); i++) {
        m_data[i] = 0;
        m_data[image.width() * (image.height() - 1) + i] = 0;
    }

    for (uint32_t i = 0; i < image.height(); i++) {
        m_data[i * image.height()] = 0;
        m_data[i * image.height() + image.width() - 1] = 0;
    }
}

std::vector<EdgeDetect::Contour> EdgeDetect::suzukiAbe(uchar* data, uint32_t size)
{
    std::vector<Contour> contours;

    // Evaluate contours, skipping borders
    uint32_t idx = size + 1;
    for (uint32_t i = 1; i < size - 1; i++){ // Rows
        for (uint32_t j = 1; j < size - 1; j++) { // Columns
            if (data[idx] == 255 && data[idx - 1] == 0)
                contours.push_back(generateContour(data, j, i, size));

            idx++;
        }

        idx += 2;
    }

    return contours;
}

EdgeDetect::Contour EdgeDetect::generateContour(uchar* data, uint32_t x, uint32_t y, uint32_t width)
{
    Contour contour = { };

    do {
        data[x + y * width] = 100; // Prevent repeating consideration
        contour.points.emplace_back(x, y);
    } while (next(data, x, y, width));

    return contour;
}

bool EdgeDetect::next(const uchar* data, uint32_t& x, uint32_t& y, uint32_t width)
{
    if (data[x + (y - 1) * width] == 255) { // Top
        y--;
        return true;
    } else if (data[x + 1 + (y - 1) * width] == 255) { // Top-Right
        x++;
        y--;
        return true;
    } else if (data[x + 1 + y * width] == 255) { // Right
        x++;
        return true;
    } else if (data[x + 1 + (y + 1) * width] == 255) { // Bottom-Right
        x++;
        y++;
        return true;
    } else if (data[x + (y + 1) * width] == 255) { // Bottom
        y++;
        return true;
    } else if (data[x - 1 + (y + 1) * width] == 255) { // Bottom-Left
        x--;
        y++;
        return true;
    } else if (data[x - 1 + y * width] == 255) { // Left
        x--;
        return true;
    } else if (data[x - 1 + (y - 1) * width] == 255) { // Top-Left
        x--;
        y--;
        return true;
    }

    return false;
}

void EdgeDetect::douglasPeucker(Contour& contour, float epsilon)
{
    if (contour.points.size() < 3) return;


    std::vector<uint32_t> limits = { 0, (uint32_t)contour.points.size() / 2, (uint32_t)contour.points.size() - 1 };

    float delta = 0;
    uint32_t current = 1, idx = 0;

    while (limits.size() > 1) {
        glm::vec2 axis = contour.points[limits[current + 1]] - contour.points[limits[current]];
        axis = { axis.y, -axis.x };

        // Identify the vertex that deviates the most from the current line
        uint32_t low = limits[current] + 1, up = limits[current + 1] - 1;
        for (uint32_t i = low; i < up; i++) {
            float test = std::abs(glm::dot(axis, contour.points[i] - contour.points[limits[current]]));
            if (test > delta) {
                delta = test;
                idx = i;
            }
        }

        if (delta > epsilon) { // Deviation is greater than allowance, another vertex needs to be inserted
            limits.insert(limits.end() - 1, idx);
        } else { // Remove all unnecessary intermediary vertices
            contour.points.erase(contour.points.begin() + low, contour.points.begin() + up + 1);
            limits.pop_back();
        }

        current = limits.size() - 2;
        delta = 0;
    }


}

RenderCapture* EdgeDetect::capture()
{
    return m_capture;
}

const QImage& EdgeDetect::source() const
{
    return m_source;
}
const QImage& EdgeDetect::sink() const
{
    return m_sink;
}