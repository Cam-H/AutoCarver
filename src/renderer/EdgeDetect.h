//
// Created by Cam on 2025-06-04.
//

#ifndef AUTOCARVER_EDGEDETECT_H
#define AUTOCARVER_EDGEDETECT_H

#include <QImage>
#include <glm/glm.hpp>

#include <vector>

class Mesh;
class RenderCapture;

class EdgeDetect {
public:

    explicit EdgeDetect(const std::shared_ptr<Mesh>& mesh);
    ~EdgeDetect();

    void setSize(uint32_t size);
    void setEpsilon(float epsilon);

    void update();

    RenderCapture* capture();

    [[nodiscard]] const QImage& source() const;
    [[nodiscard]] const QImage& sink() const;

private:

    struct Contour {
        std::vector<glm::vec2> points;
    };

    void detectEdges(const QImage& image);

    static std::vector<Contour> suzukiAbe(uchar* data, uint32_t size);
    static Contour generateContour(uchar* data, uint32_t x, uint32_t y, uint32_t width);
    static bool next(const uchar* data, uint32_t& x, uint32_t& y, uint32_t width);

    static void douglasPeucker(Contour& contour, float epsilon);

private:

    std::shared_ptr<Mesh> mesh;

    uint32_t m_size;
    float m_epsilon;

    RenderCapture *m_capture;


    QImage m_source;
    QImage m_sink;

    uchar* m_data;

    std::vector<Contour> m_contours;

};


#endif //AUTOCARVER_EDGEDETECT_H
