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

#include "geometry/ConvexHull.h"

class EdgeDetect {
public:

    explicit EdgeDetect(const std::shared_ptr<Mesh>& mesh);
    ~EdgeDetect();

    void setMesh(const std::shared_ptr<Mesh>& newMesh);

    void setSize(uint32_t size);
    void setEpsilon(float epsilon);

    void update();

    RenderCapture* capture();

    [[nodiscard]] glm::vec3 forward() const;
    [[nodiscard]] glm::vec3 vertical() const;
    [[nodiscard]] glm::vec3 horizontal() const;

    [[nodiscard]] const QImage& source() const;
    [[nodiscard]] const QImage& sink() const;

    [[nodiscard]] const std::vector<std::pair<uint32_t, glm::vec3>>& border() const;

private:

    struct Contour {
        std::vector<glm::vec2> points;
    };

    void prepareTargets();

    void detectEdges(const QImage& image);

    static std::vector<Contour> suzukiAbe(uchar* data, uint32_t size);
    static Contour traceContour(uchar* data, int x, int y, uint32_t width, uint8_t mark);
    static bool inBounds(int x, int y, uint32_t size);

    static void douglasPeucker(Contour& contour, float epsilon);

    void findBorder(std::vector<glm::vec2>& contour);

    static uint32_t limit(const std::vector<glm::vec2>& contour, const glm::vec2& axis);

    void updateModelDirections();
    void findReferences(const std::vector<uint32_t>& hull, const std::vector<glm::vec2>& contour, float& scale, uint32_t& hRef, uint32_t& cRef);

    std::vector<std::pair<size_t, size_t>> dynamicTimeWarp(const std::vector<glm::vec2>& hull, const std::vector<glm::vec2>& contour);
    static std::vector<std::pair<size_t, size_t>> dtwPath(const std::vector<std::vector<double>>& cost);

    void prepareBorder(const std::vector<std::pair<size_t, size_t>>& border, const std::vector<glm::vec2>& projection, const std::vector<glm::vec2>& contour);
    void captureConcave(const std::vector<glm::vec2>& source, uint32_t& start, uint32_t end);

    std::vector<glm::vec2> projected();

private:

    std::shared_ptr<Mesh> mesh;
    ConvexHull m_hull;

    uint32_t m_size;
    float m_epsilon;

    RenderCapture *m_capture;

    QImage m_source;
    QImage m_sink;

    glm::vec3 m_axis;
    glm::vec3 m_up;
    glm::vec3 m_right;

    std::vector<std::pair<uint32_t, glm::vec3>> m_border;

    uchar* m_data;

    std::vector<Contour> m_contours;

};


#endif //AUTOCARVER_EDGEDETECT_H
