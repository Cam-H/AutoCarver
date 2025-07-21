//
// Created by Cam on 2025-06-04.
//

#ifndef AUTOCARVER_EDGEDETECT_H
#define AUTOCARVER_EDGEDETECT_H

#include <QImage>
#include <glm.hpp>

#include <vector>

class Mesh;
class RenderCapture;

#include "geometry/primitives/ConvexHull.h"
#include "geometry/poly/Profile.h"

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

    [[nodiscard]] const Profile& profile() const;

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
    float estimateScale(const std::vector<uint32_t>& hull, const std::vector<glm::vec2>& contour);
    static std::tuple<size_t, size_t> findReferenceVertex(const std::vector<glm::vec2>& hull, const std::vector<glm::vec2>& contour);

    static std::vector<std::pair<size_t, size_t>> matchVertices(const std::vector<glm::vec2>& hull, const std::vector<glm::vec2>& contour, uint32_t hShift, uint32_t cShift);
    static std::vector<std::pair<size_t, size_t>> cyclicTimeWarp(const std::vector<glm::vec2>& hull, const std::vector<glm::vec2>& contour);
    static std::tuple<std::vector<std::vector<double>>, double> dynamicTimeWarp(const std::vector<glm::vec2>& hull, const std::vector<glm::vec2>& contour, uint32_t hShift, uint32_t cShift);
    static std::vector<std::pair<size_t, size_t>> dtwPath(const std::vector<std::vector<double>>& cost);

    void drawPolygon(const std::vector<glm::vec2>& vertices, uint8_t value);

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

    Profile m_profile;

    uchar* m_data;

    std::vector<Contour> m_contours;

};


#endif //AUTOCARVER_EDGEDETECT_H
