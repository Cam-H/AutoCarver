//
// Created by Cam on 2025-06-04.
//

#include "EdgeDetect.h"

#include "geometry/Mesh.h"
#include "renderer/RenderCapture.h"
#include "geometry/poly/Polygon.h"

#include <iostream>

EdgeDetect::EdgeDetect(const std::shared_ptr<Mesh>& mesh)
    : mesh(mesh)
    , m_hull(mesh->vertices())
    , m_size(500)
    , m_epsilon(200.0f)
    , m_capture(new RenderCapture(nullptr, QSize(m_size, m_size)))
    , m_source()
    , m_sink()
    , m_data(nullptr)
    , m_contours()
{
    prepareTargets();
}

EdgeDetect::~EdgeDetect()
{
    delete[] m_data;
}

void EdgeDetect::setMesh(const std::shared_ptr<Mesh>& newMesh)
{
    mesh = newMesh;
    m_hull = ConvexHull(mesh->vertices());

    prepareTargets();
}

void EdgeDetect::prepareTargets()
{
    m_capture->clearTargets();

//    auto hull = std::make_shared<Mesh>(ConvexHull(mesh->vertices()));
//    m_capture->addTarget(hull, QColor(0, 0, 180));

    m_capture->addTarget(mesh, QColor(255, 255, 255));
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
    // Capture an image of the scene for processing
    m_capture->capture();
    m_source = m_capture->grabFramebuffer();
    m_sink = m_source.convertToFormat(QImage::Format::Format_Grayscale8);

    delete[] m_data;
    m_data = new uchar[m_size * m_size];

    try {

        // Apply kernel to come up with only the edges of the render
        detectEdges(m_sink);

        // Find contours from continuous edge segments
        m_contours = suzukiAbe(m_data, m_size);

        // Approximate and simplify the contour by reducing the number of vertices
        for (Contour& contour : m_contours) {
            douglasPeucker(contour, m_epsilon);
        }

        if (!m_contours.empty()) {

            // A bit of styling for debugging purposes
            for (const glm::vec2& vertex : m_contours[0].points) {
                m_data[(int)(std::round(vertex.x + vertex.y * m_size))] = 150;
            }

            // Try to find a border from the discovered contours (silhouette is necessarily the first contour discovered)
            findBorder(m_contours[0].points);
        }

    } catch (const std::runtime_error& e) {
        std::cerr << "Caught exception: " << e.what() << ". The silhouette could not be calculated\n";
        m_profile = {};
    }

    // Generate an output image for debugging purposes
    m_sink = QImage(m_data, m_size, m_size, QImage::Format::Format_Grayscale8);
}

uint32_t EdgeDetect::limit(const std::vector<glm::vec2>& contour, const glm::vec2& axis)
{
    uint32_t idx = 0;
    float value = std::numeric_limits<float>::lowest();

    for (uint32_t i = 0; i < contour.size(); i++) {
        float delta = glm::dot(axis, contour[i]);
        if (delta > value) {
            value = delta;
            idx = i;
        }
    }

    return idx;
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
    uint32_t mark = 50;

    // Evaluate contours, skipping borders
    uint32_t idx = size + 1;
    for (int i = 1; i < size - 1; i++){ // Rows
        for (int j = 1; j < size - 1; j++) { // Columns
            if (data[idx] == 255 && data[idx - 1] == 0) {
                try {
                    contours.push_back(traceContour(data, j, i, size, mark++));
                } catch (const std::runtime_error& e) {
                    std::cerr << "Caught exception: " << e.what() << ".\n";
                }
            }

            idx++;
        }

        idx += 2;
    }

    return contours;
}

EdgeDetect::Contour EdgeDetect::traceContour(uchar* data, int x, int y, uint32_t width, uint8_t mark)
{
    const static std::vector<glm::ivec2> directions = {
            {-1, -1},
            { 0, -1},
            { 1, -1},
            { 1,  0},
            { 1,  1},
            { 0,  1},
            {-1,  1},
            {-1,  0}
    };

    Contour contour = { };

    const int sx = x, sy = y;
    int direction = 7;

    do {
        contour.points.emplace_back(x, y);

        data[x + y * width] = mark; // Mark visitation

        int nextDirection = -1;
        for (int i = 0; i < 8; i++) {
            int idx = (direction + i) % 8;
            int nx = x + directions[idx].x, ny = y + directions[idx].y;

            if (inBounds(nx, ny, width) && data[nx + ny * width] >= mark) {
                x = nx;
                y = ny;
                nextDirection = idx;
                break;
            }
        }

        if (nextDirection == -1) { // No neighbor found (should not happen in well-formed image)
            throw std::runtime_error("[EdgeDetect] Failed to trace contour!");
        }

        direction = (nextDirection + 5) % 8;

    } while (!(x == sx && y == sy));

    return contour;
}

bool EdgeDetect::inBounds(int x, int y, uint32_t size)
{
    return x >= 0 && y >= 0 && x < size && y < size;
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

void EdgeDetect::findBorder(std::vector<glm::vec2>& contour)
{

    updateModelDirections();

    // Convex border around the mesh
    std::vector<uint32_t> hull = m_hull.horizon(m_axis, m_up);

    // Convex border around the render
    std::vector<uint32_t> cHull = Polygon::hull(contour);
    std::vector<glm::vec2> hContour;
    hContour.reserve(cHull.size());
    for (uint32_t idx : cHull) hContour.push_back(contour[idx]);

    // Estimate scalar between model and render co-ordinates
    float scale = estimateScale(hull, hContour);

    // Project model hull on to the render
    std::vector<glm::vec2> projection;
    projection.reserve(hull.size());

    for (uint32_t i : hull) {
        projection.emplace_back(scale * glm::vec2{
                glm::dot(m_right, m_hull.vertices()[i]),
                glm::dot(-m_up, m_hull.vertices()[i])
        });
    }

//    Polygon::cullCollinear(projection);

    // Find a reasonable reference vertex to use as the origin & offset projection accordingly
    auto [hShift, cShift] = findReferenceVertex(projection, hContour);
    glm::vec2 offset = hContour[cShift] - projection[hShift];
    for (glm::vec2& proj : projection) proj += offset;

    // Match model vertices to the image corners
    auto border = matchVertices(projection, hContour, hShift, cShift);

    // TODO adjustment based on match


    // Apply calculated offsets to render to convert to model space
    scale = 1 / scale;

    for (glm::vec2& vertex : contour) {
        vertex = {
                (vertex.x - offset.x) * scale,
                (offset.y - vertex.y) * scale
        };
    }
//    std::reverse(contour.begin(), contour.end());

//    for (glm::vec2& vertex : contour) vertex = (vertex - offset) * scale;

    m_profile = Profile(contour, m_axis, m_right, m_up);

    drawPolygon(projection, 200);

}

float EdgeDetect::estimateScale(const std::vector<uint32_t>& hull, const std::vector<glm::vec2>& contour)
{
    // Image scale calculation
    uint32_t top = limit(contour, {0, -1}), bot = limit(contour, {0, 1});

    const float imgScale = glm::dot({0, -1}, contour[top] - contour[bot]);

    // Model scale calculation
    bot = hull[1];
    for (uint32_t i = 2; i < hull.size(); i++)
        if (glm::dot(m_up, m_hull.vertices()[hull[i]] - m_hull.vertices()[bot]) < 0) bot = hull[i];

    const float modelScale = glm::dot(m_up, m_hull.vertices()[hull[0]] - m_hull.vertices()[bot]);

    return imgScale / modelScale;
}

std::tuple<size_t, size_t> EdgeDetect::findReferenceVertex(const std::vector<glm::vec2>& hull, const std::vector<glm::vec2>& contour)
{
    glm::vec2 prev = hull[hull.size() - 1] - hull[0];
    float pLength = glm::length(prev); prev *= 1 / pLength;

    uint32_t hShift = 0, cShift = 0;
    glm::vec2 shiftDirection = {};
    float best = std::numeric_limits<float>::lowest();

    // Try to select the most isolated vertex to use as a reference (sharp angle + distance from neighbors)
    for (uint32_t i = 0; i < hull.size(); i++) {
        glm::vec2 next = hull[(i + 1) % hull.size()] - hull[i];
        float nLength = glm::length(next); next *= 1 / nLength;

        float value = 1 + glm::dot(prev, next);
        value *= value * (pLength + nLength);

        if (value > best) {
            best = value;
            shiftDirection = -0.5f * (prev + next);
            hShift = i;
        }

        pLength = nLength;
        prev = -next;
    }

    // Select a matching contour vertex in the same direction
    best = std::numeric_limits<float>::lowest();
    for (uint32_t i = 0; i < contour.size(); i++) {
        float value = glm::dot(contour[i], shiftDirection);
        if (value > best) {
            best = value;
            cShift = i;
        }
    }

    return { hShift, cShift };
}

std::vector<std::pair<size_t, size_t>> EdgeDetect::matchVertices(const std::vector<glm::vec2>& hull, const std::vector<glm::vec2>& contour, uint32_t hShift, uint32_t cShift)
{

    // Determine the warping path
    auto [matrix, cost] = dynamicTimeWarp(hull, contour, hShift, cShift);
    auto path = dtwPath(matrix);

    // Correct indexing
    for (std::pair<size_t, size_t>& step : path) {
        step.first = (step.first + hShift) % hull.size();
        step.second = (step.second + cShift) % contour.size();
    }

    return path;
}

std::vector<std::pair<size_t, size_t>> EdgeDetect::cyclicTimeWarp(const std::vector<glm::vec2>& hull, const std::vector<glm::vec2>& contour)
{
    uint32_t minHShift = 0, minCShift = 0;
    std::vector<std::vector<double>> minMatrix;
    double minCost = std::numeric_limits<double>::max();



    return { {0, 0 }};

    for (uint32_t i = 0; i < hull.size(); i++) {
        for (uint32_t j = 0; j < contour.size(); j++) {
            auto [matrix, cost] = dynamicTimeWarp(hull, contour, i, j);
            std::cout << i << " " << minHShift << " " << minCShift << " " << cost << " " << minCost << " CTW\n";
            if (cost < minCost) {
                minHShift = i;
                minCShift = j;
                minMatrix = matrix;
                minCost = cost;
            }
        }
    }

    // Determine the warping path
    auto path = dtwPath(minMatrix);
    std::cout << "CTW " << minHShift << " " << minCShift << " " << minCost << " " << path.size() << "\n";

    // Correct indexing
    for (std::pair<size_t, size_t>& step : path) {
        step.first = (step.first + minHShift) % hull.size();
        step.second = (step.second + minCShift) % contour.size();
    }

    for (std::pair<size_t, size_t>& step : path) std::cout << "SP " << step.first << " " << step.second << "\n";

//    std::cout << "DTW distance: " << cost[n][m] << " PS " << path.size() << " " << n << " " << m << "\n";
    return path;
}

std::tuple<std::vector<std::vector<double>>, double> EdgeDetect::dynamicTimeWarp(const std::vector<glm::vec2>& hull, const std::vector<glm::vec2>& contour, uint32_t hShift, uint32_t cShift)
{
    size_t n = hull.size(), m = contour.size();

    std::tuple<std::vector<std::vector<double>>, double> result = {
            std::vector<std::vector<double>>(n + 1, std::vector<double>(m + 1, std::numeric_limits<double>::max())),
            0
    };

    auto& cost = std::get<0>(result);
    cost[0][0] = 0.0;

    // Develop the cost matrix
    for (size_t i = 1; i <= n; i++) {
        uint32_t idx = (i - 1 + hShift) % hull.size();
        for (size_t j = 1; j <= m; j++) {
            glm::vec2 delta = hull[idx] - contour[(j - 1 + cShift) % contour.size()];
            cost[i][j] = glm::dot(delta, delta)
                    + std::min({ cost[i - 1][j], cost[i][j - 1], cost[i - 1][j - 1] });
        }
    }

    std::get<1>(result) = cost[n][m];

    return result;
}

std::vector<std::pair<size_t, size_t>> EdgeDetect::dtwPath(const std::vector<std::vector<double>>& cost)
{
    std::vector<std::pair<size_t, size_t>> path;
    size_t i = cost.size() - 1, j = cost[0].size() - 1;

    while (i > 0 && j > 0) {
        path.emplace_back(i - 1, j - 1);

        // Choose direction with minimal cost
        double diag = cost[i - 1][j - 1];
        double left = cost[i][j - 1];
        double up = cost[i - 1][j];

        if (diag <= left && diag <= up) { i--; j--; }
        else if (left < diag && left < up) j--;
        else i--;
    }

    // Handle remaining steps
    while (i > 0) {
        path.emplace_back(i - 1, j - 1);
        i--;
    }
    while (j > 0) {
        path.emplace_back(i - 1, j - 1);
        j--;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

void EdgeDetect::drawPolygon(const std::vector<glm::vec2>& vertices, uint8_t value)
{
    for (uint32_t i = 0; i < vertices.size(); i++) {
        uint32_t next = (i + 1) % vertices.size();

        glm::vec2 delta = vertices[next] - vertices[i];
        const float length = glm::length(delta), step = 1.0f;
        float j = 0;
        delta *= step / length;

        while (j < length) {
            const glm::vec2 vec = vertices[i] + delta * j;

            int idx = (int)(std::round(vec.x) + (float)m_size * std::round(vec.y));
            if (idx > 0 && idx < m_size * m_size) m_data[idx] = value;

            j += step;
        }
    }
}

RenderCapture* EdgeDetect::capture()
{
    return m_capture;
}

void EdgeDetect::updateModelDirections()
{
    QVector3D fwd = m_capture->camera().forward();
    QVector3D vert = m_capture->camera().vertical();
    QVector3D horz = m_capture->camera().horizontal();

    m_axis = { fwd.x(), fwd.y(), fwd.z() };
    m_up = { vert.x(), vert.y(), vert.z() };
    m_right = { horz.x(), horz.y(), horz.z() };
}

glm::vec3 EdgeDetect::forward() const
{
    return m_axis;
}
glm::vec3 EdgeDetect::vertical() const
{
    return m_up;
}
glm::vec3 EdgeDetect::horizontal() const
{
    return m_right;
}

const QImage& EdgeDetect::source() const
{
    return m_source;
}
const QImage& EdgeDetect::sink() const
{
    return m_sink;
}

const Profile& EdgeDetect::profile() const
{
    return m_profile;
}