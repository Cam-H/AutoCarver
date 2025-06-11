//
// Created by Cam on 2025-06-04.
//

#include "EdgeDetect.h"

#include "geometry/Mesh.h"
#include "renderer/RenderCapture.h"

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

    m_capture->clearTargets();
    prepareTargets();
}

void EdgeDetect::prepareTargets()
{
    auto hull = std::make_shared<Mesh>(ConvexHull(mesh->vertices()));
    m_capture->addTarget(hull, QColor(0, 0, 180));

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
    m_capture->capture();
    m_source = m_capture->grabFramebuffer();
    m_sink = m_source.convertToFormat(QImage::Format::Format_Grayscale8);

    delete[] m_data;
    m_data = new uchar[m_size * m_size];

    detectEdges(m_sink);

    m_contours = suzukiAbe(m_data, m_size);
    for (Contour& contour : m_contours) {
        std::cout << "Size: " << contour.points.size() << " -> ";
        douglasPeucker(contour, m_epsilon);
        std::cout << contour.points.size() << "\n";

//        for (auto v : contour.points) std::cout << v.x << " " << v.y << "\n";
    }

    for (const glm::vec2& vertex : m_contours[0].points) {
        m_data[(int)(std::round(vertex.x + vertex.y * m_size))] = 150;
    }

    findBorder(m_contours[0].points);

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
                contours.push_back(traceContour(data, j, i, size, mark++));
//                return contours;
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

        if (nextDirection == -1) {
            std::cout << "Failed to trace contour!\n";
            break; // No neighbor found (should not happen in well-formed image)
        }

        direction = (nextDirection + 5) % 8;

    } while (!(x == sx && y == sy));

//    contour.points = std::vector<glm::vec2>(contour.points.rbegin(), contour.points.rend());

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

// Merge vertices that are sufficiently close together
//void EdgeDetect::mergeVertices(Contour& contour)
//{
//    for (uint32_t i = 0; i < contour.points.size() - 1; i++) {
//        glm::vec2 delta = contour.points[i + 1] - contour.points[i];
//        if (glm::dot(delta, delta) < 0.01f * m_size) {
//            contour.points[i] += 0.5f * delta;
//            contour.points.erase(contour.points.begin() + i + 1);
//            i--;
//        }
//    }
//}

void EdgeDetect::findBorder(std::vector<glm::vec2>& contour)
{

    updateModelDirections();

    // Convex border around the mesh
    std::vector<uint32_t> hull = m_hull.horizon(m_axis, m_up);

    float scale;
    uint32_t hRef, cRef;
    findReferences(hull, contour, scale, hRef, cRef);

    glm::vec3 origin = m_hull.vertices()[hRef] - m_axis * glm::dot(m_axis, m_hull.vertices()[hRef]);
    glm::vec2 offset = contour[cRef];

    // Rearrange contour so the anchor vertex comes first
    contour.insert(contour.end(), contour.begin(), contour.begin() + cRef);
    contour.erase(contour.begin(), contour.begin() + cRef);

    std::vector<glm::vec2> projection;
    projection.reserve(hull.size());

    std::cout << hRef << " " << scale * origin.x << " " << scale * origin.y << " " << scale * origin.z << "~~ ORIGIN\n";
    std::cout << cRef << " " << offset.x << " " << offset.y << " " << 0 << "~~ OFFSET\n";
//    scale /= 2;

    for (uint32_t i : hull) {
        glm::vec3 delta = m_hull.vertices()[i] - origin;
        projection.emplace_back(offset + scale * glm::vec2{
                glm::dot(m_right, delta),
                glm::dot(-m_up, delta)
        });
    }


    for (uint32_t j : hull) std::cout << j << " ";
    std::cout << "!!!!\n";

    for (glm::vec2 proj : projection) {
        int idx = (int)(std::round(proj.x) + (float)m_size * std::round(proj.y));

        if (proj.x > 0 && proj.y > 0) {
            if (idx > 0 && idx < m_size * m_size) m_data[idx] = 255;
        }
    }

    return;

    // Match model vertices to image corners
    auto border = dynamicTimeWarp(projection, contour);

    std::cout << " PS " << border.size() << "\n";
    for (std::pair<size_t, size_t>& step : border) std::cout << step.first << " " << step.second << "\n";

    prepareBorder(border, projection, contour);

    // Map border indices and projected vertices back into 3D space
    scale = 1.0f / scale;
    for (std::pair<uint32_t, glm::vec3>& vertex : m_border) {
        if (vertex.first == std::numeric_limits<uint32_t>::max()) { // Unmatched vertex (Probably concave)
            vertex.second = scale * (vertex.second - glm::vec3{ offset.x, offset.y, 0 });
            vertex.second = (m_right * vertex.second.x - m_up * vertex.second.y + origin);
        } else { // Matched vertex (On convex hull)
            vertex.first = hull[vertex.first];
            vertex.second = m_hull.vertices()[vertex.first] - m_axis * glm::dot(m_axis, m_hull.vertices()[vertex.first]);
        }
    }


    // Map indices
    uint32_t idx = 0;
    for (auto step : m_border) {
        std::cout << idx++ << " " << step.second.x << " " << step.second.y << " " << step.second.z << " " << step.first << "\n";

    }

}

void EdgeDetect::findReferences(const std::vector<uint32_t>& hull, const std::vector<glm::vec2>& contour, float& scale, uint32_t& hRef, uint32_t& cRef)
{
    float modelScale, imgScale;

    uint32_t bot;

    // Model scale calculation
    hRef = hull[0], bot = hull[1];
    for (uint32_t i = 2; i < hull.size(); i++)
        if (glm::dot(m_up, m_hull.vertices()[hull[i]] - m_hull.vertices()[bot]) < 0) bot = hull[i];

    modelScale = glm::dot(m_up, m_hull.vertices()[hRef] - m_hull.vertices()[bot]);

    std::cout << "MODEL Top: " << hRef << ", bot: " << bot << " MS " << modelScale << "\n";

    // Image scale calculation
    cRef = limit(contour, {1, -5}), bot = limit(contour, {0, 1});
    imgScale = glm::dot({0, -1}, contour[cRef] - contour[bot]);

    std::cout << "IMG Top: " << cRef << ", bot: " << bot << " " << imgScale << "\n";

    scale = imgScale / modelScale;
}

void EdgeDetect::prepareBorder(const std::vector<std::pair<size_t, size_t>>& border, const std::vector<glm::vec2>& projection, const std::vector<glm::vec2>& contour)
{

    m_border.clear();
    m_border.reserve(border.size());

    glm::vec2 sum = {}, min;
    uint32_t count = 0, current = 0, k = 0, minIdx;

    for (uint32_t i = 0; i < projection.size(); i++) {

        // Identify the best match when multiple vertices are matched to the same hull vertex
        minIdx = k + i;
        min = contour[border[minIdx].second] - projection[i];
        while (i + k < border.size() - 1 && border[k + i].first == border[k + i + 1].first) {
            glm::vec2 test = contour[border[k + i + 1].second] - projection[i];
            if (glm::dot(min, min) > glm::dot(test, test)) {
                minIdx = k + i + 1;
                min = test;
            }
            k++;
        }

        bool aberrant = glm::dot(min, min) > 100;

        std::cout << i << " " << minIdx << " "
            << projection[i].x << " " << projection[i].y << " " << contour[border[minIdx].second].x << " " << contour[border[minIdx].second].y
            << " | " << glm::dot(min, min) << "\n";
        // Capture concave sections between matched vertices
        captureConcave(contour, current, border[minIdx].second + aberrant);

        // Calculate average offset to reduce error
        if (!aberrant) { // Aberrant value - Probably failed to match vertex to hull
            sum += min;
            current++;
            count++;
        }

        m_border.emplace_back(i, glm::vec3{});
    }

    // Capture remaining concave vertices (between first and last hull vertices)
    captureConcave(contour, current, contour.size());

    std::cout << "ED " << m_border.size() << "\n";

//    glm::vec3 avg = glm::vec3{ sum.x, sum.y, 0} * (1.0f / (float)count);
//    for (auto& vertex : m_border) vertex.second += avg;
//
//    std::cout << count << " " << avg.x << " " << avg.y << "\n";
}

void EdgeDetect::captureConcave(const std::vector<glm::vec2>& source, uint32_t& start, uint32_t end)
{
    while (start < end) {
        m_border.emplace_back(std::numeric_limits<uint32_t>::max(), glm::vec3{
                source[start].x, source[start].y, start
        });
        start++;
    }
}

std::vector<std::pair<size_t, size_t>> EdgeDetect::dynamicTimeWarp(const std::vector<glm::vec2>& hull, const std::vector<glm::vec2>& contour)
{
    for (glm::vec2 v : hull) std::cout << "HULL " << v.x << " " << v.y << "\n";
    for (glm::vec2 v : contour) std::cout << "CONTOUR " << v.x << " " << v.y << "\n";

    // Develop a map of convex vertices (concave vertices are excluded because they necessarily can not match the hull)
//    std::vector<uint32_t> map = { 0 };
//    map.reserve(contour.size());
//
//    glm::vec2 prev = contour[0] - contour[1];
//    for (uint32_t i = 1; i < contour.size(); i++) {
//        glm::vec2 next = contour[(i + 1) % contour.size()] - contour[i];
//        if (prev.y * next.x - prev.x * next.y < 0) map.push_back(i);
//        std::cout << i << " " << (prev.y * next.x - prev.x * next.y > 0) << "| " << contour[i].x << " " << contour[i].y << "\n";
//        prev = -next;
//    }

//    size_t n = hull.size(), m = map.size();
    size_t n = hull.size(), m = contour.size();

    std::vector<std::vector<double>> cost(n + 1, std::vector<double>(m + 1, std::numeric_limits<double>::infinity()));
    cost[0][0] = 0.0;

    // Develop the cost matrix
    for (size_t i = 1; i <= n; i++) {
        for (size_t j = 1; j <= m; j++) {
//            std::cout << i << " " << j << "\n";
            glm::vec2 delta = hull[i - 1] - contour[j - 1];
            cost[i][j] = glm::dot(delta, delta)
                    + std::min({ cost[i - 1][j], cost[i][j - 1], cost[i - 1][j - 1] });
        }
    }

    // Determine the warping path
    auto path = dtwPath(cost);
    for (std::pair<size_t, size_t>& step : path) std::cout << "PSSS " << step.second << "\n";
//    for (std::pair<size_t, size_t>& step : path) step.second = map[step.second];

    std::cout << "DTW distance: " << cost[n][m] << " PS " << path.size() << " " << n << " " << m << "\n";

    return path;
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

    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
    std::cout << m_axis.x << " " << m_axis.y << " " << m_axis.z << "\n";
    std::cout << m_up.x << " " << m_up.y << " " << m_up.z << "\n";
    std::cout << m_right.x << " " << m_right.y << " " << m_right.z << "\n";
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

const std::vector<std::pair<uint32_t, glm::vec3>>& EdgeDetect::border() const
{
    return m_border;
}