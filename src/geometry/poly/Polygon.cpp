//
// Created by cameronh on 02/06/24.
//

#include "Polygon.h"

#include <iostream>

Polygon::Polygon(const std::vector<QVector2D> &border)
    : m_loops({0})
    , m_area(-1)
    , m_areaOK(false)
{
    initializeLoop(border);
}

Polygon::Polygon(const std::vector<std::vector<QVector2D>> &loops)
    : m_loops(loops.size())
    , m_area(-1)
    , m_areaOK(false)
{
    for (uint32_t i = 0; i < loops.size(); i++) {
        m_loops[i] = m_links.size();

        initializeLoop(loops[i]);
    }
}

Polygon::Polygon(const std::vector<QVector3D> &border, const QVector3D &normal)
    : m_loops({0})
    , m_area(-1)
    , m_areaOK(false)
{
    if (border.size() < 3) return;

    initializeLoop(reduce(border, normal));
}

Polygon::Polygon(const std::vector<std::vector<QVector3D>> &loops, const QVector3D &normal)
    : m_loops(loops.size())
    , m_area(-1)
    , m_areaOK(false)
{
    if (loops.empty() || loops[0].size() < 3) return;

    QVector3D xAxis = (loops[0][1] - loops[0][0]).normalized();
    QVector3D yAxis = QVector3D::crossProduct(normal, xAxis).normalized();
    QVector3D zAxis = QVector3D::crossProduct(xAxis, yAxis).normalized();

    System sys = { loops[0][0], xAxis, yAxis, zAxis };

    for (uint32_t i = 0; i < loops.size(); i++) {
        m_loops[i] = m_links.size();

        initializeLoop(reduce(loops[i], normal, sys));
    }

}

void Polygon::initializeLoop(const std::vector<QVector2D> &border)
{
    int start = m_links.size(), index = start;
    for (const QVector2D &vertex : border) {
        m_links.push_back({
            index,
            index - 1,
            ++index,
            VertexType::NORMAL
        });
    }

    m_vertices.insert(m_vertices.end(), border.begin(), border.end());

    // Correct boundary edge links
    m_links[start].prev = (int)m_links.size() - 1;
    m_links[m_links.size() - 1].next = start;
}

std::vector<QVector2D> Polygon::reduce(const std::vector<QVector3D> &loop, const QVector3D &normal, const System &sys)
{
    std::vector<QVector2D> reduction;
    reduction.reserve(loop.size());

    for(const QVector3D &vertex : loop) {
        QVector3D rel = vertex - sys.origin;
        reduction.emplace_back(QVector3D::dotProduct(rel, sys.xAxis), QVector3D::dotProduct(rel, sys.yAxis));
        if (std::abs(QVector3D::dotProduct(rel, sys.zAxis)) > std::numeric_limits<float>::epsilon()) {
            std::cout << "\033[31mERROR! Can not reduce polygon, vertices are not in-plane\033[0m\n";
            break;
        }
    }

    return reduction;
}

void Polygon::invalidate()
{
    m_areaOK = false;
}

void Polygon::removeVertex(uint32_t index, bool maintainIntegrity)
{
    if (index < m_links.size()) {

        uint32_t loop = identifyParentLoop(index);
        if (loopLength(loop) <= 3) return;

        // If integrity should be maintained, verify vertex deletion does not result in self-intersecting edge
        if (maintainIntegrity) {
            if (intersects(m_links[m_links[index].prev], m_links[m_links[index].next])) {
                return;
            }
        }

        Vertex &prev = m_links[m_links[index].prev];
        Vertex &next = m_links[m_links[index].next];

        // Determine whether the neighboring vertices need a type change
//        identifyVertexType(m_hull[prev.prev].p, prev, next.p);
//        identifyVertexType(prev.p, next, m_hull[next.next].p);

        // Repair the hole created by the removal
        prev.next = m_links[index].next;
        next.prev = m_links[index].prev;

        // Ensure loop index still exists within the desired loop
        if (m_loops[loop] == index) m_loops[loop] = m_links[index].next;

        // Remove the vertex
        m_links.erase(m_links.begin() + index);
        m_vertices.erase(m_vertices.begin() + index);

        // Update all indices affected by removal of a vertex
        for (uint32_t &idx : m_loops) idx -= idx > index;
        for (Vertex &hv : m_links) {
            hv.prev -= hv.prev > index;
            hv.index -= hv.index > index;
            hv.next -= hv.next > index;
        }

        invalidate();
    }
}

void Polygon::insertVertex(uint32_t reference, QVector2D vertex)
{
    if (reference >= m_links.size()) reference = m_links.size() - 1;

    // Update all indices affected by addition of a vertex
    for (uint32_t &idx : m_loops) idx += idx > reference;
    for (Vertex &hv : m_links) {
        hv.prev += hv.prev > reference;
        hv.index += hv.index > reference;
        hv.next += hv.next > reference;
    }

    // Insert the vertex
    int index = (int)reference + 1;
    int nextIndex = m_links[reference].next;
    m_links.insert(m_links.begin() + index, {index, (int)reference, nextIndex, VertexType::NORMAL});
    m_vertices.insert(m_vertices.begin() + index, vertex);

    // Link neighbor vertices to the new vertex
    Vertex &prev = m_links[m_links[index].prev];
    Vertex &next = m_links[m_links[index].next];

    prev.next = index;
    next.index = nextIndex;
    next.prev = index;
    m_links[next.next].prev = nextIndex;

    // Determine the new type of the vertex and its immediate neighbors
//    identifyVertexType(m_hull[prev.prev].p, prev, m_hull[index].p);
//    identifyVertexType(prev.p, m_hull[index], next.p);
//    identifyVertexType(m_hull[index].p, next, m_hull[next.next].p);

    invalidate();
}

void Polygon::positionVertex(uint32_t index, QVector2D position, bool maintainIntegrity)
{
    if (index < m_links.size()) {

        Vertex &prev = m_links[m_links[index].prev];
        Vertex &next = m_links[m_links[index].next];

        QVector2D temp = m_vertices[m_links[index].index];
        m_vertices[m_links[index].index] = position;

        // If integrity should be maintained, verify vertex position does not result in self-intersecting geometry
        if (maintainIntegrity) {
            if (intersects(prev, m_links[index]) || intersects(m_links[index], next)) {
                m_vertices[m_links[index].index] = temp;
                return;
            }
        }

        // Update types of neighboring vertices (those that could have changed)
//        identifyVertexType(m_hull[prev.prev].p, prev, m_hull[index].p);
//        identifyVertexType(prev.p, m_hull[index], next.p);
//        identifyVertexType(m_hull[index].p, next, m_hull[next.next].p);

        // Invalidate results if the movement changes the type of the vertex
        invalidate();
    }
}

void Polygon::translate(QVector2D delta) {
    for (QVector2D &vertex : m_vertices) vertex += delta;
}

void Polygon::rotate(QVector2D center, float theta)
{
    for (QVector2D &vertex : m_vertices) {
        QVector2D delta = vertex - center;
        vertex = {
                center.x() + delta.x() * std::cos(theta) - delta.y() * std::sin(theta),
                center.y() + delta.x() * std::sin(theta) + delta.y() * std::cos(theta)
        };
    }
}

uint32_t Polygon::loopCount()
{
    return m_loops.size();
}

uint32_t Polygon::loopLength(uint32_t index)
{
    return getLoop(index).size();
}

uint32_t Polygon::identifyParentLoop(uint32_t index) {
    uint32_t idx = 0, current = 0;

    for (uint32_t start : m_loops) {
        current = start;

        do {
            if (current == index) return idx;
            current = m_links[current].next;
        } while(start != current);

        idx++;
    }

    return m_loops.size();
}

std::vector<uint32_t> Polygon::getLoop(uint32_t index)
{
    std::vector<uint32_t> loop;

    if (index < m_loops.size()) {
        uint32_t start = m_loops[index], current = start;

        do {
            loop.push_back(current);
            current = m_links[current].next;
        } while (start != current);
    }

    return loop;
}

uint32_t Polygon::vertexCount()
{
    return m_vertices.size();
}

QVector2D Polygon::getVertex(uint32_t index)
{
    if (index < m_vertices.size()) return m_vertices[index];

    return {0, 0};
}

bool Polygon::encloses(const QVector2D &p)
{
//    triangulation();
//
//    for (const Triangle &tri : m_triangles) {
//        if (Triangle::encloses(m_hull[tri.m_I0].p, m_hull[tri.m_I1].p, m_hull[tri.m_I2].p, p)) return true;
//    }
    std::cout << "\033[93mPolygon enclosure not yet programmed!\033[0m\n";
    return false;
}

float Polygon::area()
{
    if (!m_areaOK) {
        float sum = area(0);

        for (uint32_t i = 1; i < m_loops.size(); i++) {
            sum += area(i);
        }

        m_area = std::abs(sum);
        m_areaOK = true;
    }

    return m_area;
}

float Polygon::area() const
{
    if (m_areaOK) return m_area;
    return -1;
}

float Polygon::area(uint32_t loopIndex)
{
    if (loopIndex >= m_loops.size()) return 0;

    uint32_t start = m_loops[loopIndex], current = start;
    std::vector<QVector2D> border;

    do {
        border.push_back(m_vertices[current]);
        current = m_links[current].next;
    } while(start != current);

    return area(border);
}

float Polygon::area(const std::vector<QVector2D> &border)
{
    float area = 0;

    for (uint32_t i = 0; i < border.size(); i++) {
        uint32_t next = i + 1 == border.size() ? 0 : i + 1;
        area += (border[i].y() + border[next].y()) * (border[i].x() - border[next].x());
    }

    return area / 2;
}

std::vector<QVector2D> Polygon::reduce(const std::vector<QVector3D> &loop, const QVector3D &normal)
{
    QVector3D xAxis = (loop[1] - loop[0]).normalized();
    QVector3D yAxis = QVector3D::crossProduct(normal, xAxis).normalized();
    QVector3D zAxis = QVector3D::crossProduct(xAxis, yAxis).normalized();

    System sys = { loop[0], xAxis, yAxis, zAxis };
    return reduce(loop, normal, sys);
}

Triangle Polygon::ccwTriangle(uint32_t I0, uint32_t I1, uint32_t I2)
{
    if (Triangle::cross(m_vertices[I0], m_vertices[I1], m_vertices[I2]) < 0) {
        return {I0, I1, I2};
    } else {
        return {I0, I2, I1};
    }
}

float Polygon::interpolate(const QVector2D &start, const QVector2D &end, float dy)
{
    return start.x() + (end.x() - start.x()) * (dy - start.y()) / (end.y() - start.y());
}

bool Polygon::intersects(const Vertex &a, const Vertex &b)
{
    for (uint32_t start : m_loops) {
        uint32_t last = start, current = m_links[start].next;

        do {
            if (!(a.index == last || a.index == current || b.index == last || b.index == current)) {
                if (segmentIntersection(m_vertices[a.index], m_vertices[b.index], m_vertices[last], m_vertices[current])) return true;
            }

            last = current;
            current = m_links[current].next;
        } while (start != last);
    }

    return false;
}

bool Polygon::segmentIntersection(const QVector2D &a, const QVector2D &b, const QVector2D &c, const QVector2D &d)
{
    float a1 = Triangle::signedArea(a, b, d);
    float a2 = Triangle::signedArea(a, b, c);

    if (a1 * a2 < 0) {
        float a3 = Triangle::signedArea(c, d, a);
        float a4 = a3 + a2 - a1;

        if (a3 * a4 < 0) {
            // t = a3 / (a3 - a4)
            // p = a + t * (b - a)
            return true;
        }
    }

    return false;
}