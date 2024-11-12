//
// Created by Cam on 2024-10-10.
//

#include "CompositePolygon.h"

#include <numeric>

#include <iostream>
#include <deque>

CompositePolygon::CompositePolygon(const std::vector<QVector2D> &border)
    : Polygon(border)
    , m_diagonalized(false)
{
}

CompositePolygon::CompositePolygon(const std::vector<std::vector<QVector2D>> &loops)
    : Polygon(loops)
    , m_diagonalized(false)
{
}

CompositePolygon::CompositePolygon(const std::vector<QVector3D> &border)
    : Polygon(border, -QVector3D::crossProduct(border[1] - border[0], border[2] - border[0]).normalized())
    , m_diagonalized(false)
{

}

CompositePolygon::CompositePolygon(const std::vector<QVector3D> &border, const QVector3D &normal)
    : Polygon(border, normal)
    , m_diagonalized(false)
{
}

CompositePolygon::CompositePolygon(const std::vector<std::vector<QVector3D>> &loops, const QVector3D &normal)
    : Polygon(loops, normal)
    , m_diagonalized(false)
{
}

void CompositePolygon::invalidate()
{

    m_diagonalized = false;
    m_diagonals.clear();
    m_partitions.clear();
    m_triangles.clear();

    m_eventQueue.clear();
    intervals.clear();
    helper.clear();
}

void CompositePolygon::removeVertex(uint32_t index, bool maintainIntegrity)
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
        identifyVertexType(m_vertices[prev.prev], prev, m_vertices[next.index]);
        identifyVertexType(m_vertices[prev.index], next, m_vertices[next.next]);

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

void CompositePolygon::insertVertex(uint32_t reference, QVector2D vertex)
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
    identifyVertexType(m_vertices[prev.prev], prev, m_vertices[index]);
    identifyVertexType(m_vertices[prev.index], m_links[index], m_vertices[next.index]);
    identifyVertexType(m_vertices[index], next, m_vertices[next.next]);

    invalidate();
}

void CompositePolygon::positionVertex(uint32_t index, QVector2D position, bool maintainIntegrity)
{
    if (index < m_links.size()) {

        Vertex &prev = m_links[m_links[index].prev];
        Vertex &next = m_links[m_links[index].next];

        QVector2D temp = m_vertices[index];
        m_vertices[index] = position;

        // If integrity should be maintained, verify vertex position does not result in self-intersecting geometry
        if (maintainIntegrity) {
            if (intersects(prev, m_links[index]) || intersects(m_links[index], next)) {
                m_vertices[index] = temp;
                return;
            }
        }

        // Update types of neighboring vertices (those that could have changed)
        identifyVertexType(m_vertices[prev.prev], prev, m_vertices[index]);
        identifyVertexType(m_vertices[prev.index], m_links[index], m_vertices[next.index]);
        identifyVertexType(m_vertices[index], next, m_vertices[next.next]);

        // Invalidate results if the movement changes the type of the vertex
        invalidate();
    }
}

void CompositePolygon::boolean(const Polygon &poly, BooleanOperation operation)
{

}

const std::vector<std::pair<int, int>>& CompositePolygon::diagonals()
{
    if (!m_diagonalized) {
        diagonalize();
    }

    return m_diagonals;
}

const std::vector<CompositePolygon::IndexedBorder>& CompositePolygon::partitions()
{
    // Skip process if results have been previously calculated
    if (!m_partitions.empty()) {
        return m_partitions;
    }

    if (!m_diagonalized) {
        diagonalize();
    }

    partition(m_diagonals);

    return m_partitions;
}

const std::vector<Triangle>& CompositePolygon::triangulation()
{
    if (m_triangles.empty()) {
        if (m_partitions.empty()) {
            partitions();
        }

        for (const IndexedBorder &partition : m_partitions) {
            triangulate(partition);
        }
    }

    return m_triangles;
}

const std::vector<std::pair<int, int>>& CompositePolygon::diagonals() const
{
    return m_diagonals;
}

const std::vector<CompositePolygon::IndexedBorder>& CompositePolygon::partitions() const
{
    return m_partitions;
}
const std::vector<Triangle>& CompositePolygon::triangulation() const
{
    return m_triangles;
}

void CompositePolygon::diagonalize(){

    identifyVertexTypes();

    helper = std::vector<int>(m_links.size(), -1);

    m_eventQueue = m_links;
    sort(m_eventQueue.begin(), m_eventQueue.end(), [this] (const Vertex &a, const Vertex &b) {
        return compare(a, b);
    });

    for(uint32_t i = 0; i < m_links.size(); i++){
        Vertex current = m_eventQueue[i];

        switch(current.type){
            case VertexType::NORMAL:
                handleRegularVertex(current);
                break;
            case VertexType::START:
                handleStartVertex(current);
                break;
            case VertexType::END:
                handleEndVertex(current);
                break;
            case VertexType::SPLIT:
                handleSplitVertex(current);
                break;
            case VertexType::MERGE:
                handleMergeVertex(current);
                break;
        }
    }

    // Prevent any diagonals between direct neighbors
    for (uint32_t i = 0; i < m_diagonals.size(); i++) {
        if (m_links[m_diagonals[i].first].prev == m_diagonals[i].second || m_links[m_diagonals[i].first].next == m_diagonals[i].second) {
            m_diagonals.erase(m_diagonals.begin() + i);
            i--;
        }
    }

    m_diagonalized = true;
}

void CompositePolygon::identifyVertexTypes(){
    for(Vertex& v : m_links){
        if(v.type != VertexType::NORMAL){
            continue;
        }

        const QVector2D &prev = m_vertices[v.prev];
        const QVector2D &next = m_vertices[v.next];

        identifyVertexType(prev, v, next);
    }
}

void CompositePolygon::identifyVertexType(const QVector2D &prev, Vertex &vertex, const QVector2D &next)
{
    if(prev.y() <= m_vertices[vertex.index].y() && next.y() < m_vertices[vertex.index].y()){// Current greater than connected vertices
        if(angle(m_vertices[vertex.index], next, prev)){
            vertex.type = VertexType::END;
        }else{
            vertex.type = VertexType::MERGE;
        }
    }else if(prev.y() >= m_vertices[vertex.index].y() && next.y() > m_vertices[vertex.index].y()){// Current less than connected vertices
        if(angle(m_vertices[vertex.index], next, prev)){
            vertex.type = VertexType::START;
        }else{
            vertex.type = VertexType::SPLIT;
        }
    }else{
        vertex.type = VertexType::NORMAL;
    }
}

void CompositePolygon::handleRegularVertex(const Vertex& current){

    // If the edge made by this vertex encloses the area to the right
    if(m_vertices[current.prev].y() < m_vertices[current.index].y()) {

        tryMergeDiagonalInsertion(current, helper[current.prev]);

        removeInterval(current.prev);

        // Skip horizontal intervals
        if (m_vertices[current.index].y() != m_vertices[current.next].y()) insertInterval(current, m_links[current.next]);

    } else { // Area enclosed is to the left
        if(!intervals.empty()){
            int index = getLeftNeighborIndex(current);

            tryMergeDiagonalInsertion(current, helper[index]);

            helper[index] = current.index;
        }
    }
}

void CompositePolygon::handleStartVertex(const Vertex& current){
    insertInterval(current, m_links[current.next]);
}

void CompositePolygon::handleEndVertex(const Vertex& current){
    tryMergeDiagonalInsertion(current, helper[current.prev]);

    removeInterval(current.prev);
}

void CompositePolygon::handleSplitVertex(const Vertex& current){
    if(!intervals.empty()){

        int index = getLeftNeighborIndex(current);

        m_diagonals.emplace_back(current.index, helper[index]);
        helper[index] = current.index;
    }

    insertInterval(current, m_links[current.next]);
}

void CompositePolygon::handleMergeVertex(const Vertex& current){
    tryMergeDiagonalInsertion(current, helper[current.prev]);

    removeInterval(current.prev);

    if(!intervals.empty()){

        int index = getLeftNeighborIndex(current);
        tryMergeDiagonalInsertion(current, helper[index]);

        helper[index] = current.index;
    }
}

void CompositePolygon::tryMergeDiagonalInsertion(const Vertex& current, int index){
    if(m_links[index].type == VertexType::MERGE){
        m_diagonals.emplace_back(current.index, index);
    }
}

void CompositePolygon::insertInterval(const Vertex& current, const Vertex& end){

    // Identify where the interval should be inserted
    uint32_t index = 0;
    while(index < intervals.size()){
        if(interpolate(m_vertices[intervals[index].first], m_vertices[intervals[index].second], m_vertices[current.index].y()) > m_vertices[current.index].x()){
            break;
        }

        index++;
    }

    // Create interval
    intervals.insert(intervals.begin() + index, {current.index, end.index});
    helper[current.index] = current.index;
}

void CompositePolygon::removeInterval(int index){
    for(uint32_t i = 0; i < intervals.size(); i++){
        if(intervals[i].first == index){
            intervals.erase(intervals.begin() + i);
            break;
        }
    }

}

int CompositePolygon::getLeftNeighborIndex(const Vertex& current){
    int left = -1;

    for(const auto &interval : intervals){
        if (interpolate(m_vertices[interval.first], m_vertices[interval.second], m_vertices[current.index].y()) > m_vertices[current.index].x()) break;

        left = interval.first;
    }

    return left;
}

void CompositePolygon::partition(const std::vector<std::pair<int, int>> &diagonals)
{
    // If no partitions are needed, populate contents with the hull itself
    if (diagonals.empty()) {
        m_partitions.push_back(IndexedBorder{std::vector<uint32_t>(m_links.size())});
        std::iota(m_partitions[0].vertices.begin(), m_partitions[0].vertices.end(), 0);
        return;
    }

    // Record number of usages of each vertex
    int remainder = m_links.size() + 2 * diagonals.size();
    std::vector<int> count(m_links.size(), 1);
    std::vector<std::vector<uint32_t>> map(m_links.size());
    for (const auto &diagonal : diagonals) {
        map[diagonal.first].push_back(diagonal.second);
        map[diagonal.second].push_back(diagonal.first);

        count[diagonal.first]++;
        count[diagonal.second]++;
    }

    uint32_t top = 0;
    while (remainder > 0) {
        IndexedBorder partition;

        // Identify first vertex of partition (top-most of remaining set)
        uint32_t current = 0, next = 0;
        for (uint32_t j = top; j < m_eventQueue.size(); j++) {
            if (count[m_eventQueue[j].index] > 0) {
                current = m_eventQueue[j].index;
                top = j;

                break;
            }
        }

        next = m_links[current].next;

        QVector2D ref = {m_vertices[current].x() - 100, m_vertices[current].y()};

        int prev = -1;
        int limit = 10000;
        do {
            partition.vertices.push_back(current);
            count[current]--;
            remainder--;

            // Select the most internal edge to traverse
            float value = internalAngle(m_vertices[current], ref, m_vertices[next]);
            for (uint32_t cut : map[current]) {

                // Limit selection to avoid backtracking and retraversal
                if (prev == cut || (count[cut] <= 0 && cut != partition.vertices[0])) continue;

                // Choose the smallest internal angle
                float temp = internalAngle(m_vertices[current], ref, m_vertices[cut]) ;

                if (value < temp || count[next] == 0) {
                    next = cut;
                    value = temp;
                }
            }

            // If a partition boundary was traversed, remove link to prevent retraversal
            if (next != m_links[current].next) {
                map[current].erase(std::find(map[current].begin(), map[current].end(), next));
            }


            ref = m_vertices[current];
            prev = current;
            current = next;
            next = m_links[current].next;

        } while (current != partition.vertices[0] && --limit > 0); // Continue until loop is completed

        if (limit == 0) std::cout << "\033[91mPolygon partition failed! Limit reached!\033[0m\n";

        m_partitions.push_back(partition);
    }
}

void CompositePolygon::triangulate(const IndexedBorder &partition)
{
    if (partition.vertices.size() == 3) {
        m_triangles.emplace_back(partition.vertices[0], partition.vertices[1], partition.vertices[2]);
//        m_triangles.push_back(ccwTriangle(partition.vertices[0], partition.vertices[1], partition.vertices[2]));
        return;
    }

    std::vector<Vertex> list(partition.vertices.size());
    std::vector<QVector2D> vertices(partition.vertices.size());

    std::vector<uint32_t> indexMap(list.size());

    // Populate list with vertices of the partition
    for (int i = 0; i < partition.vertices.size(); i++) {
        list[i] = { i };
        vertices[i] = m_vertices[partition.vertices[i]];
        indexMap[i] = m_links[partition.vertices[i]].index;
    }

    // Find the highest vertex
    uint32_t top = 0;
    for (uint32_t i = 0; i < list.size(); i++) {
        if (vertices[list[top].index].y() > vertices[list[i].index].y()) top = i;
    }

    // Develop links for vertex chains from the top most to the bottom most vertex, sorting in descending order

    std::vector<Vertex> set;
    set.reserve(list.size());
    set.push_back(list[top]);

    int prevL = -1, prevR = 0, left = top == list.size() - 1 ? 0 : top + 1, right = top == 0 ? list.size() - 1 : top - 1;//, limit = list.size();
    while (left != right) {
        int size = (int)set.size();

        if (vertices[list[left].index].y() < vertices[list[right].index].y()) {
            set.push_back(list[left]);
            left = left == list.size() - 1 ? 0 : left + 1;
            if (prevL != -1) set[prevL].next = size;
            prevL = size;
        } else {
            set.push_back(list[right]);
            right = right == 0 ? list.size() - 1 : right - 1;
            set[prevR].next = size;
            prevR = size;
        }
    }
    set.push_back(list[right]);

    list = set;

    std::deque<Vertex> queue;
    queue.push_back(list[0]);
    queue.push_back(list[1]);

    float reference = 1 - 2 * (float)(list[0].next != 1);
    for(size_t i = 2; i < list.size(); i++){
        if(i == queue.back().next){// Check whether next vertex is part of the same chain

            while(queue.size() >= 2){

                Vertex v = queue.back();
                queue.pop_back();

                // Check for exterior angle - do not proceed if found
                if(Triangle::cross(vertices[v.index], vertices[queue.back().index], vertices[list[i].index]) * reference > 0){
                    queue.push_back(v);
                    break;
                }

                m_triangles.push_back(ccwTriangle(
                        indexMap[v.index], indexMap[list[i].index], indexMap[queue.back().index]
                ));
            }


            queue.push_back(list[i]);
        }else{

            // Capture free vertices from the opposite chain within separate triangles
            while(queue.size() >= 2){
                Vertex v = queue.front();
                queue.pop_front();

                m_triangles.push_back(ccwTriangle(
                        indexMap[v.index], indexMap[list[i].index], indexMap[queue.front().index]
                ));
            }

            queue.push_back(list[i]);

            // Update reference angle for interior angle checks
            reference = -reference;
        }
    }
}

bool CompositePolygon::compare(const Vertex& v1, const Vertex& v2){
    if(m_vertices[v1.index].y() == m_vertices[v2.index].y()) {
        return v1.type != VertexType::NORMAL;
    }

    return m_vertices[v1.index].y() < m_vertices[v2.index].y();
}

bool CompositePolygon::angle(const QVector2D &pivot, const QVector2D &a, const QVector2D &b)
{
    return Triangle::cross(pivot, a, b) <= 0;
}

float CompositePolygon::internalAngle(const QVector2D &pivot, const QVector2D &a, const QVector2D &b)
{
    return !angle(pivot, a, b) ? dot(pivot, a, b) : -2 - dot(pivot, a, b);
}

float CompositePolygon::dot(const QVector2D &pivot, const QVector2D &a, const QVector2D &b)
{
    return dot(a - pivot, b - pivot);
}

float CompositePolygon::dot(const QVector2D &a, const QVector2D &b) {
    return (a.x() * b.x() + a.y() * b.y()) / a.length() / b.length();
}

