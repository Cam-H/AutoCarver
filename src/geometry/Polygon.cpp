//
// Created by cameronh on 02/06/24.
//

#include "Polygon.h"

#include <numeric>

#include <iostream>
#include <deque>

Polygon::Polygon(const std::vector<QVector2D> &border) : m_diagonalized(false), m_loops({0})
{

    std::vector<QVector2D> invBorder(border.rbegin(), border.rend());
    for(const QVector2D &v : border) {
        std::cout << v.x() << " " << v.y() << "\n";
    }
    std::cout << "br\n";
    for(const QVector2D &v : invBorder) {
        std::cout << v.x() << " " << v.y() << "\n";
    }

    initializeLoop(border);
}

Polygon::Polygon(const std::vector<std::vector<QVector2D>> &loops) : m_diagonalized(false), m_loops(loops.size())
{
    bool internal = false;
    for (uint32_t i = 0; i < loops.size(); i++) {
        m_loops[i] = m_hull.size();

        initializeLoop(loops[i], internal);
        internal = true;
    }
}

void Polygon::initializeLoop(const std::vector<QVector2D> &border, bool internal)
{
    int start = m_hull.size(), index = start;
    for (const QVector2D &vertex : border) {
        m_hull.push_back({
                                 vertex,
                                 index,
                                 index - 1,
                                 ++index,
                                 VertexType::NORMAL,
                                 internal
                         });
    }

    // Correct boundary edge links
    m_hull[start].prev = (int)m_hull.size() - 1;
    m_hull[m_hull.size() - 1].next = start;
}

std::vector<Polygon::IndexedBorder> Polygon::partition()
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

void Polygon::diagonalize(){
    if (m_diagonalized) return;

    identifyVertexTypes();

    helper = std::vector<int>(m_hull.size(), -1);

    m_eventQueue = m_hull;
    sort(m_eventQueue.begin(), m_eventQueue.end(), compare);
    for(const auto& dx : m_eventQueue) {
        std::cout << dx.index << " ";
    }
    std::cout << "| zwo\n";

    for(uint32_t i = 0; i < m_hull.size(); i++){
        Vertex current = m_eventQueue[i];

        std::cout << "State: ";
        for(const auto& iv : intervals) {
            std::cout << iv.first << "(" << iv.second << ")[" << helper[iv.first] << "] ";
        }
        std::cout << "\n";

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

//        sort(intervals.begin(), intervals.end(), compx);
//        cout<<"\nTree structure at iteration "<<i+1<<":\n\t";
//        for(int j=0;j<intervals.size();j++){
//            cout<<intervals[j].first<<" ";
//        }
    }

    std::cout<<"\nEnd of algorithm. Partitioned into monotone pieces\n\n";
    std::cout<<"No of diagonals inserted:"<<m_diagonals.size();
    std::cout<<"\n The diagonals are inserted between:\n";
    for(int i=0;i<m_diagonals.size();i++){
        std::cout<<m_diagonals[i].first
                 <<"\t"<<m_diagonals[i].second <<"\n";
    }

    // Prevent any diagonals between direct neighbors
    for (uint32_t i = 0; i < m_diagonals.size(); i++) {
        if (m_hull[m_diagonals[i].first].prev == m_diagonals[i].second || m_hull[m_diagonals[i].first].next == m_diagonals[i].second) {
            m_diagonals.erase(m_diagonals.begin() + i);
            i--;
        }
    }

    m_diagonalized = true;
}

std::vector<Triangle> Polygon::tesselate()
{
    if (m_triangles.empty()) {
        if (m_partitions.empty()) {
            partition();
        }

        for (const IndexedBorder &partition : m_partitions) {
            tesselate(partition);
        }
    }

    return m_triangles;
}

void Polygon::tesselate(const IndexedBorder &partition)
{
    if (partition.vertices.size() == 3) {
        m_triangles.emplace_back(partition.vertices[0], partition.vertices[1], partition.vertices[2]);
        return;
    }

    std::vector<Vertex> list(partition.vertices.size());
    std::vector<uint32_t> indexMap(list.size());

    // Populate list with vertices of the partition
    for (int i = 0; i < partition.vertices.size(); i++) {
        list[i] = { m_hull[partition.vertices[i]].p, i };
        indexMap[i] = m_hull[partition.vertices[i]].index;
    }

    // Find the highest vertex
    uint32_t top = 0;
    for (uint32_t i = 1; i < list.size(); i++) {
        if (list[top].p.y() > list[i].p.y()) top = i;
    }

    // Develop links for vertex chains from the top most to the bottom most vertex, sorting in descending order
    std::vector<Vertex> set;
    set.reserve(list.size());
    set.push_back(list[top]);
    int left = top == 0 ? list.size() - 1 : top - 1, prevL = -1, prevR = 0, limit = list.size();
    for (uint32_t i = 1; i < limit; i++) {
        uint32_t idx = (top + i) % list.size();

//        std::cout << left << " " << prevL << " " << prevR << " ---\n";
        if (list[left].p.y() <= list[idx].p.y()) {
            set.push_back(list[left]);
            left = left == 0 ? list.size() - 1 : left - 1;

            if (prevL != -1) set[prevL].next = set.size() - 1;
            prevL = set.size() - 1;

            limit--;
            i--;
        } else {
            set.push_back(list[idx]);

            set[prevR].next = set.size() - 1;
            prevR = set.size() - 1;
        }
    }

    list = set;
    for (uint32_t i = 0; i < list.size(); i++){
//        std::cout << i << " " << list[i].index << " " << list[i].next << " M \n";
    }

    std::deque<Vertex> queue;
    queue.push_back(list[0]);
    queue.push_back(list[1]);

    float reference = 1 - 2 * (float)(list[0].next != 1);
    for(size_t i = 2; i < list.size(); i++){
        if(i == queue.back().next){// Check whether next vertex is part of the same chain
//            printf("Same chain: %d (%d)\n", i, list[i].index);

            while(queue.size() >= 2){

                Vertex v = queue.back();
                queue.pop_back();

//                printf("\n%d %d %d %f %f %f\n", queue.back().index, v.index, list[i].index, reference, cross(queue.back().p, v.p, list[i].p), reference * cross(queue.back().p, v.p, list[i].p));

                // Check for exterior angle - do not proceed if found
//                std::cout << "Angle: " << cross(v.p, queue.back().p, list[i].p) << " " << i << " " << list[i].index << " " << indexMap[list[i].index]
//                          << " | " << v.index << " " << queue.back().index << " " << list[i].index << "\n";
                if(cross(v.p, queue.back().p, list[i].p) * reference > 0){
//                    std::cout << "Exterior Angle\n";
                    queue.push_back(v);
                    break;
                }

//                std::cout << "TA: " << queue.back().index << " " << v.index << " " << list[i].index << "\n";
                m_triangles.emplace_back(Triangle{
                    indexMap[queue.back().index], indexMap[v.index], indexMap[list[i].index]
                });
            }


            queue.push_back(list[i]);
        }else{
//            printf("Swap chain: %d (%d)\n", i, list[i].index);
            // Capture free vertices from the opposite chain within separate triangles
            while(queue.size() >= 2){
                Vertex v = queue.front();
                queue.pop_front();

//                std::cout << "TB: " << list[i].index << " " << queue.front().index << " " << v.index << "\n";
                m_triangles.emplace_back(Triangle{
                    indexMap[list[i].index], indexMap[queue.front().index], indexMap[v.index]
                });
            }

            queue.push_back(list[i]);

            // Update reference angle for interior angle checks
            reference = -reference;
        }
    }
}

void Polygon::invalidate()
{
    m_diagonalized = false;
    m_diagonals.clear();
    m_partitions.clear();
    m_triangles.clear();

    m_eventQueue.clear();
    intervals.clear();
    helper.clear();
}

void Polygon::removeVertex(uint32_t index, bool maintainIntegrity)
{
    if (index < m_hull.size()) {

        uint32_t loop = identifyParentLoop(index);
        if (loopLength(loop) <= 3) return;

        // If integrity should be maintained, verify vertex deletion does not result in self-intersecting edge
        if (maintainIntegrity) {
            if (intersects(m_hull[m_hull[index].prev], m_hull[m_hull[index].next])) {
                std::cout << "Intersection!\n";
                return;
            }
        }

        std::cout << "PRE:\n";
        for (uint32_t i = 0; i < m_hull.size(); i++) {
            std::cout << m_hull[i].prev << " " << m_hull[i].index << " " << m_hull[i].next << " " << static_cast<std::underlying_type<VertexType>::type>(m_hull[i].type) << "\n";
        }

        Vertex &prev = m_hull[m_hull[index].prev];
        Vertex &next = m_hull[m_hull[index].next];

        // Determine whether the neighboring vertices need a type change
        identifyVertexType(m_hull[prev.prev].p, prev, next.p);
        identifyVertexType(prev.p, next, m_hull[next.next].p);

        // Repair the hole created by the removal
        prev.next = m_hull[index].next;
        next.prev = m_hull[index].prev;

        // Ensure loop index still exists within the desired loop
        if (m_loops[loop] == index) m_loops[loop] = m_hull[index].next;

        // Remove the vertex
        m_hull.erase(m_hull.begin() + index);

        // Update all indices affected by removal of a vertex
        for (uint32_t &idx : m_loops) idx -= idx > index;
        for (Vertex &hv : m_hull) {
            hv.prev -= hv.prev > index;
            hv.index -= hv.index > index;
            hv.next -= hv.next > index;
        }

        std::cout << "POST:\n";
        for (uint32_t i = 0; i < m_hull.size(); i++) {

            std::cout << m_hull[i].prev << " " << m_hull[i].index << " " << m_hull[i].next << " " << static_cast<std::underlying_type<VertexType>::type>(m_hull[i].type) << "\n";
        }
        for (uint32_t &idx : m_loops) std::cout << idx << "\n";


        invalidate();
    }
}

void Polygon::insertVertex(uint32_t reference, QVector2D vertex)
{
    if (reference >= m_hull.size()) reference = m_hull.size() - 1;

    std::cout << "PRE:\n";
    for (uint32_t i = 0; i < m_hull.size(); i++) {
        std::cout << m_hull[i].prev << " " << m_hull[i].index << " " << m_hull[i].next << " " << static_cast<std::underlying_type<VertexType>::type>(m_hull[i].type) << "\n";
    }

    int prevIndex = reference;

    // Update all indices affected by addition of a vertex
    for (uint32_t &idx : m_loops) idx += idx > reference;
    for (Vertex &hv : m_hull) {
        hv.prev += hv.prev > reference;
        hv.index += hv.index > reference;
        hv.next += hv.next > reference;
    }

    // Insert the vertex
    std::cout << "| " << m_hull[reference].next << " ";
    int index = (int)reference + 1;
    int nextIndex = m_hull[reference].next;
    m_hull.insert(m_hull.begin() + index, {vertex, index, (int)reference, nextIndex, VertexType::NORMAL});
    std::cout << "P: " << m_hull.size() << " " << reference << " " << prevIndex << " " << nextIndex << "\n";

    // Link neighbor vertices to the new vertex
    Vertex &prev = m_hull[m_hull[index].prev];
    Vertex &next = m_hull[m_hull[index].next];

    prev.next = index;
    next.index = nextIndex;
    next.prev = index;
    m_hull[next.next].prev = nextIndex;

    std::cout << prev.prev << " " << prev.index << " " << prev.next << "--\n";
    std::cout << m_hull[index].prev << " " << m_hull[index].index << " " << m_hull[index].next << "\n";
    std::cout << next.prev << " " << next.index << " " << next.next << "--\n";

    // Determine the new type of the vertex and its immediate neighbors
    identifyVertexType(m_hull[prev.prev].p, prev, m_hull[index].p);
    identifyVertexType(prev.p, m_hull[index], next.p);
    identifyVertexType(m_hull[index].p, next, m_hull[next.next].p);

    m_hull[index].internal = prev.internal;

    std::cout << "POST:\n";
    for (uint32_t i = 0; i < m_hull.size(); i++) {

        std::cout << m_hull[i].prev << " " << m_hull[i].index << " " << m_hull[i].next << " " << static_cast<std::underlying_type<VertexType>::type>(m_hull[i].type) << "\n";
    }
    for (uint32_t &idx : m_loops) std::cout << idx << "\n";


    invalidate();
}

void Polygon::positionVertex(uint32_t index, QVector2D position, bool maintainIntegrity)
{
    if (index < m_hull.size()) {

        Vertex &prev = m_hull[m_hull[index].prev];
        Vertex &next = m_hull[m_hull[index].next];

        QVector2D temp = m_hull[index].p;
        m_hull[index].p = position;

        // If integrity should be maintained, verify vertex position does not result in self-intersecting geometry
        if (maintainIntegrity) {
            if (intersects(prev, m_hull[index]) || intersects(m_hull[index], next)) {
                m_hull[index].p = temp;
                return;
            }
        }

        // Update types of neighboring vertices (those that could have changed)
        identifyVertexType(m_hull[prev.prev].p, prev, m_hull[index].p);
        identifyVertexType(prev.p, m_hull[index], next.p);
        identifyVertexType(m_hull[index].p, next, m_hull[next.next].p);

        // Invalidate results if the movement changes the type of the vertex
        invalidate();
    }
}

uint32_t Polygon::vertexCount()
{
    return m_hull.size();
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
            current = m_hull[current].next;
        } while(start != current);

        idx++;
    }

    return m_loops.size();
}

QVector2D Polygon::getVertex(uint32_t index)
{
    if (index < m_hull.size()) return m_hull[index].p;

    return {0, 0};
}

std::vector<uint32_t> Polygon::getLoop(uint32_t index)
{
    std::vector<uint32_t> loop;

    if (index < m_loops.size()) {
        uint32_t start = m_loops[index], current = start;

        do {
            loop.push_back(current);
            current = m_hull[current].next;
        } while (start != current);
    }

    return loop;
}
//const std::vector<QVector2D> &Polygon::getVertices()
//{
//    return m_
//}

void Polygon::identifyVertexTypes(){
    for(Vertex& v : m_hull){
        if(v.type != VertexType::NORMAL){
            continue;
        }

        const QVector2D &prev = m_hull[v.prev].p;
        const QVector2D &next = m_hull[v.next].p;

        identifyVertexType(prev, v, next);
    }
}

void Polygon::identifyVertexType(const QVector2D &prev, Vertex &vertex, const QVector2D &next)
{
    if((prev.y() < vertex.p.y()) && (vertex.p.y() >= next.y())){// Current greater than connected vertices
        if(angle(vertex.p, next, prev)){
            vertex.type = VertexType::END;
        }else{
            vertex.type = VertexType::MERGE;
        }
    }else if((prev.y() > vertex.p.y()) && (vertex.p.y() <= next.y())){// Current less than connected vertices
        if(angle(vertex.p, next, prev)){
            vertex.type = VertexType::START;
        }else{
            vertex.type = VertexType::SPLIT;
        }
    }else{
        vertex.type = VertexType::NORMAL;
    }
}

void Polygon::handleRegularVertex(const Vertex& current){
    std::cout<<"\nREGULAR:"<<current.index<<"\n\t";
    if(right(current, m_hull[current.prev])){
        std::cout<<"if(right) case:\n\t";

        tryMergeDiagonalInsertion(current, helper[current.next]);

        removeInterval(current.next);
//        const auto &end = m_hull[current.prev].p.y() > m_hull[current.next].p.y() ?
//                          m_hull[current.prev] : m_hull[current.next];
        insertInterval(current, m_hull[current.prev]);

        std::cout<<"Insert"<<current.index<<" into the tree ans set helper("<<current.index<<")="<<current.index<<"\n";
    }else{
        std::cout<<"Else case:\n\t";
        if(!intervals.empty()){

            int index = getLeftNeighborIndex(current);
            std::cout<<"Left neighbor:"<<index<<"\n\t";
            tryMergeDiagonalInsertion(current, helper[index]);

            helper[index] = current.index;
            std::cout<<"Set helper("<<index<<")="<<current.index<<"\n\t";
        }
    }
}

void Polygon::handleStartVertex(const Vertex& current){
    std::cout<<"\nSTART:"<<current.index<<"\n\t";
//    std::cout << "insx " << m_hull[current.prev].p.x() << " " << m_hull[current.next].p.x() << "\n";
//    const auto &end = m_hull[current.prev].p.x() < m_hull[current.next].p.x() ?
//                      m_hull[current.prev] : m_hull[current.next];
    insertInterval(current, m_hull[current.prev]);
//    Tree.emplace_back(current.index, current.p.x);
//    helper[current.index] = current.index;
    std::cout<<"Edge"<<current.index<<" is inserted and helper("<<current.index<<")="<<current.index<<"\n";
}

void Polygon::handleEndVertex(const Vertex& current){
    std::cout<<"\nEND:"<<current.index<<"\n\t";

    tryMergeDiagonalInsertion(current, helper[current.next]);

    removeInterval(current.next);
}

void Polygon::handleSplitVertex(const Vertex& current){
    std::cout<<"\nSPLIT:"<<current.index<<"\n\t";
    if(!intervals.empty()){

        int index = getLeftNeighborIndex(current);

        std::cout<<"Left neighbor:"<<index<<"\n\t";
        std::cout<<"Insert Diagonal between "<<current.index<<" and "<<helper[index]<<"\n\t";
        m_diagonals.emplace_back(current.index, helper[index]);
        helper[index] = current.index;
        std::cout<<"Set helper("<<index<<")="<<current.index<<"\n\t";
    }

//    std::cout << "insxs " << m_hull[current.prev].p.x() << " " << m_hull[current.next].p.x() << "\n";
//    const auto &end = m_hull[current.prev].p.x() > m_hull[current.next].p.x() ?
//                      m_hull[current.prev] : m_hull[current.next];
    insertInterval(current, m_hull[current.prev]);
//    Tree.emplace_back(current.index, current.p.x);
//    helper[current.index] = current.index;

    std::cout<<"Insert"<<current.index<<" into the tree ans set helper("<<current.index<<")="<<current.index<<"\n";

}

void Polygon::handleMergeVertex(const Vertex& current){
    std::cout<<"\nMERGE:"<<current.index<<"\n\t";

    std::cout<<"Prev node:"<<current.prev<<"\n\t";
    tryMergeDiagonalInsertion(current, helper[current.next]);

    removeInterval(current.next);

    if(!intervals.empty()){
        std::cout<<"Left neighbor:";

        int index = getLeftNeighborIndex(current);
        tryMergeDiagonalInsertion(current, helper[index]);

        helper[index] = current.index;
        std::cout<<"Set helper("<<index<<")="<<current.index<<"\n";
    }
}

void Polygon::tryMergeDiagonalInsertion(const Vertex& current, int index){
    if(m_hull[index].type == VertexType::MERGE){
        std::cout<<"Insert Diagonal between "<<current.index<<" and "<<index<<"\n\t";
        m_diagonals.emplace_back(current.index, index);
    }
}

void Polygon::insertInterval(const Vertex& current, const Vertex& end){

    // Identify where the interval should be inserted
    size_t index = 0;
    while(index < intervals.size()){
        if(interpolate(m_hull[intervals[index].first].p, m_hull[intervals[index].second].p, current.p.y()) > current.p.x()){
            break;
        }

        index++;
    }

    // Create interval
    intervals.insert(intervals.begin() + index, {current.index, end.index});
    helper[current.index] = current.index;
}

void Polygon::removeInterval(int index){
    for(size_t i = 0; i < intervals.size(); i++){
        if(intervals[i].first == index){
            std::cout<<"Delete "<<intervals[i].first<<" from the tree\n\t";
            intervals.erase(intervals.begin() + i);
            break;
        }
    }

}

int Polygon::getLeftNeighborIndex(const Vertex& current){
    int left = -1;

    for(size_t i = 0; i < intervals.size(); i++){
        std::cout << "Check: " << intervals[i].first << "~" <<current.index << "\n";
        float rx = interpolate(m_hull[intervals[i].first].p, m_hull[intervals[i].second].p, current.p.y());
        std::cout << "-> " << rx << " " << current.p.x() << "\n";
        if (rx > current.p.x()) break;

        left = intervals[i].first;
        std::cout<<left <<"\n\t";
    }

    return left;
}

float Polygon::interpolate(const QVector2D &start, const QVector2D &end, float dy)
{
    return start.x() + (end.x() - start.x()) * (dy - start.y()) / (end.y() - start.y());
}

void Polygon::partition(std::vector<std::pair<int, int>> boundaries)
{
    // If no partitions are needed, populate contents with the hull itself
    if (boundaries.empty()) {
        m_partitions.push_back(IndexedBorder{std::vector<uint32_t>(m_hull.size())});
        std::iota(m_partitions[0].vertices.begin(), m_partitions[0].vertices.end(), 0);
        return;
    }

    // Record number of usages of each vertex
    int remainder = m_hull.size() + 2 * boundaries.size();
    std::vector<int> count(m_hull.size(), 1);
    std::vector<std::vector<uint32_t>> map(m_hull.size());
    for (const auto &diagonal : boundaries) {
        map[diagonal.first].push_back(diagonal.second);
        map[diagonal.second].push_back(diagonal.first);

        count[diagonal.first]++;
        count[diagonal.second]++;
    }

    std::cout << "MAP:\n";
    for(const std::vector<uint32_t> &verts : map) {
        for(uint32_t v : verts) {
            std::cout << v << " ";
        }
        std::cout << "\n";
    }

    for (const Vertex &vertex : m_eventQueue) {
        std::cout << vertex.index << " ";
    }
    std::cout << "\n";

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

        next = m_hull[current].next;

        QVector2D ref = {m_hull[current].p.x() - 100, m_hull[current].p.y()};
//        std::cout << "XZS: " << partition.vertices[0] << " " << partition.vertices.size() << " " << current << "\n";

        std::cout << "REMAINDER:\n";
        for(int c : count){
            std::cout << c << " ";
        }std::cout << "\nStart: " << current << "\n";

        int prev = -1;
        int limit = 20;
        do {
            partition.vertices.push_back(current);
            count[current]--;
            remainder--;

            // Select the most internal edge to traverse
            float value = internalAngle(m_hull[current].p, ref, m_hull[next].p);
            for (uint32_t cut : map[current]) {
                std::cout << "CHECK: " << prev << " " << current << " " << next << " vs " << cut << " B " <<
                        (prev == cut) << " " << (count[cut] <= 0 && cut != partition.vertices[0]) << " " << !angle(m_hull[current].p, m_hull[cut].p, ref) << "\n";
                // Limit selection to avoid backtracking and retraversal
                if (prev == cut || (count[cut] <= 0 && cut != partition.vertices[0])) continue;

                // Choose the smallest internal angle
                float temp = internalAngle(m_hull[current].p, ref, m_hull[cut].p) ;
                std::cout << " I" << next << " " << angle(ref, m_hull[current].p, m_hull[next].p) << " " << value
                          << " I" << cut << " " << angle(ref, m_hull[current].p, m_hull[cut].p) << " " << temp << "\n";
                if (value < temp) {
                    next = cut;
                    value = temp;
                }
            }

            // If a partition boundary was traversed, remove link to prevent retraversal
            if (next != m_hull[current].next) {
                map[current].erase(std::find(map[current].begin(), map[current].end(), next));
            }

            std::cout << "INS " << current << "->" << next << " [" << partition.vertices[0] << "]\n";

            ref = m_hull[current].p;
            prev = current;
            current = next;
            next = m_hull[current].next;

//            if (m_hull[next].)
        } while (current != partition.vertices[0] && limit-- > 0); // Continue until loop is completed

        std::cout << "P: ";
        for (uint32_t idx : partition.vertices) {
            std::cout << idx << " ";
        }
        std::cout << "\n";

        m_partitions.push_back(partition);
    }
}

bool Polygon::compare(const Vertex& v1, const Vertex& v2){
    if(v1.p.y() == v2.p.y()) {
        return v1.p.x() < v2.p.x();
    }

    return v1.p.y() < v2.p.y();
}

bool Polygon::right(const Vertex& current, const Vertex& prev){
    if(current.p.y() < prev.p.y()){
        return true;
    }
//    else if(current.p.y == prev.p.y){
//        return current.p.x >= prev.p.x;
//    }

    return false;
}

bool Polygon::angle(const QVector2D &pivot, const QVector2D &a, const QVector2D &b)
{
    return cross(pivot, a, b) >= 0;
}

float Polygon::internalAngle(const QVector2D &pivot, const QVector2D &a, const QVector2D &b)
{
    return !angle(pivot, a, b) ? dot(pivot, a, b) : -2 - dot(pivot, a, b);
}

float Polygon::dot(const QVector2D &pivot, const QVector2D &a, const QVector2D &b)
{
    return dot(a - pivot, b - pivot);
}

float Polygon::dot(const QVector2D &a, const QVector2D &b) {
    return (a.x() * b.x() + a.y() * b.y()) / a.length() / b.length();
}

float Polygon::cross(const QVector2D &pivot, const QVector2D &a, const QVector2D &b)
{
    return cross(a - pivot, b - pivot);
}

float Polygon::cross(const QVector2D &v1, const QVector2D &v2)
{
    return v1.x() * v2.y() - v2.x() * v1.y();
}

bool Polygon::intersects(const Vertex &a, const Vertex &b)
{
    for (uint32_t start : m_loops) {
        uint32_t last = start, current = m_hull[start].next;

        do {
            if (!(a.index == last || a.index == current || b.index == last || b.index == current)) {
                if (segmentIntersection(a.p, b.p, m_hull[last].p, m_hull[current].p)) return true;
            }

            last = current;
            current = m_hull[current].next;
        } while (start != last);
    }

    return false;
}

bool Polygon::segmentIntersection(const QVector2D &a, const QVector2D &b, const QVector2D &c, const QVector2D &d)
{
    float a1 = triArea(a, b, d);
    float a2 = triArea(a, b, c);

    if (a1 * a2 < 0) {
        float a3 = triArea(c, d, a);
        float a4 = a3 + a2 - a1;

        if (a3 * a4 < 0) {
            // t = a3 / (a3 - a4)
            // p = a + t * (b - a)
            return true;
        }
    }

    return false;
}

float Polygon::triArea(const QVector2D &a, const QVector2D &b, const QVector2D &c)
{
    return (a.x() - c.x()) * (b.y() - c.y()) - (a.y() - c.y()) * (b.x() - c.x());
}