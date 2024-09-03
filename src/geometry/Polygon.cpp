//
// Created by cameronh on 02/06/24.
//

#include "Polygon.h"

#include <numeric>

#include <iostream>

Polygon::Polygon(const std::vector<QVector2D> &border)
{
    int index = 0;

    std::vector<QVector2D> invBorder(border.rbegin(), border.rend());
    for(const QVector2D &v : border) {
        std::cout << v.x() << " " << v.y() << "\n";
    }
    std::cout << "br\n";
    for(const QVector2D &v : invBorder) {
        std::cout << v.x() << " " << v.y() << "\n";
    }

    for (const QVector2D &vertex : border) {
        m_hull.push_back({
            vertex,
            index,
            VertexType::NORMAL,
            index - 1,
            ++index,
            {}
        });
        helper.push_back(-1);
    }

    // Correct boundary edge links
    m_hull[0].prev = (int)m_hull.size() - 1;
    m_hull[m_hull.size() - 1].next = 0;
}

std::vector<Polygon::IndexedBorder> Polygon::partition()
{
    // Skip process if results have been previously calculated
    if (!m_partitions.empty()) {
        return m_partitions;
    }

    identifyVertexTypes();

//    int idx = m_hull.size();
//    m_hull.push_back({
//        QVector2D(250, 150),
//        idx,
//        VertexType::SPLIT,
//        idx + 3,
//        idx + 1,
//        {}
//    });
//    m_hull.push_back({
//        QVector2D(150, 250),
//        idx + 1,
//        VertexType::NORMAL,
//        idx,
//        idx + 2,
//        {}
//    });
//    m_hull.push_back({
//        QVector2D(250, 350),
//        idx + 2,
//        VertexType::MERGE,
//        idx + 1,
//        idx + 3,
//        {}
//    });
//    m_hull.push_back({
//        QVector2D(350, 250),
//        idx + 3,
//        VertexType::NORMAL,
//        idx + 2,
//        idx,
//        {}
//    });

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
    std::cout<<"No of diagonals inserted:"<<diagonals.size();
    std::cout<<"\n The diagonals are inserted between:\n";
    for(int i=0;i<diagonals.size();i++){
        std::cout<<diagonals[i].first
            <<"\t"<<diagonals[i].second <<"\n";
    }

//    for(Vertex v : m_hull) {
//        std::cout << v.index << " (" << asInteger(v.type) << ")\n";
//    }

    partition(diagonals);

    return m_partitions;
}

void Polygon::identifyVertexTypes(){
    for(Vertex& v : m_hull){
        if(v.type != VertexType::NORMAL){
            continue;
        }

        const QVector2D &prev = m_hull[v.prev].p;
        const QVector2D &next = m_hull[v.next].p;

        if((prev.y() < v.p.y()) && (v.p.y() >= next.y())){// Current greater than connected vertices
            if(angle(v.p, next, prev)){
                v.type = VertexType::END;
            }else{
                v.type = VertexType::MERGE;
            }
        }else if((prev.y() > v.p.y()) && (v.p.y() <= next.y())){// Current less than connected vertices
            if(angle(v.p, next, prev)){
                v.type = VertexType::START;
            }else{
                v.type = VertexType::SPLIT;
            }
        }
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
        diagonals.emplace_back(current.index, helper[index]);
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
        diagonals.emplace_back(current.index, index);
    }
}

void Polygon::insertInterval(const Vertex& current, const Vertex& end){

    // Identify where the interval should be inserted
    size_t index = 0;
    while(index < intervals.size()){
        if(m_hull[intervals[index].second].p.x() > current.p.x()){
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
        const auto &start = m_hull[intervals[i].first].p;
        const auto &end = m_hull[intervals[i].second].p;
        float rx = start.x() + (end.x() - start.x()) * (current.p.y() - start.y()) / (end.y() - start.y());
        std::cout << "-> " << rx << " " << current.p.x() << "\n";
        if (rx > current.p.x()) break;

        left = intervals[i].first;
        std::cout<<left <<"\n\t";
    }

    return left;
}

void Polygon::partition(std::vector<std::pair<int, int>> boundaries)
{
    m_partitions.clear();

    // Record remaining usages of each vertex
    std::vector<uint32_t> remainder(m_hull.size(), 1);

    //
    std::vector<
//    std::iota(std::begin(remainder), std::end(remainder), 0);

    for (const auto &boundary : boundaries) {
        remainder[boundary.first]++;
        remainder[boundary.second]++;
    }

    // Sort boundaries by number of vertices enclosed
//    for (auto &boundary : boundaries) {
//        if (boundary.first > boundary.second) std::swap(boundary.first, boundary.second);
//    }
//
//    std::sort(boundaries.begin(), boundaries.end(), [](const std::pair<int, int> &a, const std::pair<int, int> &b) {
//        return (a.second - a.first) < (b.second - b.first);
//    });
//
//    for (auto boundary : boundaries) {
//        std::cout << boundary.first << " " << boundary.second << "\n";
//    }
//
//    // Cut partitions from the base along the defined boundaries
//    for (const auto &boundary : boundaries) {
//        auto begin = std::find(remainder.begin(), remainder.end(), boundary.first);
//        auto end = std::find(remainder.begin(), remainder.end(), boundary.second);
//
//        if (begin == end || begin == remainder.end()) {
//            std::cout << "ERROR!\n";
//            continue;
//        }
//
//        std::cout << "S: " << (end - begin) << " " << *begin << " " << *end << "\n";
//        IndexedBorder partition{std::vector<uint32_t>(end - begin + 1)};
//        std::copy(begin, end + 1, partition.vertices.begin());
//
//        remainder.erase(begin + 1, end);
//
//        m_partitions.push_back(partition);
//    }
//
//    // Capture remainder after processing boundaries
//    m_partitions.push_back({remainder});
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

bool Polygon::angle(const QVector2D &a, const QVector2D &b, const QVector2D &c)
{
    return cross(a, b, c) >= 0;
}

float Polygon::cross(const QVector2D &a, const QVector2D &b, const QVector2D &c)
{
    return cross(b - a, c - a);
}

float Polygon::cross(const QVector2D &v1, const QVector2D &v2)
{
    return v1.x() * v2.y() - v2.x() * v1.y();
}