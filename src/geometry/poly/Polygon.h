//
// Created by cameronh on 02/06/24.
//

#ifndef AUTOCARVER_POLYGON_H
#define AUTOCARVER_POLYGON_H

#include <vector>
#include <glm/glm.hpp>

#include "geometry/Triangle.h"

class Polygon {
public:

    explicit Polygon(const std::vector<glm::vec2> &border);

    std::vector<Triangle> bowyerWatson();
    static std::vector<Triangle> bowyerWatson(const std::vector<glm::vec2>& border);

private:

    // Edge represented as unordered pair (with sorting)
    struct Edge {
        uint32_t u, v;
        Edge(uint32_t a, uint32_t b) : u(std::min(a,b)), v(std::max(a,b)) {}
        bool operator<(const Edge& e) const {
            return std::tie(u,v) < std::tie(e.u,e.v);
        }
    };

protected:

    std::vector<glm::vec2> m_vertices; // Plain list of the vertex coordinates that compose the polygon

};


#endif //AUTOCARVER_POLYGON_H
