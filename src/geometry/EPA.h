//
// Created by Cam on 2025-04-10.
//

#ifndef AUTOCARVER_EPA_H
#define AUTOCARVER_EPA_H

#include <string>
#include <vector>
#include <glm/glm.hpp>

class ConvexHull;

#include "Triangle.h"
#include "Simplex.h"

class EPA {
public:

    EPA(const ConvexHull& a, const ConvexHull& b, Simplex simplex);

    bool colliding() const;

    glm::vec3 overlap() const;
    glm::vec3 offset() const;

    std::pair<uint32_t, uint32_t> nearest() const;


private:

    struct Facet {
        Triangle triangle;
        glm::vec3 normal;
        float value;

        std::vector<uint32_t> neighbors;
        bool onHull;
    };

    static Simplex::Vertex support(const ConvexHull& a, const ConvexHull& b, const glm::vec3& axis, std::pair<uint32_t, uint32_t> idx);

    glm::vec3 normal(uint32_t a, uint32_t b, uint32_t c);

    void expandSimplex(const ConvexHull& a, const ConvexHull& b, Simplex& simplex);
    void prepareFacets(const Simplex& simplex, std::vector<std::pair<float, uint32_t>>& order);
    void prepareFacets(const std::vector<uint32_t>& horizon, std::vector<std::pair<float, uint32_t>>& order);

    void calculateHorizon(const glm::vec3& apex, int64_t last, uint32_t current, std::vector<uint32_t>& horizon);

    glm::vec3 barycentric(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& p);

    void exportState(const std::string& path);
private:

    std::vector<Facet> facets;
    std::vector<Simplex::Vertex> vertices;

    bool m_colliding;
    glm::vec3 m_offset;

    std::pair<uint32_t, uint32_t> m_nearest;

};


#endif //AUTOCARVER_EPA_H
