//
// Created by Cam on 2025-04-07.
//

#ifndef AUTOCARVER_SIMPLEX_H
#define AUTOCARVER_SIMPLEX_H

#include <glm/glm.hpp>
#include <array>

class ConvexHull;

class Simplex {
public:

    struct Vertex {
        glm::vec3 val;
        std::pair<uint32_t, uint32_t> idx;
    };

    explicit Simplex(const Vertex& start);

    void add(const Vertex& vertex);

    void correctWinding();

    Vertex operator[](uint32_t idx) const;

    [[nodiscard]] uint32_t size() const;

    bool evaluate(glm::vec3& axis);
    void evaluateLine(glm::vec3& axis);
    void evaluateLine(glm::vec3& axis, const glm::vec3& AB, const glm::vec3& AO);
    void evaluateTriangle(glm::vec3& axis);
    bool evaluateTetrahedron(glm::vec3& axis3);

    [[nodiscard]] bool colliding() const;

private:
    std::array<Vertex, 4> vertices;
    uint32_t m_size;

    bool m_colliding;
};


#endif //AUTOCARVER_SIMPLEX_H
