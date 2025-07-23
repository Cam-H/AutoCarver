//
// Created by Cam on 2025-04-07.
//

#ifndef AUTOCARVER_SIMPLEX_H
#define AUTOCARVER_SIMPLEX_H

#include "glm.hpp"
#include <array>

class ConvexHull;

class Simplex {
public:

    struct Vertex {
        std::pair<uint32_t, uint32_t> idx;
        glm::vec3 val;

        Vertex() : idx(0, 0), val() {}
        explicit Vertex(const std::tuple<uint32_t, uint32_t, glm::vec3>& value)
            : idx(std::get<0>(value), std::get<1>(value))
            , val(std::get<2>(value)) {}
        Vertex(const std::pair<uint32_t, uint32_t>& idx, const glm::vec3& val) : idx(idx), val(val) {}
    };

    explicit Simplex(const std::tuple<uint32_t, uint32_t, glm::vec3>& start);
    explicit Simplex(const Vertex& start);

    void add(const Vertex& vertex);

    void correctWinding();

    Vertex operator[](uint32_t idx) const;

    [[nodiscard]] uint32_t size() const;

    bool evaluate(glm::vec3& axis);
    void evaluateLine(glm::vec3& axis);
    void evaluateTriangle(glm::vec3& axis);
    bool evaluateTetrahedron(glm::vec3& axis3);

    [[nodiscard]] bool colliding() const;

    void print() const;

private:
    std::array<Vertex, 4> vertices;
    uint32_t m_size;

    bool m_colliding;

    const float TOLERANCE = 1e-6;
};


#endif //AUTOCARVER_SIMPLEX_H
