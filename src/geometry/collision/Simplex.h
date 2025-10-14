//
// Created by Cam on 2025-04-07.
//

#ifndef AUTOCARVER_SIMPLEX_H
#define AUTOCARVER_SIMPLEX_H

#include <glm.hpp>
#include <array>

class ConvexHull;

template<uint16_t dims>
class Simplex {
public:

    // TODO expand - Assumed 3D, unless dims == 2
    using vec = std::conditional_t<dims == 2, glm::dvec2, glm::dvec3>;

    struct Vertex {
        std::pair<uint32_t, uint32_t> idx;
        vec val;

        Vertex() : idx(0, 0), val() {}
        explicit Vertex(const std::tuple<uint32_t, uint32_t, vec>& value)
            : idx(std::get<0>(value), std::get<1>(value))
            , val(std::get<2>(value)) {}
        Vertex(const std::pair<uint32_t, uint32_t>& idx, const vec& val) : idx(idx), val(val) {}
    };

    explicit Simplex(const std::tuple<uint32_t, uint32_t, vec>& start)
            : Simplex(Vertex(start)) {}

    explicit Simplex(const Vertex& start)
            : vertices{ start }
            , m_size(1)
            , m_colliding(false) {}

    void add(const Vertex& vertex);

    void purgeDuplicates();
    void correctWinding();

    Vertex operator[](uint32_t idx) const;

    [[nodiscard]] uint32_t size() const;

    void evaluateLine(vec& axis);
    bool evaluateTriangle(vec& axis);
    bool evaluateTetrahedron(glm::dvec3& axis);

    bool evaluate(vec& axis);

    [[nodiscard]] bool colliding() const;

    void print() const;

private:
    std::array<Vertex, dims + 1> vertices;
    uint32_t m_size;

    bool m_colliding;

    static double TOLERANCE;
};

#include "Simplex.tpp"

#endif //AUTOCARVER_SIMPLEX_H
