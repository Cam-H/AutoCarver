//
// Created by Cam on 2025-04-07.
//

#ifndef AUTOCARVER_SIMPLEX_H
#define AUTOCARVER_SIMPLEX_H

#include <glm/glm.hpp>
#include <array>

class Simplex {
public:

    explicit Simplex(const glm::vec3& start);
//    Simplex(uint32_t a, uint32_t b);
//    Simplex(uint32_t a, uint32_t b, uint32_t c);

    void add(const glm::vec3& vertex);

    glm::vec3 operator[](uint32_t idx) const;

    uint32_t size() const;

    bool evaluate(glm::vec3& axis);
    void evaluateLine(glm::vec3& axis);
    void evaluateLine(glm::vec3& axis, const glm::vec3& AB, const glm::vec3& AO);
    void evaluateTriangle(glm::vec3& axis);
    bool evaluateTetrahedron(glm::vec3& axis3);

    bool colliding() const;
    glm::vec3 overlap() const;

    void evaluateOffset();
    const glm::vec3& offset() const;

private:
//    const glm::vec3& vertex(uint32_t idx);
    glm::vec3 projectOnTriangle();
    glm::vec3 projectOnTetrahedron();

private:
    std::array<glm::vec3, 4> vertices;
    uint32_t m_size;

    glm::vec3 m_offset;
    bool m_colliding;
};


#endif //AUTOCARVER_SIMPLEX_H
