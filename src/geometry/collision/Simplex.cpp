//
// Created by Cam on 2025-04-07.
//

#include "Simplex.h"

#include <iostream>

#include "geometry/primitives/ConvexHull.h"

Simplex::Simplex(const std::tuple<uint32_t, uint32_t, glm::dvec3>& start)
    : Simplex(Vertex(start))
{

}

Simplex::Simplex(const Vertex& start)
    : vertices{ start }
    , m_size(1)
    , m_colliding(false)
{
}

// Adds the given vertex to the simplex. Only adds unique vertices, in which case 'true' is returned
void Simplex::add(const Vertex& vertex)
{
    vertices[3] = vertices[2];
    vertices[2] = vertices[1];
    vertices[1] = vertices[0];
    vertices[0] = vertex;

    m_size++;
}

void Simplex::purgeDuplicates()
{
    for (uint32_t i = 0; i < m_size; i++) {
        for (uint32_t j = i + 1; j < m_size; j++) {
            glm::dvec3 delta = vertices[j].val - vertices[i].val;
            if (glm::dot(delta, delta) < TOLERANCE) {
                std::swap(vertices[j], vertices[m_size]);
                m_size--;
                j--;
            }
        }
    }
}

void Simplex::correctWinding()
{
    if (m_size < 4) return;

    // Correct winding order
    glm::dvec3 v30 = vertices[0].val - vertices[3].val;
    glm::dvec3 v31 = vertices[1].val - vertices[3].val;
    glm::dvec3 v32 = vertices[2].val - vertices[3].val;

    if (glm::dot(v30, glm::cross(v31, v32)) < 0) std::swap(vertices[0], vertices[1]);
}

Simplex::Vertex Simplex::operator[](uint32_t idx) const
{
    return vertices[idx];
}

uint32_t Simplex::size() const
{
    return m_size;
}

bool Simplex::evaluate(glm::dvec3& axis)
{
    glm::dvec3 a = vertices[0].val;
    glm::dvec3 ao = -a;

    switch (m_size) {
        case 2: evaluateLine(axis); break;
        case 3: evaluateTriangle(axis); break;
        case 4: return evaluateTetrahedron(axis);
        default: throw std::runtime_error("[Simplex] Failed to evaluate. Impossible size reached");
    }

    return false;
}

void Simplex::evaluateLine(glm::dvec3& axis)
{
    glm::dvec3 AB = vertices[1].val - vertices[0].val, AO = -vertices[0].val;
    if (glm::dot(AB, AO) < 0) throw std::runtime_error("[Simplex] EL Less");

    axis = glm::cross(glm::cross(AB, AO), AB);
}

void Simplex::evaluateTriangle(glm::dvec3& axis)
{
    auto AB = vertices[1].val - vertices[0].val, AC = vertices[2].val - vertices[0].val, AO = -vertices[0].val;
    auto ABC = glm::cross(AB, AC);

    auto perp = glm::cross(ABC, AC);
    if (glm::dot(perp, AO) > 0) {
        vertices[1] = vertices[2];
        m_size = 2;
        axis = perp;
        return;
    }

    perp = glm::cross(AB, ABC);
    if (glm::dot(perp, AO) > 0) {
        m_size = 2;
        axis = perp;
        return;
    }

    axis = glm::dot(ABC, AO) > 0 ? ABC : -ABC;
}

bool Simplex::evaluateTetrahedron(glm::dvec3& axis)
{
    auto AB = vertices[1].val - vertices[0].val, AC = vertices[2].val - vertices[0].val, AD = vertices[3].val - vertices[0].val, AO = -vertices[0].val;
    auto ABC = glm::cross(AB, AC), ACD = glm::cross(AC, AD), ADB = glm::cross(AD, AB);

    if (glm::dot(ABC, AO) > 0) {
        m_size = 3;
        axis = ABC;
        return false;
    } else if (glm::dot(ACD, AO) > 0) {
        vertices[1] = vertices[2];
        vertices[2] = vertices[3];
        m_size = 3;
        axis = ACD;
        return false;
    } else if (glm::dot(ADB, AO) > 0) {
        vertices[2] = vertices[1];
        vertices[1] = vertices[3];
        m_size = 3;
        axis = ADB;
        return false;
    }

    return (m_colliding = true);
}

bool Simplex::colliding() const
{
    return m_colliding;
}

void Simplex::print() const
{
    std::cout << "[Simplex] size: " << m_size << ", colliding: " << m_colliding << "\n";
    for (uint32_t i = 0; i < m_size; i++)
        std::cout << "V" << i << " [" << vertices[i].idx.first << ", " << vertices[i].idx.second << "]: (" << vertices[i].val.x << ", " << vertices[i].val.y << ", " << vertices[i].val.z << ")\n";
}