//
// Created by Cam on 2025-04-07.
//

#include "Simplex.h"

#include <iostream>

#include "ConvexHull.h"

Simplex::Simplex(const Vertex& start)
    : vertices{start, {}, {}, {}}
    , m_size(1)
    , m_colliding()
{
}

void Simplex::add(const Vertex& vertex)
{
    vertices[3] = vertices[2];
    vertices[2] = vertices[1];
    vertices[1] = vertices[0];
    vertices[0] = vertex;

    m_size++;
}

void Simplex::correctWinding()
{
    if (m_size < 4) return;

    // Correct winding order
    glm::vec3 v30 = vertices[0].val - vertices[3].val;
    glm::vec3 v31 = vertices[1].val - vertices[3].val;
    glm::vec3 v32 = vertices[2].val - vertices[3].val;

    if (glm::dot(v30, glm::cross(v31, v32)) < 0.0f) std::swap(vertices[0], vertices[1]);
}

Simplex::Vertex Simplex::operator[](uint32_t idx) const
{
    return vertices[idx];
}

uint32_t Simplex::size() const
{
    return m_size;
}

bool Simplex::evaluate(glm::vec3& axis)
{
    switch (m_size) {
        case 2: evaluateLine(axis); break;
        case 3: evaluateTriangle(axis); break;
        case 4: return evaluateTetrahedron(axis);
    }

    return false;
}

void Simplex::evaluateLine(glm::vec3& axis)
{
    evaluateLine(axis, vertices[1].val - vertices[0].val, -vertices[0].val);
}

void Simplex::evaluateLine(glm::vec3& axis, const glm::vec3& AB, const glm::vec3& AO)
{
    if (glm::dot(AB, AO) > 0) {
        axis = glm::cross(glm::cross(AB, AO), AB);
    } else {
        m_size = 1;
        axis = AO;
    }
}

void Simplex::evaluateTriangle(glm::vec3& axis)
{
    auto AB = vertices[1].val - vertices[0].val, AC = vertices[2].val - vertices[0].val, AO = -vertices[0].val;
    auto ABC = glm::cross(AB, AC);

    if (glm::dot(glm::cross(ABC, AC), AO) > 0) {
        if (glm::dot(AC, AO) > 0) {
            vertices[1] = vertices[2]; m_size = 2;
            axis = glm::cross(glm::cross(AC, AO), AC);
        } else {
            m_size = 2;
            evaluateLine(axis, AB, AO);
        }
    } else {
        if (glm::dot(glm::cross(AB, ABC), AO) > 0) {
            m_size = 2;
            evaluateLine(axis, AB, AO);
        } else {
            if (glm::dot(ABC, AO) > 0) {
                axis = ABC;
            } else {
                auto temp = vertices[1];
                vertices[1] = vertices[2];
                vertices[2] = temp;
                axis = -ABC;
            }
        }
    }
}

bool Simplex::evaluateTetrahedron(glm::vec3& axis)
{
    auto AB = vertices[1].val - vertices[0].val, AC = vertices[2].val - vertices[0].val, AD = vertices[3].val - vertices[0].val, AO = -vertices[0].val;
    auto ABC = glm::cross(AB, AC), ACD = glm::cross(AC, AD), ADB = glm::cross(AD, AB);

    if (glm::dot(ABC, AO) > 0) {
        m_size = 3;
        evaluateTriangle( axis);
    } else if (glm::dot(ACD, AO) > 0) {
        vertices[1] = vertices[2]; vertices[2] = vertices[3]; m_size = 3;
        evaluateTriangle(axis);
    } else if (glm::dot(ADB, AO) > 0) {
        vertices[2] = vertices[1]; vertices[1] = vertices[3]; m_size = 3;
        evaluateTriangle(axis);
    } else {
        return (m_colliding = true);
    }

    return false;
}

bool Simplex::colliding() const
{
    return m_colliding;
}