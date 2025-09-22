//
// Created by Cam on 2025-04-07.
//

#include "Simplex.h"

#include <iostream>

// Implementation for 2D simplex
template<> void Simplex<2>::add(const Vertex& vertex)
{
    vertices[2] = vertices[1];
    vertices[1] = vertices[0];
    vertices[0] = vertex;

    m_size++;
}

//template<> void Simplex<2>::correctWinding()
//{
//    if (m_size < 3) return;
//
//    // Correct winding order
//    auto v20 = vertices[0].val - vertices[2].val;
//    auto v21 = vertices[1].val - vertices[2].val;
//
//    if (glm::dot(v30, glm::cross(v31, v32)) < 0) std::swap(vertices[0], vertices[1]);
//}

template<> void Simplex<2>::evaluateLine(vec& axis)
{
    auto AB = vertices[1].val - vertices[0].val, AO = -vertices[0].val;
    if (glm::dot(AB, AO) < 0) throw std::runtime_error("[Simplex2D] EL Less");

    axis = { AB.y, -AB.x };
    axis *= 1 - 2.0 * (glm::dot(axis, AO) < 0); // Point axis towards the origin
}

template<> bool Simplex<2>::evaluateTriangle(vec& axis)
{
    auto AB = vertices[1].val - vertices[0].val, AC = vertices[2].val - vertices[0].val, AO = -vertices[0].val;

    auto perp = vec(AB.y, -AB.x);
    perp *= 1.0 - 2.0 * (glm::dot(perp, AC) > 0); // Ensure vector is directed outwards
    if (glm::dot(perp, AO) > 0) {
        m_size = 2;
        axis = perp;
        return false;
    }

    perp = vec(AC.y, -AC.x);
    perp *= 1.0 - 2.0 * (glm::dot(perp, AB) > 0); // Ensure vector is directed outwards
    if (glm::dot(perp, AO) > 0) {
        vertices[1] = vertices[2];
        m_size = 2;
        axis = perp;
        return false;
    }

    return (m_colliding = true);
}

template<> bool Simplex<2>::evaluate(vec& axis)
{
    switch (m_size) {
        case 2: evaluateLine(axis); break;
        case 3: return evaluateTriangle(axis);
        default: throw std::runtime_error("[Simplex2D] Failed to evaluate. Impossible size reached");
    }

    return false;
}



// Implementation for 3D simplex

// Adds the given vertex to the simplex. Only adds unique vertices, in which case 'true' is returned
template<> void Simplex<3>::add(const Vertex& vertex)
{
    vertices[3] = vertices[2];
    vertices[2] = vertices[1];
    vertices[1] = vertices[0];
    vertices[0] = vertex;

    m_size++;
}

template<> void Simplex<3>::correctWinding()
{
    if (m_size < 4) return;

    // Correct winding order
    auto v30 = vertices[0].val - vertices[3].val;
    auto v31 = vertices[1].val - vertices[3].val;
    auto v32 = vertices[2].val - vertices[3].val;

    if (glm::dot(v30, glm::cross(v31, v32)) < 0) std::swap(vertices[0], vertices[1]);
}

template<> void Simplex<3>::evaluateLine(vec& axis)
{
    auto AB = vertices[1].val - vertices[0].val, AO = -vertices[0].val;
    axis = glm::cross(glm::cross(AB, AO), AB);
}

template<> bool Simplex<3>::evaluateTriangle(vec& axis)
{
    auto AB = vertices[1].val - vertices[0].val, AC = vertices[2].val - vertices[0].val, AO = -vertices[0].val;
    auto ABC = glm::cross(AB, AC);

    auto perp = glm::cross(ABC, AC);
    if (glm::dot(perp, AO) > 0) {
        vertices[1] = vertices[2];
        m_size = 2;
        axis = perp;
        return false;
    }

    perp = glm::cross(AB, ABC);
    if (glm::dot(perp, AO) > 0) {
        m_size = 2;
        axis = perp;
        return false;
    }

    axis = glm::dot(ABC, AO) > 0 ? ABC : -ABC;
    return false;
}

template<> bool Simplex<3>::evaluateTetrahedron(glm::dvec3& axis)
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

template<> bool Simplex<3>::evaluate(vec& axis)
{
    switch (m_size) {
        case 2: evaluateLine(axis); break;
        case 3: evaluateTriangle(axis); break;
        case 4: return evaluateTetrahedron(axis);
        default: throw std::runtime_error("[Simplex3D] Failed to evaluate. Impossible size reached");
    }

    return false;
}