//
// Created by Cam on 2025-04-07.
//

#include "Simplex.h"

#include <iostream>

Simplex::Simplex(const glm::vec3& start)
    : vertices{start, {}, {}, {}}
    , m_size(1)
    , m_offset()
    , m_colliding()
{

}


void Simplex::add(const glm::vec3& vertex)
{
    vertices[3] = vertices[2];
    vertices[2] = vertices[1];
    vertices[1] = vertices[0];
    vertices[0] = vertex;

    m_size++;
}

glm::vec3 Simplex::operator[](uint32_t idx) const
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
    evaluateLine(axis, vertices[1] - vertices[0], -vertices[0]);
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
    auto AB = vertices[1] - vertices[0], AC = vertices[2] - vertices[0], AO = -vertices[0];
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
    auto AB = vertices[1] - vertices[0], AC = vertices[2] - vertices[0], AD = vertices[3] - vertices[0], AO = -vertices[0];
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
        evaluateOffset();
        return (m_colliding = true);
    }

    return false;
}

void Simplex::evaluateOffset()
{
    if (m_colliding) return; // Skip duplicate calculation (done by default when simplex collision is found)

    switch (m_size) {
        case 3: m_offset = projectOnTriangle(); break;
        case 4: m_offset = projectOnTetrahedron(); break;
        default:
            std::cout << "EO: " << m_size << "\n";
    }
}

glm::vec3 Simplex::projectOnTriangle()
{
    std::cout << "PoT\n";
    auto AB = vertices[1] - vertices[0], AC = vertices[2] - vertices[0], AO = -vertices[0];
    auto ABC = glm::normalize(glm::cross(AB, AC));

    return ABC * glm::dot(ABC, AO);
}

glm::vec3 Simplex::projectOnTetrahedron()
{
    auto AB = vertices[1] - vertices[0], AC = vertices[2] - vertices[0], AD = vertices[3] - vertices[0], AO = -vertices[0];
    auto ABC = glm::normalize(glm::cross(AB, AC)), ACD = glm::normalize(glm::cross(AC, AD)), ADB = glm::normalize(glm::cross(AD, AB));
    auto BCD = glm::normalize(glm::cross(vertices[2] - vertices[1], vertices[3] - vertices[2]));

    std::array<float, 4> result = { glm::dot(ABC, AO), glm::dot(ACD, AO), glm::dot(ADB, AO), glm::dot(BCD, AO)};
    std::array<float, 4> absResult = { std::abs(result[0]), std::abs(result[1]), std::abs(result[2]), std::abs(result[3]) };

    std::cout << "========================================\n";
    std::cout << ABC.x << " " << ABC.y << " " << ABC.z << " | " << result[0] << "\n";
    std::cout << ACD.x << " " << ACD.y << " " << ACD.z << " | " << result[1] << "\n";
    std::cout << ADB.x << " " << ADB.y << " " << ADB.z << " | " << result[2] << "\n";
    std::cout << BCD.x << " " << BCD.y << " " << BCD.z << " | " << result[3] << "\n";
    std::cout << "========================================\n";

    if (absResult[0] < absResult[1] && absResult[0] < absResult[2])      return ABC * result[0];
    else if (absResult[1] < absResult[0] && absResult[1] < absResult[2]) return ACD * result[1];
    else                                                                 return ADB * result[2];
}

bool Simplex::colliding() const
{
    return m_colliding;
}

glm::vec3 Simplex::overlap() const
{
    if (m_colliding) return m_offset;
    return {};
}

const glm::vec3& Simplex::offset() const
{
    return m_offset;
}