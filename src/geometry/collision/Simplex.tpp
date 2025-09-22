#include "Simplex.h"

#include <iostream>

template<uint16_t dims>
double Simplex<dims>::TOLERANCE = 1e-12;

template<uint16_t dims>
void Simplex<dims>::purgeDuplicates()
{
    for (uint32_t i = 0; i < m_size; i++) {
        for (uint32_t j = i + 1; j < m_size; j++) {
            vec delta = vertices[j].val - vertices[i].val;
            if (glm::dot(delta, delta) < TOLERANCE) {
                std::swap(vertices[j], vertices[m_size]);
                m_size--;
                j--;
            }
        }
    }
}

template<uint16_t dims>
typename Simplex<dims>::Vertex Simplex<dims>::operator[](uint32_t idx) const
{
    return vertices[idx];
}

template<uint16_t dims>
uint32_t Simplex<dims>::size() const
{
    return m_size;
}

template<uint16_t dims>
bool Simplex<dims>::colliding() const
{
    return m_colliding;
}

template<uint16_t dims>
void Simplex<dims>::print() const
{
    std::cout << "[Simplex] dimensions: " << dims << ", size: " << m_size << ", colliding: " << m_colliding << "\n";
    for (uint32_t i = 0; i < m_size; i++)
        std::cout << "V" << i << " [" << vertices[i].idx.first << ", " << vertices[i].idx.second << "]: (" << vertices[i].val.x << ", " << vertices[i].val.y << ", " << vertices[i].val.z << ")\n";
}