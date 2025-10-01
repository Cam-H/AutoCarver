//
// Created by cjhat on 2025-09-27.
//

#include "Line.h"

#include "core/Functions.h"

Line::Line(const glm::vec2& a, const glm::dvec2& b)
    : first(a)
    , second(b)
{

}

void Line::translate(const glm::dvec2& translation)
{
    first += translation;
    second += translation;
}

bool Line::isValid() const
{
    glm::dvec2 delta = second - first;
    return glm::dot(delta, delta) > 1e-12;
}


glm::vec2 Line::start() const
{
    return first;
}

uint32_t Line::supportIndex(const glm::dvec2& axis, uint32_t startIndex)
{
    return 0;
}
std::tuple<uint32_t, glm::dvec2> Line::extreme(const glm::dvec2& axis, uint32_t startIndex) const
{
    return { 0, glm::dot(axis, first) > glm::dot(axis, second) ? first : second };
}