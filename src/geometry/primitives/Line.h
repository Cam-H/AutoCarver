//
// Created by cjhat on 2025-09-27.
//

#ifndef AUTOCARVER_LINE_H
#define AUTOCARVER_LINE_H

#include <glm.hpp>

class Line {
public:

    Line(const glm::vec2& a, const glm::dvec2& b);

    void translate(const glm::dvec2& translation);

    [[nodiscard]] bool isValid() const;

    // Required functions for GJK collision tests
    [[nodiscard]] glm::vec2 start() const;
    static uint32_t supportIndex(const glm::dvec2& axis, uint32_t startIndex = 0);
    [[nodiscard]] std::tuple<uint32_t, glm::dvec2> extreme(const glm::dvec2& axis, uint32_t startIndex = 0) const;

public:
    glm::dvec2 first;
    glm::dvec2 second;
};


#endif //AUTOCARVER_LINE_H
