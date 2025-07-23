//
// Created by Cam on 2025-06-11.
//

#ifndef AUTOCARVER_CIRCLE_H
#define AUTOCARVER_CIRCLE_H

#include "glm.hpp"

class Circle {
public:

    Circle();
    Circle(const glm::vec2& center, float radius);

    [[nodiscard]] bool isValid() const;

    // Required functions for GJK collision tests
    [[nodiscard]] glm::vec2 start() const;
//    [[nodiscard]] uint32_t supportIndex(const glm::vec2& axis) const;
//    [[nodiscard]] std::tuple<uint32_t, glm::vec2> extreme(const glm::vec2& axis) const;

    bool encloses(const glm::vec2& vertex, float tolerance = 1e-9f) const;

    static Circle triangleCircumcircle(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c);

public:
    glm::vec2 center;
    float radius;
};


#endif //AUTOCARVER_CIRCLE_H
