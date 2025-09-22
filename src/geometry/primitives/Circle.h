//
// Created by Cam on 2025-06-11.
//

#ifndef AUTOCARVER_CIRCLE_H
#define AUTOCARVER_CIRCLE_H

#include "glm.hpp"

class Circle {
public:

    Circle();
    Circle(const glm::vec2& center, double radius);
    explicit Circle(const std::tuple<glm::dvec2, double>& circle);

    static Circle enclose(const std::vector<glm::dvec2>& vertices);

    [[nodiscard]] bool isValid() const;

    // Required functions for GJK collision tests
    [[nodiscard]] glm::vec2 start() const;
    static uint32_t supportIndex(const glm::dvec2& axis, uint32_t startIndex = 0);
    [[nodiscard]] std::tuple<uint32_t, glm::dvec2> extreme(const glm::dvec2& axis, uint32_t startIndex = 0) const;

    static Circle triangleCircumcircle(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c);

private:

    static Circle welzl(std::vector<glm::dvec2>& vertices, std::vector<glm::dvec2> set, uint32_t n);


public:
    glm::dvec2 center;
    double radius;
};


#endif //AUTOCARVER_CIRCLE_H
