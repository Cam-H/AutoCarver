//
// Created by Cam on 2025-06-11.
//

#ifndef AUTOCARVER_CIRCLE_H
#define AUTOCARVER_CIRCLE_H

#include <glm/glm.hpp>

class Circle {
public:

    Circle();
    Circle(const glm::vec2& center, float radius);

    bool encloses(const glm::vec2& vertex, float tolerance = 1e-9f) const;

    static Circle triangleCircumcircle(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c);

public:
    glm::vec2 center;
    float radius;
};


#endif //AUTOCARVER_CIRCLE_H
