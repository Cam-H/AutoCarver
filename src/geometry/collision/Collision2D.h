//
// Created by cjhat on 2025-09-21.
//

#ifndef AUTOCARVER_COLLISION2D_H
#define AUTOCARVER_COLLISION2D_H

#include <glm.hpp>

class Circle;
class Polygon;

// An array of static collision detection methods for 2D shapes
class Collision2D {
public:

    static bool test(const Circle& bodyA, const glm::dvec2& a, const glm::dvec2& b);
    static bool test(const Polygon& bodyA, const glm::dvec2& a, const glm::dvec2& b);

    static bool test(const Circle& bodyA, const Circle& bodyB);

    static bool test(const Polygon& bodyA, const Circle& bodyB);
    static bool test(const Polygon& bodyA, const Polygon& bodyB);


    static bool encloses(const Circle& body, const glm::dvec2& vertex);


};


#endif //AUTOCARVER_COLLISION2D_H
