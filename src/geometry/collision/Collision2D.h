//
// Created by cjhat on 2025-09-21.
//

#ifndef AUTOCARVER_COLLISION2D_H
#define AUTOCARVER_COLLISION2D_H

#include <glm.hpp>

class Circle;
class Line;
class Polygon;

#include "geometry/primitives/Ray.h"

#include "GJK.h"
#include "Simplex.h"

// An array of static collision detection methods for 2D shapes
class Collision2D {
public:

    static bool test(const Circle& bodyA, const Line& bodyB);
    static bool test(const Polygon& bodyA, const Line& bodyB);

    static bool test(const Circle& bodyA, const Circle& bodyB);

    static bool test(const Polygon& bodyA, const Circle& bodyB);
    static bool test(const Circle& bodyA, const Polygon& bodyB);

    static bool test(const Polygon& bodyA, const Polygon& bodyB);

    static bool encloses(const Circle& body, const glm::dvec2& vertex);

    static std::tuple<bool, double> intersection(const Line& bodyA, const Ray<2>& bodyB);
    static std::tuple<bool, double, double> intersection(const Polygon& bodyA, const Ray<2>& bodyB);

private:

    template<class T1, class T2>
    static bool gjkTest(const T1& bodyA, const T2& bodyB)
    {
        auto simplex = GJK::solve2D(bodyA, bodyB, { 0, 0 });
        return simplex.colliding();
    }

    static bool edgeTest(const Polygon& bodyA, const Polygon& bodyB);

};


#endif //AUTOCARVER_COLLISION2D_H
