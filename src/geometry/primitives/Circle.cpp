//
// Created by Cam on 2025-06-11.
//

#include "Circle.h"

#include <stdexcept>
#include <random>

#include "core/Functions.h"
#include "geometry/collision/Collision2D.h"

Circle::Circle() : center(0, 0), radius(-1.0f) {}

Circle::Circle(const glm::vec2& center, double radius) : center(center), radius(radius) {}

Circle::Circle(const std::tuple<glm::dvec2, double>& circle) : center(std::get<0>(circle)), radius(std::get<1>(circle)){}


Circle Circle::enclose(const std::vector<glm::dvec2>& vertices){
    if (!vertices.empty()) {

        // Shuffle vertices to prevent consecutive consideration of adjacent vertices (massively reduces performance)
        std::vector<glm::dvec2> copy = vertices;
        std::shuffle(copy.begin(), copy.end(), std::mt19937(std::random_device()()));

        return welzl(copy, {}, copy.size());
    }

    return { { 0, 0 }, 0 };
}

bool Circle::isValid() const
{
    return radius >= 0;
}


glm::vec2 Circle::start() const
{
    return center;
}

uint32_t Circle::supportIndex(const glm::dvec2& axis, uint32_t startIndex)
{
    return 0;
}
std::tuple<uint32_t, glm::dvec2> Circle::extreme(const glm::dvec2& axis, uint32_t startIndex) const
{
    return { 0, center - glm::normalize(axis) * radius };
}

// Compute circumcenter and radius
Circle Circle::triangleCircumcircle(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c) {
    double d = 2 * (a.x*(b.y - c.y) + b.x*(c.y - a.y) + c.x*(a.y - b.y));
    if (std::abs(d) < 1e-9) throw std::runtime_error("Degenerate triangle");

    double ad = glm::dot(a, a), bd = glm::dot(b, b), cd = glm::dot(c, c);

    auto center = 1 / d * glm::dvec2{
            ad * (b.y - c.y) + bd * (c.y - a.y) + cd * (a.y - b.y),
            ad * (c.x - b.x) + bd * (a.x - c.x) + cd * (b.x - a.x)
    };

    auto del = a - center;
    return { center, sqrt(glm::dot(del, del)) };

//    glm::vec2 ab = b - a;
//    glm::vec2 ac = c - a;
//    glm::vec2 bc = c - b;
//
//    double la = glm::dot(a, a), lb = glm::dot(b, b), lc = glm::dot(c, c);
//
//    double ux = (-la * bc.y + lb * ac.y - lc * ab.y) / d;
//    double uy = ( la * bc.x - lb * ac.x + lc * ab.x) / d;
//
//    return {
//        glm::vec2(ux, uy),
//        (a.x - ux) * (a.x - ux) + (a.y - uy) * (a.y - uy)
//    };
//    glm::dvec2 ab = b - a;
//    glm::dvec2 ac = c - a;
//
//    double lab = glm::dot(ab, ab);
//    double lac = glm::dot(ac, ac);
//
//    glm::dvec3 cbc = glm::cross(ab, ac);
//
//    glm::dvec3 center = (lac * glm::cross(cbc, ab) + lab * glm::cross(ac, cbc)) / (2.0 * glm::dot(cbc, cbc)) + a;
//    return { center, glm::length(center - a) };
}

Circle Circle::welzl(std::vector<glm::dvec2>& vertices, std::vector<glm::dvec2> set, uint32_t n){

    // Trivial cases
    if (n == 0 || set.size() == 3) {
        switch(set.size()) {
            case 0:
                return { { 0, 0 }, 0 };
            case 1:
                return { { set[0] }, 0 };
            case 2:
                return Circle(Functions::midCircum(set[0], set[1]));
            case 3:
                return triangleCircumcircle(set[0], set[1], set[2]);
        }
    }

    glm::dvec2 vertex = vertices[0];
    std::swap(vertices[0], vertices[n - 1]);

    Circle test = welzl(vertices, set, n - 1);
    if (Collision2D::encloses(test, vertex)) {
        return test;
    }

    set.push_back(vertex);

    return welzl(vertices, set, n - 1);
}