//
// Created by Cam on 2025-04-15.
//

#include "Sphere.h"
#include "geometry/Collision.h"
#include "ConvexHull.h"

#include <iostream>
#include <random>

Sphere::Sphere() : center({0, 0, 0}), radius(-1.0f){}

Sphere::Sphere(glm::vec3 center, float radius) : center(center), radius(radius) {}

Sphere Sphere::enclose(const ConvexHull& hull)
{
    return Sphere::enclose(hull.vertices());
}

bool Sphere::isValid() const
{
    return radius > 0;
}

Sphere Sphere::enclose(const VertexArray& vertices)
{
    return Sphere::enclose(vertices.vertices());
}

Sphere Sphere::enclose(const std::vector<Sphere>& spheres){
    std::vector<glm::vec3> vertices;

    // Approximate sphere as 6 cardinal vertices
    for (const Sphere& sphere : spheres) {
        vertices.emplace_back(sphere.center + glm::vec3{sphere.radius, 0, 0});
        vertices.emplace_back(sphere.center + glm::vec3{-sphere.radius, 0, 0});
        vertices.emplace_back(sphere.center + glm::vec3{0, sphere.radius, 0});
        vertices.emplace_back(sphere.center + glm::vec3{0, -sphere.radius, 0});
        vertices.emplace_back(sphere.center + glm::vec3{0, 0, sphere.radius});
        vertices.emplace_back(sphere.center + glm::vec3{0, 0, -sphere.radius});
    }

    // Standard enclosure of a set of vertices
    Sphere bounds = enclose(vertices);

    // Resize approximate bounds if needed to include extremities
    for (const Sphere& sphere : spheres) {
        float delta = bounds.radius - sphere.radius - glm::length(bounds.center - sphere.center);
        if (delta < 0) bounds.radius -= delta;
    }

    return bounds;
}

Sphere Sphere::enclose(const std::vector<glm::vec3>& vertices){
    if (!vertices.empty()) {

//        uint32_t i = 0;
//        for (const auto& vertex : vertices) {
//            std::cout << i++ << " " << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
//        }

        // Shuffle vertices to prevent consecutive consideration of adjacent vertices (massively reduces performance)
        std::vector<glm::vec3> copy = vertices;
        std::shuffle(copy.begin(), copy.end(), std::mt19937(std::random_device()()));

        return welzl(copy, {}, copy.size());
    }

    return { { 0, 0, 0 }, 0 };
}

Sphere Sphere::midpointSphere(const glm::vec3& a, const glm::vec3& b){
    return { { (a + b) / 2.0f }, glm::length(b - a) / 2.0f };
}

Sphere Sphere::triangleCircumsphere(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c){
    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;

    float lab = glm::dot(ab, ab);
    float lac = glm::dot(ac, ac);

    glm::vec3 cbc = glm::cross(ab, ac);

    glm::vec3 center = (lac * glm::cross(cbc, ab) + lab * glm::cross(ac, cbc)) / (2.0f * glm::dot(cbc, cbc)) + a;
    return { center, glm::length(center - a) };
}

Sphere Sphere::tetrahedronCircumsphere(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d){
    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 ad = d - a;

    glm::vec3 ccd = glm::cross(ac, ad);

    float den = glm::dot(ab, ccd);
    den = 0.5f / den;

    glm::vec3 cdb = glm::cross(ad, ab);
    glm::vec3 cbc = glm::cross(ab, ac);

    float lab = glm::dot(ab, ab);
    float lac = glm::dot(ac, ac);
    float lad = glm::dot(ad, ad);

    glm::vec3 center = (lab * ccd + lac * cdb + lad * cbc) * den + a;
    return { center, glm::length(center - a) };
}

Sphere Sphere::welzl(std::vector<glm::vec3>& vertices, std::vector<glm::vec3> set, uint32_t n){

    // Trivial cases
    if (n == 0 || set.size() == 4) {
        switch(set.size()) {
            case 0:
                return { { 0, 0, 0 }, 0 };
            case 1:
                return { { set[0] }, 0 };
            case 2:
                return midpointSphere(set[0], set[1]);
            case 3:
                return triangleCircumsphere(set[0], set[1], set[2]);
            case 4:
                return tetrahedronCircumsphere(set[0], set[1], set[2], set[3]);
        }
    }

    glm::vec3 vertex = vertices[0];
    std::swap(vertices[0], vertices[n - 1]);

    Sphere test = welzl(vertices, set, n - 1);
    if (Collision::encloses(test, vertex)) {
        return test;
    }

    set.push_back(vertex);

    return welzl(vertices, set, n - 1);
}

bool Sphere::raycast(const glm::vec3& origin, const glm::vec3& direction) const {
    float a, b, c;

    return raycast(origin, direction, a, b, c);
}

bool Sphere::raycast(const glm::vec3& origin, const glm::vec3& direction, float& t1, float& t2) const {
    float a, b, c;

    if (raycast(origin, direction, a, b, c)) {
        float den = sqrt(b * b - 4 * a * c);
        t1 = (-b - den) / (2 * a);
        t2 = (-b + den) / (2 * a);

        if (t1 > t2) {//Should never happen todo remove
            std::cout << "\033[31mT ERROR!\033[0m\n";
        }
        return true;
    }

    t1 = t2 = -1;

    return false;
}

bool Sphere::raycast(const glm::vec3& origin, const glm::vec3& direction, float& a, float& b, float& c) const {

    glm::vec3 offset = origin - center;

    a = 1;// glm::length2(direction);
    b = 2 * glm::dot(direction, offset);
    c = glm::dot(offset, offset) - radius * radius;

    return b * b - 4 * a * c > 0 && (b <= 0 || c < 0);
}

glm::vec3 Sphere::extreme(const glm::vec3& axis) const
{
    return center - glm::normalize(axis) * radius;
}