//
// Created by Cam on 2025-04-15.
//

#ifndef AUTOCARVER_SPHERE_H
#define AUTOCARVER_SPHERE_H

#include "glm.hpp"

#include "geometry/VertexArray.h"

class ConvexHull;

class Sphere {
public:

    Sphere();
    Sphere(glm::dvec3 center, double radius);

    [[nodiscard]] bool isValid() const;

    static Sphere enclose(const ConvexHull& hull);
    static Sphere enclose(const VertexArray& vertices);
    static Sphere enclose(const std::vector<Sphere>& spheres);
    static Sphere enclose(const std::vector<glm::dvec3>& vertices);

    static Sphere midpointSphere(const glm::dvec3& a, const glm::dvec3& b);
    static Sphere triangleCircumsphere(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c);
    static Sphere tetrahedronCircumsphere(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, const glm::dvec3& d);

    [[nodiscard]] bool raycast(const glm::dvec3& origin, const glm::dvec3& direction) const;
    bool raycast(const glm::dvec3& origin, const glm::dvec3& direction, double& t1, double& t2) const;

    [[nodiscard]] const glm::dvec3& start() const;
    [[nodiscard]] uint32_t supportIndex(const glm::dvec3& axis, uint32_t startIndex = 0) const;
    [[nodiscard]] std::tuple<uint32_t, glm::dvec3> extreme(const glm::dvec3& axis, uint32_t startIndex = 0) const;

private:
    static Sphere welzl(std::vector<glm::dvec3>& vertices, std::vector<glm::dvec3> set, uint32_t n);

    bool raycast(const glm::dvec3& origin, const glm::dvec3& direction, double& a, double& b, double& c) const;

public:
    glm::dvec3 center;
    double radius;
};


#endif //AUTOCARVER_SPHERE_H
