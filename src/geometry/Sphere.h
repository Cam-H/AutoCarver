//
// Created by Cam on 2025-04-15.
//

#ifndef AUTOCARVER_SPHERE_H
#define AUTOCARVER_SPHERE_H

#include <glm.hpp>

#include "VertexArray.h"

class Sphere {
public:

    Sphere();
    Sphere(glm::vec3 center, float radius);

    static Sphere enclose(const VertexArray& vertices);
    static Sphere enclose(const std::vector<Sphere>& spheres);
    static Sphere enclose(const std::vector<glm::vec3>& vertices);

    static Sphere midpointSphere(const glm::vec3& a, const glm::vec3& b);
    static Sphere triangleCircumsphere(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);
    static Sphere tetrahedronCircumsphere(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d);

    [[nodiscard]] bool encloses(const glm::vec3& vertex, float tolerance = std::numeric_limits<float>::epsilon()) const;

    [[nodiscard]] bool intersects(const Sphere& sphere) const;

    [[nodiscard]] bool raycast(const glm::vec3& origin, const glm::vec3& direction) const;
    bool raycast(const glm::vec3& origin, const glm::vec3& direction, float& t1, float& t2) const;

    [[nodiscard]] uint32_t vertexCount() const;


private:
    static Sphere welzl(std::vector<glm::vec3>& vertices, std::vector<glm::vec3> set, uint32_t n);

    bool raycast(const glm::vec3& origin, const glm::vec3& direction, float& a, float& b, float& c) const;

public:
    glm::vec3 center;
    float radius;
};


#endif //AUTOCARVER_SPHERE_H
