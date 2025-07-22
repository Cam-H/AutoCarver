//
// Created by cjhat on 2025-07-21.
//

#ifndef AUTOCARVER_AXIS3D_H
#define AUTOCARVER_AXIS3D_H

#include <glm.hpp>

class Axis3D {
public:

    Axis3D();
    explicit Axis3D(const glm::vec3& axis);
    Axis3D(const glm::vec3& xAxis, const glm::vec3& yAxis);
    Axis3D(const glm::vec3& xAxis, const glm::vec3& yAxis, const glm::vec3& zAxis);

    void print() const;

public:
    glm::vec3 xAxis;
    glm::vec3 yAxis;
    glm::vec3 zAxis;

};


#endif //AUTOCARVER_AXIS3D_H
