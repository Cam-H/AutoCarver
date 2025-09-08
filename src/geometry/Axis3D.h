//
// Created by cjhat on 2025-07-21.
//

#ifndef AUTOCARVER_AXIS3D_H
#define AUTOCARVER_AXIS3D_H

#include <glm.hpp>

const static glm::dvec3 UP = { 0, 1, 0};

class Axis3D {
public:

    Axis3D();
    explicit Axis3D(const glm::dvec3& axis);
    Axis3D(const glm::dvec3& xAxis, const glm::dvec3& yAxis);
    Axis3D(const glm::dvec3& xAxis, const glm::dvec3& yAxis, const glm::dvec3& zAxis);
    Axis3D(const glm::dmat3& matrix);

    friend bool operator==(const Axis3D& lhs, const Axis3D& rhs);
    friend bool operator!=(const Axis3D& lhs, const Axis3D& rhs);

    friend Axis3D operator*(const Axis3D& axes, const glm::dmat3& rotation);
    friend Axis3D operator*(const Axis3D& axes, const glm::dquat& rotation);

    static Axis3D faceAligned(const glm::dvec3& normal, const glm::dvec3& direction, bool alignHorizontal);


    void rotateY();

    void rotateX(double theta);
    void rotateY(double theta);
    void rotateZ(double theta);

    void rotate(const glm::dvec3& axis, double theta);
    void rotate(const glm::dquat& rotation);

    void flipXY();
    void flipXZ();
    void flipYZ();

    [[nodiscard]] bool isValid() const;
    static bool compare(const Axis3D& lhs, const Axis3D& rhs, double tolerance);

    [[nodiscard]] glm::dmat3 toTransform() const;
    [[nodiscard]] glm::dquat toQuat() const;

    [[nodiscard]] glm::dquat relative(const Axis3D& axes) const;

    [[nodiscard]] glm::dvec3 localize(const glm::dvec3& vertex) const;
    [[nodiscard]] glm::dvec3 delocalize(const glm::dvec3& vertex) const;

    void print() const;

private:

    static inline bool checkAxis(const glm::dvec3& axis, const glm::dvec3& ref, double tolerance)
    {
        double val = glm::dot(axis, ref);
        return 1 - tolerance < val && val < 1 + tolerance;
    }

public:
    glm::dvec3 xAxis;
    glm::dvec3 yAxis;
    glm::dvec3 zAxis;

};


#endif //AUTOCARVER_AXIS3D_H
