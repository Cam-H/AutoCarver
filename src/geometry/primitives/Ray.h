//
// Created by cjhat on 2025-07-18.
//

#ifndef AUTOCARVER_RAY_H
#define AUTOCARVER_RAY_H

#include <glm.hpp>

template<typename Derived, uint16_t dims>
class RayBase {
public:

    using vec = std::conditional_t<dims == 2, glm::dvec2, glm::dvec3>;

    RayBase(const vec& origin, const vec& axis);

    friend vec operator*(const Derived& ray, double t) { return ray.origin + t * ray.axis; }
    friend vec operator*(double t, const Derived& ray) { return ray.origin + t * ray.axis; }

    void reverse();

    [[nodiscard]] bool isValid() const;

    [[nodiscard]] double squareDistance(const vec& vertex) const;

public:
    vec origin;
    vec axis;
};

template<uint16_t dims>
class Ray : public RayBase<Ray<dims>, dims> {
public:
    using RayBase<Ray<dims>, dims>::data_;
};

template<>
class Ray<2> : public RayBase<Ray<2>, 2> {
public:

    Ray();
    Ray(const glm::dvec3& location);
    Ray(const glm::dvec2& origin, const glm::dvec2& axis);

    void print() const;
};

template<>
class Ray<3> : public RayBase<Ray<3>, 3> {
public:

    Ray();
    Ray(const glm::dvec3& origin, const glm::dvec3& axis);

    friend Ray<3> operator*(const glm::dmat4& transform, const Ray& ray);

    static double axialRotation(const glm::dvec3& reference, const glm::dvec3& axis);
    [[nodiscard]] double axialRotation(const glm::dvec3& axis) const;

    void print() const;
};

template class RayBase<Ray<2>, 2>;
template class RayBase<Ray<3>, 3>;

#endif //AUTOCARVER_RAY_H
