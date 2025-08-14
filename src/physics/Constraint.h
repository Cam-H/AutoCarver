//
// Created by Cam on 2025-06-01.
//

#ifndef AUTOCARVER_CONSTRAINT_H
#define AUTOCARVER_CONSTRAINT_H

#include <memory>

#include <glm.hpp>

#include "geometry/Axis3D.h"

class RigidBody;
class EPA;

struct Jacobian {
    glm::dvec3 VA;
    glm::dvec3 WA;
    glm::dvec3 VB;
    glm::dvec3 WB;

    double mass;
};

class Constraint {
public:

    Constraint(const std::shared_ptr<RigidBody>& a, const std::shared_ptr<RigidBody>& b, const EPA& collision);

    void iterateNormal(double step);
    void iterateFriction();

    [[nodiscard]] double penetration() const;

protected:

    Jacobian jacobian(const glm::dvec3& axis);
    void calculateEffectiveMass(Jacobian& J);

    void iterateFriction(const Jacobian& J, double& impulse);


    double calculateImpulse(const Jacobian& J, double bias);

    [[nodiscard]] double baumgarteContribution(double step) const;
    [[nodiscard]] double restitutionContribution() const;

    void apply(const Jacobian& J, double impulse);

protected:

    std::shared_ptr<RigidBody> rb1;
    std::shared_ptr<RigidBody> rb2;

    glm::dvec3 lra;
    glm::dvec3 lrb;

    Axis3D system;
    double depth;

    double frictionCoefficient;
    double resitutionCoefficient;

    double m_normalImpulse;
    double m_tangent1Impulse;
    double m_tangent2Impulse;

private:

    // Jacobian
    Jacobian m_JN, m_JT1, m_JT2;

public:

    static double BETA;
    static double DEPTH_SLOP;
    static double RESTITUTION_SLOP;
};

#endif //AUTOCARVER_CONSTRAINT_H
