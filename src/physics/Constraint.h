//
// Created by Cam on 2025-06-01.
//

#ifndef AUTOCARVER_CONSTRAINT_H
#define AUTOCARVER_CONSTRAINT_H

#include <memory>

#include <glm/glm.hpp>

class RigidBody;
class EPA;

struct Jacobian {
    glm::vec3 VA;
    glm::vec3 WA;
    glm::vec3 VB;
    glm::vec3 WB;

    float mass;
};

class Constraint {
public:

    Constraint(const std::shared_ptr<RigidBody>& a, const std::shared_ptr<RigidBody>& b, const EPA& collision);

    void iterateNormal(float step);
    void iterateFriction();

    float penetration() const;

protected:

    Jacobian jacobian(const glm::vec3& axis);
    void calculateEffectiveMass(Jacobian& J);

    void iterateFriction(const Jacobian& J, float& impulse);


    float calculateImpulse(const Jacobian& J, float bias);

    float baumgarteContribution(float step) const;
    float restitutionContribution() const;

    void apply(const Jacobian& J, float impulse);

protected:

    std::shared_ptr<RigidBody> rb1;
    std::shared_ptr<RigidBody> rb2;

    glm::vec3 lra;
    glm::vec3 lrb;

    glm::vec3 normal;
    float depth;

    glm::vec3 m_tangent1;
    glm::vec3 m_tangent2;

    float frictionCoefficient;
    float resitutionCoefficient;

    float m_normalImpulse;
    float m_tangent1Impulse;
    float m_tangent2Impulse;

private:

    // Jacobian
    Jacobian m_JN, m_JT1, m_JT2;
};


#endif //AUTOCARVER_CONSTRAINT_H
