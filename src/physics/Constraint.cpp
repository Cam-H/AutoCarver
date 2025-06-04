//
// Created by Cam on 2025-06-01.
//

#include "Constraint.h"

#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

#include "geometry/RigidBody.h"
#include "geometry/EPA.h"

const float BETA = 0.01f;
const float DEPTH_SLOP = 0.01f;
const float RESTITUTION_SLOP = 0.1f;

Constraint::Constraint(const std::shared_ptr<RigidBody>& a, const std::shared_ptr<RigidBody>& b, const EPA& collision)
    : rb1(a)
    , rb2(b)
    , lra(collision.colliderAClosestLocal())
    , lrb(collision.colliderBClosestLocal())
    , normal(-glm::normalize(collision.overlap()))
    , depth(collision.distance())
    , m_JN(jacobian(normal))

    , m_normalImpulse(0)
    , m_tangent1Impulse(0)
    , m_tangent2Impulse(0)
{

    glm::vec3 ca = a->mesh()->centroid(), cb = b->mesh()->centroid();
    std::cout << a << " " << ca.x << " " << ca.y << " " << ca.z << " || " << b << " " << cb.x << " " << cb.y << " " << cb.z << "\n";

    glm::vec3 ref = normal.x * normal.x == 1 ? glm::vec3{ 0, 1, 0 } : glm::vec3{ 1, 0, 0 };
    m_tangent1 = glm::normalize(glm::cross(normal, ref));
    m_tangent2 = glm::normalize(glm::cross(normal, m_tangent1));

    m_JT1 = jacobian(m_tangent1);
    m_JT2 = jacobian(m_tangent2);
//    std::cout << "Axes: " << glm::to_string(normal) << "\n" << glm::to_string(m_tangent1) << "\n" << glm::to_string(m_tangent2) << "\n";

    // Stop calculations from factoring in motion of static/kinematic bodies
    if (b->getType() != RigidBody::Type::DYNAMIC) {
        m_JN.VB = m_JN.WB = m_JT1.VB = m_JT1.WB = m_JT2.VB = m_JT2.WB = {};
    }

    // Calculate the effective mass from linear and rotational contributions
    calculateEffectiveMass(m_JN);
    calculateEffectiveMass(m_JT1);
    calculateEffectiveMass(m_JT2);

    // TODO select based on materials of colliding bodies
    frictionCoefficient = 0.1f;
    resitutionCoefficient = 0.0f;
}

Jacobian Constraint::jacobian(const glm::vec3& axis)
{
    return { -axis, glm::cross(-lra, axis), axis, glm::cross(lrb, axis) };
}

void Constraint::calculateEffectiveMass(Jacobian& J)
{
    J.mass = glm::dot(J.VA, J.VA) / rb1->mass() + glm::dot(J.WA * rb1->inertiaTensor(), J.WA)
           + glm::dot(J.VB, J.VB) / rb2->mass() + glm::dot(J.WB * rb2->inertiaTensor(), J.WB);
}

void Constraint::iterateNormal(float step)
{
    float lambda = calculateImpulse(m_JN, baumgarteContribution(step) + restitutionContribution());

    float sum = std::max(0.0f, m_normalImpulse + lambda);

//    std::cout << "IT " << lambda << " -> " << sum << " " << m_normalImpulse << "\n";

    if (m_normalImpulse != sum) std::cout << sum - m_normalImpulse << " ";
    apply(m_JN, sum - m_normalImpulse);
    m_normalImpulse = sum;
}

void Constraint::iterateFriction()
{
    if (frictionCoefficient == 0) return;
    iterateFriction(m_JT1, m_tangent1Impulse);
    iterateFriction(m_JT2, m_tangent2Impulse);

//    std::cout << "\nFriction: " << m_normalImpulse << " " << m_tangent1Impulse << " " << m_tangent2Impulse << "\n";
//    std::cout << glm::to_string(m_JT1.VA * m_tangent1Impulse / rb1->mass())
//            << "\n" << glm::to_string(m_JT2.VA * m_tangent2Impulse / rb2->mass()) << "\n";
}

void Constraint::iterateFriction(const Jacobian& J, float& impulse)
{
    float lambda = calculateImpulse(J, 0);

    float limit = std::abs(frictionCoefficient * m_normalImpulse);
    float sum = std::clamp(impulse + lambda, -limit, limit);

//    std::cout << "IT " << lambda << " -> " << sum << " " << m_normalImpulse << "\n";

//    if (impulse != sum) std::cout << sum - impulse << " ";
    apply(J, sum - impulse);
    impulse = sum;
}

float Constraint::calculateImpulse(const Jacobian& J, float bias)
{

    float numerator = glm::dot(J.VA, rb1->getLinearVelocity()) + glm::dot(J.WA, rb1->getAngularVelocity())
                    + glm::dot(J.VB, rb2->getLinearVelocity()) + glm::dot(J.WB, rb2->getAngularVelocity());

//    std::cout << bias << " " << numerator << " " << m_effectiveMass << "\n";
//    std::cout << "T: " << glm::dot(m_JVA, rb1->getLinearVelocity()) << " " << glm::dot(m_JWA, rb1->getAngularVelocity())
//        << " " << glm::dot(m_JVB, rb2->getLinearVelocity()) << " " << glm::dot(m_JWB, rb2->getAngularVelocity()) << "\n";

    numerator = -(numerator + bias);

    return numerator / J.mass;
}

float Constraint::penetration() const
{
    return depth;
}

float Constraint::baumgarteContribution(float step) const
{
    return -BETA / step * std::max(depth - DEPTH_SLOP, 0.0f);
}
float Constraint::restitutionContribution() const
{
    float speed = glm::dot(normal, -rb1->getLinearVelocity() + rb2->getLinearVelocity()
                            - glm::cross(rb1->getAngularVelocity(), lra)
                            + glm::cross(rb2->getAngularVelocity(), lrb));

    return resitutionCoefficient * std::max(speed - RESTITUTION_SLOP, 0.0f);
}

void Constraint::apply(const Jacobian& J, float impulse)
{

//    std::cout << "Delta: " << depth << " " << impulse << "\n";
//    std::cout << glm::to_string(m_JVA * impulse / rb1->mass()) << "\n";
//    std::cout << glm::to_string(m_JWA) << " " << glm::to_string(rb1->inertiaTensor()) << "\n";
//    std::cout << glm::to_string(rb1->inertiaTensor() * m_JWA * impulse) << "\n";

    rb1->setLinearVelocity(rb1->getLinearVelocity() + J.VA * impulse / rb1->mass());
    rb1->setAngularVelocity(rb1->getAngularVelocity() + rb1->inertiaTensor() * J.WA * impulse);

    if (rb2->getType() == RigidBody::Type::DYNAMIC) {
        rb2->setLinearVelocity(rb2->getLinearVelocity() + J.VB * impulse / rb2->mass());
        rb2->setAngularVelocity(rb2->getAngularVelocity() + rb2->inertiaTensor() * J.WB * impulse);
    }
}