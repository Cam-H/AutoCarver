//
// Created by Cam on 2025-06-01.
//

#include "Constraint.h"

#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/string_cast.hpp>

#include "RigidBody.h"
#include "geometry/collision/EPA.h"

double Constraint::BETA = 0.0005;
double Constraint::DEPTH_SLOP = 0.01;
double Constraint::RESTITUTION_SLOP = 0.1;

Constraint::Constraint(const std::shared_ptr<RigidBody>& a, const std::shared_ptr<RigidBody>& b, const EPA& collision)
    : rb1(a)
    , rb2(b)
    , lra(collision.colliderAClosestLocal())
    , lrb(collision.colliderBClosestLocal())
    , system(-glm::normalize(collision.overlap()))
    , depth(collision.distance())
    , m_JN(jacobian(system.zAxis))
    , m_JT1(jacobian(system.xAxis))
    , m_JT2(jacobian(system.yAxis))

    , m_normalImpulse(0)
    , m_tangent1Impulse(0)
    , m_tangent2Impulse(0)
{

    // Calculate the effective mass from linear and rotational contributions
    calculateEffectiveMass(m_JN);
    calculateEffectiveMass(m_JT1);
    calculateEffectiveMass(m_JT2);

    // TODO select based on materials of colliding bodies
    frictionCoefficient = 0.3;
    resitutionCoefficient = 0.0;
}

Jacobian Constraint::jacobian(const glm::dvec3& axis)
{
    return { -axis, glm::cross(-lra, axis), axis, glm::cross(lrb, axis) };
}

void Constraint::calculateEffectiveMass(Jacobian& J)
{
    J.mass = glm::dot(J.VA, J.VA) / rb1->mass() + glm::dot(rb1->inertiaTensor() * J.WA, J.WA);

    if (rb2->getType() == RigidBody::Type::DYNAMIC) { // Non-dynamic bodies should in effect have infinite mass
        J.mass += glm::dot(J.VB, J.VB) / rb2->mass() + glm::dot(J.WB * rb2->inertiaTensor(), J.WB);
    }
}

void Constraint::iterateNormal(double step)
{
    double lambda = calculateImpulse(m_JN, baumgarteContribution(step) + restitutionContribution());

    double sum = std::max(0.0, m_normalImpulse + lambda);

//    std::cout << "IT " << lambda << " -> " << sum << " " << m_normalImpulse << "\n";

//    if (m_normalImpulse != sum) std::cout << sum - m_normalImpulse << " ";
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

void Constraint::iterateFriction(const Jacobian& J, double& impulse)
{
    double lambda = calculateImpulse(J, 0);

    double limit = std::abs(frictionCoefficient * m_normalImpulse);
    double sum = std::clamp(impulse + lambda, -limit, limit);

//    std::cout << "IT " << lambda << " -> " << sum << " " << m_normalImpulse << "\n";

//    if (impulse != sum) std::cout << sum - impulse << " ";
    apply(J, sum - impulse);
    impulse = sum;
}

double Constraint::calculateImpulse(const Jacobian& J, double bias)
{

    double numerator = glm::dot(J.VA, rb1->getLinearVelocity()) + glm::dot(J.WA, rb1->getAngularVelocity())
                     + glm::dot(J.VB, rb2->getLinearVelocity()) + glm::dot(J.WB, rb2->getAngularVelocity());

//    std::cout << bias << " " << numerator << " " << m_effectiveMass << "\n";
//    std::cout << "T: " << glm::dot(m_JVA, rb1->getLinearVelocity()) << " " << glm::dot(m_JWA, rb1->getAngularVelocity())
//        << " " << glm::dot(m_JVB, rb2->getLinearVelocity()) << " " << glm::dot(m_JWB, rb2->getAngularVelocity()) << "\n";

    numerator = -(numerator + bias);

    return numerator / J.mass;
}

double Constraint::penetration() const
{
    return depth;
}

double Constraint::baumgarteContribution(double step) const
{
    return -BETA / step * std::max(depth - DEPTH_SLOP, 0.0);
}
double Constraint::restitutionContribution() const
{
    double speed = glm::dot(system.zAxis, -rb1->getLinearVelocity() + rb2->getLinearVelocity()
                            - glm::cross(rb1->getAngularVelocity(), lra)
                            + glm::cross(rb2->getAngularVelocity(), lrb));

    return resitutionCoefficient * std::max(speed - RESTITUTION_SLOP, 0.0);
}

void Constraint::apply(const Jacobian& J, double impulse)
{

//    std::cout << "Delta: " << depth << " " << impulse << "\n";
//    std::cout << glm::to_string(m_JVA * impulse / rb1->mass()) << "\n";
//    std::cout << glm::to_string(m_JWA) << " " << glm::to_string(rb1->inertiaTensor()) << "\n";
//    std::cout << glm::to_string(rb1->inertiaTensor() * m_JWA * impulse) << "\n";

    // First body is required to be dynamic
    rb1->setLinearVelocity(rb1->getLinearVelocity() + J.VA * impulse / rb1->mass());
    rb1->setAngularVelocity(rb1->getAngularVelocity() + rb1->inertiaTensor() * J.WA * impulse);

    if (rb2->getType() == RigidBody::Type::DYNAMIC) {
        rb2->setLinearVelocity(rb2->getLinearVelocity() + J.VB * impulse / rb2->mass());
        rb2->setAngularVelocity(rb2->getAngularVelocity() + rb2->inertiaTensor() * J.WB * impulse);
    }
}