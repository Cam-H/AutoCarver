//
// Created by cjhat on 2025-07-26.
//

#include "Interpolator.h"

#include "core/Functions.h"

#include <iostream>

Interpolator::Interpolator(double start, double end, SolverType solver)
        : m_constraints({ start, 0, 0, end, 0, 0 })
        , m_coeffs({ 0, 0, 0, 0 })
        , m_solver(solver)
{

    updateCoefficients();
}

Interpolator::Interpolator(const std::array<double, 6>& constraints, SolverType solver)
        : m_constraints(constraints)
        , m_coeffs({ 0, 0, 0, 0 })
        , m_solver(solver)
{
    updateCoefficients();
}

void Interpolator::setSolver(SolverType solver)
{
    m_solver = solver;
    updateCoefficients();
}

void Interpolator::setPositionEndpoints(double qo, double qf)
{
    m_constraints[0] = qo;
    m_constraints[3] = qf;
    updateCoefficients();
}
void Interpolator::setVelocityEndpoints(double vo, double vf)
{
    if (m_solver == SolverType::LINEAR) throw std::runtime_error("[Interpolator] Velocity is not controllable with linear solvers");
    m_constraints[1] = vo;
    m_constraints[4] = vf;
    updateCoefficients();

    print();
}
void Interpolator::setAccelerationEndpoints(double ao, double af)
{
    if (m_solver != SolverType::QUINTIC) throw std::runtime_error("[Interpolator] Acceleration is only controllable with quintic solvers");
    m_constraints[2] = ao;
    m_constraints[5] = af;
    updateCoefficients();
}

void Interpolator::setInitialPosition(double qo)
{
    m_constraints[0] = qo;
    updateCoefficients();
}
void Interpolator::setInitialVelocity(double vo)
{
    if (m_solver == SolverType::LINEAR) throw std::runtime_error("[Interpolator] Velocity is not controllable with linear solvers");
    m_constraints[1] = vo;
    updateCoefficients();
}
void Interpolator::setInitialAcceleration(double ao)
{
    if (m_solver != SolverType::QUINTIC) throw std::runtime_error("[Interpolator] Acceleration is only controllable with quintic solvers");
    m_constraints[2] = ao;
    updateCoefficients();
}

void Interpolator::setFinalPosition(double qf)
{
    m_constraints[3] = qf;
    updateCoefficients();
}
void Interpolator::setFinalVelocity(double vf)
{
    if (m_solver == SolverType::LINEAR) throw std::runtime_error("[Interpolator] Velocity is not controllable with linear solvers");
    m_constraints[4] = vf;
    updateCoefficients();
}
void Interpolator::setFinalAcceleration(double af)
{
    if (m_solver != SolverType::QUINTIC) throw std::runtime_error("[Interpolator] Acceleration is only controllable with quintic solvers");
    m_constraints[5] = af;
    updateCoefficients();
}

// Correct constraints (prioritizing position -> velocity -> acceleration)
void Interpolator::correctConstraints()
{
    switch (m_solver) {
        case SolverType::LINEAR: // Assign velocity constraints based on linear position change
            m_constraints[1] = m_constraints[4] = -m_constraints[0] + m_constraints[3];
            m_constraints[2] = m_constraints[5] = 0;
            break;
        case SolverType::CUBIC: // Assign acceleration constraints based on linear velocity change
            m_constraints[2] = m_constraints[5] = -m_constraints[1] + m_constraints[4];
            break;
        case SolverType::QUINTIC:
            break;
    }
}

void Interpolator::updateCoefficients()
{
    correctConstraints();

    switch (m_solver) {
        case SolverType::LINEAR:
            prepareLinearCoefficients();
            break;
        case SolverType::CUBIC:
            prepareCubicCoefficients();
            break;
        case SolverType::QUINTIC:
            prepareQuinticCoefficients();
            break;
    }
}

// Calculate interpolation coefficients for a linear function
// The function is assumed to range from to=0 to tf=1 so that the matrix may be precomputed
void Interpolator::prepareLinearCoefficients()
{
    m_coeffs[0] = m_constraints[0];
    m_coeffs[1] = m_constraints[1];
}

// Calculate interpolation coefficients for a cubic polynomial
// The function is assumed to range from to=0 to tf=1 so that the matrix may be precomputed
void Interpolator::prepareCubicCoefficients()
{
    m_coeffs[0] = m_constraints[0];
    m_coeffs[1] = m_constraints[1];
    m_coeffs[2] = multiply(std::array<double, 4>{ -3, -2,  3, -1 });
    m_coeffs[3] = multiply(std::array<double, 4>{  2,  1, -2,  1 });
}

// Calculate interpolation coefficients for a quintic polynomial
// The function is assumed to range from to=0 to tf=1 so that the matrix may be precomputed
// a0 and a1 are equivalent to q0 and v0, respectively, so they are not stored again to save memory
void Interpolator::prepareQuinticCoefficients()
{
    m_coeffs[0] = 0.5 * m_constraints[2];
    m_coeffs[1] = multiply(std::array<double, 6>{ -10, -6, -1.5,  10, -4, 0.5 });
    m_coeffs[2] = multiply(std::array<double, 6>{  15,  8,  1.5, -15,  7,  -1 });
    m_coeffs[3] = multiply(std::array<double, 6>{  -6, -3, -0.5,   6, -3, 0.5 });
}

double Interpolator::multiply(const std::array<double, 4>& scalars) const
{
    return scalars[0] * m_constraints[0]
         + scalars[1] * m_constraints[1]
         + scalars[2] * m_constraints[3]
         + scalars[3] * m_constraints[4];
}

double Interpolator::multiply(const std::array<double, 6>& scalars) const
{
    return scalars[0] * m_constraints[0]
         + scalars[1] * m_constraints[1]
         + scalars[2] * m_constraints[2]
         + scalars[3] * m_constraints[3]
         + scalars[4] * m_constraints[4]
         + scalars[5] * m_constraints[5];
}

double Interpolator::position(double t) const
{
    switch (m_solver) {
        case SolverType::LINEAR:
            return m_coeffs[0] + t * m_coeffs[1];
        case SolverType::CUBIC:
            return m_coeffs[0] + t * m_coeffs[1] + t*t * m_coeffs[2] + t*t*t * m_coeffs[3];
        case SolverType::QUINTIC:
            return m_constraints[0] + t * m_constraints[1] + t*t * m_coeffs[0] + t*t*t * m_coeffs[1]
                   + t*t*t*t * m_coeffs[2] + t*t*t*t*t * m_coeffs[3];
    }

    return 0;
}

double Interpolator::velocity(double t) const
{
    switch (m_solver) {
        case SolverType::LINEAR:
            return m_coeffs[1];
        case SolverType::CUBIC:
            return m_coeffs[1] + 2*t * m_coeffs[2] + 3*t*t * m_coeffs[3];
        case SolverType::QUINTIC:
            return m_constraints[1] + 2*t * m_coeffs[0] + 3*t*t * m_coeffs[1] + 4*t*t*t * m_coeffs[2]
                   + 5*t*t*t*t * m_coeffs[3];
    }

    return 0;
}
double Interpolator::acceleration(double t) const
{
    switch (m_solver) {
        case SolverType::LINEAR:
            return 0;
        case SolverType::CUBIC:
            return 2 * m_coeffs[2] + 6*t * m_coeffs[3];
        case SolverType::QUINTIC:
            return 2 * m_coeffs[0] + 6*t * m_coeffs[1] + 12*t*t * m_coeffs[2] + 20*t*t*t * m_coeffs[3];
    }

    return 0;
}

double Interpolator::delta() const
{
    return m_constraints[3] - m_constraints[0];
}

double Interpolator::maxVelocity() const
{
    switch (m_solver) {
        case SolverType::LINEAR: return m_coeffs[1];
        case SolverType::CUBIC:  return velocity(-m_coeffs[2] / (3 * m_coeffs[3]));
        case SolverType::QUINTIC:
        { // Find local minima/maxima through the cubic roots of the quintic acceleration function. Select largest
            double max = 0;
            auto roots = Functions::cubic(20 * m_coeffs[3], 12 * m_coeffs[2], 6 * m_coeffs[1], 2 * m_coeffs[0]);
            for (double root : roots) {
                double vel = std::abs(velocity(std::clamp(root, 0.0, 1.0)));
                if (vel > max) max = vel;
            }

            return max;
        }
    }

    return 0;
}
double Interpolator::maxAcceleration() const
{
    switch (m_solver) {
        case SolverType::LINEAR: return 0;
        case SolverType::CUBIC:  return 6 * m_coeffs[3];
        case SolverType::QUINTIC:
        { // Find local minima/maxima through the quadratic roots of the quintic jerk function. Select largest
            auto [x1, x2] = Functions::quadratic(60 * m_coeffs[3], 24 * m_coeffs[2], 6 * m_coeffs[1]);
            x1 = acceleration(std::clamp(x1, 0.0, 1.0));
            x2 = acceleration(std::clamp(x2, 0.0, 1.0));
            return std::max(std::abs(x1), std::abs(x2));
        }
    }

    return 0;
}

std::vector<double> Interpolator::pTrajectory(double tStep) const
{
    return trajectory(std::bind(&Interpolator::position, this, std::placeholders::_1), tStep);
}
std::vector<double> Interpolator::vTrajectory(double tStep) const
{
    return trajectory(std::bind(&Interpolator::velocity, this, std::placeholders::_1), tStep);
}
std::vector<double> Interpolator::aTrajectory(double tStep) const
{
    return trajectory(std::bind(&Interpolator::acceleration, this, std::placeholders::_1), tStep);
}

std::vector<double> Interpolator::trajectory(const std::function<double (double)>& func, double tStep) const
{
    std::vector<double> values;

    int count = (int)(1 / tStep) + 1;
    values.reserve(count);

    double t = 0;
    for (uint32_t i = 0; i < count; i++) {
        values.emplace_back(func(t));
        t += tStep;
    }

    return values;
}

double Interpolator::initialPosition() const
{
    return m_constraints[0];
}
double Interpolator::initialVelocity() const
{
    return m_constraints[1];
}
double Interpolator::initialAcceleration() const
{
    return m_constraints[2];
}

double Interpolator::finalPosition() const
{
    return m_constraints[3];
}
double Interpolator::finalVelocity() const
{
    return m_constraints[4];
}
double Interpolator::finalAcceleration() const
{
    return m_constraints[5];
}

std::vector<double> Interpolator::t(double tStep)
{
    std::vector<double> values;
    int count = (int)(1 / tStep) + 1;
    values.reserve(count);

    double val = 0;
    for (uint32_t i = 0; i < count; i++) {
        values.emplace_back(val);
        val += tStep;
    }

    return values;
}

std::string Interpolator::solverType() const
{
    switch (m_solver) {
        case SolverType::LINEAR:  return "Linear";
        case SolverType::CUBIC:   return "Cubic";
        case SolverType::QUINTIC: return "Quintic";
    }

    return "";
}

void Interpolator::print() const
{
    std::cout << "Interpolator [" << solverType() << "]\nConstraints:\nposition: ["
        << Functions::toString(m_constraints[0], 2) << ", " << Functions::toString(m_constraints[3], 2) << "]\nvelocity: ["
        << Functions::toString(m_constraints[1], 2) << ", " << Functions::toString(m_constraints[4], 2) << "]\n";

    if (m_solver == SolverType::QUINTIC)
        std::cout <<"acceleration: [" << Functions::toString(m_constraints[2], 2) << ", " << Functions::toString(m_constraints[5], 2) << "]\n";

    std::cout << "Coefficients: [";

    switch (m_solver) {
        case SolverType::LINEAR:
            std::cout << m_coeffs[0] << ", " << m_coeffs[1] << "]\n";
            break;
        case SolverType::QUINTIC:
            std::cout << m_constraints[0] << ", " << m_constraints[1] << ", ";
        case SolverType::CUBIC:
            std::cout << m_coeffs[0] << ", " << m_coeffs[1] << " " << m_coeffs[2] << " " << m_coeffs[3] << "]\n";
            break;

    }
}