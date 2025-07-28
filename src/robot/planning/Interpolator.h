//
// Created by cjhat on 2025-07-26.
//

#ifndef AUTOCARVER_INTERPOLATOR_H
#define AUTOCARVER_INTERPOLATOR_H

#include <array>
#include <functional>

class Interpolator {
public:

    // Defines how trajectories should be interpolated based on endpoints (constraints)
    // LINEAR - Only position endpoints, linear interpolation
    // CUBIC - Addition of velocity endpoints
    // QUINTIC - Addition of acceleration endpoints
    enum class SolverType {
        LINEAR = 0, CUBIC, QUINTIC
    };

    Interpolator(double start, double end, SolverType solver = SolverType::LINEAR);
    explicit Interpolator(const std::array<double, 6>& constraints, SolverType solver = SolverType::LINEAR);

    void setSolver(SolverType solver);

    void setPositionEndpoints(double qo, double qf);
    void setVelocityEndpoints(double vo, double vf);
    void setAccelerationEndpoints(double ao, double af);

    [[nodiscard]] double position(double t) const;
    [[nodiscard]] double velocity(double t) const;
    [[nodiscard]] double acceleration(double t) const;

    [[nodiscard]] double delta() const;

    [[nodiscard]] double maxVelocity() const;
    [[nodiscard]] double maxAcceleration() const;

    [[nodiscard]] std::vector<double> pTrajectory(double tStep = 0.05f) const;
    [[nodiscard]] std::vector<double> vTrajectory(double tStep = 0.05f) const;
    [[nodiscard]] std::vector<double> aTrajectory(double tStep = 0.05f) const;

    [[nodiscard]] double initialPosition() const;
    [[nodiscard]] double initialVelocity() const;
    [[nodiscard]] double initialAcceleration() const;

    [[nodiscard]] double finalPosition() const;
    [[nodiscard]] double finalVelocity() const;
    [[nodiscard]] double finalAcceleration() const;

    [[nodiscard]] static std::vector<double> t(double tStep = 0.05);

private:

    void correctConstraints();
    void updateCoefficients();

    void prepareLinearCoefficients();
    void prepareCubicCoefficients();
    void prepareQuinticCoefficients();

    [[nodiscard]] inline double multiply(const std::array<double, 4>& scalars) const;
    [[nodiscard]] inline double multiply(const std::array<double, 6>& scalars) const;

    [[nodiscard]] std::vector<double> trajectory(const std::function<double (double)>& func, double tStep = 0.05) const;

private:

    // Endpoint constraints: Initial position, velocity, acceleration, then final values
    std::array<double, 6> m_constraints;

    // Calculated coefficients for the interpolation function (a0 and a1 are not recorded as they are just q0 and v0)
    std::array<double, 4> m_coeffs;

    SolverType m_solver;

};


#endif //AUTOCARVER_INTERPOLATOR_H
