//
// Created by cjhat on 2025-07-31.
//

#include "CubicSpline.h"

#include <iostream>

CubicSpline::CubicSpline () : m_s(0) {}

void CubicSpline::fit(const std::vector<double>& s, const std::vector<double>& q)
{
    assert(s.size() == q.size());

    uint32_t n = s.size();
    if (n < 2) throw std::runtime_error("[CubicSpline] At least 2 points required for fit");
    m_s = s[n - 1];

    if (n == 2) { // Handle simplified problem
        m_segments.emplace_back(
                0,
                0,
                0,
                (q[1] - q[0]) / (s[1] - s[0]),
                q[0]
                );
        return;
    }

    // Evaluate step sizes
    uint32_t numSegments = n - 1;
    std::vector<double> h(numSegments);
    for (size_t i = 0; i < numSegments; i++) h[i] = s[i + 1] - s[i];


    // Set up the tridiagonal system for second derivatives
    std::vector<double> beta(n, 0), rhs(n, 0);

    for (uint32_t i = 1; i < numSegments; i++) {
        beta[i] = 2.0 * (h[i - 1] + h[i]);
        rhs[i] = 6.0 * ((q[i + 1] - q[i]) / h[i] - (q[i] - q[i - 1]) / h[i - 1]);
    }

    // Natural spline boundary conditions
    std::vector<double> m(n, 0);  // second derivatives

    // Solve tridiagonal system (Thomas algorithm)
    for (uint32_t i = 2; i < numSegments; i++) {
        double factor = h[i - 1] / beta[i - 1];
        beta[i] -= factor * h[i - 1];
        rhs[i] -= factor * rhs[i - 1];
    }

    m[n - 2] = rhs[n - 2] / beta[n - 2];
    for (int i = n - 3; i >= 1; i--) m[i] = (rhs[i] - h[i] * m[i + 1]) / beta[i];

    // Compute coefficients
    m_segments.clear();
    for (size_t i = 0; i < numSegments; ++i) {
        m_segments.emplace_back(
                s[i],
                (m[i + 1] - m[i]) / (6 * h[i]),
                m[i] / 2,
                (q[i + 1] - q[i]) / h[i] - h[i] * (2 * m[i] + m[i + 1]) / 6,
                q[i]
                );
    }
}

double CubicSpline::s() const
{
    return m_s;
}

double CubicSpline::evaluate(double s) const {
    return evaluate(findSegment(s), s);
}

double CubicSpline::evaluateFirstDerivative(double s) const {
    return evaluateFirstDerivative(findSegment(s), s);
}

double CubicSpline::evaluateSecondDerivative(double s) const {
    return evaluateSecondDerivative(findSegment(s), s);
}

double CubicSpline::evaluate(uint32_t idx, double s) const
{
    if (idx >= m_segments.size()) throw std::runtime_error("[CubicSpline] Index out of bounds");
    return evaluate(m_segments[idx], s);
}
double CubicSpline::evaluateFirstDerivative(uint32_t idx, double s) const
{
    if (idx >= m_segments.size()) throw std::runtime_error("[CubicSpline] Index out of bounds");
    return evaluateFirstDerivative(m_segments[idx], s);
}
double CubicSpline::evaluateSecondDerivative(uint32_t idx, double s) const
{
    if (idx >= m_segments.size()) throw std::runtime_error("[CubicSpline] Index out of bounds");
    return evaluateSecondDerivative(m_segments[idx], s);
}

const CubicSpline::Segment& CubicSpline::findSegment(double s) const {
    if (m_segments.empty()) throw std::runtime_error("[CubicSpline] Not initialized");

    if (s <= m_segments[0].sInitial) return m_segments[0];
    if (s >= m_segments.back().sInitial) return m_segments.back();

    for (uint32_t i = 0; i < m_segments.size() - 1; i++) {
        if (s <= m_segments[i + 1].sInitial + 1e-12) return m_segments[i];
    }

    return m_segments.back();
}