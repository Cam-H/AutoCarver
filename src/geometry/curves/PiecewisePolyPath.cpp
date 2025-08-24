//
// Created by cjhat on 2025-07-31.
//

#include "PiecewisePolyPath.h"

#include <cmath>
#include <iostream>

PiecewisePolyPath::PiecewisePolyPath(const std::vector<Waypoint>& waypoints)
{
    if (waypoints.size() < 2) return; // No initialization with insufficient waypoints

    uint32_t order = waypoints[0].values.size();
    for (uint32_t i = 1; i < waypoints.size(); i++) {
        assert(order == waypoints[i].values.size());
        assert(waypoints[0].inDg == waypoints[i].inDg);
    }

    // TODO convert dg/rad if needed rather than error
//    for (Waypoint& waypoint : m_waypoints) {
//        if (m_order != waypoint.values.size()) throw std::runtime_error("[TOPPTrajectory] Invalid waypoints. Sizes do not match");
//
//        if (m_inDg != waypoint.inDg) {
//            if (m_inDg) waypoint = waypoint.toDg();
//            else waypoint = waypoint.toRad();
//        }
//    }

    double delta;
    m_breakpoints = std::vector<double>(waypoints.size(), 0);
    for (uint32_t i = 1; i < m_breakpoints.size(); i++) {

        for (uint32_t j = 0; j < order; j++) {
            delta = waypoints[i].values[j] - waypoints[i - 1].values[j];
            m_breakpoints[i] += delta * delta;
        }

        m_breakpoints[i] = m_breakpoints[i - 1] + sqrt(m_breakpoints[i]);
    }


    for (uint32_t i = 0; i < order; i++) {
        std::vector<double> path(waypoints.size());
        for (uint32_t j = 0; j < waypoints.size(); j++) path[j] = waypoints[j].values[i];

        m_splines.emplace_back();
        m_splines.back().fit(m_breakpoints, path);
    }
}


uint32_t PiecewisePolyPath::order() const
{
    return m_splines.size();
}

double PiecewisePolyPath::sEnd() const
{
    return !m_breakpoints.empty() ? m_breakpoints.back() : 0;
}

bool PiecewisePolyPath::empty() const
{
    return m_splines.empty() || m_breakpoints.empty();
}

uint32_t PiecewisePolyPath::segments() const
{
    return m_breakpoints.size();
}

std::vector<double> PiecewisePolyPath::evaluate(double s) const
{
    std::vector<double> position;
    position.reserve(m_splines.size());

    uint32_t idx = regionIdx(s);

    for (const CubicSpline& spline : m_splines) position.emplace_back(spline.evaluate(idx, s));

    return position;
}

std::vector<double> PiecewisePolyPath::evaluateFirstDerivative(double s) const
{
    std::vector<double> velocity;
    velocity.reserve(m_splines.size());

    uint32_t idx = regionIdx(s);

    for (const CubicSpline& spline : m_splines) velocity.emplace_back(spline.evaluateFirstDerivative(idx, s));

    return velocity;
}
std::vector<double> PiecewisePolyPath::evaluateSecondDerivative(double s) const
{
    std::vector<double> acceleration;
    acceleration.reserve(m_splines.size());

    uint32_t idx = regionIdx(s);

    for (const CubicSpline& spline : m_splines) acceleration.emplace_back(spline.evaluateSecondDerivative(idx, s));

    return acceleration;
}

uint32_t PiecewisePolyPath::regionIdx(double s) const
{
    if (m_breakpoints.empty()) throw std::runtime_error("[PiecewisePolyPath] Improper definition");
    else if (s <= m_breakpoints[0]) return 0;
    else if (s >= m_breakpoints.back()) return m_breakpoints.size() - 2;

    for (uint32_t i = 0; i < m_breakpoints.size() - 2; i++) {
        if (s <= m_breakpoints[i + 1] + 1e-12) return i;
    }

    return m_breakpoints.size() - 2;
}