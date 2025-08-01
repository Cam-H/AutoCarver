//
// Created by cjhat on 2025-07-31.
//

#include "TOPP.h"

#include <cmath>
#include <iostream>

#include "core/Functions.h"

TOPP::TOPP(const PiecewisePolyPath& path, uint32_t N)
    : m_order(path.order())
{
    if (path.order() == 0) throw std::runtime_error("[TOPP] Can not evaluate. Path is empty");

    m_ds = path.sEnd() / (N - 1);
    m_path.reserve(N);
    discretize(path);
}

void TOPP::discretize(const PiecewisePolyPath& path)
{
    double s = 0;
    while (s < path.sEnd() + 1e-12) {
        m_path.push_back({
            s,
            path.evaluate(s),
            path.evaluateFirstDerivative(s),
            path.evaluateSecondDerivative(s)
        });

        s += m_ds;
    }
}

void TOPP::compute(const std::vector<double>& vLims, const std::vector<double>& aLims)
{
    assert(m_order == vLims.size());
    assert(m_order == aLims.size());

    forwardPass(vLims, aLims);
    reversePass(aLims);

    uint32_t N = m_path.size();
    m_sv = std::vector<double>(N);
    m_t = std::vector<double>(N);

    // Merge results from forward and reverse pass to obtain the maximum feasible velocity profile
    for (size_t i = 0; i < N; ++i) m_sv[i] = std::min(m_fwd[i], m_rev[i]);

    // Integrate time
    m_t[0] = 0.0;
    for (int i = 1; i < N; ++i) {
        m_t[i] = m_t[i - 1] + (2 * m_ds / (m_sv[i - 1] + m_sv[i] + 1e-6));
    }
}

void TOPP::forwardPass(const std::vector<double>& vLims, const std::vector<double>& aLims)
{
    m_fwd = std::vector<double>(m_path.size());
    m_fwd[0] = 0;

    for (uint32_t i = 0; i < m_fwd.size() - 1; i++) {
        double svMax = maxPathVelocity(m_path[i + 1], vLims);
        double saMax = maxPathAcceleration(m_path[i], m_fwd[i], aLims);

        // Take the lowest between maximum velocity and the prev velocity + integrated acceleration
        m_fwd[i + 1] = std::min(svMax, sqrt(m_fwd[i]*m_fwd[i] + 2 * saMax * m_ds));
    }
}

void TOPP::reversePass(const std::vector<double>& aLims)
{
    m_rev = std::vector<double>(m_path.size());
    m_rev.back() = 0;

    for (int i = m_rev.size() - 2; i >= 0; i--) {
        double saMin = minPathAcceleration(m_path[i], m_rev[i + 1], aLims);

        m_rev[i] = std::min(m_fwd[i], sqrt(std::max(0.0, m_rev[i + 1]*m_rev[i + 1] + 2 * saMin * m_ds)));
    }
}

double TOPP::maxPathVelocity(const PathVertex& vertex, const std::vector<double>& vLims)
{
    double vel = std::numeric_limits<double>::max();
    for (uint32_t i = 0; i <  vLims.size(); i++) {
        if (std::abs(vertex.q_dot[i]) > 1e-8) {
            vel = std::min(vel, vLims[i] / std::abs(vertex.q_dot[i]));
        }
    }

    return vel;
}

double TOPP::maxPathAcceleration(const PathVertex& vertex, double sv, const std::vector<double>& aLims)
{
    double accel = std::numeric_limits<double>::max();
    for (uint32_t i = 0; i < aLims.size(); i++) {
        if (std::abs(vertex.q_dot[i]) > 1e-8) {
            accel = std::min(accel, (aLims[i] - vertex.q_ddot[i] * sv*sv) / vertex.q_dot[i]);
        }
    }

    return accel;
}

double TOPP::minPathAcceleration(const PathVertex& vertex, double sv, const std::vector<double>& aLims)
{
    double accel = std::numeric_limits<double>::lowest();
    for (uint32_t i = 0; i < aLims.size(); i++) {
        if (std::abs(vertex.q_dot[i]) > 1e-8) {
            accel = std::max(accel, (-aLims[i] - vertex.q_ddot[i] * sv*sv) / vertex.q_dot[i]);
        }
    }

    return accel;
}

std::vector<double> TOPP::sVelocity() const
{
    return m_sv;
}
std::vector<double> TOPP::timestep() const
{
    return m_t;
}

void TOPP::print() const
{
    std::cout << "t\ts\tsv\tq\n";
    for (int i = 0; i < m_path.size(); i++) {
        std::cout << Functions::toString(m_t[i], 2) << "\t"
                  << Functions::toString(m_path[i].s, 2) << "\t"
                  << Functions::toString(m_sv[i], 2) << "\t";

        for (uint32_t j = 0; j < m_path[i].q.size(); j++)
            std::cout << Functions::toString(m_path[i].q[j], 2) << "\t";

        std::cout << "\n";
    }
}
