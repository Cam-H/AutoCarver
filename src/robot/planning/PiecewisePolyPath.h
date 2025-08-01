//
// Created by cjhat on 2025-07-31.
//

#ifndef MATHTEST_PIECEWISEPOLYPATH_H
#define MATHTEST_PIECEWISEPOLYPATH_H

#include <vector>

#include "Waypoint.h"
#include "CubicSpline.h"

class PiecewisePolyPath {
public:

    explicit PiecewisePolyPath(const std::vector<Waypoint>& waypoints);

    [[nodiscard]] uint32_t order() const;
    [[nodiscard]] double sEnd() const;

    [[nodiscard]] bool empty() const;

    [[nodiscard]] std::vector<double> evaluate(double s) const;
    [[nodiscard]] std::vector<double> evaluateFirstDerivative(double s) const;
    [[nodiscard]] std::vector<double> evaluateSecondDerivative(double s) const;

private:

    [[nodiscard]] uint32_t regionIdx(double s) const;

private:

    std::vector<double> m_breakpoints;
    std::vector<CubicSpline> m_splines;
};


#endif //MATHTEST_PIECEWISEPOLYPATH_H
