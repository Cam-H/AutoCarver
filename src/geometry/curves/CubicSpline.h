//
// Created by cjhat on 2025-07-31.
//

#ifndef MATHTEST_CUBICSPLINE_H
#define MATHTEST_CUBICSPLINE_H

#include <vector>
#include <cassert>
#include <stdexcept>

class CubicSpline {
public:

    // Each segment has 4 coefficients: a(s - sInitial)^3 + b(s - sInitial)^2 + c(s - sInitial) + d
    struct Segment {

        Segment(double s, double a, double b, double c, double d)
            : sInitial(s)
            , a(a)
            , b(b)
            , c(c)
            , d(d) {}

        double sInitial;
        double a, b, c, d;
    };

    CubicSpline();

    void fit(const std::vector<double>& s, const std::vector<double>& q);

    [[nodiscard]] double s() const;

    [[nodiscard]] double evaluate(double s) const;
    [[nodiscard]] double evaluateFirstDerivative(double s) const;
    [[nodiscard]] double evaluateSecondDerivative(double s) const;

    [[nodiscard]] double evaluate(uint32_t idx, double s) const;
    [[nodiscard]] double evaluateFirstDerivative(uint32_t idx, double s) const;
    [[nodiscard]] double evaluateSecondDerivative(uint32_t idx, double s) const;

private:

    [[nodiscard]] inline static double evaluate(const Segment& seg, double s)
    {
        double x = s - seg.sInitial;
        return seg.a * x*x*x + seg.b * x*x + seg.c * x + seg.d;
    }
    [[nodiscard]] inline static double evaluateFirstDerivative(const Segment& seg, double s)
    {
        double x = s - seg.sInitial;
        return 3.0 * seg.a * x*x + 2.0 * seg.b * x + seg.c;
    }

    [[nodiscard]] inline static double evaluateSecondDerivative(const Segment& seg, double s)
    {
        double x = s - seg.sInitial;
        return 6.0 * seg.a * x + 2.0 * seg.b;
    }

    [[nodiscard]] const Segment& findSegment(double s) const;

private:
    std::vector<Segment> m_segments;
    double m_s;
};


#endif //MATHTEST_CUBICSPLINE_H
