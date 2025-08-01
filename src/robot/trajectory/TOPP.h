//
// Created by cjhat on 2025-07-31.
//

#ifndef MATHTEST_TOPP_H
#define MATHTEST_TOPP_H

#include "geometry/curves/PiecewisePolyPath.h"

class TOPP {
public:

    TOPP(const PiecewisePolyPath& path, uint32_t N);

    void compute(const std::vector<double>& vLims, const std::vector<double>& aLims);

    [[nodiscard]] std::vector<double> sVelocity() const;
    [[nodiscard]] std::vector<double> timestep() const;

    void print() const;

private:

    struct PathVertex {
        double s;
        std::vector<double> q;        // joint position
        std::vector<double> q_dot;    // dq/ds
        std::vector<double> q_ddot;   // d2q/ds2
    };

    void discretize(const PiecewisePolyPath& path);

    void forwardPass(const std::vector<double>& vLims, const std::vector<double>& aLims);
    void reversePass(const std::vector<double>& aLims);

    [[nodiscard]] static double maxPathVelocity(const PathVertex& vertex, const std::vector<double>& vLims);

    [[nodiscard]] static double maxPathAcceleration(const PathVertex& vertex, double sv, const std::vector<double>& aLims);
    [[nodiscard]] static double minPathAcceleration(const PathVertex& vertex, double sv, const std::vector<double>& aLims);

private:

    std::vector<PathVertex> m_path;
    uint32_t m_order;
    double m_ds;

    std::vector<double> m_fwd, m_rev;

    std::vector<double> m_sv;
    std::vector<double> m_t;
};


#endif //MATHTEST_TOPP_H
