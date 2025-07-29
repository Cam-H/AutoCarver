
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>

#include <glm.hpp>

namespace Functions {
    inline static std::string toString(double value, int precision)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << value;
        return oss.str();
    }

    // Apply the quadratic formula to calculate the roots of a degree 2 polynomial function
    inline static std::tuple<double, double> quadratic(double A, double B, double C)
    {
        double discriminant = B*B - 4 * A * B;
        if (discriminant < 0) throw std::runtime_error("[Functions] Can not solve quadratic. Discriminant is negative");

        A = 1 / (2 * A);
        discriminant = sqrt(discriminant);
        return { (-B + discriminant) * A, (B + discriminant) * A };
    }


    // Solve the depressed cubic (t^3 + pt + q = 0)
    inline static std::vector<double> cubic(double P, double Q)
    {
        double sub = P*P*P / 27 + Q*Q / 4; // Actual discriminant = -(4p^3 + 27q^2) | Sign is inverted from discriminant

        if (sub < 0) { // Three distinct real roots - Use trigonometric solution
            double A = 2*sqrt(-P/3), B = acos(3*Q*sqrt(-3/P)/(2*P))/3, C = -2*M_PI/3;
            return {
                A * cos(B),
                A * cos(B + C),
                A * cos(B + 2*C)
            };
        } else if (sub > 0) { // 2 Complex conjugate roots, one real - Apply Cardano's formula
            sub = sqrt(sub);
            double u1 = -0.5*Q + sub, u2 = -0.5*Q - sub;
            return { std::cbrt(u1) + std::cbrt(u2) };
        }

        // Discriminant = 0 -> Repeated roots
        if (P == 0) return { 0 }; // Only care about unique roots (Actually triple 0)

        return {
            3 * Q / P, // Simple root
            -3 * Q / (2 * P) // Double root
        };
    }

    // Apply the cubic formula(s) to calculate the (real) roots of a degree 3 polynomial function
    inline static std::vector<double> cubic(double A, double B, double C, double D)
    {
        double A3 = 3*A;

        auto roots = cubic((A3*C - B*B) / A3*A, (2*B*B*B - 9*A*B*C + 27*A*A*D) / (27*A*A*A));

        // Apply transform to the depressed solution to derive actual roots
        double delta = -B / A3;
        for (double& x : roots) x += delta;

        return roots;
    }
}

