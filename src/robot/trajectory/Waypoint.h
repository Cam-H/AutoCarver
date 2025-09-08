//
// Created by cjhat on 2025-07-26.
//

#ifndef AUTOCARVER_WAYPOINT_H
#define AUTOCARVER_WAYPOINT_H

#include <vector>
#include <iostream>
#include <cstdint>
#include <cmath>

const static double TO_RAD = M_PI / 180;
const static double TO_DG = 180 / M_PI;

class Waypoint {
public:

    // values - The joint angles that represent the waypoint
    // inDg - Specifies the units of values (true = degrees, false = radians)
    explicit Waypoint(const std::vector<double>& values = {}, bool inDg = false);

    friend bool operator==(const Waypoint& lhs, const Waypoint& rhs);
    friend bool operator!=(const Waypoint& lhs, const Waypoint& rhs);

    [[nodiscard]] bool isValid() const;

    [[nodiscard]] std::string toString() const;

    [[nodiscard]] Waypoint toDg() const;
    [[nodiscard]] Waypoint toRad() const;

    [[nodiscard]] double delta(const Waypoint& wp) const;

    static Waypoint midpoint(const Waypoint& lhs, const Waypoint& rhs);
    inline static bool compare(const Waypoint& lhs, const Waypoint& rhs, double tolerance = 1e-3) { return lhs.delta(rhs) < tolerance; }

private:

    [[nodiscard]] Waypoint scalarApplied(double scalar) const;

public:

    std::vector<double> values;
    bool inDg;

    bool collides;
};

std::ostream& operator<<(std::ostream& stream, const Waypoint& waypoint);

#endif //AUTOCARVER_WAYPOINT_H
