//
// Created by cjhat on 2025-07-26.
//

#include "Waypoint.h"

#include "core/Functions.h"

Waypoint::Waypoint(const std::vector<double>& values, bool inDg)
    : values(values)
    , inDg(inDg)
    , collides(false)
{

}

bool operator==(const Waypoint& lhs, const Waypoint& rhs)
{
    if (lhs.values.size() != rhs.values.size()) return false;

    if (lhs.inDg != rhs.inDg) {
        if (lhs.inDg) {
            auto wp = rhs.toDg();
            for (uint32_t i = 0; i < lhs.values.size(); i++) {
                if (std::abs(lhs.values[i] - wp.values[i]) > 1e-12) return false;
            }

            return true;
        } else {
            auto wp = lhs.toDg();
            for (uint32_t i = 0; i < wp.values.size(); i++) {
                if (std::abs(wp.values[i] - rhs.values[i]) > 1e-12) return false;
            }

            return true;
        }
    }

    for (uint32_t i = 0; i < lhs.values.size(); i++) {
        if (std::abs(lhs.values[i] - rhs.values[i]) > 1e-12) return false;
    }

    return true;
}

bool operator!=(const Waypoint& lhs, const Waypoint& rhs)
{
    return !(lhs == rhs);
}

bool Waypoint::isValid() const
{
    return !values.empty();
}

std::string Waypoint::toString() const
{
    if (values.empty()) return "[]";

    Waypoint wp = toDg();

    std::string data = "[";
    for (uint32_t i = 0; i < values.size() - 1; i++) data += Functions::toString(wp.values[i], 2) + ", ";
    data += Functions::toString(wp.values.back(), 2) + "]";
    return data;
}

Waypoint Waypoint::toDg() const
{
    if (inDg) return *this;

    auto wp = scalarApplied(TO_DG);
    wp.inDg = true;

    return wp;
}
Waypoint Waypoint::toRad() const
{
    if (!inDg) return *this;

    auto wp = scalarApplied(TO_RAD);
    wp.inDg = false;

    return wp;
}

Waypoint Waypoint::scalarApplied(double scalar) const
{
    Waypoint wp = *this;
    for (double& value : wp.values) value *= scalar;

    return wp;
}

std::ostream& operator<<(std::ostream& stream, const Waypoint& waypoint)
{
    stream << "{ ";
    for (double value : waypoint.values) {
        stream << value << ", ";
    }

    stream << "[" << waypoint.inDg << "] }";
    return stream;
}