//
// Created by cjhat on 2025-08-25.
//

#ifndef AUTOCARVER_SECTIONOPERATION_H
#define AUTOCARVER_SECTIONOPERATION_H

#include <glm.hpp>

class Profile;
class Polygon;

#include "geometry/primitives/Triangle.h"

// Helper class to decompose refinement steps into a series of cuts based on what is physically accessible during the process
// In cases with insufficient clearance, tries to find a set of relief cuts to enable the cut
// Also finds free paths between cuts
class SectionOperation {
public:

    SectionOperation();

    SectionOperation(const glm::dvec2& start, const glm::dvec2& split, const glm::dvec2& end,
                     double width, double thickness);

    void prepareReliefs(const Profile* profile);
    void adjustPositions();

    struct Set {

        Set(const glm::dvec2& axis, const glm::dvec2& normal)
            : axis(axis), normal(normal), travel(axis) {}

        Set(const glm::dvec2& axis, const glm::dvec2& normal, const std::vector<std::pair<glm::dvec2, double>>& motions)
                : axis(axis), normal(normal), travel(axis), motions(motions) {}

        Set(const glm::dvec2& axis, const glm::dvec2& normal, const glm::dvec2& travel)
            : axis(axis), normal(normal), travel(travel) {}

        Set(const std::vector<glm::dvec3>& approach)
                : axis(), normal(), travel(), approach(approach) {}

        void offset(double dx, double dy);

        void flipNormal();

        [[nodiscard]] glm::dvec3 start() const;
        [[nodiscard]] glm::dvec3 end() const;

        [[nodiscard]] glm::dvec2 initialNormal() const;
        [[nodiscard]] glm::dvec2 finalNormal() const;

        [[nodiscard]] std::vector<glm::dvec3> path() const;

        glm::dvec2 axis; // Along the flat of the blade
        glm::dvec2 normal; // Normal to the flat of the blade
        glm::dvec2 travel; // Direction of travel of the blade

        std::vector<std::pair<glm::dvec2, double>> motions; // First = start position, second = final distance from start

        std::vector<glm::dvec3> approach; // Sequence of motions to transit to beginning of the set
    };

    struct Relief {

        Relief(const glm::dvec2& start,
               const glm::dvec2& internal, const glm::dvec2& external, const glm::dvec2& opposite,
               const glm::dvec2& normal,
               double length, double width, double thickness);

        [[nodiscard]] double step(double t) const;
        [[nodiscard]] double reduction(double t) const;

        [[nodiscard]] double projection(double t) const;

        [[nodiscard]] std::vector<double> depths(double t) const;

        [[nodiscard]] const glm::dvec2& apex();
        [[nodiscard]] glm::dvec2 center() const;

        void print() const;

        glm::dvec2 start; // External triangle vertex for the relief

        glm::dvec2 internal; // Direction of internal edge of cut
        glm::dvec2 external; // Direction of external edge of cut
        glm::dvec2 normal; // Direction of intended cut [Outwards]

        double length; // Distance along internal that needs to be cut
        double extLength; // Distance along external that needs to be cut with reliefs
        double theta; // Angle formed between the cut direction and the external cut surface
        double phi; // Angle formed in the parallelogram

        double sink; // Record offset for first depth (For collisions with opposing edge)

        std::vector<glm::dvec2> edges; // (continuous) edges of the relief, excluding edges shared with the profile
    };

    bool clear(const Polygon& hull);

    [[nodiscard]] inline bool isValid() const { return m_valid; }

    [[nodiscard]] glm::dvec2 splitVertex() const;
    [[nodiscard]] const glm::dvec2& external() const;
    [[nodiscard]] glm::dvec2 normal() const;

    [[nodiscard]] bool reliefLeft() const;
    [[nodiscard]] bool reliefRight() const;
    [[nodiscard]] bool requiresRelief() const;

    [[nodiscard]] Triangle2D bounds(double expansion = -RUN_UP) const;
//
//    [[nodiscard]] Polygon bounds() const;
    [[nodiscard]] Polygon boundsNoLeft() const;
    [[nodiscard]] Polygon boundsNoRight() const;

    [[nodiscard]] inline const std::vector<Relief>& reliefs() const { return m_reliefs; }

//    [[nodiscard]] std::vector<Set> connected(const Profile* profile, const glm::dvec2& start, double theta) const;

    [[nodiscard]] std::vector<Set> left() const;
    [[nodiscard]] std::vector<Set> right() const;

    void print() const;

    static double RUN_UP;
    static uint8_t ITERATIONS;

private:

    [[nodiscard]] double leftOrientation() const;
    [[nodiscard]] double rightOrientation() const;

    [[nodiscard]] glm::dvec2 reliefStart(const SectionOperation::Relief& relief, double offset) const;

    [[nodiscard]] bool checkFloor(const glm::dvec2& initial, const glm::dvec2& normal) const;

    bool prepareRelief(const glm::dvec2& initial,
                       const glm::dvec2& internal, const glm::dvec2& external, const glm::dvec2& opposite,
                       const glm::dvec2& normal,
                       double length,
                       const Profile* profile);

    [[nodiscard]] Set direct(const glm::dvec2& origin, const glm::dvec2& axis, const glm::dvec2& ref, double depth) const;
    [[nodiscard]] std::vector<Set> sequence(const SectionOperation::Relief& relief) const;

private:

    bool m_valid; // Operation is possible given constraints

    glm::dvec2 m_split;
    glm::dvec2 m_left, m_right; // Initial positions to perform operations from
    double width, thickness, floor;

    glm::dvec2 BA, BC, AC; // Edge unit vectors
    double lBA, lBC, lAC; // Edge lengths

    double theta; // Internal angle between BA and BC
    double reduction; // Depth reduction due to thickness (to prevent overrun)

    std::vector<Relief> m_reliefs;

    bool m_cutBA, m_cutBC;
};


#endif //AUTOCARVER_SECTIONOPERATION_H
