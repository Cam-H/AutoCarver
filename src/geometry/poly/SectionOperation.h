//
// Created by cjhat on 2025-08-25.
//

#ifndef AUTOCARVER_SECTIONOPERATION_H
#define AUTOCARVER_SECTIONOPERATION_H

#include <glm.hpp>

class Profile;

// Helper class to decompose refinement steps into a series of cuts based on what is physically accessible during the process
// In cases with insufficient clearance, tries to find a set of relief cuts to enable the cut
class SectionOperation {
public:

    SectionOperation();

    SectionOperation(const glm::dvec2& start, const glm::dvec2& split, const glm::dvec2& end,
                     double width, double thickness);

    void prepareReliefs(const Profile* profile);


    struct Set {

        Set(const glm::dvec2& axis, const glm::dvec2& normal)
            : axis(axis), normal(normal), travel(axis) {}

        Set(const glm::dvec2& axis, const glm::dvec2& normal, const std::vector<std::pair<glm::dvec2, double>>& motions)
                : axis(axis), normal(normal), travel(axis), motions(motions) {}

        Set(const glm::dvec2& axis, const glm::dvec2& normal, const glm::dvec2& travel)
            : axis(axis), normal(normal), travel(travel) {}

        std::vector<std::pair<glm::dvec2, double>> motions; // First = start position, second = final distance from start
        glm::dvec2 axis;
        glm::dvec2 normal;
        glm::dvec2 travel;
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

        glm::dvec2 start; // Start position for relief

        glm::dvec2 internal; // Direction of internal edge of cut
        glm::dvec2 external; // Direction of external edge of cut
        glm::dvec2 normal; // Direction of intended cut [Outwards]

        double length; // Distance along internal that needs to be cut
        double extLength; // Distance along external that needs to be cut with reliefs
        double theta; // Angle formed between the cut direction and the external cut surface
        double phi; // Angle formed in the parallelogram

        double sink; // Record offset for first depth (For collisions with opposing edge)

        std::vector<glm::dvec2> edges; // (continuous) edges of the relief, excluding edges shared with the profile

        bool valid;
    };

    [[nodiscard]] glm::dvec2 startVertex(double offset = 0) const;
    [[nodiscard]] glm::dvec2 endVertex(double offset = 0) const;

    [[nodiscard]] glm::dvec2 splitVertex() const;

    [[nodiscard]] std::vector<Set> cuts() const;

    [[nodiscard]] std::vector<Set> left() const;
    [[nodiscard]] std::vector<Set> right() const;


    static double RUN_UP;
    static uint8_t ITERATIONS;

private:

    [[nodiscard]] bool checkFloor(const glm::dvec2& initial, const glm::dvec2& normal, const Profile* profile) const;

    bool prepareRelief(const glm::dvec2& initial,
                       const glm::dvec2& internal, const glm::dvec2& external, const glm::dvec2& opposite,
                       const glm::dvec2& normal,
                       double length,
                       const Profile* profile);

    [[nodiscard]] Set direct(const glm::dvec2& origin, const glm::dvec2& axis, const glm::dvec2& ref, double depth) const;
    [[nodiscard]] std::vector<Set> sequence(const SectionOperation::Relief& relief) const;


public:

    bool valid; // Operation is possible given constraints

    glm::dvec2 start, end; // Endpoints
    double width, thickness;

    glm::dvec2 BA, BC, AC; // Edge unit vectors
    double lBA, lBC, lAC; // Edge lengths


    double theta; // Internal angle between BA and BC
    double reduction; // Depth reduction due to thickness (to prevent overrun)

    std::vector<Relief> reliefs;

private:

    bool m_cutBA, m_cutBC;

};


#endif //AUTOCARVER_SECTIONOPERATION_H
