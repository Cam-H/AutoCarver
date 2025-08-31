//
// Created by cjhat on 2025-08-25.
//

#include "SectionOperation.h"

#include <iostream>

#include "Profile.h"

double SectionOperation::RUN_UP = 0.02;

SectionOperation::SectionOperation()
        : valid(false)
{

}

SectionOperation::SectionOperation(const glm::dvec2& start, const glm::dvec2& split, const glm::dvec2& end,
                                   double width, double thickness)
    : valid(true)
    , start(start)
    , end(end)
    , width(width)
    , thickness(thickness)
    , BA(start - split)
    , BC(end - split)
    , AC(end - start)
    , lBA(glm::length(BA))
    , lBC(glm::length(BC))
    , lAC(glm::length(AC))
    , m_cutBA(true)
    , m_cutBC(true)
{
    BA /= lBA;
    BC /= lBC;
    AC /= lAC;

    theta = acos(glm::dot(BA, BC));
    reduction = theta > 0.5 * M_PI ? 0 : thickness / tan(theta);
}

SectionOperation::SectionOperation(const glm::dvec2& start, const glm::dvec2& split, const glm::dvec2& end,
                                   const glm::dvec2& pre, const glm::dvec2& post,
                                   const std::pair<double, double>& clearance,
                                   const Profile* profile,
                                   double width, double thickness)
    : SectionOperation(start, split, end, width, thickness)

{
    assert(profile != nullptr);

    if (valid && clearance.first < width) {
        reliefs.emplace_back(start, -BA, AC, pre,
                             std::max(0.0, lBA - reduction), width + RUN_UP, thickness);

        valid &= profile->validate(reliefs.back());

        m_cutBA = false;
    }

    if (valid && clearance.second < width) {
        reliefs.emplace_back(end, -BC, -AC, post,
                             std::max(0.0, lBC - reduction), width + RUN_UP, thickness);

        valid &= profile->validate(reliefs.back());

        m_cutBC = false;
    }
}

glm::dvec2 SectionOperation::startVertex(double offset) const
{
    if (!m_cutBA) return start;
    return start + BA * (RUN_UP + offset);
}

glm::dvec2 SectionOperation::endVertex(double offset) const
{
    if (!m_cutBC) return end;
    return end + BC * (RUN_UP + offset);
}

glm::dvec2 SectionOperation::splitVertex() const
{
    return start - BA * lBA;
}

std::vector<SectionOperation::Set> SectionOperation::cuts() const
{
    std::vector<Set> cuts;

    if (valid) {

        if (m_cutBA) { // Direct blind cut, if no relief operation is required on edge AB
            cuts.emplace_back( -BA, glm::dvec2(BA.y, -BA.x));
            if (glm::dot(cuts.back().normal, BC) < 0) cuts.back().normal = -cuts.back().normal;
            cuts.back().motions.emplace_back(startVertex(), lBA + RUN_UP - reduction);
        }

        if (m_cutBC) { // Direct blind cut, if no relief operation is required on edge BC
            cuts.emplace_back( -BC, glm::dvec2(BC.y, -BC.x));
            if (glm::dot(cuts.back().normal, BA) < 0) cuts.back().normal = -cuts.back().normal;
            cuts.back().motions.emplace_back(endVertex(), lBC + RUN_UP - reduction);
        }

        // Perform any required reliefs
        for (const SectionOperation::Relief& relief : reliefs) {

            glm::dvec2 position = relief.start + relief.extLength * relief.external + relief.normal * RUN_UP;

            auto step = relief.step(thickness);
            auto depths = relief.depths(thickness);
            cuts.emplace_back(-relief.normal, glm::dvec2(relief.normal.y, -relief.normal.x));
            if (glm::dot(cuts.back().normal, relief.external) > 0) cuts.back().normal = -cuts.back().normal;

            for (double depth : depths) {
                cuts.back().motions.emplace_back(position, depth + RUN_UP);
                position -= step * relief.external;
            }

//            double direction = 1 - 2 * (glm::dot(pose.axes.xAxis, travel) < 0);
//            glm::dvec3 origin = pose.position + 0.5 * m_bladeWidth * direction * pose.axes.xAxis;
//            double teff = std::abs(m_bladeThickness * glm::dot(normal, pose.axes.yAxis));

            // Add a milling operation to clean up surface (Two passes)
            double millDistance = relief.projection(thickness);
            double teff = 0.5 * std::abs(thickness * glm::dot(-relief.normal, relief.internal));
            cuts.emplace_back(-relief.normal, -cuts.back().normal, relief.internal);
            cuts.back().motions.emplace_back(relief.start + teff * relief.normal, millDistance);
            cuts.emplace_back(-relief.normal, cuts.back().normal, relief.internal);
            cuts.back().motions.emplace_back(relief.start, millDistance);

            // Perform a blind cut against the newly accessible surface, if the milling did not entirely clear it
            if (millDistance < relief.length && width < millDistance) {
                cuts.emplace_back( relief.internal, glm::dvec2(relief.internal.y, -relief.internal.x));
                if (glm::dot(cuts.back().normal, relief.external) < 0) cuts.back().normal = -cuts.back().normal;
                cuts.back().motions.emplace_back(relief.start + relief.internal * width, relief.length - width);
            }
        }
    }

    return cuts;
}

// TODO Currently only tries forming a relief cut with the adjacent edge as a guide. There are more (infinitely many) options. Trying against the opposite face would also capture some situations


// Forms a parallelogram representing the required empty region to perform a relief cut
SectionOperation::Relief::Relief(const glm::dvec2& start,
                                 const glm::dvec2& internal, const glm::dvec2& external, const glm::dvec2& help,
                                 double length, double width, double thickness)
    : start(start)
    , internal(internal)
    , external(external)
    , normal(help - start)
    , length(length)
    , sink(0)
    , valid(false)
{

    double normalLength = glm::length(normal);
    normal /= normalLength;

    theta = acos(glm::dot(internal, external));
    phi = acos(glm::dot(external, normal));

    double del = step(thickness), fi = sin(M_PI - theta - phi) / sin(phi);
    extLength = std::min(length, width) * fi + del;

//    std::cout << "RELIEF: " << cutLength << " -> " << extLength << " " << width << " " << (180*theta/M_PI) << " " << (180*phi/M_PI) << " " << normalLength << "\n";

    // TODO calculate sink
//    auto psi = acos(glm::dot(-normal, glm::normalize(end - split)));
//    firstDepth = extLength * del / fi - std::max(0.0, thickness / tan(psi));

    auto apex = start + external * extLength + normal * width;

    edges = {
            apex - normal * width,
            apex,
            apex - external * extLength
    };

    if (normalLength < width) edges.emplace_back(help);
}

// Returns the distance along external to travel for each relief cut. Based on the projection of thickness
double SectionOperation::Relief::step(double t) const
{
    return t / cos(0.5 * M_PI - phi);
}

// Returns the depth decrease (constant between each step) based on the thickness of the cutting tool
double SectionOperation::Relief::reduction(double t) const
{
    return std::max(0.0, t / tan(M_PI - theta - phi));
}

// Returns the equivalent distance travelled along internal when cutting along external
double SectionOperation::Relief::projection(double t) const
{
    double distance = extLength - step(t);
//    std::cout << theta << " " << phi << " " << 0.5 * M_PI << "\n";
//    if (theta + phi > 0.5 * M_PI) distance -= step(t);
    return sin(phi) * distance / sin(M_PI - theta - phi);
}

std::vector<double> SectionOperation::Relief::depths(double t) const
{
    if (extLength < 0) {
//        throw std::runtime_error("[Profile::Relief] Can not evaluate depths. Relief improperly defined");
        std::cout << "\033[93m[Profile::Relief] Can not evaluate depths. Relief improperly defined\033[0m\n";
        return {};
    }

    double x = extLength, del = step(t), reduc = reduction(t), factor = sin(theta) / sin(M_PI - theta - phi);

    std::vector<double> depth;
    depth.reserve((size_t)std::floor(extLength / del));

    while (x > del) {
        depth.emplace_back(x * factor - reduc);
        x -= del;
    }

    depth[0] += sink;
    return depth;
}

const glm::dvec2& SectionOperation::Relief::apex()
{
    return edges[1];
}