//
// Created by cjhat on 2025-08-25.
//

#include "SectionOperation.h"

#include <iostream>

#include "Profile.h"
#include "fileIO/SerialList.h"
#include "core/Functions.h"

double SectionOperation::RUN_UP = 0.02;
uint8_t SectionOperation::ITERATIONS = 8;

SectionOperation::SectionOperation()
        : m_valid(false)
{

}

SectionOperation::SectionOperation(const glm::dvec2& start, const glm::dvec2& split, const glm::dvec2& end,
                                   double width, double thickness)
    : m_valid(true)
    , m_split(split)
    , m_left(start)
    , m_right(end)
    , width(width)
    , thickness(thickness)
    , floor(std::numeric_limits<double>::lowest())
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

void SectionOperation::prepareReliefs(const Profile* profile)
{
    assert(profile != nullptr);

    const auto& tri = profile->nextTriangle();
    auto clearance = profile->clearance();

    auto left  = clearance.first < width + RUN_UP
            ? glm::normalize(profile->border()[profile->prevVertex(tri.I0)] - profile->border()[tri.I0]) : BA;
    auto right  = clearance.second < width + RUN_UP
            ? glm::normalize(profile->border()[profile->nextVertex(tri.I2)] - profile->border()[tri.I2]) : BC;

    std::cout << clearance.first << " " << clearance.second << "\n";

    auto ts = atan2(left.y, left.x), dt = glm::atan(right.y, right.x) - ts;
    if (ts < 0) ts += 2 * M_PI;
    if (dt < 0) dt += 2 * M_PI;

    double delta = dt / (double)(ITERATIONS - 1);

    floor = profile->floor();
    if (clearance.first < width || !checkFloor(m_left, left)) {
        m_cutBA = m_valid = false;

        double t = ts;
        for (uint8_t i = 0; i < ITERATIONS; i++) {
            glm::dvec2 normal(cos(t), sin(t));
            if (prepareRelief(m_left, -BA, AC, BC, normal, std::max(0.0, lBA - reduction), profile)) {
                m_valid = true;
                break;
            }

            t += delta;
        }
    }

    if (m_valid && (clearance.second < width || !checkFloor(m_right, right))) {
        m_cutBC = m_valid = false;

        double t = ts + dt;
        for (uint8_t i = 0; i < ITERATIONS; i++) {
            glm::dvec2 normal(cos(t), sin(t));
            if (prepareRelief(m_right, -BC, -AC, BA, normal, std::max(0.0, lBC - reduction), profile)) {
                m_valid = true;
                break;
            }

            t -= delta;
        }
    }

    adjustPositions();
}

// Apply offsets to the starting positions to account for tool width and runup
void SectionOperation::adjustPositions()
{
    if (m_valid) {
        double offset = 0.5 * width + RUN_UP, ht = 0.5 * thickness;

        if (m_cutBA) {
            glm::dvec2 norm = { BA.y, -BA.x };
            if (glm::dot(norm, BC) < 0) norm = -norm;

            m_left += offset * BA + ht * norm;
            lBA += RUN_UP;
        } else m_left = reliefStart(m_reliefs[0], offset);

        if (m_cutBC) {
            glm::dvec2 norm = { BC.y, -BC.x };
            if (glm::dot(norm, BA) < 0) norm = -norm;

            m_right += offset * BC + ht * norm;
            lBC += RUN_UP;
        } else m_right = reliefStart(m_reliefs[!m_cutBA], offset);
    }
}

// Offsets endpoints such that they do not collide with the hull. Returns true if the system remains possible (No floor contact + no reliefs)
bool SectionOperation::clear(const Polygon& hull)
{
    if (!m_cutBA || !m_cutBC) return false;

    glm::dvec3 left = { m_left.x, m_left.y, leftOrientation() };
    glm::dvec3 right = { m_right.x, m_right.y, rightOrientation() };

    double clearance = RUN_UP + 0.5 * width;
    double ts = hull.clear(clearance, left);
    double te = hull.clear(clearance, right);
    assert(ts > -1e-12 && te > -1e-12); // Would only be negative if cut is in some way reversed

    if (!checkFloor({ left.x, left.y }, BA) || !checkFloor({ right.x, right.y }, BC)) return false;

    if (ts > 1e-12) {
        m_left = { left.x, left.y };
        lBA += ts;
    }

    if (te > 1e-12) {
        m_right = { right.x, right.y };
        lBC += te;
    }

    return true;
}

void SectionOperation::Set::offset(double dx, double dy)
{
    glm::dvec2 delta = dx * axis + dy * normal;

    for (std::pair<glm::dvec2, double>& motion : motions) {
        motion.first += delta;
    }
}

void SectionOperation::Set::flipNormal()
{
    for (glm::dvec3& vertex : approach) {
        vertex.z = Functions::radWrap(vertex.z + M_PI);
    }

    normal = -normal;
}

glm::dvec3 SectionOperation::Set::start() const
{
    if (!approach.empty()) return approach[0];

    assert(!motions.empty());

    return {
        motions.front().first.x,
        motions.front().first.y,
        atan2(axis.y, axis.x)
    };
}

glm::dvec3 SectionOperation::Set::end() const
{
    if (!motions.empty()) {
        return {
            motions.back().first.x,
            motions.back().first.y,
            atan2(axis.y, axis.x)
        };
    }

    assert(!approach.empty());
    return approach.back();
}

glm::dvec2 SectionOperation::Set::initialNormal() const
{
    if (!approach.empty()) return { -sin(approach[0].z), cos(approach[0].z) };
    return normal;
}
glm::dvec2 SectionOperation::Set::finalNormal() const
{
    if (!motions.empty()) return normal;
    return { -sin(approach.back().z), cos(approach.back().z) };
}

std::vector<glm::dvec3> SectionOperation::Set::path() const
{
//    return { start(), end() };

    std::vector<glm::dvec3> path = approach;

    double z = atan2(-normal.x, normal.y);
    for (const std::pair<glm::dvec2, double>& motion : motions) {
        path.emplace_back(motion.first.x, motion.first.y, z);

        glm::dvec2 peak = motion.first + motion.second * travel;
        path.emplace_back(peak.x, peak.y, z);

        path.emplace_back(motion.first.x, motion.first.y, z);
    }

    return path;
}

std::vector<SectionOperation::Set> SectionOperation::left() const
{
    if (m_valid) {
        if (m_cutBA) return { direct(m_left, BA, BC, lBA) };
        else return sequence(m_reliefs[0]);
    }

    return {};
}
std::vector<SectionOperation::Set> SectionOperation::right() const
{
    if (m_valid) {
        if (m_cutBC) return { direct(m_right, BC, BA, lBC) };
        else return sequence(m_reliefs[!m_cutBA]);
    }

    return {};
}

bool SectionOperation::checkFloor(const glm::dvec2& initial, const glm::dvec2& normal) const
{
    return (initial + (width + RUN_UP) * normal).y >= floor;
}

// Prepare relief, returns true (success) if the relief would not hit the floor, the relief is not near-perpendicular
// to the external surface, and if there are no internal collisions
bool SectionOperation::prepareRelief(const glm::dvec2& initial,
                   const glm::dvec2& internal, const glm::dvec2& external, const glm::dvec2& opposite,
                   const glm::dvec2& normal,
                   double length,
                   const Profile* profile)
{
    if (checkFloor(initial, normal) && std::abs(glm::dot(external, normal)) > 0.2) {
        m_reliefs.emplace_back(initial, internal, external, opposite, normal, length, width + RUN_UP, thickness);
        if (m_reliefs.back().step(thickness) < lAC && profile->validate(m_reliefs.back())) return true;
        m_reliefs.pop_back();
    }

    return false;
}

glm::dvec2 SectionOperation::reliefStart(const SectionOperation::Relief& relief, double offset) const
{
    glm::dvec2 position = relief.start + relief.extLength * relief.external + relief.normal * RUN_UP;

    if (glm::dot(relief.internal, relief.normal) > 0) { // Relief cut direction is inverted
        double step = relief.step(thickness);
        position -= std::floor(relief.extLength / step) * step * relief.external;
    }

    return position + relief.normal * offset;
}

double SectionOperation::leftOrientation() const
{
    const glm::dvec2& delta = m_cutBA ? BA : -m_reliefs[0].normal;
    return atan2(delta.y, delta.x);
}
double SectionOperation::rightOrientation() const
{
    const glm::dvec2& delta = m_cutBC ? BC : -m_reliefs[!m_cutBA].normal;
    return atan2(delta.y, delta.x);
}

glm::dvec2 SectionOperation::splitVertex() const
{
    return m_split;
}

const glm::dvec2& SectionOperation::external() const
{
    return AC;
}

glm::dvec2 SectionOperation::normal() const
{
    glm::dvec2 norm = { AC.y, -AC.x };
    if (glm::dot(norm, BA) < 0) norm = -norm;
    return norm;
}

bool SectionOperation::reliefLeft() const
{
    return !m_cutBA;
}
bool SectionOperation::reliefRight() const
{
    return !m_cutBC;
}

bool SectionOperation::requiresRelief() const
{
    return !m_cutBA || !m_cutBC;
}

Triangle2D SectionOperation::bounds(double expansion) const
{
    glm::dvec2 a, b;
    if (m_cutBA) {
        a = m_split + (lBA + expansion) * BA;
    } else a = m_reliefs[0].start;

    if (m_cutBC) {
        b = m_split + (lBC + expansion) * BC;
    } else b = m_reliefs[!m_cutBA].start;

    return { a, m_split, b };
}

// Returns bounds, but with the left relief removed
Polygon SectionOperation::boundsNoLeft() const
{
    auto tri = bounds();

    if (!m_cutBA) {
        const Relief& relief = m_reliefs[0];
        double proj = relief.projection(thickness);

        std::vector<glm::dvec2> border;
        if (proj + 1e-12 < lBA) border.emplace_back(relief.start - BA * proj);
        border.emplace_back(relief.start + AC * relief.extLength);
        border.emplace_back(tri.c);
        border.emplace_back(tri.b);

        return Polygon(border);
    }

    return Polygon({ tri.a, tri.b, tri.c });
}

// Returns bounds, but with the right relief removed
Polygon SectionOperation::boundsNoRight() const
{
    auto tri = bounds();

    if (!m_cutBC) {
        const Relief& relief = m_reliefs[!m_cutBA];
        double proj = relief.projection(thickness);

        std::vector<glm::dvec2> border;
        border.emplace_back(relief.start - AC * relief.extLength);
        if (proj + 1e-12 < lBC) border.emplace_back(relief.start - BC * proj);
        border.emplace_back(tri.b);
        border.emplace_back(tri.a);
        return Polygon(border);
    }

    return Polygon({ tri.a, tri.b, tri.c });
}

// Direct blind cut, if no relief operation is required on edge
SectionOperation::Set SectionOperation::direct(const glm::dvec2& origin, const glm::dvec2& axis, const glm::dvec2& ref, double depth) const
{
    glm::dvec2 normal = { axis.y, -axis.x };
//    if (atan2(normal.y, normal.x) < 0) normal = -normal;

    return { -axis, normal, {{ origin, depth - reduction }} };
}

std::vector<SectionOperation::Set> SectionOperation::sequence(const SectionOperation::Relief& relief) const
{
    std::vector<Set> cuts;

    glm::dvec2 position = relief.start + relief.extLength * relief.external + relief.normal * RUN_UP;

    auto step = relief.step(thickness);
    auto depths = relief.depths(thickness);

    glm::dvec2 normal = { relief.normal.y, -relief.normal.x };
    if (glm::dot(normal, relief.external) > 0) normal = -normal;

    cuts.emplace_back(-relief.normal, normal);

    for (double depth : depths) {
        cuts.back().motions.emplace_back(position, depth + RUN_UP);
        position -= step * relief.external;
    }

    bool reverse = glm::dot(relief.internal, relief.normal) > 0;
    if (reverse) std::reverse(cuts.back().motions.begin(), cuts.back().motions.end());

//            double direction = 1 - 2 * (glm::dot(pose.axes.xAxis, travel) < 0);
//            glm::dvec3 origin = pose.position + 0.5 * m_bladeWidth * direction * pose.axes.xAxis;
//            double teff = std::abs(m_bladeThickness * glm::dot(normal, pose.axes.yAxis));

    // Add a milling operation to clean up surface (Two passes)
    double millDistance = relief.projection(thickness);
    double teff = 0.5 * std::abs(thickness * glm::dot(relief.normal, relief.internal));
    auto travel = (1.0 - 2.0 * reverse) * relief.internal;
    auto initial = relief.start + reverse * millDistance * relief.internal - !reverse * thickness * normal;
    cuts.emplace_back(-relief.normal, normal, travel);
    cuts.back().motions.emplace_back(initial + teff * relief.normal, millDistance);
    cuts.emplace_back(-relief.normal, normal, travel);
    cuts.back().motions.emplace_back(initial, millDistance);

    // Perform a blind cut against the newly accessible surface, if the milling did not entirely clear it
    if (millDistance + 1e-12 < relief.length && width < millDistance) {
        normal = { relief.internal.y, -relief.internal.x };

        initial = relief.start + relief.internal * (width + 0.5 * RUN_UP);

        glm::dvec3 prev = cuts.back().end();

        cuts.emplace_back(relief.internal, normal);
        cuts.back().motions.emplace_back(initial, relief.length - width - 0.5 * RUN_UP);
//        cuts.back().approach = { prev, cuts.back().start() };

        // TODO backcut
//        cuts.emplace_back( relief.internal, normal, -relief.internal);
//        cuts.back().motions.emplace_back(initial, -0.5 * m_runup);
    }

    // Applies offsets to account for tool dimensions
    double dx = -0.5 * width, dy = 0.5 * thickness;
    for (Set& set : cuts) {
        set.offset(dx, dy);

//        if (atan2(set.normal.y, set.normal.x) < 0) set.normal = -set.normal;
    }

    return cuts;
}

// Forms a parallelogram representing the required empty region to perform a relief cut
SectionOperation::Relief::Relief(const glm::dvec2& start,
                                 const glm::dvec2& internal, const glm::dvec2& external, const glm::dvec2& opposite,
                                 const glm::dvec2& normal,
                                 double length, double width, double thickness)
    : start(start)
    , internal(internal)
    , external(external)
    , normal(normal)
    , length(length)
    , sink(0)
{

    theta = acos(glm::dot(internal, external));
    phi = acos(glm::dot(external, normal));

    double del = step(thickness), fi = sin(M_PI - theta - phi) / sin(phi);
    extLength = std::min(length, width) * fi + del;

    // If cut exceeds internal length, first relief cut depth should be reduced to avoid hitting opposite edge
    if (extLength + 1e-12 >= length) {
        auto psi = acos(glm::dot(normal, opposite));
        if (psi < 0.5 * M_PI) sink = -thickness / tan(psi);
    }

    auto apex = start + external * extLength + normal * width;

    edges = {
            apex - normal * width,
            apex,
            apex - external * extLength
    };

    edges.emplace_back(edges[0] - external * extLength);
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
    double distance = extLength - step(t); // * (1.0 + (sink < 0)); TODO fix to prevent overrun by up to 1 step
//    std::cout << extLength << " " << distance << " " << step(t) << " " << (1.0 + sink < 0) << " " << (1.0 + (sink < 0)) << " VSZ\n";
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

    assert(depth.size() == (size_t)std::floor(extLength / del));

    depth[0] += sink;
    return depth;
}

const glm::dvec2& SectionOperation::Relief::apex()
{
    return edges[1];
}

// Calculates and returns the center of the parallelogram (bounds) of this relief
glm::dvec2 SectionOperation::Relief::center() const
{
    return 0.5 * (edges[0] + edges[2]);
}

void SectionOperation::print() const
{
    std::cout << "[SectionOperation] valid: " << m_valid << ", rleft: " << !m_cutBA << ", rright: " << !m_cutBC << ", floor: " << floor
        << "\nwidth: " << width << ", thickness: " << thickness << ", angle: " << theta << ", reduction: " << reduction
        << "\nTRI: [ (" << m_left.x << ", " << m_left.y << "), (" << m_split.x << ", " << m_split.y << "), (" << m_right.x << ", " << m_right.y << ") ]\n";
}

void SectionOperation::Relief::print() const
{
    std::cout << "[SectionOperation::Relief] start: (" << start.x << ", " << start.y << ")"
        << "\ninternal: [" << internal.x << ", " << internal.y << "] " << length
        << "\nexternal: [" << external.x << ", " << external.y << "] " << extLength
        << "\ntheta: " << theta << ", phi: " << phi << ", sink " << sink
        << "\nstep: " << step(1.0) << "x\n";
}