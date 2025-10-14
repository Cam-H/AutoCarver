//
// Created by cjhat on 2025-08-25.
//

#include "SectionOperation.h"

#include <iostream>

#include "Profile.h"

double SectionOperation::RUN_UP = 0.02;
uint8_t SectionOperation::ITERATIONS = 8;

SectionOperation::SectionOperation()
        : m_valid(false)
{

}

SectionOperation::SectionOperation(const glm::dvec2& start, const glm::dvec2& split, const glm::dvec2& end,
                                   double width, double thickness)
    : m_valid(true)
    , m_start(start)
    , m_end(end)
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

    auto ts = atan2(left.y, left.x), dt = glm::atan(right.y, right.x) - ts;
    if (ts < 0) ts += 2 * M_PI;
    if (dt < 0) dt += 2 * M_PI;

    double delta = dt / (double)(ITERATIONS - 1);

    floor = profile->floor();
    if (clearance.first < width || !checkFloor(m_start, left)) {
        m_cutBA = m_valid = false;

        double t = ts;
        for (uint8_t i = 0; i < ITERATIONS; i++) {
            glm::dvec2 normal(cos(t), sin(t));
            if (prepareRelief(m_start, -BA, AC, BC, normal, std::max(0.0, lBA - reduction), profile)) {
                m_valid = true;
                break;
            }

            t += delta;
        }
    }

    if (m_valid && (clearance.second < width || !checkFloor(m_end, right))) {
        m_cutBC = m_valid = false;

        double t = ts + dt;
        for (uint8_t i = 0; i < ITERATIONS; i++) {
            glm::dvec2 normal(cos(t), sin(t));
            if (prepareRelief(m_end, -BC, -AC, BA, normal, std::max(0.0, lBC - reduction), profile)) {
                m_valid = true;
                break;
            }

            t -= delta;
        }
    }
}

// Offsets endpoints such that they do not collide with the hull. Returns true if the system remains possible (No floor contact + no reliefs)
bool SectionOperation::clear(const Polygon& hull)
{
    if (!m_cutBA || !m_cutBC) return false;

    glm::dvec3 start = { m_start.x, m_start.y, startOrientation() };
    glm::dvec3 end = { m_end.x, m_end.y, endOrientation() };

    double clearance = RUN_UP + 0.5 * width;
    double ts = hull.clear(clearance, start);
    double te = hull.clear(clearance, end);
    assert(ts > -1e-12 && te > -1e-12); // Would only be negative if cut is in some way reversed

    if (!checkFloor({ start.x, start.y }, BA) || !checkFloor({ end.x, end.y }, BC)) return false;

    if (ts > 1e-12) {
        m_start = { start.x, start.y };
        lBA += ts;
    }

    if (te > 1e-12) {
        m_end = { end.x, end.y };
        lBC += te;
    }

    return true;
}

glm::dvec3 SectionOperation::initialLeft() const
{
    glm::dvec2 pos = start();
    return { pos.x, pos.y, startOrientation() };
}
glm::dvec3 SectionOperation::initialRight() const
{
    glm::dvec2 pos = end();
    return { pos.x, pos.y, endOrientation() };
}

bool SectionOperation::plan(RRTPlanner& planner, std::vector<Polygon>& env)
{
    if (m_valid) {

        static int iter = 0;
        planner.setStart(initialLeft());
        planner.setGoal(initialRight());
        planner.save("../out/B" + std::to_string(iter++) + ".bin");

        {
            m_sequence = left();
            std::vector<Set> rem = right();
            m_sequence.insert(m_sequence.end(), rem.begin(), rem.end());
            return true;
        }


//        glm::dvec2 sv = startVertex(0.5 * planner.width()), ev = endVertex(0.5 * planner.width());
        glm::dvec2 sv = start(), ev = end();

        glm::dvec3 first = { sv.x, sv.y, startOrientation() }, second = { ev.x, ev.y, endOrientation() };

        std::vector<Set> rem;

        // Choose the nearer cut to begin from TODO path plan in sequence creation
        glm::dvec2 initial = { planner.first().x, planner.first().y };
        glm::dvec2 ds = sv - initial, de = ev - initial;
        bool rightFirst = glm::dot(de, de) < glm::dot(ds, ds);

        if (rightFirst) {
            std::swap(first, second);
            m_sequence = right();
            rem = left();
        } else {
            m_sequence = left();
            rem = right();
        }

        std::cout << "SG " << first.x << " " << first.y << " " << first.z << " | " << second.x << " " << second.y << " " << second.z << "\n";

        // Develop approach to the first cut
        planner.setGoal(first);

        if (!planner.solved() && !planner.solve(env, planner.iterations() == 0 ? 40000 : 20000)) {
            std::cout << "\033[93mFailed to find approach\033[0m\n";
            return false;
        }

        // Remove relief material from subsequent consideration
        if (requiresRelief()) env.back() = midBounds(rightFirst);

        m_sequence.front().approach = planner.prunedPath(env);
        std::cout << "Possible approach identified\n";

        // Begin next plan from the end of the cut sequence
//        if (planner.setStart(env, m_sequence.back().end())) {
//            std::cout << "Failed to change start point!\n";
//        }
        if (!planner.moveToGoal()) {
            std::cout << "\033[93mFailed to change start point!\033[0m\n";
        }

        // Develop approach between first and second cuts
        planner.setGoal(second);

        // If a solution is already available, try to find a better one (And help rebase tree after rerooting)
        if (planner.solved()) planner.iterate(env, 1000);

        if (!planner.solved() && !planner.solve(env, 20000)) {
            std::cout << "\033[93mFailed to find intermediary\033[0m\n";
            return false;
        }

        rem.front().approach = planner.prunedPath(env);

        m_sequence.insert(m_sequence.end(), rem.begin(), rem.end());

        return true;
    }

    m_sequence = {};
    return false;
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

std::vector<SectionOperation::Set> SectionOperation::left() const
{
    if (m_valid) {
        if (m_cutBA) return { direct(m_start, BA, BC, lBA) };
        else return sequence(m_reliefs[0]);
    }

    return {};
}
std::vector<SectionOperation::Set> SectionOperation::right() const
{
    if (m_valid) {
        if (m_cutBC) return { direct(m_end, BC, BA, lBC) };
        else return sequence(m_reliefs[!m_cutBA]);
    }

    return {};
}

bool SectionOperation::checkFloor(const glm::dvec2& initial, const glm::dvec2& normal) const
{
    return (initial + (0.5 * width + RUN_UP) * normal).y >= floor;
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
        if (profile->validate(m_reliefs.back())) return true;
        m_reliefs.pop_back();
    }

    return false;
}

glm::dvec2 SectionOperation::start() const
{
    if (m_cutBA) return m_start;
    return reliefStart(m_reliefs[0], 0);
}

glm::dvec2 SectionOperation::end() const
{
    if (m_cutBC) return m_end;
    return reliefStart(m_reliefs[!m_cutBA], 0);
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

double SectionOperation::startOrientation() const
{
    const glm::dvec2& delta = m_cutBA ? BA : -m_reliefs[0].normal;
    return atan2(delta.y, delta.x);
}
double SectionOperation::endOrientation() const
{
    const glm::dvec2& delta = m_cutBC ? BC : -m_reliefs[!m_cutBA].normal;
    return atan2(delta.y, delta.x);
}

glm::dvec2 SectionOperation::splitVertex() const
{
    return m_start - BA * lBA;
}

bool SectionOperation::requiresRelief() const
{
    return !m_cutBA || !m_cutBC;
}

Triangle2D SectionOperation::bounds(double expansion) const
{
    return {
        m_start + BA * (expansion + (m_cutBA ? width + RUN_UP : 0.0)),
        splitVertex(),
        m_end   + BC * (expansion + (m_cutBC ? width + RUN_UP : 0.0))
    };
}

// Returns bounds
Polygon SectionOperation::bounds() const
{
    return Polygon({ splitVertex(), m_start, m_end });
}

// Returns bounds, but with the first relief removed
Polygon SectionOperation::midBounds(bool rightFirst) const
{
    std::vector<glm::dvec2> border;
    if (!rightFirst && !m_cutBA) {
        double proj = m_reliefs[0].projection(thickness);
        if (proj + 1e-12 < lBA) border.emplace_back(m_start - BA * proj);
        border.emplace_back(m_start + AC * m_reliefs[0].extLength);
    } else border.emplace_back(m_start);

    if (rightFirst && !m_cutBC) {
        double proj = m_reliefs[!m_cutBA].projection(thickness);
        border.emplace_back(m_end - AC * m_reliefs[!m_cutBA].extLength);
        if (proj + 1e-12 < lBC) border.emplace_back(m_end - BC * proj);
    } else border.emplace_back(m_end);

    border.emplace_back(splitVertex());

    return Polygon(border);
}

std::vector<SectionOperation::Set> SectionOperation::sequence() const
{
    return m_sequence;
}

// Direct blind cut, if no relief operation is required on edge
SectionOperation::Set SectionOperation::direct(const glm::dvec2& origin, const glm::dvec2& axis, const glm::dvec2& ref, double depth) const
{
    glm::dvec2 normal = { axis.y, -axis.x };
    if (glm::dot(normal, ref) < 0) normal = -normal;

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
        if (reverse ^ (glm::dot(normal, relief.external) > 0)) normal = -normal;

        initial = relief.start - !reverse * thickness * normal + relief.internal * (width + 0.5 * RUN_UP);

        cuts.emplace_back(relief.internal, normal);
        cuts.back().motions.emplace_back(initial, relief.length - width - 0.5 * RUN_UP);

        // TODO backcut
//        cuts.emplace_back( relief.internal, normal, -relief.internal);
//        cuts.back().motions.emplace_back(initial, -0.5 * m_runup);
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
    , valid(false)
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
