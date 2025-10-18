//
// Created by Cam on 2025-06-19.
//

#include "Profile.h"

#include <iostream>
#include "fileIO/SerialList.h"
#include "robot/planning/RRTPlanner.h"
#include "geometry/collision/Collision2D.h"

#include <gtc/quaternion.hpp>

Profile::Profile()
    : Profile({}, { 1, 0,  0 }, { 0, 0, -1 }, { 0, 1, 0 })
{
}

Profile::Profile(const std::vector<glm::dvec2>& contour, const glm::dvec3& normal, const glm::dvec3& xAxis, const glm::dvec3& yAxis)
    : Polygon(contour, false)
    , m_system(xAxis, yAxis,normal)
    , m_method(RefinementMethod::DIRECT)
    , m_minimumArea(0.0002)
    , m_floorIndex(0)
    , m_width(0.108)
    , m_thickness(0.011)
    , m_planner(nullptr)
    , m_wrapper(Polygon::convex(contour))
{
    initialize();
}

Profile::Profile(const std::string& filename)
        : Profile()
{
    Serializable::load(filename);
}

Profile::Profile(std::ifstream& file)
        : Profile()
{
    if (!Profile::deserialize(file)) std::cerr << "Failed to deserialize profile properly!\n";
}

bool Profile::serialize(std::ofstream& file) const
{
    Polygon::serialize(file);

    Serializer::writeDVec3(file, m_system.xAxis);
    Serializer::writeDVec3(file, m_system.yAxis);
    Serializer::writeDVec3(file, m_system.zAxis);

    switch (m_method) {
        case RefinementMethod::DIRECT:   Serializer::writeUint(file, 0); break;
        case RefinementMethod::DELAUNEY: Serializer::writeUint(file, 1); break;
        case RefinementMethod::TEST:     Serializer::writeUint(file, 2); break;
    }

    Serializer::writeUint(file, m_hull.size());
    for (uint32_t idx : m_hull) Serializer::writeUint(file, idx);

    Serializer::writeUint(file, m_operations.size());
    for (const auto& pair : m_operations) {
        Serializer::writeUint(file, pair.first);
        Serializer::writeUint(file, pair.second);
    }

    Serializer::writeUint(file, m_sections.size());
    for (const Section& section : m_sections) {
        Serializer::writeUint(file, section.triangle.I0);
        Serializer::writeUint(file, section.triangle.I1);
        Serializer::writeUint(file, section.triangle.I2);

        Serializer::writeUint(file, section.depth);
        Serializer::writeUint(file, section.children);

        Serializer::writeBool(file, section.external);
        Serializer::writeBool(file, section.skipped);
    }

    SerialList<Polygon>(m_remainder).serialize(file);

    Serializer::writeDouble(file, m_thickness);
    Serializer::writeDouble(file, m_width);

    Serializer::writeDouble(file, m_minimumArea);

    Serializer::writeUint(file, m_floorIndex);

    m_wrapper.serialize(file);

//    std::cout << "Serialize\n";
//    print();

    return true;
}

// TODO safety checks
bool Profile::deserialize(std::ifstream& file)
{
    Polygon::deserialize(file);

    m_system.xAxis = Serializer::readDVec3(file);
    m_system.yAxis = Serializer::readDVec3(file);
    m_system.zAxis = Serializer::readDVec3(file);

    // Don't call setter because internal state should not be recalculated (Loaded instead)
    switch (Serializer::readUint(file)) {
        case 0: m_method = RefinementMethod::DIRECT; break;
        case 1: m_method = RefinementMethod::DELAUNEY; break;
        case 2: m_method = RefinementMethod::TEST; break;
    }

    m_hull = std::vector<uint32_t>(Serializer::readUint(file));
    for (uint32_t& idx : m_hull) idx = Serializer::readUint(file);

    m_operations = std::vector<std::pair<uint32_t, uint32_t>>(Serializer::readUint(file));
    for (std::pair<uint32_t, uint32_t>& pair : m_operations) {
        pair.first = Serializer::readUint(file);
        pair.second = Serializer::readUint(file);
    }

    m_sections.clear();
    uint32_t size = Serializer::readUint(file), count = 0;
    for (uint32_t i = 0; i < size; i++) {
        uint32_t I0 = Serializer::readUint(file), I1 = Serializer::readUint(file), I2 = Serializer::readUint(file);
        uint32_t depth = Serializer::readUint(file), children = Serializer::readUint(file);
        m_sections.emplace_back(TriIndex(I0, I1, I2), count++, depth, children);
        m_sections.back().external = Serializer::readBool(file);
        m_sections.back().skipped = Serializer::readBool(file);
    }

    m_sequence = std::vector<Sequence>(size);

    m_remainder = SerialList<Polygon>(file).objects;

    m_thickness = Serializer::readDouble(file);
    m_width = Serializer::readDouble(file);

    m_minimumArea = Serializer::readDouble(file);

    m_floorIndex = Serializer::readUint(file);

    m_wrapper = Polygon(file);
//    std::cout << "Deserialize\n";
//    print();

//    sortOperations();

    return true;
}

void Profile::initialize()
{
    if (m_vertices.size() < 4) return;

    correctWinding();

    m_hull = hull();

    // Reorganize recorded border to ensure it is in ascending order (Needed for future operations)
    uint32_t last = m_hull[0];
    for (uint32_t i = 1; i < m_hull.size(); i++) {
        if (m_hull[i] < last) {
            last = m_hull.size();
            m_hull.insert(m_hull.begin(), m_hull.end(), m_hull.end() + m_hull.size() - i);
            m_hull.erase(m_hull.begin() + last, m_hull.end());
            break;
        }
        last = m_hull[i];
    }

    // Identify concave faces, noting boundaries with the convex hull
    for (uint32_t i = 0; i < m_hull.size(); i++) {
        uint32_t idx = (i + 1) % m_hull.size(), count = 1 + difference(m_hull[i], m_hull[idx], m_vertices.size());
        if (count > 2) emplaceOperation(m_hull[i], count);
    }

    sortOperations();

    findFloor();
}

// Sort operations by normal in a counter-clockwise direction beginning from 6 o'clock ([0, -1])
void Profile::sortOperations()
{
    std::vector<std::pair<uint32_t, double>> positions;
    positions.reserve(m_operations.size());

    for (uint32_t i = 0; i < m_operations.size(); i++) {
        glm::dvec2 ref = m_vertices[offsetIndex(m_operations[i].first, m_operations[i].second - 1)] - m_vertices[m_operations[i].first];
        glm::dvec2 ext = glm::normalize(glm::dvec2(ref.y, -ref.x));
        ref = m_vertices[offsetIndex(m_operations[i].first, 1)] - m_vertices[m_operations[i].first];
        if (glm::dot(ext, ref) > 0) ext = -ext;

        positions.emplace_back(i, atan2(ext.x, ext.y));
    }

//    for (const std::pair<uint32_t, double>& position : positions) {
//        std::cout << position.first << " " << position.second << "\n";
//    }

    std::sort(positions.begin(), positions.end(), [](const std::pair<uint32_t, double>& lhs, const std::pair<uint32_t, double>& rhs){
       return lhs.second > rhs.second; // Reverse for clockwise
    });

    std::vector<std::pair<uint32_t, uint32_t>> operations;
    operations.reserve(m_operations.size());

    for (const std::pair<uint32_t, double>& position : positions) {
        operations.emplace_back(m_operations[position.first]);
//        std::cout << "-> " << position.first << " " << operations.back().first << "\n";
    }

    m_operations = std::move(operations);
}

// Returns the index of the terminal vertex for the step
uint32_t Profile::terminus(uint32_t step) const
{
    if (step >= m_operations.size()) throw std::runtime_error("[Profile] Array index out of bounds");
    return offsetIndex(m_operations[step].first + m_operations[step].second - 2);
}

uint32_t Profile::offsetIndex(uint32_t idx, uint32_t offset) const
{
    return (idx + offset) % m_vertices.size();
}

void Profile::emplaceOperation(uint32_t start, uint32_t count)
{
    auto axis = edgeNormal(start, offsetIndex(start, count - 1));
    uint32_t subCount;

    while ((subCount = subdivide(axis, start, count)) < count) {
        // Additional check to remove adjacent collinear vertices
        if (subCount >= 2) m_operations.emplace_back(start, subCount + 1);

        // Prepare for next division
        start = offsetIndex(start, subCount);
        count -= subCount;
    }

    // Capture remaining
    if(count > 2) m_operations.emplace_back(start, count);
}

// Splits a section into subcomponents along protrusions. This is necessary because
// later triangulation would fail due to 0 thickness sections that would otherwise result
uint32_t Profile::subdivide(const glm::dvec2& normal, uint32_t start, uint32_t count)
{
    for (uint32_t i = 1; i < count - 1; i++) {
        double result = glm::dot(normal, glm::normalize(m_vertices[offsetIndex(start, i)] - m_vertices[start]));
        if (result > -0.02f) return i;
    }

    return count;
}

glm::dvec2 Profile::edgeNormal(uint32_t start, uint32_t end)
{
    glm::dvec2 edge = m_vertices[end] - m_vertices[start];
    return glm::normalize(glm::dvec2{ edge.y, -edge.x });
}

void Profile::setRefinementMethod(RefinementMethod method)
{
    m_sections.clear();

    if (!m_operations.empty()) {
        m_method = method;

        // More involved decomposition of concave regions for processing
        switch (m_method) {
//        case RefinementMethod::DIRECT: directRefinement(); break;
            case RefinementMethod::DELAUNEY: delauneyRefinement(); break;
//        case RefinementMethod::TEST: testRefinement(); break;
            default: throw std::runtime_error("[Profile] Unrecognized refinement method");
        }
    }

    print();
}

void Profile::setMimimumArea(double area)
{
    m_minimumArea = area;
}
//void Profile::translate(const glm::dvec2& translation)
//{
//    Polygon::translate(translation);
//}

void Profile::translate(const glm::dvec3& translation)
{
    Polygon::translate({
        glm::dot(translation, m_system.xAxis),
        glm::dot(translation, m_system.yAxis)
    });
}

void Profile::positionVertex(uint32_t index, const glm::dvec2& position)
{
    Polygon::positionVertex(index, position);
    findFloor();
}

void Profile::rotateAbout(const glm::dvec3& axis, double theta)
{
    m_system.rotate(axis, theta);
    auto rotation = glm::angleAxis(theta, axis);
//    m_normal = rotation * m_normal;
//    m_xAxis = rotation * m_xAxis;
//    m_yAxis = rotation * m_yAxis;
}

void Profile::inverseWinding()//TODO update
{
    Polygon::inverseWinding();

    for (auto& remainder : m_operations) {

        // Offset index by count (Required because inversion causes initiation from opposite side of section)
        remainder.first += remainder.second;
        if (remainder.first >= m_vertices.size()) remainder.first -= m_vertices.size();

        remainder.first = m_vertices.size() - remainder.first; // Invert index
    }
}

void Profile::setToolDimensions(double width, double thickness)
{
    m_width = width;
    m_thickness = thickness;
}

std::tuple<double, double> Profile::toolDimensions() const
{
    return { m_width, m_thickness };
}

// Skips the next section along with any further sections dependent on it
// IF the section is internal and has not yet been skipped - Delay processing for later in case the operation becomes possible
// OTHERWISE (If external or already seen == section is infeasible) Remove section and its dependents
void Profile::skip()
{
    if (!m_sections.empty()) {
        const std::vector<Polygon>& skips = nextSkips();
        m_remainder.insert(m_remainder.end(), skips.begin(), skips.end());

        uint32_t count = 1 + m_sections[0].children;
        if (count < m_sections.size()) {

            // Directly remove section when delaying can not make a section possible
            if (m_sections[0].external || m_sections[0].skipped || m_sections[count].external || m_sections[count].skipped) {
                if (m_sections[count].skipped) count += m_sections[count].children + 1;
                for (uint32_t i = 0; i < count; i++) m_sections.pop_front();
            } else { // Swap internal equal-depth sections
                m_sections[0].skipped = true;
                auto copy = std::deque(m_sections.begin(), m_sections.begin() + count);

                m_sections[0] = m_sections[count];
                for (uint32_t i = 1; i < m_sections[0].children + 1; i++) {
                    m_sections[i] = m_sections[count + i];
                }

                count = m_sections[0].children + 1;
                for (uint32_t i = 0; i < copy.size(); i++) {
                    m_sections[count + i] = copy[i];
                }
            }

        } else m_sections.clear(); // No use skipping the sole independent section - Always delete
    }
}

std::vector<Polygon> Profile::nextSkips()
{
    std::vector<Polygon> skips;

    if (!m_sections.empty()) {
        uint32_t count = std::min(m_sections[0].children + 1, (uint32_t)m_sections.size());

        if (count == m_sections.size() || m_sections[0].external || m_sections[0].skipped
            || m_sections[count].external || m_sections[count].skipped) {

            if (count < m_sections.size() && m_sections[count].skipped) count += m_sections[count].children + 1;

            for (uint32_t i = 0; i < count; i++) {
                const TriIndex& tri = m_sections[i].triangle;
                skips.emplace_back(std::vector<glm::dvec2>{
                    m_vertices[tri.I0],
                    m_vertices[tri.I1],
                    m_vertices[tri.I2]
                });
            }
        }
    }

    return skips;
}

const RRTPlanner* Profile::planner() const
{
    return m_planner;
}

const Polygon& Profile::wrapper() const
{
    return m_wrapper;
}

void Profile::refine()
{
    if (!m_sections.empty()) {
        if (m_sequence[m_sections[0].index].cutWrap) { // Adjust wrapper to account for now removed material
            m_wrapper = Collision2D::fragment(m_wrapper, Ray<2>(m_vertices[m_sections[0].triangle.I0], -nextExternal()));
        }

        if (m_planner != nullptr) { // Ensure planner starts from the latest position
            const Sequence& seq = m_sequence[m_sections[0].index];
            if (seq.leftFirst && !seq.right.empty()) m_planner->setStart(seq.right.back().end());
            else if (!seq.left.empty()) m_planner->setStart(seq.left.back().end());
        }

        uint32_t children = m_sections[0].children;

        // Remove prior skipped flag - If an equal-depth internal section is removed it is worth considering again
        if (children + 1 < m_sections.size()) m_sections[children + 1].skipped = false;

        m_sections.pop_front();

        for (uint32_t i = 0; i < children; i++) m_sections[i].depth--;
    }
}

void Profile::prepare()
{
    prepare(m_wrapper);
}

void Profile::prepare(const Polygon& wrapper)
{
    delete m_planner;
    m_planner = new RRTPlanner();

    m_wrapper = wrapper;

    m_planner->setTargetDimensions(m_width, m_thickness);

    auto [xMin, xMax] = m_wrapper.xExtents();
    auto [yMin, yMax] = m_wrapper.yExtents();

    double margin = 2.0 * m_planner->radius();

    glm::dvec2 initial = { xMax + margin, yMax + margin };
    m_planner->setBounds({ xMin - margin, yMin }, initial);

    // Start refinement from a position clear of the profile (top right)
    m_planner->setStart({ initial.x, initial.y, 0.0 });
}

bool Profile::planReturn()
{
    auto [min, max] = m_planner->getBounds();
    m_planner->setGoal({ max.x, max.y, 0.0 });

    std::vector<Polygon> decomposition = decompose();
    if (m_planner->solve(decomposition, 10000)) {
        m_return = m_planner->prunedPath(decomposition);
        return true;
    }

    return false;
}

bool Profile::planNext()
{
    if (m_sections.empty()) return false;

    auto op = nextOperation();

    if (op.isValid()) {

        const glm::dvec3 initialStart = m_planner->start();
        bool cutWrap = isNextExternal() && !op.clear(m_wrapper);

        std::cout << "PN " << isNextExternal() << " " << cutWrap << " " << initialStart.x << " " << initialStart.y << "\n";

        m_sequence[m_sections[0].index] = { op.bounds(0.5 * m_width), op.left(), op.right() };
        Sequence& seq = m_sequence[m_sections[0].index];

        if (cutWrap && !planSlice(op)) {
            std::cout << "\033[93mFailed to perform slice!\n\033[0m";
            return false;
        }

        std::vector<Polygon> decomposition = decompose();

        if (planSequence(decomposition, op)) {
            seq.valid = true;

            m_planner->setStart(initialStart);
            if (developApproach(decomposition, 10000)) return true;
        } else std::cout << "\033[93m[Profile] Failed to develop sequence!\n\033[0m";

        m_planner->setStart(initialStart); // Restore planner start when planning failed
    } else std::cout << "\033[93m[Profile] Next section operation is invalid!\n\033[0m";

    return false;
}

bool Profile::developApproach(const std::vector<Polygon>& decomposition, uint32_t iterations)
{
    if (!m_sections.empty()) {
        const glm::dvec3 initialStart = m_planner->start();

        Sequence& seq = m_sequence[m_sections[0].index];
        seq.approach.clear();

        std::vector<Polygon> bounds = { m_wrapper };
        bool success = true;

        if (seq.cutWrap) { // Begin with approach to the through cut, if needed
            double z = atan2(seq.cutDirection.y, seq.cutDirection.x);
            if (seq.cutApproach.empty()) { // Don't bother recalculating

                // Plan wrapper clear if needed
                seq.cutApproach = planClear(decomposition, bounds, iterations, success);
                if (!success) {
                    seq.cutApproach.clear();
                    return false;
                }

                // Plan travel to the slice position
                m_planner->setGoal({ seq.cutPosition.x, seq.cutPosition.y, z });
                if (m_planner->solve({ m_wrapper }, iterations)) {
                    auto path = m_planner->prunedPath(bounds);
                    seq.cutApproach.insert(seq.cutApproach.end(), path.begin(), path.end());
                } else {
                    std::cout << "\033[93m[Profile] Failed to develop slice approach!\n\033[0m";
                    seq.cutApproach.clear();
                    return false;
                }
            }

            if (!seq.cutApproach.empty()) { // Prepare for subsequent approach planning (towards actual cuts)
                glm::dvec2 end = seq.cutPosition + seq.cutDepth * seq.cutDirection;
                end += 0.02 * glm::dvec2(1, 0) * (1.0 - 2.0 * (seq.cutDirection.x > 0)); // Move off piece a bit to help path planning
                m_planner->setStart({ end.x, end.y, z });

                bounds[0] = Collision2D::fragment(bounds[0], Ray<2>(m_vertices[m_sections[0].triangle.I0], -nextExternal()));
                seq.approach.emplace_back(end.x, end.y, z);

                //                return true; // todo remove
            }
        } else if (isNextExternal()) {
            seq.approach = planClear(decomposition, bounds, iterations, success);
            if (!success) {
                std::cout << "\033[93m[Profile] Failed to clear for approach!\n\033[0m";
                return false;
            }
        }


        // Select the nearer edge to begin from
        if (m_planner->nearer(seq.left.front().start(), seq.right.front().start())) m_planner->setGoal(seq.left.front().start());
        else {
            m_planner->setGoal(seq.right.front().start());
            seq.leftFirst = false;
        }

        const std::vector<Polygon>& env = isNextExternal() ? bounds : decomposition;
        bool solve = m_planner->solve(env, iterations);
        if (!solve) {
            SerialList<Polygon>(env).save("../out/apoly.bin");
            m_planner->save("../out/aplan.bin");
        }

        if (seq.cutWrap) m_planner->setStart(initialStart);

        if (solve) {
            const std::vector<glm::dvec3> path = m_planner->prunedPath(env);
            seq.approach.insert(seq.approach.end(), path.begin(), path.end());
            return true;
        } else std::cout << "\033[93m[Profile] Failed to develop approach!\n\033[0m";
    }

    return false;
}

// Develop path to clear the wrapper
std::vector<glm::dvec3> Profile::planClear(const std::vector<Polygon>& decomposition, const std::vector<Polygon>& bounds, uint32_t iterations, bool& success)
{
    success = true;

    glm::dvec3 start = m_planner->start();
    double overlap = m_wrapper.clear(SectionOperation::RUN_UP, start);
    if (std::abs(overlap) > 1e-6) {
        m_planner->setGoal(start);

        if (m_planner->solve(decomposition, iterations)) {
            m_planner->setStart(start);
            return m_planner->prunedPath(bounds);
        } else {
            std::cout << "\033[93m[Profile] Failed to clear!\n\033[0m";
            success = false;
        }
    }

    return {};
}

//glm::dvec3 Profile::Sequence::start() const
//{
//    if (cutWrap)
//}

bool Profile::planSlice(const SectionOperation& op)
{
    Ray<2> ray(m_vertices[m_sections[0].triangle.I0], op.external());

    auto [hit, tr, tf] = Collision2D::intersection(m_wrapper, ray);
    std::cout << "P " << hit << " " << tr << " " << tf << " | " << ray.axis.x << " " << ray.axis.y << " " << glm::dot(ray.axis, { 0, 1 }) << "\n";
    if (hit) {

        Sequence& seq = m_sequence[m_sections[0].index];
        double offset = 0.5 * m_width + SectionOperation::RUN_UP;

        // Ensure cut is performed downwards
        seq.cutDirection = -ray.axis;
        if (glm::dot(ray.axis, { 0, 1 }) < 0) {
            seq.cutDirection = -seq.cutDirection;
            seq.cutPosition = (tr - offset) * ray;
        } else seq.cutPosition = (tf + offset) * ray;

        seq.cutDepth = tf - tr + SectionOperation::RUN_UP;

        // Adjust (reduce) depth to prevent impacting the floor
        double angle = ray.angle(), dh = 0.5 * m_width * sin(angle) + 0.5 * m_thickness * cos(angle);
        double yc = seq.cutPosition.y + seq.cutDirection.y * seq.cutDepth, dy = -std::min(0.0, yc - dh - floor());
//        std::cout << angle << " " << cos(angle) << " " << sin(angle) << " " << yc << " | " << (yc - dh) << " ~ " << floor() << "\n";
//        std::cout << seq.cutPosition.y << " " << floor() << " " << dh << " " << std::hypot(m_width, m_thickness) << " PSSSSSSSSSS\n";
        seq.cutDepth -= dy / sin(angle);

        glm::dvec2 normal = op.normal();
        seq.cutPosition += 0.5 * m_thickness * normal;

        return (seq.cutWrap = true);
    }

    return false;
}

// No protection - known to only be called with valid op
bool Profile::planSequence(std::vector<Polygon>& decomposition, const SectionOperation& op)
{
    static int iter = 0;

    Sequence& seq = m_sequence[m_sections[0].index];

    m_planner->setStart(seq.left.front().start());
    m_planner->setGoal(seq.right.front().start());

    // Use the wrapper to develop path between external start positions
    bool useWrapper = isNextExternal() && !seq.cutWrap;
    const std::vector<Polygon>& bounds = useWrapper ? std::vector<Polygon>{ m_wrapper } : decomposition; // Don't really need to calculate the full thing every time TODO

    std::cout << "PSOP " << bounds.size() << "\n";
    m_planner->setSampler(RRTPlanner::SampleSpace::BOX);
    if (m_planner->solve(bounds, 10000)) {
        m_sequence[m_sections[0].index].transition = m_planner->prunedPath(bounds);

        std::cout << "PATH " << m_sequence[m_sections[0].index].transition.size() << " / " << m_planner->path().size() << "\n";

        if (op.reliefLeft()) {
            decomposition.back() = op.boundsNoLeft();
            if (!planTransition(seq.bounds, decomposition, seq.left)) {
                SerialList<Polygon>(decomposition).save("../out/FPOLY" + std::to_string(iter) + "-SEQL.bin");
                m_planner->save("../out/FPLAN" + std::to_string(iter++) + "-SEQL.bin");
                return false;
            }
        }

        if (op.reliefRight()) {
            decomposition.back() = op.boundsNoRight();
            if (!planTransition(seq.bounds, decomposition, seq.right)) {
                SerialList<Polygon>(decomposition).save("../out/FPOLY" + std::to_string(iter) + "-SEQR.bin");
                m_planner->save("../out/FPLAN" + std::to_string(iter++) + "-SEQR.bin");
                return false;
            }
        }

        if (op.requiresRelief()) {
            auto tri = op.bounds();
            decomposition.back() = Polygon({ tri.a, tri.b, tri.c });
        }

//            glm::dvec3 prev = sequence[0].end();
//            for (uint32_t i = 1; i < sequence.size(); i++) {
//                if (!sequence[i].approach.empty()) prev = sequence[i].approach.back();
//
//                if (!sequence[i].motions.empty()) {
//                    glm::dvec2 norm = { -sin(prev.z), cos(prev.z) };
//                    std::cout << norm.x << " " << norm.y << " " << sequence[i].normal.x << " " << sequence[i].normal.y << " -> " << glm::dot(sequence[i].normal, norm) << " zzazzm\n";
//                    if (glm::dot(sequence[i].normal, norm) < -1.0 + 1e-3) sequence[i].normal = -sequence[i].normal;
//                }
//
//                prev = sequence[i].end();
//            }

        return true;
    } else {
        std::cout << "\033[93m[Profile] Failed to develop path between edges!\n\033[0m";
        SerialList<Polygon>(bounds).save("../out/FPOLY" + std::to_string(iter) + "-SEQ.bin");
        m_planner->save("../out/FPLAN" + std::to_string(iter++) + "-SEQ.bin");
    }

    return false;
}

// Develops approaches to connect disjoint sets
bool Profile::planTransition(const Triangle2D& bounds, const std::vector<Polygon>& env, std::vector<SectionOperation::Set>& sets)
{
    glm::dvec3 start = sets.front().start(), end = sets.back().end();

    // Restrict configuration space to a smaller area (Solution is necessarily in the region)
//    auto region = Circle::enclose({ bounds.a, bounds.b, bounds.c, { start.x, start.y } });
//    region.radius += 2.0 * m_planner->radius();
//    m_planner->setBounds(region);

    // Try preparing approach to return to the start point of these sets
    m_planner->setStart(end);
    m_planner->setGoal(start);
    if (m_planner->solve(env, 5000)) {
        sets.emplace_back(m_planner->prunedPath(env));
    } else { // TODO method not guaranteed to avoid collision
        sets.emplace_back(std::vector<glm::dvec3>{ end, start });
//        return false;
    }

//    start = sets.front().end();
//    for (uint32_t i = 0; i < sets.size(); i++) {
//        std::cout << "PT " << i << " " << sets[i].approach.size() << " " << sets[i].motions.size() << " " << atan2(sets[i].axis.y, sets[i].axis.x) << " " << atan2(sets[i].normal.y, sets[i].normal.x) << "\n";
//        if (!sets[i].motions.empty() && atan2(sets[i].normal.y, sets[i].normal.x) < 0) {
//            std::cout << "REVERSING\n";
//            sets[i].normal = -sets[i].normal;
//        }
////        start = sets[i].end();
//    }

//    std::cout << sets.size() << " zzPT:\n";
//    for (const auto& set : sets) {
//        auto s = set.start(), e = set.end();
//        std::cout << set.approach.size() << " " << set.motions.size() << " " << s.x << " " << s.y << " " << s.z << " | " << e.x << " " << e.y << " " << e.z << "\n";
//    }
//
//    for (uint32_t i = 1; i < sets.size(); i++) {
//        start = sets[i - 1].end();
//        end = sets[i].start();
//
//        glm::dvec3 delta = end - start;
//        if (glm::dot(delta, delta) > 1e-3) {
//            m_planner->setStart(start);
//            m_planner->setGoal(end);
//            if (m_planner->solve(env, 5000)) {
//                sets[i].approach = m_planner->prunedPath(env);
//            } // else return false;
//        }
//    }

    return true;
}


TriIndex Profile::nextTriangle() const
{
    if (!m_sections.empty()) return m_sections[0].triangle;
    return { 0, 0, 0 };
}

glm::dvec2 Profile::nextExternal() const
{
    if (m_sections.empty()) return {};

    const TriIndex& tri = m_sections[0].triangle;
    glm::dvec2 external = m_vertices[tri.I2] - m_vertices[tri.I0];
    external = { external.y, -external.x };

    if (glm::dot(external, m_vertices[tri.I0] - m_vertices[tri.I1]) < 0) external = -external;

    return glm::normalize(external);
}

glm::dvec3 Profile::nextNormal() const
{
    if (m_sections.empty()) return {};

    return projected3D(nextExternal());
}
std::pair<glm::dvec3, glm::dvec3> Profile::nextEdge() const
{
    if (m_sections.empty()) return {};

    return {
        projected3D(m_vertices[m_sections[0].triangle.I0]),
        projected3D(m_vertices[m_sections[0].triangle.I2])
    };
}

SectionOperation Profile::nextOperation() const
{
    if (m_sections.empty()) return {};

    const TriIndex& tri = m_sections[0].triangle;
    SectionOperation operation = {
            m_vertices[tri.I0],
            m_vertices[tri.I1],
            m_vertices[tri.I2],
            m_width,
            m_thickness
    };

    operation.prepareReliefs(this);

    return operation;
}

std::vector<glm::dvec3> Profile::nextPath() const
{
    std::vector<SectionOperation::Set> sequence = nextSequence();
    std::vector<glm::dvec3> path;

    for (const SectionOperation::Set& set : sequence) {
        auto sPath = set.path();
        path.insert(path.end(), sPath.begin(), sPath.end());
    }

    return path;
}

std::pair<glm::dvec3, glm::dvec3> Profile::Sequence::cut() const
{
    glm::dvec3 pos = { cutPosition.x, cutPosition.y, atan2(cutDirection.y, cutDirection.x) };
    auto delta = cutDepth * glm::dvec3(cutDirection.x, cutDirection.y, 0);

    return { pos, pos + delta };
}

const Triangle2D& Profile::nextBounds() const
{
    assert(!m_sections.empty() && m_sections[0].index < m_sequence.size());
    return m_sequence[m_sections[0].index].bounds;
}

bool Profile::nextSequenceAccessible() const
{
    return !m_sections.empty() && m_sequence[m_sections[0].index].valid;
}
bool Profile::nextSequenceReady() const
{
    return nextSequenceAccessible() && !m_sequence[m_sections[0].index].approach.empty();
}

std::vector<SectionOperation::Set> Profile::nextSequence() const
{
    if (nextSequenceReady()) {
        const Sequence& seq = m_sequence[m_sections[0].index];

        std::vector<SectionOperation::Set> sequence;
        if (seq.cutWrap) {
            sequence.emplace_back(seq.cutDirection, glm::dvec2(seq.cutDirection.y, -seq.cutDirection.x));
            sequence[0].motions.emplace_back(seq.cutPosition, seq.cutDepth);
            sequence[0].approach = seq.cutApproach;
        }

        sequence.emplace_back(seq.approach);

        const std::vector<SectionOperation::Set>& first = seq.leftFirst ? seq.left : seq.right;
        sequence.insert(sequence.end(), first.begin(), first.end());

        const std::vector<SectionOperation::Set>& second = seq.leftFirst ? seq.right : seq.left;
        sequence.emplace_back(seq.transition);
        if (!seq.leftFirst) std::reverse(sequence.back().approach.begin(), sequence.back().approach.end());
        sequence.insert(sequence.end(), second.begin(), second.end());

        std::cout << "PRE\n";
        for (const auto& set : sequence) {
            std::cout << set.initialNormal().x << " " << set.initialNormal().y << " P " << set.finalNormal().x << " " << set.finalNormal().y << "\n";
        }

        glm::dvec2 test = { 2, 1 };
        std::cout << atan2(test.y, test.x) << " " << atan2(-test.y, -test.x) << " TEST\n";

        glm::dvec2 prev = sequence[0].initialNormal();
        for (uint32_t i = 0; i < 1; i++) {

            std::cout << i << ": " << sequence[i].approach.size() << " " << sequence[i].motions.size()
                      << " | " << glm::dot(prev, sequence[i].initialNormal()) << " " << glm::dot(sequence[i].initialNormal(), sequence[i].normal) << "\n";

            for (const glm::dvec3& v : sequence[i].approach) std::cout << "[" << v.x << " " << v.y << " " << v.z << "]\n";

            prev = sequence[i].finalNormal();
        }

        for (uint32_t i = 1; i < sequence.size(); i++) {
            std::cout << i << ": " << sequence[i].approach.size() << " " << sequence[i].motions.size()
                      << " | " << glm::dot(sequence[i - 1].finalNormal(), sequence[i].initialNormal()) << " " << glm::dot(sequence[i].initialNormal(), sequence[i].normal) << "\n";

            for (const glm::dvec3& v : sequence[i].approach) std::cout << "[" << v.x << " " << v.y << " " << v.z << "]\n";
            if (sequence[i].approach.empty()) {
                auto s = sequence[i].start(), e = sequence[i].end();
                std::cout << "[" << s.x << " " << s.y << " " << s.z << "]\n";
                std::cout << "[" << e.x << " " << e.y << " " << e.z << "]\n";
            }

            if (glm::dot(sequence[i - 1].finalNormal(), sequence[i].initialNormal()) < 0) {
                sequence[i].flipNormal();
            }
        }

        std::cout << "POST\n";
        for (const auto& set : sequence) {
            std::cout << set.initialNormal().x << " " << set.initialNormal().y << " P " << set.finalNormal().x << " " << set.finalNormal().y << "\n";
        }

        std::cout << "~~~~~~~~~~~~~~~~~~~\n";
        glm::dvec3 last = sequence[0].start();
        for (const auto& set : sequence) {
            auto s = set.start(), del = s - last;
            prev = set.end();
            std::cout << s.x << " " << s.y << " " << s.z << ", delta: " << del.x << " " << del.y << " " << del.y << "\n";
        }

        return sequence;
    }

    return {};
}

std::vector<glm::dvec3> Profile::returnPath() const
{
    return m_return;
}

uint32_t Profile::remainingSections() const
{
    return m_sections.size();
}

bool Profile::complete() const
{
    return m_sections.empty();
}

bool Profile::isVertexExternal(uint32_t index) const
{
    if (index >= m_vertices.size()) return false;
    return std::binary_search(m_hull.begin(), m_hull.end(), index);
}

bool Profile::isNextExternal() const
{
    return !m_sections.empty() && m_sections[0].external;
}

uint32_t Profile::nextDepth() const
{
    if (m_sections.empty()) return 0;
    return m_sections[0].depth;
}

// Returns the area of the next section
double Profile::area() const
{
    if (m_sections.empty()) return 0;
    return area(m_sections[0].triangle);
}

double Profile::floor() const
{
    if (m_floorIndex >= m_vertices.size()) return 0;
    return m_vertices[m_floorIndex].y;
}

std::pair<double, double> Profile::angles() const
{
    if (m_sections.empty()) return { 0, 0 };
    return angles(m_sections[0].triangle);
}
std::pair<double, double> Profile::clearance() const
{
    if (m_sections.empty()) return { 0, 0 };
    return clearance(m_sections[0].triangle);
}

// Returns the exterior angle formed between the cut and adjacent edges for both vertices [in radians]
std::pair<double, double> Profile::angles(const TriIndex& triangle) const
{
    if (!triangle.isValid(m_vertices.size())) return { 0, 0 };

    uint32_t prev = prevVertex(triangle.I0);
    uint32_t next = nextVertex(triangle.I2);

    return {
        angle(m_vertices[prev] - m_vertices[triangle.I0], m_vertices[triangle.I1] - m_vertices[triangle.I0]),
        angle(m_vertices[triangle.I1] - m_vertices[triangle.I2], m_vertices[next] - m_vertices[triangle.I2])
    };
}

double Profile::angle(const glm::dvec2& a, const glm::dvec2& b)
{
    double value = atan2(b.y, b.x) - atan2(a.y, a.x);
    return value + 2 * M_PI * (value < 0);
}

uint32_t Profile::prevVertex(uint32_t vertexIndex) const
{
    for (const Section& section : m_sections) {
        if (section.triangle.I2 == vertexIndex) return section.triangle.I0;
    }

    return Polygon::prevVertex(vertexIndex);
}
uint32_t Profile::nextVertex(uint32_t vertexIndex) const
{
    for (const Section& section : m_sections) {
        if (section.triangle.I0 == vertexIndex) return section.triangle.I2;
    }

    return Polygon::nextVertex(vertexIndex);
}

// Returns the clearance from either vertex before the first obstruction on the reversed cut direction (Margin for alignment)
std::pair<double, double> Profile::clearance(const TriIndex& triangle) const
{
    if (!triangle.isValid(m_vertices.size())) return { 0, 0 };

    auto theta = angles(triangle);

    return {
        theta.first  < M_PI ? 0 : clearance(-glm::normalize(m_vertices[triangle.I0] - m_vertices[triangle.I1]), triangle.I0),
        theta.second < M_PI ? 0 : clearance(-glm::normalize(m_vertices[triangle.I2] - m_vertices[triangle.I1]), triangle.I2)
    };
}

// Calculates clearance along the axis from the specified vertex
// Always returns DOUBLE_MAX if the vertex is on the convex hull regardless of axis direction
double Profile::clearance(const glm::dvec2& axis, uint32_t vertexIndex) const
{
    double t = std::numeric_limits<double>::max();

    if (!isVertexExternal(vertexIndex)) {
        glm::dvec2 dir = m_vertices[vertexIndex] - axis;

        auto hv = nextHullVertices(vertexIndex);
        intersection(m_vertices[vertexIndex], dir, hv.first, prevVertex(vertexIndex), t);
        intersection(m_vertices[vertexIndex], dir, nextVertex(vertexIndex), hv.second, t);
    }

    return t;
}

// Checks collisions against the parallelogram formed by the relief, returning true
// if there were no intersections (indicating that it is possible for the robot to perform the relief cuts without colliding)
bool Profile::validate(const SectionOperation::Relief& relief) const
{
    if (m_sections.empty()) return false;

    uint32_t splitIndex = m_sections[0].triangle.I1;
    auto hv = nextHullVertices(splitIndex);

    for (uint32_t i = 1; i < relief.edges.size(); i++) {
        if (intersects(relief.edges[i - 1], relief.edges[i], hv.first, hv.second)) return false;
    }

    return true;
}

bool Profile::intersects(const glm::dvec2& a, const glm::dvec2& b, uint32_t start, uint32_t end) const
{
    uint32_t index = start;
    while (start != end) {
        index = nextVertex(index);
        auto [hit, tIntersection] = Polygon::intersection(a, b, m_vertices[start], m_vertices[index]);
        if (hit && 1e-12 < tIntersection && tIntersection < 1 - 1e-12) {
//            std::cout << "Intersection: " << start << " " << end << " | " << index << " " << tIntersection << "\n";
            return true;
        }
        start = index;
    }

    return false;
}

void Profile::intersection(const glm::dvec2& a, const glm::dvec2& b, uint32_t start, uint32_t end, double& t) const
{
    uint32_t index = start;
    while (start != end) {
        index = nextVertex(index);
        auto [hit, tIntersection] = Polygon::intersection(a, b, m_vertices[start], m_vertices[index]);
        if (hit) t = std::min(t, tIntersection);
        start = index;
    }
}

std::pair<uint32_t, uint32_t> Profile::nextHullVertices(uint32_t vertexIndex) const
{
    auto lIt = std::lower_bound(m_hull.begin(), m_hull.end(), vertexIndex);
    auto hIt = std::upper_bound(m_hull.begin(), m_hull.end(), vertexIndex);

    return {
        lIt == m_hull.begin() ? m_hull.back() : *(lIt - 1), // index is lowest  -> wrap to the last
        hIt == m_hull.end() ? m_hull[0] : *hIt              // index is highest -> wrap to the start
    };
}

const Axis3D& Profile::system() const
{
    return m_system;
}

const glm::dvec3& Profile::normal() const
{
    return m_system.zAxis;
}

void Profile::addTriangle(uint32_t startIndex)
{
    auto tri = TriIndex(startIndex, offsetIndex(startIndex, 1), offsetIndex(startIndex, 2));
    if (area(tri) > m_minimumArea) {
        m_sections.emplace_back(tri, m_sections.size(), 0);
        m_sections.back().external = true;
    }
}

void Profile::directRefinement()
{
//    auto cut = m_operations[m_next];
//    auto indices = sectionIndices(cut);
//
//    if (isValidRefinement(indices)) {
//        m_operations.erase(m_operations.begin() + m_next);
//        return indices;
//    } else m_next++;
//
//    return {};
}


void Profile::delauneyRefinement()
{
    for (const std::pair<uint32_t, uint32_t>& rem : m_operations) {

        // Easy direct triangle cutout
        if (rem.second == 3) {
            addTriangle(rem.first);
            continue;
        }

        // Generate chain of just the vertices that need to be refined in this section
        std::vector<uint32_t> indices = sectionIndices(rem);
        std::vector<glm::dvec2> vertices = sectionVertices(indices);

        // Find the Delauney triangulation of the section
        auto rawTriangles = Polygon::triangulate(vertices);

        std::vector<Section> sections = prepareSections(rawTriangles, indices);

        commitSections(sections);
    }

    m_sequence = std::vector<Sequence>(m_sections.size());
}

void Profile::commitSections(const std::vector<Section>& sections)
{
    for (uint32_t i = 0; i < sections.size(); i++) {
        double sum = area(sections[i].triangle);
        uint32_t j = sections[i].children * (sum == 0);

        // For small, valid sections, sum area of children
        while (sum < m_minimumArea && j < sections[i].children) {
            sum += area(sections[i + j].triangle);
        }

        // Only commit valid sections of sufficient size TODO test
        if (sum < m_minimumArea) {

            // Adjust parent counts to remove impact of now removed sections
            for (int k = (int)m_sections.size() - 1; k >= 0; k--) {
                if (m_sections[k].children == 0) break;
                m_sections[k].children -= sections[i].children;
            }

            i += sections[i].children;
        } else m_sections.emplace_back(sections[i].triangle, m_sections.size(), sections[i].depth, sections[i].children, sections[i].external);
    }
}

std::vector<Profile::Section> Profile::prepareSections(std::vector<TriIndex>& triangles, const std::vector<uint32_t>& indexMap)
{
    std::vector<Section> sections;
    sections.reserve(triangles.size());

    struct Edge {
        Edge(uint32_t first, uint32_t last, uint32_t depth) : first(first), last(last), depth(depth) {}
        uint32_t first, last, depth;
    };

    std::vector<Edge> edges = { { 0, (uint32_t)indexMap.size() - 1, 0 } };

    // Select triangles from exterior to interior like a tree
    while (!triangles.empty()) {
        if (edges.empty()) throw std::runtime_error("[Profile] Failed to partition triangles");

        auto [valid, nextTri] = nextTriangle(triangles, edges.back().first, edges.back().last);
        uint32_t depth = edges.back().depth;
        edges.pop_back();

        if (valid) {

            // Correct indexing while constructing sections
            sections.emplace_back(TriIndex(indexMap[nextTri.I0], indexMap[nextTri.I1], indexMap[nextTri.I2]), sections.size(), depth);

            edges.emplace_back(nextTri.I0, nextTri.I1, depth + 1);
            edges.emplace_back(nextTri.I1, nextTri.I2, depth + 1);
        }
    }

    // Calculate # of children of each section (Based on distance to next section of equal depth)
    for (uint32_t i = 0; i < sections.size(); i++) {
        sections[i].external = sections[i].depth == 0;

        for (uint32_t j = i + 1; j < sections.size(); j++) {
            if (sections[j].depth <= sections[i].depth) break;
            sections[i].children++;
        }
    }

    return sections;
}


// Selects the triangle connected to the given indices, returns its other vertex, and removes it from the pool
std::tuple<bool, TriIndex> Profile::nextTriangle(std::vector<TriIndex>& triangles, uint32_t first, uint32_t last)
{
    for (uint32_t i = 0; i < triangles.size(); i++) {
        if (triangles[i].has(first) && triangles[i].has(last)) {
            uint32_t splitVertex = triangles[i].last(first, last);

            std::swap(triangles[i], triangles.back());
            triangles.pop_back();

            return { true, TriIndex(first, splitVertex, last) };
        }
    }

    return { false, TriIndex(0, 0, 0) };
}

void Profile::testRefinement()
{

}

void Profile::findFloor()
{
    double min = std::numeric_limits<double>::max();
    for (uint32_t i = 0; i < m_vertices.size(); i++) {
        if (m_vertices[i].y < min) {
            min = m_vertices[i].y;
            m_floorIndex = i;
        }
    }
}

double Profile::area(const TriIndex& triangle) const
{
    if (triangle.I0 == triangle.I1) return 0; // Indicates invalid triangle

    return std::abs(Triangle2D::area(m_vertices[triangle.I0], m_vertices[triangle.I1], m_vertices[triangle.I2]));
}

uint32_t Profile::difference(uint32_t a, uint32_t b, uint32_t max)
{
    return b < a ? max - a + b : b - a;
}

std::vector<uint32_t> Profile::sectionIndices(const std::pair<uint32_t, uint32_t>& limits) const
{
    std::vector<uint32_t> indices;
    indices.reserve(limits.second);

    for (uint32_t i = 0; i < limits.second; i++) indices.emplace_back((limits.first + i) % m_vertices.size());
    return indices;
}

std::vector<glm::dvec2> Profile::sectionVertices(const std::pair<uint32_t, uint32_t>& limits) const
{
    return sectionVertices(sectionIndices(limits));
}
std::vector<glm::dvec2> Profile::sectionVertices(const std::vector<uint32_t>& indices) const
{
    std::vector<glm::dvec2> section(indices.size());
    for (uint32_t i = 0; i < indices.size(); i++) section[i] = m_vertices[indices[i]];
    return section;
}

glm::dvec2 Profile::project(const glm::dvec3& vertex) const
{
    return { glm::dot(m_system.xAxis, vertex), glm::dot(m_system.yAxis, vertex) };
}

// Project vertices into 3D space according to the defined axis system
std::vector<glm::dvec3> Profile::projected3D() const
{
    return Polygon::projected3D(m_system.xAxis, m_system.yAxis, {});
}

std::vector<glm::dvec3> Profile::projected3D(const TriIndex& triangle, const glm::dvec3& offset) const
{
    return projected3D(std::vector<uint32_t>{ triangle.I0, triangle.I1, triangle.I2 }, offset);
}

std::vector<glm::dvec3> Profile::projected3D(const std::vector<uint32_t>& indices, const glm::dvec3& offset) const
{
    std::vector<glm::dvec2> vertices;
    vertices.reserve(indices.size());

    for (uint32_t idx : indices) {
        if (idx >= m_vertices.size()) throw std::runtime_error("[Profile] Out of bounds vertex index access!");
        vertices.emplace_back(m_vertices[idx]);
    }

    return Polygon::projected3D(vertices, m_system.xAxis, m_system.yAxis, offset);
}

//std::vector<Polygon> Profile::decompose() const
//{
//    std::vector<Polygon> hulls = Polygon::decompose();
//
//    for (int i = m_sections.size() - 1; i >= 0; i--) {
//        const TriIndex& tri = m_sections[i].triangle;
//        hulls.emplace_back(std::vector<glm::dvec2>{ m_vertices[tri.I0], m_vertices[tri.I1], m_vertices[tri.I2] });
//    }
//
//    return hulls;
//}

// Decomposition based on standard polygon + any triangles that were skipped during refining
std::vector<Polygon> Profile::decompose() const
{
    if (!isValid()) return {};

    std::vector<Polygon> hulls = Polygon::decompose();
    for (const Polygon& hull : m_remainder) hulls.emplace_back(hull);

    for (int i = m_sections.size() - 1; i >= 0; i--) {
        hulls.emplace_back(std::vector<glm::dvec2>{
                m_vertices[m_sections[i].triangle.I0],
                m_vertices[m_sections[i].triangle.I1],
                m_vertices[m_sections[i].triangle.I2]
        });
    }

    return hulls;
}

// Returns a list of all hulls that make up the current frontier (Directly accessible sections to refine)
std::vector<Polygon> Profile::frontier() const
{
    std::vector<Polygon> hulls;

    for (const Section& section : m_sections) {
        if (section.depth == 0) {
            hulls.emplace_back(std::vector<glm::dvec2>{
                m_vertices[section.triangle.I0],
                m_vertices[section.triangle.I1],
                m_vertices[section.triangle.I2]
            });
        }
    }

    // TODO better - Ensure that the current operating hull is last
    std::reverse(hulls.begin(), hulls.end());

    return hulls;
}

const std::vector<Polygon>& Profile::remainder() const
{
    return m_remainder;
}

std::vector<std::pair<glm::dvec2, glm::dvec2>> Profile::debugEdges() const {
    auto edges = Polygon::debugEdges();

    // Only need to show one edge of each triangle (All except boundary are shared)
    for (const Section& section: m_sections) {
        edges.emplace_back(m_vertices[section.triangle.I0], m_vertices[section.triangle.I2]);
    }

    // Show clearances
    if (!m_sections.empty()) {
        TriIndex tri = m_sections[0].triangle;

        std::vector<uint8_t> checks;

//        auto margin = clearance();
//        if (margin.first  != 0) edges.emplace_back(m_vertices[tri.I0], m_vertices[tri.I0] + std::min(margin.first,  2.0) * glm::normalize(m_vertices[tri.I0] - m_vertices[tri.I1]));
//        else checks.emplace_back(0);
//
//        if (margin.second != 0) edges.emplace_back(m_vertices[tri.I2], m_vertices[tri.I2] + std::min(margin.second, 2.0) * glm::normalize(m_vertices[tri.I2] - m_vertices[tri.I1]));
//        else checks.emplace_back(1);

        auto op = nextOperation();

        // Show reliefs, if appropriate
//        std::cout << "XX " << op.valid << " " << op.reliefs.size() << "\n";
        for (const SectionOperation::Relief& relief : op.reliefs()) {
            for (uint32_t i = 1; i < relief.edges.size(); i++) edges.emplace_back(relief.edges[i - 1], relief.edges[i]);

            edges.emplace_back(relief.start + relief.internal * m_width,
                               relief.start + relief.internal * m_width + glm::dvec2{ 0, -0.2 });
            edges.emplace_back(relief.start + relief.internal * relief.projection(m_thickness),
                               relief.start + relief.internal * relief.projection(m_thickness) + glm::dvec2{ 0, 0.2 });
        }

//        Polygon tool(m_start, m_width, m_thickness, m_orientation);
//        edges.emplace_back(tool.border()[0], tool.border()[1]);
//        edges.emplace_back(tool.border()[1], tool.border()[2]);
//        edges.emplace_back(tool.border()[2], tool.border()[3]);
//        edges.emplace_back(tool.border()[3], tool.border()[0]);

//        Polygon collider = op.midBounds();
//        edges.emplace_back(collider.border().back(), collider.border()[0]);
//        for (uint32_t i = 0; i < collider.vertexCount() - 1; i++) {
//            edges.emplace_back(collider.border()[i], collider.border()[i + 1]);
//        }


//        auto d1 = m_vertices[tri.I0] - m_vertices[tri.I1], d2 = m_vertices[tri.I2] - m_vertices[tri.I1];

//        auto t0 = atan2(d1.y, d1.x), dt = glm::atan(d2.y, d2.x) - t0;
//        if (t0 < 0) t0 += 2 * M_PI;
//        if (dt < 0) dt += 2 * M_PI;
//        dt /= 1.0;
//
//        double t = t0;
//        for (uint32_t i = 0; i < 2; i++) {
//            edges.emplace_back(m_vertices[tri.I1], m_vertices[tri.I1] + 5.0 * glm::dvec2(cos(t), sin(t)));
//            t += dt;
//        }
    }

//    std::cout << "FLOOR: " << floor() << "\n";

    return edges;
}

void Profile::print() const
{
    std::cout << "[Profile]\n";
    m_system.print();

    Polygon::print();

    std::cout << "hull: [";
    for (uint32_t idx : m_hull) std::cout << idx << " ";

    std::cout << "]\nrem: [";
    for (const auto& rem : m_operations) std::cout << "(" << rem.first << ", " << rem.second << ") ";

    std::cout << "]\nsections: \n";
    for (const Section& section : m_sections) {
        std::cout << section.triangle.I0 << " " << section.triangle.I1 << " " << section.triangle.I2
            << " | " << section.index << " " << section.depth << " " << section.children << " | " << section.external << " " << section.skipped << "\n";
    }

    auto [xNear, xFar] = xExtents();
    auto [yNear, yFar] = yExtents();

    std::cout << "limits: (" << xNear << ", " << yNear << "), (" << xFar << ", " << yFar << ")\n";

    std::cout << "\n";
}