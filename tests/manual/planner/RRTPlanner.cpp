//
// Created by cjhat on 2025-09-20.
//


#include "RRTPlanner.h"

#include <deque>
#include <stack>

#include "core/Timer.h"

#include "geometry/primitives/Line.h"
#include "geometry/poly/Polygon.h"
#include "geometry/collision/Collision2D.h"

#include "robot/Trajectory/Waypoint.h"

uint32_t RRTPlanner::REBUILD_THRESHOLD = 20;

RRTPlanner::RRTPlanner()
    : m_tree(new KDTree2D(2, m_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10)))
    , m_start()
    , m_goal()
    , m_first(0)
    , m_min(-20.0, -20.0)
    , m_max(std::numeric_limits<double>::lowest())
    , m_width(1.0)
    , m_height(1.0)
    , m_radius(0.5 * std::hypot(1.0, 1.0))
    , m_diagonal(0.5 * M_PI)
    , m_stepSize(1.0)
    , m_goalRadius(3.0)
    , m_iterations(0)
    , m_lastRebuild(0)
    , m_solved(false)
    , m_terminus(std::numeric_limits<uint32_t>::max())
    , m_offset(0)
    , m_updateTracking(0, 0)
{
    m_rng.seed(std::random_device{}());

    m_xDist = std::uniform_real_distribution<double>(m_min.x, m_max.x);
    m_yDist = std::uniform_real_distribution<double>(m_min.y, m_max.y);

    m_cloud.pts = { Node(m_start) };
}

RRTPlanner::~RRTPlanner()
{
    delete m_tree;
}

void RRTPlanner::updateBounds(const std::vector<Polygon>& env, double margin)
{
    m_min = { std::numeric_limits<double>::max(), std::numeric_limits<double>::max() };
    m_max = { std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest() };

    for (const Polygon& poly : env) {
        auto [xNear, xFar] = poly.xExtents();
        if (xNear < m_min.x) m_min.x = xNear;
        if (xFar > m_max.x) m_max.x = xFar;

        auto [yNear, yFar] = poly.yExtents();
        if (yNear < m_min.y) m_min.y = yNear;
        if (yFar > m_max.y) m_max.y = yFar;
    }

    margin *= glm::length(m_max - m_min); // Scale margin to span

    m_min -= margin;
    m_max += margin;

    m_xDist = std::uniform_real_distribution<double>(m_min.x, m_max.x);
    m_yDist = std::uniform_real_distribution<double>(m_min.y, m_max.y);
}

void RRTPlanner::setTargetDimensions(double targetWidth, double targetHeight)
{
    m_width = targetWidth;
    m_height = targetHeight;
    m_radius = 0.5 * std::hypot(m_width, m_height);

    m_diagonal = atan2(m_height, m_width);
}

void RRTPlanner::reset()
{
    resetTree();
    resetPath();
}

void RRTPlanner::resetTree()
{
    m_iterations = 0;

    m_cloud.pts = { Node(m_start) };
    m_first = 0;

    m_tree->buildIndex();
}
void RRTPlanner::resetPath()
{
    m_solved = false;
    m_terminus = std::numeric_limits<uint32_t>::max();
    m_offset = 0;

    m_updateTracking = { 0, 0 };
}

// Sets the start position for path planning. The tree is reset when called
void RRTPlanner::setStart(const glm::dvec3& start)
{
    m_start = start;
    reset();
}

// Sets the start position for path planning. The existing tree is preserved. Fails, returning false, when
// the tree can not be modified. Such failure states include: intersection at start position, unreachable start positions (from the perspective of the tree)
bool RRTPlanner::setStart(const std::vector<Polygon>& env, const glm::dvec3& start)
{
    // Confirm no intersections on new position
    auto bounds = Polygon({ start.x, start.y }, m_width, m_height, start.z);
    if (!free(env, bounds)) return false;

    uint32_t newRoot = nearest(start);
    if (m_first != newRoot) { // Proceed if a new root is required

        // Confirm new position is accessible
        if (!free(env, m_cloud.pts[newRoot].location, start)) return false;

        m_start = start;
        m_first = newRoot;

        reroot(env);
    }

    m_tree->buildIndex();

    return true;
}

void RRTPlanner::reroot(const std::vector<Polygon>& env)
{

    uint32_t prev = m_first, target = m_cloud.pts[m_first].parent;

    // Remove old parent-child relationship to root
    removeChild(target, m_first);
    m_cloud.pts[m_first].parent = std::numeric_limits<uint32_t>::max();

    // Crawl to the old root, inverting parent-child relationships to grow from the new root
    while (m_cloud.pts[target].parent != std::numeric_limits<uint32_t>::max()) {
        uint32_t next = m_cloud.pts[target].parent;
        replaceParent(target, prev);

        prev = target;
        target = next;
    }

    // Connect old root to the chain
    m_cloud.pts[prev].children.emplace_back(target);
    m_cloud.pts[target].parent = prev;

    updateCosts();
}

// From the resultant cost [from root] at each node, calculate and overwrite with only the cost from the parent
//void RRTPlanner::propagateStepCost()
//{
//    std::deque<uint32_t> nodes;
//    for (uint32_t child : m_cloud.pts[m_first].children) nodes.emplace_back(child);
//
//    std::vector<double> costs(m_cloud.pts.size());
//    costs[m_first] = 0;
//
//    while (!nodes.empty()) {
//        uint32_t current = nodes.front();
//        nodes.pop_front();
//
//        costs[current] = m_cloud.pts[m_first].cost;
//        m_cloud.pts[m_first].cost -= costs[m_cloud.pts[current].parent];
//
//        for (uint32_t child : m_cloud.pts[current].children) nodes.emplace_back(child);
//    }
//}
//
//// Update costs after a reroot
//void RRTPlanner::propagateCost()
//{
//
//    // Store index and cost to travel to parent [from root] for each node
//    std::deque<std::pair<uint32_t, double>> nodes = {
//            { m_first, distance(m_cloud.pts[m_first], m_start) }
//    };
//
//    // Update costs for all nodes
//    while (!nodes.empty()) {
//        auto current = nodes.front();
//        nodes.pop_front();
//
//        m_cloud.pts[current.first].cost += current.second;
//
//        for (uint32_t child : m_cloud.pts[current.first].children) {
//            nodes.emplace_back(child, m_cloud.pts[child].cost);
//
//            // Determine cost of travel between nodes (Would be negative for inverted child-parent links)
//            double dist = std::abs(current.second - m_cloud.pts[child].cost);
//            m_cloud.pts[child].cost = m_cloud.pts[current.first].cost + dist;
//        }
//    }
//}

void RRTPlanner::updateCosts()
{
    std::deque<uint32_t> nodes;
    for (uint32_t child : m_cloud.pts[m_first].children) nodes.emplace_back(child);

    m_cloud.pts[m_first].cost = distance(m_cloud.pts[m_first], m_start);

    while (!nodes.empty()) {
        uint32_t parent = m_cloud.pts[nodes.front()].parent;
        m_cloud.pts[nodes.front()].cost = m_cloud.pts[parent].cost
                + distance(m_cloud.pts[parent], m_cloud.pts[nodes.front()]);

        for (uint32_t child : m_cloud.pts[nodes.front()].children) nodes.emplace_back(child);

        nodes.pop_front();
    }
}

void RRTPlanner::setGoal(const glm::dvec3& goal)
{
    m_goal = goal;
    resetPath();

    // Leverage existing tree to try and find a new path
    double r = radius(m_iterations);
    auto neighbors = near(goal, r);

    for (uint32_t idx : neighbors) checkGoal(m_cloud.pts[idx]);
}

// Iterates maximumIterations times, or until a faster solution is discovered, whichever comes first.
// Returns true if a path solution was discovered
bool RRTPlanner::solve(const std::vector<Polygon>& env, uint32_t maximumIterations) {
    ScopedTimer timer(m_solved ? "[RRTPlanner] Failed to find a better solution"
        : "[RRTPlanner] Failed to find a solution");

    maximumIterations += m_iterations; // Perform up to maximumIterations on top of what has already been done
    while (m_iterations < maximumIterations) {
        iterate(env);

        // Check goal
        if (m_solved && m_terminus == m_cloud.pts.size() - 1) {
            timer.setMessage("[RRTPlanner] Found a solution at iteration: " + std::to_string(m_iterations) + ". Cost: " + std::to_string(m_cloud.pts[m_terminus].cost));
            return true;
        }
    }

    return false;
}

// Tries to add a new node to the tree based on a random sample. Returns true if a node was successfully added
bool RRTPlanner::iterate(const std::vector<Polygon>& env)
{

    m_iterations++;

    glm::dvec3 rand = sample();
    uint32_t neighbor = nearest(rand);

    // Determine direction of travel and position of new node
    glm::dvec3 forward = glm::normalize(rand - m_cloud.pts[neighbor].location);
    glm::dvec3 next = m_cloud.pts[neighbor].location + m_stepSize * forward;

    // Confirm new position does not result in collision with the environment
    auto bounds = Polygon({ next.x, next.y }, m_width, m_height, next.z);

    if (!free(env, bounds) || !free(env, m_cloud.pts[neighbor].location, next)) return false;

    attach(env, next, neighbor);

    checkGoal(m_cloud.pts.back());

    return true;
}

void RRTPlanner::attach(const std::vector<Polygon>& env, const glm::dvec3& next, uint32_t parent)
{
    double cost = m_cloud.pts[parent].cost + distance(m_cloud.pts[parent], next);
    double r = radius(m_iterations);

    auto neighbors = near(next, r);
    std::vector<double> distances(neighbors.size());
    for (uint32_t i = 0; i < distances.size(); i++) distances[i] = distance(m_cloud.pts[neighbors[i]], next);

    // Find best parent
    for (uint32_t i = 0; i < neighbors.size(); i++) {
        double dist = m_cloud.pts[neighbors[i]].cost + distances[i];
        if (dist < cost && free(env, m_cloud.pts[neighbors[i]].location, next)) {
            parent = neighbors[i];
            cost = dist;
        }
    }

    // Add node to the best parent
    addNode(next, cost, parent);
    uint32_t index = m_cloud.pts.size() - 1;

    // Rewire neighbors
    for (uint32_t i = 0; i < neighbors.size(); i++) {
        double dist = cost + distances[i];
        if (dist < m_cloud.pts[neighbors[i]].cost && free(env, next, m_cloud.pts[neighbors[i]].location)) {
            propagateCost(neighbors[i], dist - m_cloud.pts[neighbors[i]].cost);
            replaceParent(neighbors[i], index);
        }
    }
}

void RRTPlanner::addNode(const glm::dvec3& vertex, double cost, uint32_t parent)
{
    m_cloud.pts[parent].children.emplace_back(m_cloud.pts.size());
    m_cloud.pts.emplace_back(vertex, cost, parent);
    if (m_lastRebuild++ > REBUILD_THRESHOLD) {
        m_tree->buildIndex();
        m_lastRebuild = 0;
    }
}

void RRTPlanner::replaceParent(uint32_t index, uint32_t newParent)
{
    removeChild(m_cloud.pts[index].parent, index);

    // Create link to and from new parent
    m_cloud.pts[index].parent = newParent;
    m_cloud.pts[newParent].children.emplace_back(index);
}

// Remove link from old parent
void RRTPlanner::removeChild(uint32_t parent, uint32_t child)
{
    auto it = std::find(m_cloud.pts[parent].children.begin(), m_cloud.pts[parent].children.end(), child);
    assert(it != m_cloud.pts[parent].children.end());
    m_cloud.pts[parent].children.erase(it);
}

// Applies delta to costs of specified node (by index) and all of its children
void RRTPlanner::propagateCost(uint32_t index, double delta)
{
    std::deque<uint32_t> nodes = { index };
    while (!nodes.empty()) {
        uint32_t current = nodes.front();
        nodes.pop_front();

        for (uint32_t child : m_cloud.pts[current].children) nodes.emplace_back(child);
        m_cloud.pts[current].cost += delta;
    }
}

// Update path if the specified node is a faster solution
void RRTPlanner::checkGoal(Node& node)
{
    double offset = distance(node, m_goal);
    if (offset < m_goalRadius) {
        if (!m_solved || m_cloud.pts[m_terminus].cost + m_offset > node.cost + offset) {
            m_terminus = &node - &m_cloud.pts[0];
            m_offset = offset;
            m_solved = true;
        }
    }
}

const glm::dvec3& RRTPlanner::start() const
{
    return m_start;
}

const glm::dvec3& RRTPlanner::goal() const
{
    return m_goal;
}

double RRTPlanner::width() const
{
    return m_width;
}
double RRTPlanner::height() const
{
    return m_height;
}
double RRTPlanner::radius() const
{
    return m_radius;
}

double RRTPlanner::diagonal() const
{
    return m_diagonal;
}

const std::vector<Node>& RRTPlanner::nodes() const
{
    return m_cloud.pts;
}

bool RRTPlanner::solved() const
{
    return m_solved && m_terminus != std::numeric_limits<uint32_t>::max();
}

uint32_t RRTPlanner::terminus() const
{
    return m_terminus;
}

bool RRTPlanner::solutionImproved() const
{
    return m_solved
        && (m_updateTracking.first < m_terminus
        || m_updateTracking.second > m_cloud.pts[m_terminus].cost + 1e-12);
}

void RRTPlanner::clearUpdateFlag()
{
    if (m_solved) m_updateTracking = { m_terminus, m_cloud.pts[m_terminus].cost };
}

uint32_t RRTPlanner::iterations() const
{
    return m_iterations;
}

const std::pair<uint32_t, double>& RRTPlanner::lastSolution() const
{
    return m_updateTracking;
}

// Returns the locations of all nodes along the optimal path discovered, from the start to the goal
std::vector<glm::dvec3> RRTPlanner::path() const
{
    if (!m_solved) return {};

    uint32_t index = m_terminus;
    std::vector<glm::dvec3> vertices;

    while (index != std::numeric_limits<uint32_t>::max()) {
        vertices.emplace_back(m_cloud.pts[index].location);
        index = m_cloud.pts[index].parent;

    }

    if (m_first != 0) vertices.emplace_back(m_start);
    std::reverse(vertices.begin(), vertices.end());
    vertices.emplace_back(m_goal);

    return vertices;
}

std::vector<Waypoint> RRTPlanner::waypoints() const
{
    return waypoints(path());
}

std::vector<Waypoint> RRTPlanner::waypoints(const std::vector<Polygon>& env) const
{
    return waypoints(pruned(env, path()));
}

std::vector<Waypoint> RRTPlanner::waypoints(const std::vector<glm::dvec3>& path)
{
    std::vector<Waypoint> wps(path.size());
    for (uint32_t i = 0; i < wps.size(); i++) {
        wps[i] = Waypoint({ path[i].x, path[i].y, path[i].z }, true);
    }

    return wps;
}

std::vector<glm::dvec3> RRTPlanner::pruned(const std::vector<Polygon>& env, const std::vector<glm::dvec3>& path) const
{
    if (path.empty()) return {};

    std::vector<glm::dvec3> vertices = { path[0] };

    std::stack<uint32_t> turns;
    turns.emplace(path.size() - 1);

    uint32_t current = 0;

    while (!turns.empty()) {
        if (turns.top() == current + 1                        // Section can not be further subdivided
            || free(env, path[current], path[turns.top()])) { // Free path - Intermediate points are unnecessary
            vertices.emplace_back(path[turns.top()]);
            current = turns.top();
            turns.pop();
        } else turns.emplace(0.5 * (current + turns.top()));
    }

    return vertices;
}

glm::dvec3 RRTPlanner::sample()
{
    static std::uniform_real_distribution<double> tDist(0.0, 2 * M_PI);

    return {
        m_xDist(m_rng),
        m_yDist(m_rng),
        tDist(m_rng)
    };
}

double RRTPlanner::distance(const Node& a, const glm::dvec3& b)
{
//    glm::dvec2 delta = { a.location.x - b.x, a.location.y - b.y };
    auto delta = a.location - b;
    return glm::dot(delta, delta);
}

// Returns true if bounds does not collide with any polygons in env
bool RRTPlanner::free(const std::vector<Polygon>& env, const Polygon& bounds) const
{
    for (const Polygon& poly : env) {
        if (Collision2D::test(poly, bounds)) return false;
    }

    return true;
}

// Returns true if there is no intersection along the path (start to end)
bool RRTPlanner::free(const std::vector<Polygon>& env, const glm::dvec3& start, const glm::dvec3& end) const
{
    // TODO NOTE: Collision check may introduce false negatives (Profile is good but a new polygon would need to be created to provide a perfect test)
    auto [axis, r] = profile(start, end);
    glm::dvec2 norm = { axis.y, -axis.x };

    Line top({ start.x, start.y }, { end.x, end.y }), bot = top;
    top.translate(norm * r);
    bot.translate(-norm * r);

    for (const Polygon& poly : env) {
        if (Collision2D::test(poly, top) || Collision2D::test(poly, bot)) return false;
    }

    return true;
}

std::tuple<glm::dvec2, double> RRTPlanner::profile(const glm::dvec3& start, const glm::dvec3& end) const
{
    glm::dvec2 delta = { end.x - start.x, end.y - start.y };

    double sAngle = start.z - atan2(delta.y, delta.x);
    double diff = end.z - start.z, mod = 1.0;

    if (std::abs(diff) < M_PI) {
        double gAngle = sAngle + diff, ss = sin(sAngle), sg = sin(sAngle + diff);
        if ((ss * sg > 0.0) == (tan(sAngle) * tan(gAngle) > 0.0)) {
            std::array<double, 4> set = {
                    std::abs(sin(sAngle - m_diagonal)),
                    std::abs(sin(sAngle + m_diagonal)),
                    std::abs(sin(gAngle - m_diagonal)),
                    std::abs(sin(gAngle + m_diagonal))
            };

            auto it = std::max_element(set.begin(), set.end(), [](double lhs, double rhs){
                return lhs < rhs;
            });

            mod = *it;
        }
    }

    return { glm::normalize(delta), m_radius * mod };
}

uint32_t RRTPlanner::nearest(const glm::dvec3& position) const
{
    double query[2] = { position.x, position.y }, distance;
    size_t index;

    nanoflann::KNNResultSet<double> set(1);
    set.init(&index, &distance);

    m_tree->findNeighbors(set, query, nanoflann::SearchParameters(10));
    return index;
}

std::vector<uint32_t> RRTPlanner::near(const glm::dvec3& position, double radius) const
{
    double query[2] = { position.x, position.y };
    std::vector<nanoflann::ResultItem<uint32_t, double>> matches;

    nanoflann::SearchParameters params;
    const size_t nMatches = m_tree->radiusSearch(query, radius * radius, matches, params);

    std::vector<uint32_t> neighbors;
    neighbors.reserve(matches.size());

    for (const auto& match : matches) neighbors.emplace_back(match.first);

    return neighbors;
}

// Shrinks with iterations (theory: ~O((log n / n)^(1/d)))
double RRTPlanner::radius(int iteration) {
    return m_stepSize * 5.0 / sqrt(log(iteration + 1) / iteration);
}

void RRTPlanner::print() const
{
    std::cout << "[RRTPlanner] nodes: " << m_cloud.pts.size() << ", iterations: " << m_iterations << ", solved: "
        << m_solved << "\n";

    if (m_solved) {
        std::cout << "Solution: " << m_terminus << ", cost: " << m_cloud.pts[m_terminus].cost << "\n";
    }
}