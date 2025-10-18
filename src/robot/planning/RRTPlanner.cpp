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

#include "robot/trajectory/Waypoint.h"

uint32_t RRTPlanner::REBUILD_THRESHOLD = 20;

RRTPlanner::RRTPlanner()
    : m_tree(new KDTree2D(2, m_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10)))
    , m_start()
    , m_goal()
    , m_first(0)
    , m_width(1.0)
    , m_height(1.0)
    , m_radius(0.5 * std::hypot(1.0, 1.0))
    , m_diagonal(0.5 * M_PI)
    , m_stepSize(1.0)
    , m_goalRadius(2.0)
    , m_iterations(0)
    , m_lastRebuild(0)
    , m_method(SampleSpace::BOX)
    , m_triBounds({ 0, 0 }, { 20, 0 }, { 20, 20 })
    , m_circBounds({}, 1.0)
    , m_solved(false)
    , m_terminus(std::numeric_limits<uint32_t>::max())
    , m_offset(0)
    , m_updateTracking(0, 0)
    , m_rng(std::random_device{}())
    , m_dist01(std::uniform_real_distribution<double>(0.0, 1.0))
    , m_xDist(std::uniform_real_distribution<double>(-20.0, 20.0))
    , m_yDist(std::uniform_real_distribution<double>(-20.0, 20.0))
    , m_tDist(std::uniform_real_distribution<double>(0.0, M_PI))
{
    m_cloud.pts = { Node(m_start) };
}

RRTPlanner::~RRTPlanner()
{
    delete m_tree;
}

bool RRTPlanner::serialize(std::ofstream& file) const
{
    Serializer::writeDVec3(file, m_start);
    Serializer::writeDVec3(file, m_goal);

    Serializer::writeUint(file, m_first);

    Serializer::writeDouble(file, m_width);
    Serializer::writeDouble(file, m_height);
    Serializer::writeDouble(file, m_radius);
    Serializer::writeDouble(file, m_diagonal);

    Serializer::writeDouble(file, m_goalRadius);
    Serializer::writeDouble(file, m_stepSize);

//    Serializer::writeUint(file, m_iterations);
//    Serializer::writeUint(file, m_lastRebuild);

    Serializer::writeUint(file, m_method == SampleSpace::BOX ? 0 : (m_method == SampleSpace::TRIANGLE ? 1 : 2));

    Serializer::writeDouble(file, m_xDist.min());
    Serializer::writeDouble(file, m_xDist.max());
    Serializer::writeDouble(file, m_yDist.min());
    Serializer::writeDouble(file, m_yDist.max());

    Serializer::writeDouble(file, m_circBounds.center.x);
    Serializer::writeDouble(file, m_circBounds.center.y);
    Serializer::writeDouble(file, m_circBounds.radius);


    std::cout << "BOUNDS: [" << m_xDist.min() << " " << m_xDist.max() << "], [" << m_yDist.min() << " " << m_yDist.max() << "]\n";

    return true;
}

bool RRTPlanner::deserialize(std::ifstream& file)
{
    delete m_tree;
    m_tree = new KDTree2D(2, m_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));

    m_start = Serializer::readDVec3(file);
    m_goal = Serializer::readDVec3(file);

    m_first = Serializer::readUint(file);

    m_width = Serializer::readDouble(file);
    m_height = Serializer::readDouble(file);
    m_radius = Serializer::readDouble(file);
    m_diagonal = Serializer::readDouble(file);

    m_goalRadius = Serializer::readDouble(file);
    m_stepSize = Serializer::readDouble(file);

//    Serializer::writeUint(file, m_iterations);
//    Serializer::writeUint(file, m_lastRebuild);

    switch (Serializer::readUint(file)) {
        case 0: m_method = SampleSpace::BOX; break;
        case 1: m_method = SampleSpace::TRIANGLE; break;
        case 2: m_method = SampleSpace::CIRCLE; break;
    }

    double min, max;

    min = Serializer::readDouble(file);
    max = Serializer::readDouble(file);
    m_xDist = std::uniform_real_distribution<double>(min, max);

    min = Serializer::readDouble(file);
    max = Serializer::readDouble(file);
    m_yDist = std::uniform_real_distribution<double>(min, max);

    m_circBounds = { { Serializer::readDouble(file), Serializer::readDouble(file) }, Serializer::readDouble(file) };

    std::cout << "BOUNDS: [" << m_xDist.min() << " " << m_xDist.max() << "], [" << m_yDist.min() << " " << m_yDist.max() << "]\n";

    return true;
}

// Restricts sampling within the box when SampleSpace == BOX
void RRTPlanner::setBounds(const glm::dvec2& min, const glm::dvec2& max)
{
    m_xDist = std::uniform_real_distribution<double>(min.x, max.x);
    m_yDist = std::uniform_real_distribution<double>(min.y, max.y);

    m_method = SampleSpace::BOX;
    updateParameters();
}

// Restricts sampling within the triangle when SampleSpace == TRIANGLE
void RRTPlanner::setBounds(const Triangle2D& bounds)
{
    m_triBounds = bounds;

    m_method = SampleSpace::TRIANGLE;
    updateParameters();
}

void RRTPlanner::setBounds(const Circle& bounds)
{
    m_circBounds = bounds;

    m_method = SampleSpace::CIRCLE;
    updateParameters();
}

// Specifies the method for sampling
void RRTPlanner::setSampler(SampleSpace method)
{
    if (m_method != method) {
        m_method = method;
        updateParameters();
    }
}

void RRTPlanner::updateParameters()
{
    switch (m_method) {
        case SampleSpace::BOX:
            m_stepSize = 0.04 * std::hypot(m_xDist.max() - m_xDist.min(), m_yDist.max() - m_yDist.min());
            break;
        case SampleSpace::TRIANGLE:
            m_stepSize = 0.04 * m_triBounds.perimeter();
            break;
        case SampleSpace::CIRCLE:
            m_stepSize = 0.02 * m_circBounds.radius;
            break;
    }

    m_goalRadius = 2.0 * m_stepSize;
}

void RRTPlanner::setStepSize(double stepSize)
{
    m_stepSize = stepSize;
}

std::tuple<glm::dvec2, glm::dvec2> RRTPlanner::getBounds() const
{
    return { { m_xDist.min(), m_yDist.min() }, { m_xDist.max(), m_yDist.max() } };
}
const Triangle2D& RRTPlanner::getTriBounds() const
{
    return m_triBounds;
}
const Circle& RRTPlanner::getCircBounds() const
{
    return m_circBounds;
}
RRTPlanner::SampleSpace RRTPlanner::getSampler() const
{
    return m_method;
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
    m_start = limit(start);
    reset();
}

// Sets the start position for path planning. The existing tree is preserved. Fails, returning false, when
// the tree can not be modified. Such failure states include: intersection at start position, unreachable start positions (from the perspective of the tree)
bool RRTPlanner::setStart(const std::vector<Polygon>& env, const glm::dvec3& start)
{
    if (m_cloud.pts.size() == 1) { // Use default set if the tree is empty
        setStart(start);
        return true;
    }

    // Confirm no intersections on new position
    auto bounds = Polygon({ start.x, start.y }, m_width, m_height, start.z);
    if (!free(env, bounds)) return false;

    m_start = limit(start);

    uint32_t newRoot = nearest(m_start);
    if (m_first != newRoot) { // Proceed if a new root is required

        // Confirm new position is accessible
        if (!free(env, m_cloud.pts[newRoot].location, m_start)) return false;

        m_first = newRoot;
        reroot();
    }

    return true;
}

// Rebases the start position to the goal position for further path planning. Applies only if there is a known path
// from the current start position to the goal, in which case it returns true
bool RRTPlanner::moveToGoal()
{
    if (m_solved) {
        m_start = m_goal;

        if (m_first != m_terminus) {
            m_first = m_terminus;
            reroot();
        }
    }

    return m_solved;
}

void RRTPlanner::reroot()
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

void RRTPlanner::updateCosts()
{
    std::deque<uint32_t> nodes;
    for (uint32_t child : m_cloud.pts[m_first].children) nodes.emplace_back(child);

    m_cloud.pts[m_first].cost = travelCost(m_cloud.pts[m_first], m_start);

    while (!nodes.empty()) {
        uint32_t parent = m_cloud.pts[nodes.front()].parent;
        m_cloud.pts[nodes.front()].cost = m_cloud.pts[parent].cost
                + travelCost(m_cloud.pts[parent], m_cloud.pts[nodes.front()].location);

        for (uint32_t child : m_cloud.pts[nodes.front()].children) nodes.emplace_back(child);

        nodes.pop_front();
    }
}

void RRTPlanner::setGoal(const glm::dvec3& goal)
{
    m_goal = limit(goal);

    resetPath();

//    std::cout << cos(goal.z - m_start.z) << " " << sin(goal.z - m_start.z) << "\n";

    // Leverage existing tree to try and find a new path
    double r = radius(m_iterations);
    auto neighbors = near(goal, r);

    for (uint32_t idx : neighbors) checkGoal(m_cloud.pts[idx]);
}

// Moves start and goal positions such that they are at least clearance from the polygon (Moved along x (width) direction)
std::tuple<double, double> RRTPlanner::clearObstacle(const Polygon& obstacle, double clearance)
{
    clearance += 0.5 * m_width;
    double ts = obstacle.clear(clearance, m_start);
    double tg = obstacle.clear(clearance, m_goal);
    return { ts, tg };
}

// Limits location.z range to 0-PI
glm::dvec3 RRTPlanner::limit(glm::dvec3 location)
{
    if (location.z < 0) location.z += 2 * M_PI;
    if (location.z > M_PI) location.z -= M_PI;

    return location;
}

// Iterates maximumIterations times, or until a faster solution is discovered, whichever comes first.
// Returns true if a path solution was discovered
bool RRTPlanner::solve(const std::vector<Polygon>& env, uint32_t maximumIterations) {
    ScopedTimer timer(m_solved ? "\033[93m[RRTPlanner] Failed to find a better solution\033[0m"
        : "\033[93m[RRTPlanner] Failed to find a solution\033[0m");

    if (tryDirect(env)) {
        timer.setMessage("\033[93m[RRTPlanner] Found a direct path. Cost: " + std::to_string(m_cloud.pts[m_terminus].cost) + "\033[0m");
        return true;
    }

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

bool RRTPlanner::tryDirect(const std::vector<Polygon>& env)
{
    if (free(env, m_start, m_goal)) {
        addNode(m_goal, travelCost(m_cloud.pts[m_first], m_goal), m_first);
        checkGoal(m_cloud.pts.back());
        return true;
    }

    return false;
}

// Tries to add a new node to the tree based on a random sample. Returns true if a node was successfully added
bool RRTPlanner::iterate(const std::vector<Polygon>& env)
{

    m_iterations++;

    auto [parent, forward] = sample();

    glm::dvec3 next = m_cloud.pts[parent].location + m_stepSize * forward;

    // Confirm new position does not result in collision with the environment
    auto hull = bounds(next);

    if (!free(env, hull) || !free(env, m_cloud.pts[parent].location, next)) return false;

    attach(env, next, parent);

    checkGoal(m_cloud.pts.back());

    return true;
}

// Iterate the specified number of times. Unlike solve(), does not end if a better solution is found
void RRTPlanner::iterate(const std::vector<Polygon>& env, uint32_t iterations)
{
    for (uint32_t i = 0; i < iterations; i++) iterate(env);
}

void RRTPlanner::attach(const std::vector<Polygon>& env, const glm::dvec3& next, uint32_t parent)
{
    double cost = m_cloud.pts[parent].cost + travelCost(m_cloud.pts[parent], next);
    double r = radius(m_iterations);

    auto neighbors = near(next, r);
    std::vector<double> distances(neighbors.size());
    for (uint32_t i = 0; i < distances.size(); i++) distances[i] = travelCost(m_cloud.pts[neighbors[i]], next);

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

//    tryFrontierInsertion(neighbors, distances);

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
    double offset = travelCost(node, m_goal);
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

const glm::dvec3& RRTPlanner::first() const
{
    return m_cloud.pts[m_first].location;
}


const glm::dvec3& RRTPlanner::goal() const
{
    return m_goal;
}

// Returns true if the cost to travel between test and the start is less than the travel between comparison and the start
bool RRTPlanner::nearer(const glm::dvec3& test, const glm::dvec3& comparison) const
{
    return travelCost(m_cloud.pts[m_first], test) < travelCost(m_cloud.pts[m_first], comparison);
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

double RRTPlanner::stepSize() const
{
    return m_stepSize;
}

// Returns the resultant polygonal bounds of the target, given the location
Polygon RRTPlanner::bounds(const glm::dvec3& location) const
{
    return { { location.x, location.y }, m_width, m_height, location.z };
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

double RRTPlanner::cost() const
{
    return m_terminus == std::numeric_limits<uint32_t>::max()
        ? std::numeric_limits<double>::max() : m_cloud.pts[m_terminus].cost;
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

std::vector<glm::dvec3> RRTPlanner::prunedPath(const std::vector<Polygon>& env) const
{
    return pruned(env, path());
}

std::vector<Waypoint> RRTPlanner::waypoints() const
{
    return waypoints(path());
}

std::vector<Waypoint> RRTPlanner::waypoints(const std::vector<Polygon>& env) const
{
    return waypoints(prunedPath(env));
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
            || free(env, path[current], path[turns.top()])    // Free path - Intermediate points are unnecessary
            ) {
            vertices.emplace_back(path[turns.top()]);
            current = turns.top();
            turns.pop();
        } else turns.emplace(0.5 * (current + turns.top()));
    }

    // Try to reduce unnecessary rotations
    for (uint32_t i = 0; i < vertices.size() - 2; i++) {
        glm::dvec3 test = { vertices[i + 1].x, vertices[i + 1].y, vertices[i].z };
        if (free(env, vertices[i], test) && free(env, test, vertices[i + 2])) vertices[i + 1] = test;
    }

    return vertices;
}

std::tuple<uint32_t, glm::dvec3> RRTPlanner::sample()
{
//    if (m_dist01(m_rng) < 0.05) //FRONTIER

    glm::dvec3 rand = random();
    uint32_t parent = selectParent(rand);

    return { parent, glm::normalize(rand - m_cloud.pts[parent].location) };
}

glm::dvec3 RRTPlanner::random()
{

    // Perform informed RRT* if a viable path has already been discovered
    if (m_solved) {
        auto delta = glm::dvec2(m_goal.x - m_start.x, m_goal.y - m_start.y);
        double min = glm::length(delta);

        double best = std::max(min, m_cloud.pts[m_terminus].cost);

        min *= 0.5;
        double a = 0.5 * best;              // semi-major axis
        double b = sqrt(a * a - min * min); // semi-minor axis

        // Sample uniformly inside a unit circle
        double theta = 2 * M_PI * m_dist01(m_rng);
        double r = m_dist01(m_rng);

        double x = r * cos(theta);
        double y = r * sin(theta);

        // Scale to ellipse
        x *= a;
        y *= b;

        // Rotate ellipse to align with start-goal line
        double angle = atan2(delta.y, delta.x);

        // Translate to midpoint of start-goal
        auto mid = 0.5 * glm::dvec2(m_goal.x + m_start.x, m_goal.y + m_start.y);
        return {
                mid.x + cos(angle) * x - sin(angle) * y,
                mid.y + sin(angle) * x + cos(angle) * y,
                m_tDist(m_rng)
        };
    }

    // Perform normal RRT* random sample selection within prescribed bounds
    switch (m_method) {
        case SampleSpace::BOX:
            return {
                    m_xDist(m_rng),
                    m_yDist(m_rng),
                    m_tDist(m_rng)
            };
        case SampleSpace::TRIANGLE:
        {
            double u = m_dist01(m_rng), v = m_dist01(m_rng), w = m_dist01(m_rng), norm = 1.0 / (u + v + w);
            glm::dvec2 pos = m_triBounds.sample(u * norm, v * norm, w * norm);

            return {
                pos.x,
                pos.y,
                m_tDist(m_rng)
            };
        }
        case SampleSpace::CIRCLE:
        {
            double theta = 2 * M_PI * m_dist01(m_rng);
            double r = m_circBounds.radius * m_dist01(m_rng);

            return {
                m_circBounds.center.x + r * cos(theta),
                m_circBounds.center.y + r * sin(theta),
                m_tDist(m_rng)
            };
        }
    }

    return {};
}


uint32_t RRTPlanner::selectParent(const glm::dvec3& position)
{
//    if (m_frontier.back().first != 0 && m_dist01(m_rng) < EXPLORATION_BIAS) { // Occasionally try from a frontier node if available
//        return selectFrontier();
//    }

    return nearest(position);
}

double RRTPlanner::travelCost(const Node& a, const glm::dvec3& b) const
{
    return distance(a.location, b) + rotation(a.location.z, b.z);
}

double RRTPlanner::distance(const glm::dvec3& a, const glm::dvec3& b)
{
    return hypot(a.x - b.x, a.y - b.y);
}

double RRTPlanner::rotation(double theta, double phi) const
{
    return std::abs(0.5 * m_stepSize * sin(phi - theta));
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

// Returns true if the free rotation without collision is possible from this node
bool RRTPlanner::freespin(const std::vector<Polygon>& env, const glm::dvec3& position) const
{
    Circle bounds({ position.x, position.y }, m_radius);
    for (const Polygon& poly : env) {
        if (Collision2D::test(poly, bounds)) return false;
    }

    return true;
}

std::tuple<glm::dvec2, glm::dvec2> RRTPlanner::minimumBounds(const std::vector<Polygon>& env)
{
    glm::dvec2 min = { std::numeric_limits<double>::max(), std::numeric_limits<double>::max() };
    glm::dvec2 max = { std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest() };

    for (const Polygon& poly : env) {
        auto [xNear, xFar] = poly.xExtents();
        if (xNear < min.x) min.x = xNear;
        if (xFar > max.x) max.x = xFar;

        auto [yNear, yFar] = poly.yExtents();
        if (yNear < min.y) min.y = yNear;
        if (yFar > max.y) max.y = yFar;
    }

    return { min, max };
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