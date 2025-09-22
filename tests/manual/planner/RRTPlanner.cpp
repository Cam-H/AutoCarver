//
// Created by cjhat on 2025-09-20.
//


#include "RRTPlanner.h"

#include "core/Timer.h"

#include "geometry/poly/Polygon.h"
#include "geometry/collision/Collision2D.h"

RRTPlanner::RRTPlanner()
    : min(-20.0, -20.0)
    , max(std::numeric_limits<double>::lowest())
    , width(1.0)
    , height(1.0)
    , step_size(2.0)
    , goal_radius(3.0)
    , max_iter(5000)
    , iterations(0)
    , m_solved(false)
    , m_terminus(0)
{

    std::cout << "LIMS: " << min.x << " " << min.y << " | " << max.x << " " << max.y << "\n";

    rng.seed(std::random_device{}());

    xDist = std::uniform_real_distribution<double>(min.x, max.x);
    yDist = std::uniform_real_distribution<double>(min.y, max.y);
}

void RRTPlanner::updateBounds(const std::vector<Polygon>& env, double margin)
{
    min = { std::numeric_limits<double>::max(), std::numeric_limits<double>::max() };
    max = { std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest() };

    for (const Polygon& poly : env) {
        auto [xNear, xFar] = poly.xExtents();
        if (xNear < min.x) min.x = xNear;
        if (xFar > max.x) max.x = xFar;

        auto [yNear, yFar] = poly.yExtents();
        if (yNear < min.y) min.y = yNear;
        if (yFar > max.y) max.y = yFar;
    }

    margin *= glm::length(max - min); // Scale margin to span

    min -= margin;
    max += margin;

    xDist = std::uniform_real_distribution<double>(min.x, max.x);
    yDist = std::uniform_real_distribution<double>(min.y, max.y);
}

void RRTPlanner::setTargetDimensions(double width, double height)
{
    this->width = width;
    this->height = height;
}

void RRTPlanner::reset()
{
    iterations = 0;
    nodes.clear();
}

void RRTPlanner::setStart(const glm::dvec3& start)
{
    reset();

    nodes = { Node({ start.x, start.y, 0.0 }) };
}
void RRTPlanner::setGoal(const glm::dvec3& goal)
{
    reset();

    nodes = { nodes[0] };
    this->goal = goal;
}

void RRTPlanner::solve(const std::vector<Polygon>& env) {
    ScopedTimer timer("[RRTPlanner] Solution: Failed. Process");

    while (iterations < max_iter) {
        iterate(env);

        iterations++;

        // Check goal
        if (distance(nodes.back(), goal) < goal_radius) {
            m_solved = true;
            m_terminus = nodes.size() - 1;

            timer.setMessage("[RRTPlanner] Solution: Goal reached at iteration: " + std::to_string(iterations) + ". Process");
            break;
        }
    }
}

bool RRTPlanner::iterate(const std::vector<Polygon>& env)
{
    glm::dvec3 rand = sample();
    uint32_t neighbor = nearest(rand);

    // Determine direction of travel and position of new node
    glm::dvec3 forward = glm::normalize(rand - nodes[neighbor].location);
    glm::dvec3 next = nodes[neighbor].location + step_size * forward;

    // Confirm new position does not result in collision with the environment
    auto bounds = Polygon({ next.x, next.y }, width, height, next.z);
//    std::cout << "xx\n";
//    std::cout << ": ->" << free(env, bounds) << "\n";
//    std::cout << ": ->" << free(env, x_nearest->location, x_new.location) << "\n";

    if (!free(env, bounds) || !free(env, nodes[neighbor].location, next)) return false;

    double r = radius(iterations);
    std::cout << iterations << " | " << nodes.size() << " " << r << " " << (nodes.size() - 1) << "\n";

    // Find best parent
    uint32_t parent = neighbor;
    double cost = nodes[neighbor].cost + distance(nodes[neighbor], next);
    for (const Node& node : nodes) {
        double dist = distance(node, next);

        if (dist < r) {
            if (free(env, node.location, next) && node.cost + dist < cost) {
                parent = &node - &nodes[0];
                cost = node.cost + dist;
            }
        }
    }

    // Add node
    nodes.emplace_back(next, cost, parent);
    uint32_t index = nodes.size() - 1;

    // Rewire neighbors
    for (uint32_t i = 0; i < nodes.size(); i++) {
        if (i == index) continue;

        double dist = distance(nodes[i], nodes[index]);
        if (dist < r) {
            if (free(env, nodes[index].location, nodes[i].location) && nodes[index].cost + dist < nodes[i].cost) {
                nodes[i].parent = index;
                nodes[i].cost = nodes[index].cost + dist;
            }
        }
    }

    return true;
}

glm::dvec3 RRTPlanner::start() const
{
    assert(!nodes.empty());
    return nodes[0].location;
}

std::vector<glm::dvec3> RRTPlanner::path() const
{
    if (!m_solved) return {};

    std::cout << nodes.size() << " " << m_terminus << " RV PATH\n";

//    for (const Node& node : nodes) {
//        std::cout << "N" << (&node - &nodes[0]) << ": " << node.parent << " " << (node.parent - &nodes[0]) << " " << node.cost << "\n";
//    }

    uint32_t index = m_terminus;
    std::vector<glm::dvec3> vertices;
    glm::dvec3 last = nodes[index].location;

    do {
        std::cout << index << " " << nodes[index].parent << " " << vertices.size() << " |" << glm::length(last - nodes[index].location) << "\n";
        last = nodes[index].location;

        vertices.emplace_back(nodes[index].location);
        index = nodes[index].parent;
    } while (index != std::numeric_limits<uint32_t>::max());

    std::cout << "br\n";

    std::reverse(vertices.begin(), vertices.end());
    return vertices;
}

glm::dvec3 RRTPlanner::sample()
{
    static std::uniform_real_distribution<double> tDist(0.0, 2 * M_PI);

    return {
        xDist(rng),
        yDist(rng),
        tDist(rng)
    };
}

double RRTPlanner::distance(const Node& a, const glm::dvec3& b)
{
    // Ignore orientation for the distance calculation
    return glm::length(glm::dvec2(a.location.x - b.x, a.location.y - b.y));
}

// Returns true if bounds does not collide with any polygons in env
bool RRTPlanner::free(const std::vector<Polygon>& env, const Polygon& bounds)
{
//    return true;

    for (const Polygon& poly : env) {
        if (Collision2D::test(poly, bounds)) return false;
    }

    return true;
}

// Returns true if there is no intersection along the path (start to end)
bool RRTPlanner::free(const std::vector<Polygon>& env, const glm::dvec3& start, const glm::dvec3& end)
{
    return true;

    // TODO check externals instead
    for (const Polygon& poly : env) {
        if (Collision2D::test(poly, start, end)) return false;
    }

    return true;
}

uint32_t RRTPlanner::nearest(const glm::dvec3& position) {
    uint32_t best = 0;

    double cost = std::numeric_limits<double>::max();
    for (const Node& node : nodes) {
        glm::dvec3 delta = node.location - position;
        double dist = glm::dot(delta, delta);

        if (dist < cost) {
            best = &node - &nodes[0];
            cost = dist;
        }
    }
    return best;
}

double RRTPlanner::radius(int iteration) {
    // Shrinks with iterations (theory: ~O((log n / n)^(1/d)))
    return step_size * 5.0 / std::sqrt(iteration + 1);
}