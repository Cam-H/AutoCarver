//
// Created by cjhat on 2025-09-20.
//

#ifndef AUTOCARVER_RRTPLANNER_H
#define AUTOCARVER_RRTPLANNER_H

#include <glm.hpp>

#include <iostream>
#include <random>

class Polygon;

struct Node {
    explicit Node(const glm::dvec3& location)
        : Node(location, 0, std::numeric_limits<uint32_t>::max()) {}

    Node(const glm::dvec3& location, double cost, uint32_t parent)
            : location(location)
            , cost(cost)
            , parent(parent) {}

    glm::dvec3 location;

    double cost;
    uint32_t parent; // Index of parent node
};

// Resolves 2D + rotation path planning using the RRT* algorithm
class RRTPlanner {
public:

//    RRTPlanner(const glm::dvec3& start, const glm::dvec3& goal);

    RRTPlanner();

    void updateBounds(const std::vector<Polygon>& env, double margin = 0.01);
    void setTargetDimensions(double width, double height);

    void setStart(const glm::dvec3& start);
    void setGoal(const glm::dvec3& goal);

    void reset();

    void solve(const std::vector<Polygon>& env);

    bool iterate(const std::vector<Polygon>& env);

    [[nodiscard]] glm::dvec3 start() const;
    [[nodiscard]] std::vector<glm::dvec3> path() const;

private:

    [[nodiscard]] glm::dvec3 sample();

    [[nodiscard]] static inline double distance(const Node& a, const Node& b) { return distance(a, b.location); }
    [[nodiscard]] static double distance(const Node& a, const glm::dvec3& b);

    [[nodiscard]] bool free(const std::vector<Polygon>& env, const Polygon& bounds);
    [[nodiscard]] bool free(const std::vector<Polygon>& env, const glm::dvec3& start, const glm::dvec3& end);

    uint32_t nearest(const glm::dvec3& position);

    [[nodiscard]] double radius(int iteration);

public:
    std::vector<Node> nodes;
    glm::dvec2 min, max;

    double width;
    double height;

    double step_size, goal_radius;
    uint32_t iterations, max_iter;

    glm::dvec3 goal;

    std::mt19937 rng;
    std::uniform_real_distribution<double> xDist;
    std::uniform_real_distribution<double> yDist;

private:

    bool m_solved; // Whether a path was found
    uint32_t m_terminus; // Index of the end of the best path discovered


};



#endif //AUTOCARVER_RRTPLANNER_H
