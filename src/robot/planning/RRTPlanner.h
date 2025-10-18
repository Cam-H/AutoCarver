//
// Created by cjhat on 2025-09-20.
//

#ifndef AUTOCARVER_RRTPLANNER_H
#define AUTOCARVER_RRTPLANNER_H

#include "glm.hpp"

#include <iostream>
#include <random>

#include "nanoflann.hpp"

class Waypoint;

#include "fileIO/Serializable.h"

#include "geometry/primitives/Triangle.h"
#include "geometry/primitives/Circle.h"
#include "geometry/primitives/Ray.h"

#include "geometry/poly/Polygon.h"

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
    std::vector<uint32_t> children; // Indices of child nodes
};

struct NodeCloud {
    std::vector<Node> pts;

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        switch (dim) {
            case 0: return pts[idx].location.x;
            case 1: return pts[idx].location.y;
            case 2: return pts[idx].location.z;
            default: throw std::runtime_error("[RRTPlanner:NodeCloud] Invalid dimension specified");
        }

        return 0.0;
    }

    template<class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};

using KDTree2D = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, NodeCloud>, NodeCloud, 2>;

// Resolves 2D + rotation path planning using the RRT* algorithm
class RRTPlanner : public Serializable {
public:

    enum class SampleSpace {
        BOX = 0, TRIANGLE, CIRCLE
    };

//    RRTPlanner(const glm::dvec3& start, const glm::dvec3& goal);

    RRTPlanner();
    ~RRTPlanner();

    bool serialize(std::ofstream& file) const override;
    bool deserialize(std::ifstream& file) override;

    void setBounds(const glm::dvec2& min, const glm::dvec2& max);
    void setBounds(const Triangle2D& bounds);
    void setBounds(const Circle& bounds);

    void setSampler(SampleSpace method);
    void updateParameters();

    void setStepSize(double stepSize);

    [[nodiscard]] std::tuple<glm::dvec2, glm::dvec2> getBounds() const;
    [[nodiscard]] const Triangle2D& getTriBounds() const;
    [[nodiscard]] const Circle& getCircBounds() const;

    [[nodiscard]] SampleSpace getSampler() const;

    void setTargetDimensions(double targetWidth, double targetHeight);

    void setStart(const glm::dvec3& start);
    bool setStart(const std::vector<Polygon>& env, const glm::dvec3& start);

    bool moveToGoal();

    void setGoal(const glm::dvec3& goal);

    std::tuple<double, double> clearObstacle(const Polygon& obstacle, double clearance);

    void reset();

    bool solve(const std::vector<Polygon>& env, uint32_t maximumIterations);

    bool iterate(const std::vector<Polygon>& env);
    void iterate(const std::vector<Polygon>& env, uint32_t iterations);

    bool tryDirect(const std::vector<Polygon>& env);

    void clearUpdateFlag();

    [[nodiscard]] const glm::dvec3& start() const;
    [[nodiscard]] const glm::dvec3& first() const;

    [[nodiscard]] const glm::dvec3& goal() const;

    [[nodiscard]] bool nearer(const glm::dvec3& test, const glm::dvec3& comparison) const;

    [[nodiscard]] static glm::dvec3 limit(glm::dvec3 location);

    [[nodiscard]] double width() const;
    [[nodiscard]] double height() const;
    [[nodiscard]] double radius() const;

    [[nodiscard]] double diagonal() const;

    [[nodiscard]] double stepSize() const;

    [[nodiscard]] Polygon bounds(const glm::dvec3& location) const;

    [[nodiscard]] std::tuple<glm::dvec2, glm::dvec2> minimumBounds(const std::vector<Polygon>& env);

    [[nodiscard]] std::tuple<glm::dvec2, double> profile(const glm::dvec3& start, const glm::dvec3& end) const;

    [[nodiscard]] const std::vector<Node>& nodes() const;

    [[nodiscard]] bool solved() const;
    [[nodiscard]] uint32_t terminus() const;

    [[nodiscard]] bool solutionImproved() const;

    [[nodiscard]] uint32_t iterations() const;

    [[nodiscard]] double cost() const;

    [[nodiscard]] const std::pair<uint32_t, double>& lastSolution() const;

    [[nodiscard]] std::vector<glm::dvec3> path() const;
    [[nodiscard]] std::vector<glm::dvec3> prunedPath(const std::vector<Polygon>& env) const;

    [[nodiscard]] std::vector<Waypoint> waypoints() const;
    [[nodiscard]] std::vector<Waypoint> waypoints(const std::vector<Polygon>& env) const;

    void print() const;

private:

    void resetTree();
    void resetPath();

    void reroot();

    void updateCosts();

    void attach(const std::vector<Polygon>& env, const glm::dvec3& next, uint32_t parent);
    void addNode(const glm::dvec3& vertex, double cost, uint32_t parent);

    void replaceParent(uint32_t index, uint32_t newParent);
    void removeChild(uint32_t parent, uint32_t child);

    void propagateCost(uint32_t index, double delta);

    void checkGoal(Node& node);

    [[nodiscard]] std::vector<glm::dvec3> pruned(const std::vector<Polygon>& env, const std::vector<glm::dvec3>& path) const;
    static std::vector<Waypoint> waypoints(const std::vector<glm::dvec3>& path);

    [[nodiscard]] std::tuple<uint32_t, glm::dvec3> sample();
    [[nodiscard]] inline glm::dvec3 random();

    [[nodiscard]] uint32_t selectParent(const glm::dvec3& position);

    [[nodiscard]] inline double travelCost(const Node& a, const glm::dvec3& b) const;

    [[nodiscard]] static inline double distance(const glm::dvec3& a, const glm::dvec3& b) ;
    [[nodiscard]] inline double rotation(double theta, double phi) const;

    [[nodiscard]] bool free(const std::vector<Polygon>& env, const Polygon& bounds) const;
    [[nodiscard]] bool free(const std::vector<Polygon>& env, const glm::dvec3& start, const glm::dvec3& end) const;

    [[nodiscard]] bool freespin(const std::vector<Polygon>& env, const glm::dvec3& position) const;

    [[nodiscard]] uint32_t nearest(const glm::dvec3& position) const;
    [[nodiscard]] std::vector<uint32_t> near(const glm::dvec3& position, double radius) const;


    [[nodiscard]] double radius(int iteration);

    static uint32_t REBUILD_THRESHOLD;

private:

    NodeCloud m_cloud;
    KDTree2D* m_tree;

    glm::dvec3 m_start;
    glm::dvec3 m_goal;

    uint32_t m_first; // Index of the start node (Usually 0 but can be different if start position is changed)

    double m_width;
    double m_height;
    double m_radius; // Length of hypotenuse

    double m_diagonal; // Angle formed with x-axis and target diagonal

    double m_stepSize, m_goalRadius;

    uint32_t m_iterations;
    uint32_t m_lastRebuild;

    std::mt19937 m_rng;
    std::uniform_real_distribution<double> m_dist01;

    std::uniform_real_distribution<double> m_xDist;
    std::uniform_real_distribution<double> m_yDist;
    std::uniform_real_distribution<double> m_tDist;

    SampleSpace m_method;
    Triangle2D m_triBounds;
    Circle m_circBounds;

    bool m_solved; // Whether a path was found
    uint32_t m_terminus; // Index of the end of the best path discovered
    double m_offset; // Cost from end of path to the goal

    std::pair<uint32_t, double> m_updateTracking; // Last recorded index and cost of solution
};



#endif //AUTOCARVER_RRTPLANNER_H
