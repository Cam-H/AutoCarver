//
// Created by cjhat on 2025-09-08.
//

#ifndef AUTOCARVER_PROCESSPLANNER_H
#define AUTOCARVER_PROCESSPLANNER_H

#include <deque>
#include <cstdint>
#include <memory>

#include <glm.hpp>

#include "robot/Pose.h"

#include "ProcessConfiguration.h"

class Scene;
class Sculpture;

class Profile;
class SectionOperation;
class Sequence;

class Robot;
class Trajectory;
class CompositeTrajectory;

class ConvexHull;
class Plane;

struct Cut {
    Cut(const Pose& pose, double theta, double ts, double tf, uint32_t index)
            : origin(pose.position)
            , normal(pose.axes.yAxis)
            , axis(pose.axes.xAxis)
            , theta(theta)
            , ts(ts)
            , tf(tf)
            , index(index) {}
    Cut(const glm::dvec3& origin, const glm::dvec3& normal, const glm::dvec3& axis, double theta, double ts, double tf, uint32_t index)
            : origin(origin)
            , normal(normal)
            , axis(axis)
            , theta(theta)
            , ts(ts)
            , tf(tf)
            , index(index) {}

    glm::dvec3 origin; // Initial position before beginning cut
    glm::dvec3 normal; // Normal to the cutting direction
    glm::dvec3 axis; // Axis of travel
    double theta; // Angle between blade and normal of cut surface
    double ts;
    double tf;
    uint32_t index;
};

struct Action {
    Action (const std::shared_ptr<Trajectory>& trajectory, const std::shared_ptr<Robot>& robot)
            : trajectory(trajectory)
            , target(robot)
            , started(false) {}


    std::shared_ptr<Trajectory> trajectory;
    std::shared_ptr<Robot> target;
    bool started;

    std::deque<Cut> cuts;
};

class ProcessPlanner {
public:

    ProcessPlanner();

    bool plan(const std::shared_ptr<Sculpture>& target, const std::shared_ptr<Robot>& sculptor, const std::shared_ptr<Robot>& tt);

    ProcessConfiguration& getConfiguration();
    [[nodiscard]] const ProcessConfiguration& getConfiguration() const;

    [[nodiscard]] const std::deque<Action>& actions() const;

private:

    void planConvexTrim();
    std::vector<std::pair<Plane, std::vector<glm::dvec3>>> convexTrimOrder() const;
    std::vector<Plane> volumeOptimizedOrder(const ConvexHull& base, const std::vector<Plane>& planes) const;

    std::vector<Action> planConvexTrim(const std::pair<Plane, std::vector<glm::dvec3>>& step);

    void planOutlineRefinement(double stepDg);
    std::vector<Action> planOutlineRefinement(Profile& profile);
    Action planOutlineRefinement(const Profile& profile, const SectionOperation& operation);

    bool planTranslation(const glm::dvec3& translation, Action& action);
    bool planSequence(const Sequence& sequence, Action& action);

    bool planBlindCut(const Pose& pose, double depth, Action& action);
    bool planMill(const Pose& pose, const glm::dvec3& normal, const glm::dvec3& travel, double depth, Action& action);
//    bool planReliefCuts(const Pose& pose, double depth, Action& action);

    void planFeatureRefinement();

    Action planTurntableAlignment(double start, double end);
    Action planRoboticSection(const std::shared_ptr<CompositeTrajectory>& trajectory);


    std::shared_ptr<CompositeTrajectory> preparePlanarTrajectory(const std::vector<glm::dvec3>& border, const glm::dvec3& normal);
    std::shared_ptr<CompositeTrajectory> prepareThroughCut(Pose pose, double depth, double runup, const glm::dvec3& off);

    [[nodiscard]] bool withinActionLimit() const;

    [[nodiscard]] inline bool nearest(const Sequence& test, const Sequence& comparison);

private:

    ProcessConfiguration m_config;

    std::shared_ptr<Sculpture> sculpture;

    std::shared_ptr<Robot> robot;
    std::shared_ptr<Robot> turntable;

    Waypoint m_latestRobotCommand;

    std::deque<Action> m_actions;

    std::deque<Cut> m_cuts; // Temporary container for cuts when preparing action

};


#endif //AUTOCARVER_PROCESSPLANNER_H
