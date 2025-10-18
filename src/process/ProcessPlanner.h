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
            , started(false)
            , skip(false) {}


    std::shared_ptr<Trajectory> trajectory;
    std::shared_ptr<Robot> target;
    bool started;
    bool skip;

    std::deque<Cut> cuts;
};

class ProcessPlanner {
public:

    struct Report {
        Report()
            : success(false)
            , appliedConvexTrim(false)
            , ctStepCount(0)
            , appliedSilhouetteRefinement(false)
            , srStepCount(0)
            , appliedFeatureRefinement(false)
            , frStepCount(0)
            {}

        bool success;

        bool appliedConvexTrim;
        uint32_t ctStepCount;

        bool appliedSilhouetteRefinement;
        std::vector<std::pair<double, uint32_t>> srOrientations;
        uint32_t srStepCount;

        bool appliedFeatureRefinement;
        uint32_t frStepCount;

        void print() const;
    };

    ProcessPlanner();
    explicit ProcessPlanner(ProcessConfiguration  configuration);

    Report plan(const std::shared_ptr<Sculpture>& target, const std::shared_ptr<Robot>& sculptor, const std::shared_ptr<Robot>& tt);

    ProcessConfiguration& getConfiguration();
    [[nodiscard]] const ProcessConfiguration& getConfiguration() const;

    [[nodiscard]] const std::deque<Action>& actions() const;

private:

    void complete();

    void planConvexTrim();
    [[nodiscard]] std::vector<std::pair<Plane, std::vector<glm::dvec3>>> convexTrimOrder() const;
    [[nodiscard]] std::vector<Plane> volumeOptimizedOrder(const ConvexHull& base, const std::vector<Plane>& planes) const;

    std::vector<Action> planConvexTrim(const std::pair<Plane, std::vector<glm::dvec3>>& step);

    void planSilhouetteRefinement();
    void planSilhouetteTrim();

    std::vector<Action> planProfile(Profile& profile);

    bool planSection(const Profile& profile, std::vector<Action>& actions);

    bool planTranslation(const glm::dvec3& translation, Action& action);
    bool planSequence(const Sequence& sequence, Action& action);
    bool planApproach(const Axis3D& axes, const std::vector<std::pair<glm::dvec3, double>>& approach, Action& action);
    Action planPath2D(const Profile& profile, const std::vector<glm::dvec3>& path2D);

    Action planRetraction(const Waypoint& start);

    bool planBlindCut(const Pose& pose, double depth, Action& action);
    bool planMill(const Pose& pose, const glm::dvec3& normal, const glm::dvec3& travel, double depth, Action& action);
//    bool planReliefCuts(const Pose& pose, double depth, Action& action);

    void planFeatureRefinement();

    Action planTurntableAlignment(double start, double end);
    Action planRoboticSection(const std::shared_ptr<CompositeTrajectory>& trajectory);


    std::shared_ptr<CompositeTrajectory> preparePlanarTrajectory(const std::vector<glm::dvec3>& border, const Axis3D& axes);
    std::shared_ptr<CompositeTrajectory> prepareThroughCut(Pose pose, double depth, double runup, const glm::dvec3& off);

    [[nodiscard]] bool withinActionLimit(uint32_t buffered) const;

    [[nodiscard]] glm::dvec3 projected2D(const Waypoint& wp, const Profile& profile) const;

private:

    ProcessConfiguration m_config;
    Report m_report;

    std::shared_ptr<Sculpture> sculpture;

    std::shared_ptr<Robot> robot;
    std::shared_ptr<Robot> turntable;

    Waypoint m_latestRobotCommand;

    std::deque<Action> m_actions;

    std::deque<Cut> m_cuts; // Temporary container for cuts when preparing action

};


#endif //AUTOCARVER_PROCESSPLANNER_H
