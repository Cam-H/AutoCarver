//
// Created by Cam on 2024-11-12.
//

#ifndef AUTOCARVER_SCULPTPROCESS_H
#define AUTOCARVER_SCULPTPROCESS_H

#include "core/Scene.h"

#include <vector>
#include <deque>

#include "geometry/VertexArray.h"
#include "geometry/Mesh.h"
#include "core/Sculpture.h"

#include "geometry/curves/Interpolator.h"
#include "robot/Pose.h"
#include "robot/trajectory/Waypoint.h"
#include "robot/trajectory/CompositeTrajectory.h"

#include "ProcessConfiguration.h"
#include "Sequence.h"

class Profile;
class SectionOperation;

class SculptProcess : public Scene {
public:

    explicit SculptProcess(const std::shared_ptr<Mesh>& model);
    ~SculptProcess();

    void setTarget(const std::shared_ptr<Mesh>& mesh);
    void loadSculpture(const std::string& filename);

    void reset();

    void plan();

    void blind(const Pose& pose, double depth);
    void mill(const Pose& pose, const glm::dvec3& normal, const glm::dvec3& travel, double depth);

    void proceed();
    void skip();

    void step(double delta) override;

    void setSculptingRobot(const std::shared_ptr<Robot>& robot);

    void alignToFace(uint32_t faceIdx);

    [[nodiscard]] bool simulationComplete() const;
    [[nodiscard]] bool simulationIdle() const;
    [[nodiscard]] bool simulationActive() const;

    ProcessConfiguration& getConfiguration();

    const std::shared_ptr<Sculpture>& getSculpture() const;
    const std::shared_ptr<Body>& getModel() const;
    const std::shared_ptr<Debris>& getDebris() const;

    const std::shared_ptr<Robot>& getSculptor() const;
    const std::shared_ptr<Robot>& getTurntable() const;

    const glm::dvec3& forward() const;

//    std::shared_ptr<Mesh> sculpture();

private:

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

    void readySculpture(const std::shared_ptr<Sculpture>& sculpture);

    void prepareTurntable();
    void attachSculpture();

    [[nodiscard]] bool withinActionLimit() const;

    [[nodiscard]] Waypoint alignedToFaceWP(const std::vector<glm::dvec3>& border, const glm::dvec3& normal) const;
    [[nodiscard]] Waypoint alignedToFaceWP(const Axis3D& axes, const std::vector<glm::dvec3>& border) const;
    [[nodiscard]] Waypoint alignedToVertexWP(const Axis3D& axes, const glm::dvec3& vertex) const;
    [[nodiscard]] Waypoint alignedToVertexWP(const Axis3D& axes, const glm::dvec3& vertex, const glm::dvec3& tryAxis, double tryLimit) const;

    [[nodiscard]] glm::dvec3 alignedToBlade(const Axis3D& axes, const glm::dvec3& vertex);

    [[nodiscard]] inline glm::dvec3 bladeOffset(const Axis3D& axes, const glm::dvec3& vertex) const;
    [[nodiscard]] inline glm::dvec3 bladeCenterOffset(const glm::dvec3& zAxis, const glm::dvec3& vertex) const;
    [[nodiscard]] inline glm::dvec3 bladeThicknessOffset(const Axis3D& axes) const;

    [[nodiscard]] inline bool nearest(const Sequence& test, const Sequence& comparison);

    static void toWorldSpace(glm::dvec3& normal, const glm::dquat& rotation);
    void toWorldSpace(std::vector<glm::dvec3>& border, const glm::dquat& rotation) const;

    [[nodiscard]] glm::dvec3 poseAdjustedVertex(const Axis3D& axes, const glm::dvec3& vertex) const;


    void planConvexTrim();
    std::vector<Plane> orderConvexTrim(const std::vector<Plane>& cuts);
    std::vector<Action> planConvexTrim(const ConvexHull& hull, const Plane& plane);

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

    bool testTurntableCollision(const std::shared_ptr<Trajectory>& ttTrajectory);
    Action planTurntableClearance();
    Action planApproach(const Waypoint& destination);



    std::shared_ptr<Trajectory> prepareApproach(const Waypoint& destination);

    std::shared_ptr<CompositeTrajectory> preparePlanarTrajectory(const std::vector<glm::dvec3>& border, const glm::dvec3& normal);
    std::shared_ptr<CompositeTrajectory> prepareThroughCut(Pose pose, double depth, double runup, const glm::dvec3& off);

    void nextAction();
    void link();

    void removeDebris();

    [[nodiscard]] bool validatePose(const Pose& pose) const;
    [[nodiscard]] bool validateTrajectory(const std::shared_ptr<Trajectory>& trajectory, double dt);

private:

    ProcessConfiguration m_config;

    std::shared_ptr<Mesh> model;
    std::shared_ptr<Sculpture> m_sculpture;

    std::shared_ptr<Debris> m_debris;

    std::shared_ptr<Robot> m_turntable;
    glm::dvec3 m_ttOffset; // Offset to table surface

    std::shared_ptr<Robot> m_robot;
    glm::dvec3 m_fwd; // Horizontal offset from robot to turntable
    double m_bladeLength, m_bladeWidth, m_bladeThickness;

    std::vector<bool> m_robotMask, m_eoatMask;


    std::vector<double> m_baseVelocityLimits, m_baseAccelerationLimits;
    std::vector<double> m_slowVelocityLimits; // TODO use cartesian speed limit (Needs further trajectory development)

    Waypoint m_robotHome;
    Waypoint m_robotNeutral;

    Waypoint m_latestTableCommand;
    Waypoint m_latestRobotCommand;

    Interpolator::SolverType m_solver;

    std::deque<Cut> m_cuts; // Temporary container for cuts when preparing action

    bool m_planned;

    std::deque<Action> m_actions;
    uint32_t m_step;
};


#endif //AUTOCARVER_SCULPTPROCESS_H
