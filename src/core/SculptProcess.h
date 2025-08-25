//
// Created by Cam on 2024-11-12.
//

#ifndef AUTOCARVER_SCULPTPROCESS_H
#define AUTOCARVER_SCULPTPROCESS_H

#include "Scene.h"

#include <vector>
#include <deque>

#include "geometry/VertexArray.h"
#include "geometry/Mesh.h"
#include "core/Sculpture.h"

#include "geometry/curves/Interpolator.h"
#include "robot/Pose.h"
#include "robot/trajectory/Waypoint.h"
#include "robot/trajectory/CompositeTrajectory.h"

class Profile;

const static glm::dvec3 UP = { 0, 1, 0};

class SculptProcess : public Scene {
public:

    struct Configuration {
        double materialWidth = 1.0;
        double materialHeight = 2.0;
    };

    enum class ConvexSliceOrder {
        TOP_DOWN = 0, BOTTOM_UP
    };

    explicit SculptProcess(const std::shared_ptr<Mesh>& model);
    ~SculptProcess();

    void setTarget(const std::shared_ptr<Mesh>& mesh);

    void enableConvexTrim(bool enable);
    void enableSilhouetteRefinement(bool enable);

    void enableProcessCut(bool enable);

    void enableActionLinking(bool enable);
    void enableCollisionTesting(bool enable);
    void enableCutSimulation(bool enable);
    void enableFragmentRelease(bool enable);

    void enableDebrisColoring(bool enable);

    void setSlicingOrder(ConvexSliceOrder order);
    void setActionLimit(uint32_t limit);
    void enableActionLimit(bool enable);

    void reset();

    void plan();

    void blind(const Pose& pose, double depth);
    void mill(const Pose& pose, const glm::dvec3& normal, const glm::dvec3& travel, double depth);

    void proceed();
    void skip();

    void step(double delta) override;

    void setSculptingRobot(const std::shared_ptr<Robot>& robot);
    void setContinuous(bool enable);

    void alignToFace(uint32_t faceIdx);

    uint32_t getActionLimit() const;
    bool isActionLimitEnabled() const;

    bool simulationComplete() const;
    bool simulationIdle() const;
    bool simulationActive() const;

    const std::shared_ptr<Sculpture>& getSculpture() const;
    const std::shared_ptr<RigidBody>& getModel() const;

    const std::shared_ptr<Robot>& getSculptor() const;
    const std::shared_ptr<Robot>& getTurntable() const;

    [[nodiscard]] Axis3D faceAlignedAxes(const glm::dvec3& normal, bool alignHorizontal) const;

//    std::shared_ptr<Mesh> sculpture();

private:

    struct Cut {
        Cut(const Pose& pose, double effectiveThickness, double ts, double tf, uint32_t index)
            : origin(pose.position)
            , normal(pose.axes.yAxis)
            , axis(pose.axes.xAxis)
            , thickness(effectiveThickness)
            , ts(ts)
            , tf(tf)
            , index(index) {}
        Cut(const glm::dvec3& origin, const glm::dvec3& normal, const glm::dvec3& axis, double effectiveThickness, double ts, double tf, uint32_t index)
            : origin(origin)
            , normal(normal)
            , axis(axis)
            , thickness(effectiveThickness)
            , ts(ts)
            , tf(tf)
            , index(index) {}

        glm::dvec3 origin; // Initial position before beginning cut
        glm::dvec3 normal; // Normal to the cutting direction
        glm::dvec3 axis; // Axis of travel
        double thickness; // Distance between cutting planes (Reduced from full blade thickness when axis is not perpendicular to normal)
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

    void prepareTurntable();
    void attachSculpture();

    [[nodiscard]] bool withinActionLimit() const;

    [[nodiscard]] Waypoint alignedToFaceWP(const std::vector<glm::dvec3>& border, const glm::dvec3& normal) const;
    [[nodiscard]] Waypoint alignedToFaceWP(const Axis3D& axes, const std::vector<glm::dvec3>& border) const;
    [[nodiscard]] Waypoint alignedToVertexWP(const Axis3D& axes, const glm::dvec3& vertex) const;
    [[nodiscard]] Waypoint alignedToVertexWP(const Axis3D& axes, const glm::dvec3& vertex, const glm::dvec3& tryAxis, double tryLimit) const;

    [[nodiscard]] glm::dvec3 alignedToBlade(const Axis3D& axes, const glm::dvec3& vertex);

    [[nodiscard]] inline glm::dvec3 bladeCenterOffset(const Axis3D& axes, const glm::dvec3& vertex) const;
    [[nodiscard]] inline glm::dvec3 bladeThicknessOffset(const Axis3D& axes) const;

    static void toWorldSpace(glm::dvec3& normal, const glm::dquat& rotation);
    void toWorldSpace(std::vector<glm::dvec3>& border, const glm::dquat& rotation) const;

    [[nodiscard]] glm::dvec3 poseAdjustedVertex(const Axis3D& axes, const glm::dvec3& vertex) const;


    void planConvexTrim();
    std::vector<Plane> orderConvexTrim(const std::vector<Plane>& cuts);
    std::vector<Action> planConvexTrim(const ConvexHull& hull, const Plane& plane);

    void planOutlineRefinement(double stepDg);
    std::vector<Action> planOutlineRefinement(Profile& profile);
    Action planOutlineRefinement(const Profile& profile, const Triangle3D& wTri, const glm::dvec3& wNormal);

    bool planReliefCuts(const Profile& profile, const glm::dvec3& wNormal, uint8_t edgeIndex, Action& action);
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
    std::shared_ptr<CompositeTrajectory> prepareBlindCut(Pose pose, double depth);

    void commitActions(const std::vector<Action>& actions);

    void nextAction();

    void removeDebris();

    [[nodiscard]] bool validatePose(const Pose& pose) const;
    [[nodiscard]] bool validateTrajectory(const std::shared_ptr<Trajectory>& trajectory, double dt);

    static uint32_t identifySculpture(const std::vector<std::shared_ptr<Mesh>>& fragments);

private:

    Configuration m_config;

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

    double m_minCutVolume; // Skip cuts when the reduced volume is less than this limit

    Waypoint m_robotHome;
    Waypoint m_robotNeutral;

    Waypoint m_latestTableCommand;
    Waypoint m_latestRobotCommand;

    Interpolator::SolverType m_solver;

    std::deque<Cut> m_cuts; // Temporary container for cuts when preparing action

    std::vector<Action> m_actions;

    // Planning settings

    bool m_planned;

    bool m_convexTrimEnable;
    bool m_silhouetteRefinementEnable;
    bool m_processCutEnable;

    bool m_linkActionEnable;
    bool m_collisionTestingEnable;
    bool m_cutSimulationEnable;
    bool m_fragmentReleaseEnable;

    bool m_debrisColoringEnable;

    ConvexSliceOrder m_sliceOrder;

    uint32_t m_actionLimit;
    bool m_actionLimitEnable;

    uint32_t m_step;
    bool m_continuous;

};


#endif //AUTOCARVER_SCULPTPROCESS_H
