//
// Created by Cam on 2024-11-12.
//

#ifndef AUTOCARVER_SCULPTPROCESS_H
#define AUTOCARVER_SCULPTPROCESS_H

#include "Scene.h"

#include <vector>

#include "geometry/VertexArray.h"
#include "geometry/Mesh.h"
#include "core/Sculpture.h"

#include "geometry/curves/Interpolator.h"
#include "robot/trajectory/Waypoint.h"

class Profile;

const static glm::dvec3 UP = { 0, 1, 0};

class SculptProcess : public Scene {
public:

    struct Configuration {
        double materialWidth = 1.0f;
        double materialHeight = 2.0f;
    };

    explicit SculptProcess(const std::shared_ptr<Mesh>& model);
    ~SculptProcess();

    void enableConvexTrim(bool enable);
    void enableProcessCut(bool enable);

    void plan();

    void proceed();
    void skip();

    void step(double delta) override;

    void setSculptingRobot(const std::shared_ptr<Robot>& robot);
    void setContinuous(bool enable);

    void alignToFace(uint32_t faceIdx);

    bool simulationComplete() const;
    bool simulationIdle() const;
    bool simulationActive() const;

    const std::shared_ptr<Sculpture>& getSculpture() const;

    const std::shared_ptr<Robot>& getSculptor() const;
    const std::shared_ptr<Robot>& getTurntable() const;


//    std::shared_ptr<Mesh> sculpture();

private:

    struct Action {
        Action (const std::shared_ptr<Trajectory>& trajectory, const std::shared_ptr<Robot>& robot)
                : trajectory(trajectory)
                , target(robot)
                , started(false)
                , trigger(false) {}


        std::shared_ptr<Trajectory> trajectory;
        std::shared_ptr<Robot> target;
        bool started;
        bool trigger;
    };

    void prepareTurntable();

    [[nodiscard]] Waypoint alignedToFaceWP(uint32_t faceIdx) const;
    [[nodiscard]] Waypoint alignedToFaceWP(const Axis3D& axes, const std::vector<glm::dvec3>& border) const;
    [[nodiscard]] Waypoint alignedToVertexWP(const Axis3D& axes, const glm::dvec3& vertex) const;

    void planConvexTrim();
    ConvexHull planConvexTrim(const ConvexHull& hull, const Plane& plane);

    void planOutlineRefinement(double stepDg);
    void planOutlineRefinement(Profile& profile);

    void planFeatureRefinement();

    void planTurntableAlignment(double theta);

    void planRoboticSection(const std::shared_ptr<Trajectory>& trajectory);

    std::shared_ptr<Trajectory> preparePlanarTrajectory(const Axis3D& axes, const std::vector<glm::dvec3>& border);
    void planRoboticSection(const std::vector<glm::dvec3>& border);

    void nextAction();

    static uint32_t identifySculpture(const std::vector<std::shared_ptr<Mesh>>& fragments);

private:

    Configuration m_config;

    std::shared_ptr<Mesh> model;
    std::shared_ptr<Sculpture> m_sculpture;

    std::shared_ptr<Robot> m_turntable;
    glm::dvec3 m_ttOffset; // Offset to table surface

    std::shared_ptr<Robot> m_robot;
    glm::dvec3 m_fwd; // Horizontal offset from robot to turntable
    double m_fwdOffset, m_rThickness;

    std::vector<double> m_baseVelocityLimits, m_baseAccelerationLimits;
    std::vector<double> m_slowVelocityLimits; // TODO use cartesian speed limit (Needs further trajectory development)

    Waypoint m_robotHome;
    Waypoint m_robotNeutral;

    Waypoint m_latestTableCommand;
    Waypoint m_latestRobotCommand;

    Interpolator::SolverType m_solver;

    std::vector<Action> m_actions;

    bool m_planned;
    bool m_convexTrimEnable;
    bool m_processCutEnable;

    uint32_t m_step;
    bool m_continuous;

};


#endif //AUTOCARVER_SCULPTPROCESS_H
