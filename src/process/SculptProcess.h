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


#include "ProcessPlanner.h"

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

    void alignToFace(uint32_t faceIdx);

//    void setSculptingRobot(const std::shared_ptr<Robot>& robot);

    [[nodiscard]] bool simulationComplete() const;
    [[nodiscard]] bool simulationIdle() const;
    [[nodiscard]] bool simulationActive() const;

    ProcessConfiguration& getConfiguration();
    [[nodiscard]] const ProcessConfiguration& getConfiguration() const;

    [[nodiscard]] const std::shared_ptr<Sculpture>& getSculpture() const;
    [[nodiscard]] const std::shared_ptr<Body>& getModel() const;
    [[nodiscard]] const std::shared_ptr<Debris>& getDebris() const;

    [[nodiscard]] const std::shared_ptr<Robot>& getSculptor() const;

    [[nodiscard]] const std::shared_ptr<Robot>& getTurntable() const;

    [[nodiscard]] const glm::dvec3& forward() const;

    [[nodiscard]] Waypoint alignedToFaceWP(const std::vector<glm::dvec3>& border, const glm::dvec3& normal) const;
    [[nodiscard]] Waypoint alignedToFaceWP(const Axis3D& axes, const std::vector<glm::dvec3>& border) const;
    [[nodiscard]] Waypoint alignedToVertexWP(const Axis3D& axes, const glm::dvec3& vertex) const;
    [[nodiscard]] Waypoint alignedToVertexWP(const Axis3D& axes, const glm::dvec3& vertex, const glm::dvec3& tryAxis, double tryLimit) const;

private:

    void readySculpture(const std::shared_ptr<Sculpture>& sculpture);

    void prepareRobot();
    void prepareTurntable();

    void attachSculpture();

    void nextAction();

    void removeDebris();

    std::deque<Action> smooth(const std::deque<Action>& actions);
    void link();

    Action linked(const Action& action);

    bool testTurntableCollision(const std::shared_ptr<Trajectory>& ttTrajectory);
    Action planTurntableClearance();
    Action planApproach(const Waypoint& destination);

    std::shared_ptr<Trajectory> prepareApproach(const Waypoint& destination);

    [[nodiscard]] bool validatePose(const Pose& pose) const;
    [[nodiscard]] bool validateTrajectory(const std::shared_ptr<Trajectory>& trajectory, double dt);

private:

    std::shared_ptr<Mesh> model;
    std::shared_ptr<Sculpture> m_sculpture;

    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<Robot> m_turntable;

    std::shared_ptr<Debris> m_debris;

    glm::dvec3 m_ttOffset; // Offset to table surface


    ProcessPlanner m_planner;

    bool m_planned;

    std::deque<Action> m_actions;
    uint32_t m_step;

};


#endif //AUTOCARVER_SCULPTPROCESS_H
