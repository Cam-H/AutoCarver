//
// Created by Cam on 2025-03-20.
//

#ifndef AUTOCARVER_ROBOT_H
#define AUTOCARVER_ROBOT_H

#include <vector>

#include "KinematicChain.h"
#include "physics/RigidBody.h"

#include "geometry/Transformable.h"
#include "planning/Trajectory.h"

class Axis3D;

class Robot : public Transformable { // TODO make serializable
public:

    Robot(const std::shared_ptr<KinematicChain>& kinematics, const std::shared_ptr<RigidBody>& eoat = nullptr);

    void prepareLinks();

    void moved() override;

    void step();
    void step(double delta);

    void update();

    void setEOAT(const std::shared_ptr<RigidBody>& eoat, bool preserveTransform = true);

    void setJointValue(uint32_t idx, double value);
    void setJointValueDg(uint32_t idx, double value);

    void moveTo(const glm::dvec3& position);
    void moveTo(const Axis3D& axes);

    void moveTo(const glm::dvec3& position, const Axis3D& axes);
//    void moveTo(const glm::dvec3& position, const glm::dvec3& euler);
    void moveTo(const Waypoint& waypoint);

    void traverse(const std::shared_ptr<Trajectory>& trajectory);

    const std::vector<std::shared_ptr<RigidBody>>& links();

    double getJointValue(uint32_t idx);
    double getJointValueDg(uint32_t idx);

    [[nodiscard]] Waypoint getWaypoint() const;
    [[nodiscard]] std::vector<double> getDistanceTravelled() const;
    [[nodiscard]] std::vector<double> getDistanceRemaining() const;
    [[nodiscard]] std::vector<double> waypointDelta(const Waypoint& waypoint) const;

    [[nodiscard]] std::vector<double> getJointVelocity() const;
    [[nodiscard]] std::vector<double> getJointAcceleration() const;

    [[nodiscard]] const std::shared_ptr<RigidBody>& getEOAT() const;

    [[nodiscard]] const glm::dmat4x4& getEOATTransform() const;
    [[nodiscard]] glm::dvec3 getEOATPosition() const;
    [[nodiscard]] Axis3D getEOATAxes() const;
    [[nodiscard]] glm::dvec3 getEOATEuler() const;

    bool inTransit() const;

    [[nodiscard]] Waypoint inverse(const glm::dvec3& position, const Axis3D& axes) const;
//    [[nodiscard]] Waypoint inverse(const glm::dvec3& position, const glm::dvec3& euler) const;


protected:

    void updateTransforms();

private:

    glm::dmat4 m_invTransform;

    std::shared_ptr<KinematicChain> m_kinematics;

    // Maps to the joints of the kinematic chain
    std::vector<std::shared_ptr<RigidBody>> m_links;

    std::shared_ptr<RigidBody> m_eoat;
    glm::dmat4x4 m_eoatRelativeTransform;

    //
    std::shared_ptr<Trajectory> m_currentTrajectory;

};


#endif //AUTOCARVER_ROBOT_H
