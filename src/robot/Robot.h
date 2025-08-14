//
// Created by Cam on 2025-03-20.
//

#ifndef AUTOCARVER_ROBOT_H
#define AUTOCARVER_ROBOT_H

#include <vector>

#include "KinematicChain.h"
#include "physics/RigidBody.h"

#include "geometry/Transformable.h"

class Pose;
class Trajectory;

class Robot : public Transformable { // TODO make serializable
public:

    Robot(const std::shared_ptr<KinematicChain>& kinematics, const std::shared_ptr<RigidBody>& eoat = nullptr);

    void prepareLinks();

    void moved() override;

    void step();
    void step(double delta);

    void update();

    void setEOAT(const std::shared_ptr<RigidBody>& eoat, bool preserveTransform = true);
    void translateEOAT(const glm::dvec3& translation);

    void setJointValue(uint32_t idx, double value);
    void setJointValueDg(uint32_t idx, double value);

    void moveTo(const glm::dvec3& position);
    void moveTo(const Axis3D& axes);

    void moveTo(const Pose& pose);
    void moveTo(const Waypoint& waypoint);

    void traverse(const std::shared_ptr<Trajectory>& trajectory);
    void stop();

    void setLinkMesh(uint32_t index, const std::shared_ptr<Mesh>& mesh);

    void setName(const std::string& name);
    [[nodiscard]] const std::string& getName() const;
    [[nodiscard]] bool isValid() const;
    [[nodiscard]] uint32_t dof() const;

    const std::vector<std::shared_ptr<RigidBody>>& links();

    double getJointValue(uint32_t idx);
    double getJointValueDg(uint32_t idx);

    [[nodiscard]] Waypoint getWaypoint() const;
    [[nodiscard]] std::vector<double> getDistanceTravelled() const;
    [[nodiscard]] std::vector<double> getDistanceRemaining() const;
    [[nodiscard]] std::vector<double> waypointDelta(const Waypoint& waypoint) const;

    [[nodiscard]] std::vector<double> getJointVelocity() const;
    [[nodiscard]] std::vector<double> getJointAcceleration() const;

    [[nodiscard]] std::shared_ptr<RigidBody> getEOAT() const;

    [[nodiscard]] glm::dvec3 getPosition() const;
    [[nodiscard]] Axis3D getAxes() const;

    [[nodiscard]] Pose getPose() const;
    [[nodiscard]] Pose getPose(const Waypoint& waypoint) const;

    [[nodiscard]] Waypoint inverse(const Pose& pose) const;

    [[nodiscard]] Waypoint preferredWaypoint(const Waypoint& optionA, const Waypoint& optionB) const;

    bool inTransit() const;



protected:

    void updateTransforms();

private:

    [[nodiscard]] static uint32_t linkMask(uint32_t level) ;

private:

    std::string m_name;

    glm::dmat4 m_invTransform;

    std::shared_ptr<KinematicChain> m_kinematics;

    // Maps to the joints of the kinematic chain
    std::vector<std::shared_ptr<RigidBody>> m_links;

    std::shared_ptr<RigidBody> m_eoat;
    glm::dmat4 m_eoatTransform; // Transform calculated directly from kinematic chain

    glm::dmat4 m_eoatRelativeTransform;

    //
    std::shared_ptr<Trajectory> m_currentTrajectory;

};


#endif //AUTOCARVER_ROBOT_H
