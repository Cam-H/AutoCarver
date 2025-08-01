//
// Created by Cam on 2025-03-20.
//

#include "Robot.h"

#include <utility>
#include <glm.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/matrix_decompose.hpp>
#include <gtx/string_cast.hpp>

#include "geometry/MeshBuilder.h"
#include "Pose.h"
#include "trajectory/Trajectory.h"
#include "core/Functions.h"

Robot::Robot(const std::shared_ptr<KinematicChain>& kinematics, const std::shared_ptr<RigidBody>& eoat)
    : m_kinematics(kinematics)
    , m_eoat(eoat)
    , m_eoatRelativeTransform()
    , Transformable()
    , m_invTransform(1.0)
{

}

// Prepare links based on the kinematic chain
void Robot::prepareLinks()
{
    double height = 0.3f;
    for (const Joint& joint : m_kinematics->getJoints()) {
//        auto mesh = MeshBuilder::cylinder(0.25f, height, 8);

        auto mesh = MeshBuilder::cylinder(0.12, height, 8);


        mesh->translate({ 0, -height / 2, 0 });
        mesh->rotate({ 1, 0, 0 }, M_PI / 2);
        mesh->setFaceColor({0.1f, 0.1f, 0.1f});
        mesh->setBaseColor({1, 0, 0});

        uint32_t layer = 0b0010, mask = 0b0001;

        if (joint.getParameters().len > 0 || joint.getParameters().dist > 0) {

//            auto linkMesh = MeshBuilder::box(std::max(0.2f, joint.getParameters().len), std::max(0.2f, joint.getParameters().dist), 0.2);
            auto linkMesh = MeshBuilder::box(std::max(0.08, joint.getParameters().len), std::max(0.08, joint.getParameters().dist), 0.08);
            linkMesh->translate({ joint.getParameters().len / 2, 0, joint.getParameters().dist / 2 });
            linkMesh->setFaceColor({ 1.0f, 1.0f, 0.0f });
            mesh = MeshBuilder::merge(mesh, linkMesh);
        } else {
            mask = layer = 0; // Ignore collisions on joints without a footprint
        }

        auto link = std::make_shared<RigidBody>(mesh);

        link->setLayer(layer);
        link->setMask(mask);
        link->prepareColliderVisuals();

        m_links.push_back(link);
    }


    auto eoatMesh = MeshBuilder::axes(Axis3D());
    m_links.push_back(std::make_shared<RigidBody>(eoatMesh));

    // Block collisions of last joint with EOAT
    m_links[m_links.size() - 2]->setLayer(0b0100);
    m_links.back()->setLayer(0b0010);
    m_links.back()->setMask(0b0011);

    updateTransforms();
}

// Only calculate inverse when updated to save time
void Robot::moved()
{
    m_invTransform = glm::inverse(m_transform);
}

void Robot::step()
{
    if (inTransit()) moveTo(m_currentTrajectory->next());
}

void Robot::step(double delta)
{
    if (inTransit()) moveTo(m_currentTrajectory->timestep(delta));
}

void Robot::update()
{
    updateTransforms();
}

void Robot::setEOAT(const std::shared_ptr<RigidBody>& eoat, bool preserveTransform)
{
    m_eoat = eoat;

    if (m_eoat != nullptr && preserveTransform) {
        m_eoatRelativeTransform = glm::inverse(getEOATTransform()) * m_eoat->getTransform();
    } else m_eoatRelativeTransform = glm::dmat4x4(1.0f);
}

void Robot::setJointValue(uint32_t idx, double value)
{
    if (idx < m_kinematics->jointCount()) {
        m_kinematics->getJoint(idx).setValue(value);
        update();
    } else {
        std::cout << "WARN. Joint idx specified does not exist!\n";
    }
}

void Robot::setJointValueDg(uint32_t idx, double value)
{
    setJointValue(idx, (double)(value * M_PI / 180));
}

void Robot::moveTo(const glm::dvec3& position)
{
    moveTo(Pose(position, getEOATAxes()));
}
void Robot::moveTo(const Axis3D& axes)
{
    moveTo(Pose(getEOATPosition(), axes));
}

void Robot::moveTo(const Pose& pose)
{
//    glm::dvec4 transformed = m_invTransform * glm::dvec4(position.x, position.y, position.z, 1);
//    m_kinematics->moveTo({ transformed.x, transformed.y, transformed.z }, axes * m_invTransform);

    m_kinematics->moveTo(m_invTransform * pose);
    update();
}

//void Robot::moveTo(const glm::dvec3& position, const glm::dvec3& euler)
//{
//    m_kinematics->moveTo(position, euler);
//    update();
//}

void Robot::moveTo(const Waypoint& waypoint)
{
    m_kinematics->moveTo(waypoint);
    update();
}

void Robot::traverse(const std::shared_ptr<Trajectory>& trajectory)
{
    if (m_kinematics == nullptr) throw std::runtime_error("[Robot] Can not traverse. The kinematic chain is undefined");
    m_currentTrajectory = trajectory;
}

void Robot::updateTransforms()
{
    const std::vector<glm::dmat4> transforms = m_kinematics->jointTransforms();
    for (uint32_t i = 0; i < transforms.size(); i++) { // -1 if no EOAT
        m_links[i]->setTransform(m_transform * transforms[i]);
    }

    if (m_eoat != nullptr) {
        m_eoat->setTransform(getEOATTransform() * m_eoatRelativeTransform);
    }
}

bool Robot::isValid() const
{
    return m_kinematics != nullptr && m_kinematics->jointCount() > 0;
}

uint32_t Robot::dof() const
{
    if (m_kinematics == nullptr) return 0;
    return m_kinematics->jointCount();
}

const std::vector<std::shared_ptr<RigidBody>>& Robot::links()
{
    return m_links;
}

double Robot::getJointValue(uint32_t idx)
{
    return m_kinematics->getJoint(idx).getValue();
}
double Robot::getJointValueDg(uint32_t idx)
{
    return (double)(getJointValue(idx) * 180.0f / M_PI);
}

Waypoint Robot::getWaypoint() const
{
    if (m_kinematics == nullptr) return Waypoint();
    else return m_kinematics->getWaypoint();
}

std::vector<double> Robot::getDistanceTravelled() const
{
    std::vector<double> delta(m_kinematics->jointCount(), 0.0);
    if (!inTransit()) return std::vector<double>(m_kinematics->jointCount(), 0.0);
    return waypointDelta(m_currentTrajectory->start());
}
std::vector<double> Robot::getDistanceRemaining() const
{
    std::vector<double> delta(m_kinematics->jointCount(), 0.0);
    if (!inTransit()) return std::vector<double>(m_kinematics->jointCount(), 0.0);
    return waypointDelta(m_currentTrajectory->end());
}

std::vector<double> Robot::waypointDelta(const Waypoint& waypoint) const
{
    if (m_kinematics == nullptr || waypoint.values.size() != m_kinematics->jointCount()) throw std::runtime_error("[Robot] Invalid data. Can not calculate delta");

    Waypoint current = getWaypoint();
    if (waypoint.inDg) current = current.toDg();

    std::vector<double> delta(waypoint.values.size());
    for (uint32_t i = 0; i < m_kinematics->jointCount(); i++) delta[i] = current.values[i] - waypoint.values[i];

    return delta;
}

std::vector<double> Robot::getJointVelocity() const
{
    if (!inTransit()) return std::vector<double>(m_kinematics->jointCount(), 0.0);
    return m_currentTrajectory->velocity();
}
std::vector<double> Robot::getJointAcceleration() const
{
    if (!inTransit()) return std::vector<double>(m_kinematics->jointCount(), 0.0);
    return m_currentTrajectory->acceleration();
}

const std::shared_ptr<RigidBody>& Robot::getEOAT() const
{
    if (m_links.empty()) throw std::runtime_error("[Robot] The robot does not have an EOAT");
    return m_links.back();
}

const glm::dmat4x4& Robot::getEOATTransform() const
{
    if (m_links.empty()) throw std::runtime_error("[Robot] No Links!");
    return m_links.back()->getTransform();
}

glm::dvec3 Robot::getEOATPosition() const
{
    if (m_links.empty()) return {};
    return Transformable::position(m_links.back()->getTransform());
}

Axis3D Robot::getEOATAxes() const
{
    if (m_links.empty()) return {};

    return { m_links.back()->getRotation() };
}

glm::dvec3 Robot::getEOATEuler() const
{
    if (m_links.empty()) return {};

    const glm::dmat4& transform = m_links.back()->getTransform();
    return glm::eulerAngles(glm::quat_cast(transform));
}

Pose Robot::getPose() const
{
    return { getEOATPosition(), getEOATAxes() };
}
Pose Robot::getPose(const Waypoint& waypoint) const
{
    return (m_transform * m_kinematics->getPose(waypoint)); // TODO * m_eoatRelativeTransform;
}

Waypoint Robot::inverse(const Pose& pose) const
{
    return Waypoint(m_kinematics->invkin(m_invTransform * pose), false);
}

bool Robot::inTransit() const
{
    return m_currentTrajectory != nullptr && !m_currentTrajectory->complete();
}