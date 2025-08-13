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
        , m_name("ROBOT")
        , m_eoatTransform(1.0)
        , m_eoatRelativeTransform()
        , m_invTransform(1.0)
        , Transformable()
{

}

// Prepare links based on the kinematic chain
// Layers and masks are prepared so each link ignores collisions with the adjacent links
void Robot::prepareLinks()
{
    double height = 0.3f;
    static uint32_t level = 0x00000001;
    /*
     * TODO Current implementation will cause collision testing issues with enough links
     */

    auto baseMesh = MeshBuilder::box(1.0);
    m_links.push_back(std::make_shared<RigidBody>(baseMesh));
    m_links.back()->prepareColliderVisuals();
    m_links.back()->setMask(0); // Base is fixed so explicit collision tests are unnecessary
    m_links.back()->setLayer(level <<= 1); // Prevent overlap with mask of previous robot

    for (const Joint& joint : m_kinematics->getJoints()) {
//        auto mesh = MeshBuilder::cylinder(0.25f, height, 8);

        auto mesh = MeshBuilder::cylinder(0.12, height, 8);


        mesh->translate({ 0, -height / 2, 0 });
        mesh->rotate({ 1, 0, 0 }, M_PI / 2);
        mesh->setFaceColor({0.1f, 0.1f, 0.1f});
        mesh->setBaseColor({1, 0, 0});

        std::shared_ptr<RigidBody> link = nullptr;
        if (joint.getParameters().len > 0 || joint.getParameters().dist > 0) {
//            auto linkMesh = MeshBuilder::box(std::max(0.2f, joint.getParameters().len), std::max(0.2f, joint.getParameters().dist), 0.2);
            auto linkMesh = MeshBuilder::box(std::max(0.08, joint.getParameters().len), std::max(0.08, joint.getParameters().dist), 0.08);
            linkMesh->translate({ joint.getParameters().len / 2, 0, joint.getParameters().dist / 2 });
            linkMesh->setFaceColor({ 1.0f, 1.0f, 0.0f });
            link = std::make_shared<RigidBody>(MeshBuilder::merge(mesh, linkMesh));

            link->setLayer(level <<= 1);
            link->setMask(linkMask(level));

            link->prepareColliderVisuals();

        } else {
            link = std::make_shared<RigidBody>(mesh);

            // Ignore collisions on joints without a footprint
            link->setMask(0);
            link->setLayer(0);
        }

        m_links.push_back(link);
    }

    setName(m_name); // Set link names

    updateTransforms();
}

uint32_t Robot::linkMask(uint32_t level)
{
    return ~((level << 2) | (level << 1) | (level >> 1) | (level >> 2));
}

// Only calculate inverse when updated to save time
void Robot::moved()
{
    updateTransforms();
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
    if (m_eoat != nullptr) { // Cleanup old EOAT
        m_eoat->setAxisEnabled(false);
        m_eoat->resetMask();
        m_eoat->resetLayer();
//        m_eoat->mesh()->transform(m_kinematics->inversion());
    }

    m_eoat = eoat;

    if (m_eoat == nullptr || m_links.empty()) return;

    m_eoat->setAxisEnabled(true);

    // Take the end layer (Should be last unless the end is not a collider)
    uint32_t layer = 0;
    for (const std::shared_ptr<RigidBody>& link : m_links) layer = std::max(layer, link->layer());

    m_eoat->setLayer(layer <<= 1);
    m_eoat->setMask(~((layer >> 1) | (layer >> 2)));

//    m_eoat->mesh()->rotate(glm::quat_cast(glm::inverse(m_kinematics->axisTransform())));

    m_eoatRelativeTransform = glm::inverse(m_kinematics->inversion());
//    m_eoatRelativeTransform = glm::dmat4(1.0);
//    m_eoat->mesh()->transform(glm::inverse(m_kinematics->inversion()));

    if (preserveTransform) {
//        m_eoatRelativeTransform = glm::inverse(m_links.back()->getTransform()) * m_eoat->getTransform() * m_eoatRelativeTransform;//TODO test
    }


    updateTransforms();
}

void Robot::translateEOAT(const glm::dvec3& translation)
{
    m_eoatRelativeTransform = glm::translate(m_eoatRelativeTransform, translation);
    updateTransforms();
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
    moveTo(Pose(position, getAxes()));
}
void Robot::moveTo(const Axis3D& axes)
{
    moveTo(Pose(getPosition(), axes));
}

void Robot::moveTo(const Pose& pose)
{
    m_kinematics->moveTo(m_invTransform * pose);
    update();
}

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

void Robot::setLinkMesh(uint32_t index, const std::shared_ptr<Mesh>& mesh)
{
    if (index >= m_links.size()) throw std::runtime_error("[Robot] Index out of bounds. Can not update link mesh");

    // The base link is not affected by the kinematic chains axis inversion
    if (index != 0 && index < m_links.size()) {
        mesh->rotate(glm::quat_cast(glm::inverse(m_kinematics->axisInversion())));
    }

    m_links[index]->setMesh(mesh, true);
}

void Robot::updateTransforms()
{
    if (m_links.empty()) return;

    m_links[0]->setTransform(m_transform); // Robot base

    const std::vector<glm::dmat4> transforms = m_kinematics->jointTransforms();
    for (uint32_t i = 0; i < transforms.size() - 1; i++) {
        m_links[i + 1]->setTransform(m_transform * transforms[i]);
    }

    m_eoatTransform = m_transform * transforms.back();

    if (m_eoat != nullptr) {
        m_eoat->setTransform(m_eoatTransform * m_eoatRelativeTransform);
    }

    Transformable::moved();
}

void Robot::setName(const std::string& name)
{
    m_name = name;
    for (uint32_t i = 0; i < m_links.size(); i++) {
        m_links[i]->setName(m_name + "-LINK" + std::to_string(i));
    }
}

const std::string& Robot::getName() const
{
    return m_name;
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

std::shared_ptr<RigidBody> Robot::getEOAT() const
{
    return m_eoat;
}

glm::dvec3 Robot::getPosition() const
{
    return Transformable::position(m_eoatTransform);
}
Axis3D Robot::getAxes() const
{
    return { Transformable::rotation(m_eoatTransform) };
}

Pose Robot::getPose() const
{
    return { getPosition(), getAxes() };
}
Pose Robot::getPose(const Waypoint& waypoint) const
{
    return (m_transform * m_kinematics->getPose(waypoint)); // TODO * m_eoatRelativeTransform;
}

Waypoint Robot::inverse(const Pose& pose) const
{
    return Waypoint(m_kinematics->invkin(m_invTransform * pose), false);
}

// Selects the better waypoint based on distance from joint limits
Waypoint Robot::preferredWaypoint(const Waypoint& optionA, const Waypoint& optionB) const
{
    // Confirm that both waypoints are valid
    bool vA = m_kinematics->validate(optionA), vB = m_kinematics->validate(optionB);
    if (!vA || !vB) {
        if (vA) return optionA;
        if (vB) return optionB;
        return Waypoint();
    }

    double aWeight = 0, bWeight = 0;
    for (uint32_t i = 0; i < optionA.values.size(); i++) {
        const auto& joint = m_kinematics->getJoints()[i];
        aWeight += 1 - (joint.remainingLimit(optionA.values[i]) / joint.getRange());
        bWeight += 1 - (joint.remainingLimit(optionB.values[i]) / joint.getRange());
    }

    return aWeight < bWeight ? optionA : optionB;
}

bool Robot::inTransit() const
{
    return m_currentTrajectory != nullptr && !m_currentTrajectory->complete();
}