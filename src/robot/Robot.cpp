//
// Created by Cam on 2025-03-20.
//

#include "Robot.h"

#include <utility>
#include <glm.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/matrix_decompose.hpp>

#include "geometry/MeshBuilder.h"

Robot::Robot(const std::shared_ptr<KinematicChain>& kinematics, const std::shared_ptr<RigidBody>& eoat)
    : m_kinematics(kinematics)
    , m_eoat(eoat)
    , m_eoatRelativeTransform()
    , Transformable()
{

}

// Prepare links based on the kinematic chain
void Robot::prepareLinks()
{
    float height = 0.3f;
    for (const Joint& joint : m_kinematics->getJoints()) {
//        auto mesh = MeshBuilder::cylinder(0.25f, height, 8);

        auto mesh = MeshBuilder::cylinder(0.12f, height, 8);


        mesh->translate({ 0, -height / 2, 0 });
        mesh->rotate({ 1, 0, 0 }, M_PI / 2);
        mesh->setFaceColor({0.1f, 0.1f, 0.1f});
        mesh->setBaseColor({1, 0, 0});

        uint32_t layer = 0b0010, mask = 0b0001;

        if (joint.getParameters().len > 0 || joint.getParameters().dist > 0) {

//            auto linkMesh = MeshBuilder::box(std::max(0.2f, joint.getParameters().len), std::max(0.2f, joint.getParameters().dist), 0.2);
            auto linkMesh = MeshBuilder::box(std::max(0.08f, joint.getParameters().len), std::max(0.08f, joint.getParameters().dist), 0.08f);
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



    auto eoatMesh = MeshBuilder::box(0.2f, 0.1f, 0.05f);
    eoatMesh->setBaseColor({0, 0, 1});
    m_links.push_back(std::make_shared<RigidBody>(eoatMesh));

    // Block collisions of last joint with EOAT
    m_links[m_links.size() - 2]->setLayer(0b0100);
    m_links[m_links.size() - 1]->setLayer(0b0010);
    m_links[m_links.size() - 1]->setMask(0b0011);

    updateTransforms();
}

void Robot::step()
{
    if (inTransit()) moveTo(m_currentTrajectory->next());
}

void Robot::step(float delta)
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
    } else m_eoatRelativeTransform = glm::mat4x4(1.0f);
}

void Robot::setJointValue(uint32_t idx, float value)
{
    if (idx < m_kinematics->jointCount()) {
        m_kinematics->getJoint(idx).setValue(value);
        update();
    } else {
        std::cout << "WARN. Joint idx specified does not exist!\n";
    }
}

void Robot::setJointValueDg(uint32_t idx, float value)
{
    setJointValue(idx, (float)(value * M_PI / 180));
}

void Robot::moveTo(const glm::vec3& position, const glm::vec3& euler)
{
    m_kinematics->moveTo(position, euler);
    update();
}

void Robot::moveTo(const Waypoint& waypoint)
{
    uint32_t end = std::min(m_kinematics->jointCount(), (uint32_t)waypoint.values.size());
    for (uint32_t i = 0; i < end; i++) m_kinematics->getJoint(i).setValue(waypoint.toRad * waypoint.values[i]);

    update();
}

void Robot::traverse(const std::shared_ptr<Trajectory>& trajectory)
{
    m_currentTrajectory = trajectory;
}

void Robot::updateTransforms()
{
    const std::vector<glm::mat4> transforms = m_kinematics->jointTransforms();
    for (uint32_t i = 0; i < transforms.size(); i++) { // -1 if no EOAT
        m_links[i]->setTransform(m_transform * transforms[i]);
    }

    if (m_eoat != nullptr) {
        m_eoat->setTransform(getEOATTransform() * m_eoatRelativeTransform);
    }
}

const std::vector<std::shared_ptr<RigidBody>>& Robot::links()
{
    return m_links;
}

float Robot::getJointValue(uint32_t idx)
{
    return m_kinematics->getJoint(idx).getValue();
}
float Robot::getJointValueDg(uint32_t idx)
{
    return (float)(getJointValue(idx) * 180.0f / M_PI);
}

Waypoint Robot::getWaypoint() const
{
    if (m_kinematics == nullptr) return {};

    std::vector<float> values;
    values.reserve(m_kinematics->jointCount());
    for (uint32_t i = 0; i < m_kinematics->jointCount(); i++) values.emplace_back(m_kinematics->getJoint(i).getValue());
    return {
        values,
        1.0f,
        false
    };
}

const glm::mat4x4& Robot::getEOATTransform() const
{
    if (m_links.empty()) throw std::runtime_error("[Robot] No Links!");
    return m_links[m_links.size() - 1]->getTransform();
}

glm::vec3 Robot::getEOATPosition() const
{
    if (m_links.empty()) return {};

    const glm::mat4& transform = m_links[m_links.size() - 1]->getTransform();
    return { transform[3][0], transform[3][1], transform[3][2] };
}

glm::vec3 Robot::getEOATEuler() const
{
    if (m_links.empty()) return {};

    const glm::mat4& transform = m_links[m_links.size() - 1]->getTransform();
    return glm::eulerAngles(glm::quat_cast(transform));
}

bool Robot::inTransit()
{
    return m_currentTrajectory != nullptr && !m_currentTrajectory->complete();
}

Waypoint Robot::inverse(const glm::vec3& position, const Axis3D& axes) const
{
    return {
            m_kinematics->invkin(position, axes),
            1.0f,
            false
    };
}

Waypoint Robot::inverse(const glm::vec3& position, const glm::vec3& euler) const
{
    auto values = m_kinematics->invkin(position, euler);

    return {
        values,
        1.0f,
        false
    };
}