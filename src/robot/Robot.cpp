//
// Created by Cam on 2025-03-20.
//

#include "Robot.h"

#include <utility>
#include <glm/glm.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_decompose.hpp>

#include "geometry/MeshBuilder.h"

Robot::Robot(KinematicChain* kinematics)
    : m_kinematics(kinematics)
    , m_transform(1, 0, 0, 0,
                  0, 0, 1, 0,
                  0, -1, 0, 0,
                  0, 0, 0, 1)
{

}

// Prepare links based on the kinematic chain
void Robot::prepareLinks(rp3d::PhysicsCommon *phys, rp3d::PhysicsWorld *world)
{
    float height = 0.8f;
    for (const Joint& joint : m_kinematics->getJoints()) {
        auto mesh = MeshBuilder::cylinder(0.25f, height, 8);
        mesh->translate(0, -height / 2, 0);
        mesh->rotate(1, 0, 0, M_PI / 2);
        mesh->setBaseColor(glm::vec3{0.1f, 0.1f, 0.1f});

        if (joint.getParameters().len > 0 || joint.getParameters().dist > 0) {
            auto linkMesh = MeshBuilder::box(std::max(0.2f, joint.getParameters().len), std::max(0.2f, joint.getParameters().dist), 0.2);
            linkMesh->translate(joint.getParameters().len / 2, 0, joint.getParameters().dist / 2);
            linkMesh->setBaseColor(glm::vec3{ 1.0f, 1.0f, 0.0f });
            mesh = MeshBuilder::merge(mesh, linkMesh);
        }


        m_links.push_back(new Body(phys, world, mesh));
        m_links[m_links.size() - 1]->prepareHullMesh();
    }

    auto eoatMesh = MeshBuilder::box(0.2f, 0.1f, 0.05f);
    eoatMesh->setBaseColor({0, 0, 1});
    m_links.push_back(new Body(phys, world, eoatMesh));

    updateTransforms();
}

void Robot::update()
{
    updateTransforms();
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

void Robot::updateTransforms()
{
    const std::vector<glm::mat4> transforms = m_kinematics->jointTransforms();
    for (uint32_t i = 0; i < transforms.size(); i++) { // -1 if no EOAT
        m_links[i]->setTransform(transforms[i]);
    }
}

//KinematicChain& Robot::kinematics()
//{
//    return m_kinematics;
//}

const std::vector<Body*>& Robot::links()
{
    return m_links;
}

//Joint& Robot::getJoint(uint32_t idx)
//{
//    return m_kinematics.getJoint(idx);
//}

float Robot::getJointValue(uint32_t idx)
{
    return m_kinematics->getJoint(idx).getValue();
}
float Robot::getJointValueDg(uint32_t idx)
{
    return (float)(getJointValue(idx) * 180.0f / M_PI);
}

glm::vec3 Robot::getEOATPosition() const
{
    if (m_links.empty()) return {};

    const glm::mat4& transform = m_links[m_links.size() - 1]->getTransform();
//            std::cout << " GLM Transform:\n"
//                  << transform[0][0] << " " << transform[0][1] << " " << transform[0][2] << " " << transform[0][3] << "\n"
//                  << transform[1][0] << " " << transform[1][1] << " " << transform[1][2] << " " << transform[1][3] << "\n"
//                  << transform[2][0] << " " << transform[2][1] << " " << transform[2][2] << " " << transform[2][3] << "\n"
//                  << transform[3][0] << " " << transform[3][1] << " " << transform[3][2] << " " << transform[3][3] << "\n";


    return { transform[0][3], transform[1][3], transform[2][3] };
}

glm::vec3 Robot::getEOATEuler() const
{
    const glm::mat4& transform = m_links[m_links.size() - 1]->getTransform();
    glm::mat3 rotation = {
            transform[0][0], transform[0][1], transform[0][2],
            transform[1][0], transform[1][1], transform[1][2],
            transform[2][0], transform[2][1], transform[2][2]
    };

    glm::vec3 t = glm::eulerAngles(glm::quat_cast(rotation));
    std::cout << "CEEuler: " << t.x << " " << t.y << " "<< t.z << "\n";

    return glm::eulerAngles(glm::quat_cast(rotation));
}