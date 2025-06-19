//
// Created by Cam on 2024-10-21.
//

#include "Scene.h"

// Rendering
#include <Qt3DCore/QTransform>

#include "fileIO/MeshHandler.h"
#include "geometry/ConvexHull.h"
#include "geometry/EPA.h"
#include "physics/Constraint.h"

Scene::Scene()
    : m_updateThread(nullptr)
    , m_running(false)
    , m_paused(false)
{

}

Scene::~Scene()
{

}


bool Scene::serialize(const std::string& filename)
{
    return Serializable::serialize(filename);
}
bool Scene::serialize(std::ofstream& file)
{

    Serializer::writeUint(file, m_bodies.size());
    for (const std::shared_ptr<RigidBody>& body : m_bodies) {
        if (!body->serialize(file)) return false;
    }

    return true;
}

bool Scene::deserialize(const std::string& filename)
{
    return Serializable::deserialize(filename);
}
bool Scene::deserialize(std::ifstream& file)
{
    m_bodies.clear();

    uint32_t bodyCount = Serializer::readUint(file);

    for (uint32_t i = 0; i < bodyCount; i++) {
        auto body = std::make_shared<RigidBody>((std::shared_ptr<Mesh>)nullptr);
        if (!body->deserialize(file)) return false;

        m_bodies.push_back(body);
    }


    // TODO serialize robots

    return true;
}

void Scene::start()
{
    m_running = true;
    m_paused = false;

    m_updateThread = std::make_unique<std::thread>(&Scene::run, this);
}

void Scene::pause()
{
    m_paused = !m_paused;
}

void Scene::step(float delta)
{
    // Update robots
    for (const std::shared_ptr<Robot>& robot : m_robots) robot->step();

    // Check for robot collisions with the environment
    for (const std::shared_ptr<Robot>& robot : m_robots) {
        for (const std::shared_ptr<RigidBody>& link: robot->links()) {
            bool collision = false;
            for (const std::shared_ptr<RigidBody>& body : m_bodies) {
                collision |= link->collides(body);
            }

            link->mesh()->overrideColor(collision);
        }
    }

    // TODO broadphase selection narrowing
    std::vector<std::pair<std::shared_ptr<RigidBody>, std::shared_ptr<RigidBody>>> couples;
    for (uint32_t i = 0; i < m_bodies.size(); i++) {
        if (m_bodies[i]->layer() == 0) continue;
        if (m_bodies[i]->getType() == RigidBody::Type::DYNAMIC) {
//            std::cout << "Speed: " << m_bodies[i]->getLinearVelocity().x << " " << m_bodies[i]->getLinearVelocity().y << " " << m_bodies[i]->getLinearVelocity().z << "\n";
            for (uint32_t j = i + 1; j < m_bodies.size(); j++) {
                couples.emplace_back(m_bodies[i], m_bodies[j]);
            }
        } else {
            for (uint32_t j = i + 1; j < m_bodies.size(); j++) {
                if (m_bodies[j]->getType() == RigidBody::Type::DYNAMIC) couples.emplace_back(m_bodies[j], m_bodies[i]);
            }
        }
    }

    // Check dynamic body collisions
    std::vector<Constraint> constraints;
    for (const std::pair<std::shared_ptr<RigidBody>, std::shared_ptr<RigidBody>>& couple : couples) {
        auto result = couple.first->collision(couple.second);
        if (result.colliding()) constraints.emplace_back(couple.first, couple.second, result);
    }

    for (uint32_t i = 0; i < 1; i++)
        for (Constraint& constraint : constraints) constraint.iterateNormal(delta);

    for (uint32_t i = 0; i < 1; i++)
        for (Constraint& constraint : constraints) constraint.iterateFriction();

    if (!constraints.empty()) {
//        std::cout << "\n" << constraints.size() << " " << constraints[0].penetration() << "----------------------\n";
    }

    // Update bodies
    for (const std::shared_ptr<RigidBody>& body : m_bodies) body->step(delta);

    for (auto callback : callbacks) callback();

}

void Scene::stop()
{
    m_running = false;
}

void Scene::connect(void(*function)())
{
    callbacks.push_back(function);
}

void Scene::run()
{
    auto startTime = std::chrono::system_clock::now();

    while (m_running) {
        auto endTime = std::chrono::system_clock::now();

        if (!m_paused) {
            float delta = 1e-9f * (endTime - startTime).count(); // Calculate timestep in seconds
//            std::cout << "Delta: " << delta << "\n";
            step(delta);
            update();
        }

        startTime = endTime;

        // TODO Adapt sleep time based on target update rate
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void Scene::update()
{
    for (const std::shared_ptr<Robot>& robot : m_robots) robot->update();



}

//void Scene::update(float timestep)
//{
//    m_world->update(timestep);
//}

void Scene::clear(uint8_t level)
{
    uint32_t count = 0;

//    for (uint32_t i = 0; i < m_entities.size(); i++) {
//        if (m_entities[i].level <= level) {
//            if (m_entities[i].body->physicsBody() != nullptr) delete m_entities[i].body;
//            if (m_entities[i].render != nullptr) m_entities[i].render->deleteLater();
//
//            std::swap(m_entities[i], m_entities[m_entities.size() - 1]);
//            count++;
//            i--;
//        }
//    }
//
//    m_entities.erase(m_entities.begin() + count);
}

std::shared_ptr<RigidBody> Scene::createBody(const std::string &filepath, RigidBody::Type type)
{
    return createBody(MeshHandler::loadAsMeshBody(filepath), type);
}

std::shared_ptr<RigidBody> Scene::createBody(const std::shared_ptr<Mesh>& mesh, RigidBody::Type type)
{
    if (mesh == nullptr) return nullptr;

    auto body = std::make_shared<RigidBody>(mesh);
    body->setType(type);

    prepareBody(body);

    return body;
}

std::shared_ptr<RigidBody> Scene::createBody(const ConvexHull& hull, RigidBody::Type type)
{
    auto body = std::make_shared<RigidBody>(hull);
    body->setType(type);

    prepareBody(body);

    return body;
}


std::shared_ptr<Robot> Scene::createRobot(KinematicChain* kinematics)
{
    auto robot = std::make_shared<Robot>(kinematics);

    robot->prepareLinks();

    for (const auto& link : robot->links()) {
        m_bodies.push_back(link);
    }

    m_robots.push_back(robot);
    return robot;
}

void Scene::prepareBody(const std::shared_ptr<RigidBody>& body, uint8_t level)
{
//    if (m_root == nullptr) {
//        m_entities.push_back({body, nullptr, level});
//        return;
//    }
//
//    m_entities.push_back({body, prepareRender(body), level});
    m_bodies.push_back(body);
}

const std::vector<std::shared_ptr<RigidBody>>& Scene::bodies()
{
    return m_bodies;
}

uint32_t Scene::bodyCount()
{
    return m_bodies.size();
}

//std::vector<const std::shared_ptr<Mesh>&> Scene::meshes()
//{
//    std::vector<const std::shared_ptr<Mesh>&> result;
//    if (!m_entities.empty()) result.push_back(m_entities[0].body->mesh());
//
//    return result;
//}