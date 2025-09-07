//
// Created by Cam on 2024-10-21.
//

#include "Scene.h"

#include "fileIO/MeshHandler.h"
#include "geometry/primitives/Plane.h"
#include "geometry/primitives/AABB.h"
#include "geometry/primitives/ConvexHull.h"
#include "geometry/collision/EPA.h"
#include "physics/Constraint.h"
#include "geometry/collision/Collision.h"
#include "Timer.h"

#include "renderer/RenderBuffer.h"

Scene::Scene()
    : m_updateThread(nullptr)
    , m_running(false)
    , m_paused(false)
    , m_timeScalar(1.0)
    , m_total(0)
    , m_colorCollisions(true)
{

}

Scene::~Scene() = default;

bool Scene::serialize(std::ofstream& file) const
{

    Serializer::writeUint(file, m_bodies.size());
    for (const std::shared_ptr<RigidBody>& body : m_bodies) {
        if (!body->serialize(file)) return false;
    }

    return true;
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

// Check for robot collisions with the environment
void Scene::colorCollision(const std::shared_ptr<Robot>& robot)
{
    for (const std::shared_ptr<RigidBody>& link: robot->links()) {
        link->mesh()->enableColorOverride(test(link));
    }

    auto eoat = robot->getEOAT();
    if (eoat != nullptr) eoat->mesh()->enableColorOverride(test(eoat));
}

bool Scene::test(const std::shared_ptr<Robot>& robot) const
{
    for (const std::shared_ptr<RigidBody>& link: robot->links()) {
        if (test(link)) return true;
    }

    auto eoat = robot->getEOAT();
    return (eoat != nullptr && test(eoat));
}

bool Scene::test(const std::shared_ptr<RigidBody>& body) const
{
    for (const std::shared_ptr<RigidBody>& collider : m_bodies) { // TODO use broadphase selection
//        std::cout << "Check: " <<
        if (body->test(collider)) {
//            std::cout << "Collision: \n";
//            body->print();
//            collider->print();
            return true;
        }
    }

    return false;
}

void Scene::step(double delta)
{
    delta *= m_timeScalar;

    // Update robots
//    for (const std::shared_ptr<Robot>& robot : m_robots) robot->step();
    for (const std::shared_ptr<Robot>& robot : m_robots) robot->step(delta);

//    return;

    // TODO broadphase selection narrowing
    // Develop a list of required collision checks
    std::vector<std::pair<std::shared_ptr<RigidBody>, std::shared_ptr<RigidBody>>> couples;
    for (uint32_t i = 0; i < m_bodies.size(); i++) {
        if (m_bodies[i]->layer() == 0) continue;

        // Only explicitly test bodies that have recently moved for collision
        if (m_bodies[i]->checkMoveState()) {
            switch(m_bodies[i]->getType()) {
                case RigidBody::Type::DYNAMIC:

                    // Capture any preceding couplings missed due to lack of motion
                    for (uint32_t j = 0; j < i - 1; j++) {
                        if (!m_bodies[j]->checkMoveState()) couples.emplace_back(m_bodies[i], m_bodies[j]);
                    }

                    // Capture all subsequent couplings (Ignores older bodies to avoid duplicates)
                    for (uint32_t j = i + 1; j < m_bodies.size(); j++) {
                        couples.emplace_back(m_bodies[i], m_bodies[j]);
                    }

                    break;
                case RigidBody::Type::KINEMATIC:
                case RigidBody::Type::STATIC:

                    // Capture couplings between dynamic bodies and others that have been recently moved
                    for (const std::shared_ptr<RigidBody>& body : m_bodies) {
                        if (body->getType() == RigidBody::Type::DYNAMIC) couples.emplace_back(body, m_bodies[i]);
                    }

                    break;

            }
//            std::cout << "Speed: " << m_bodies[i]->getLinearVelocity().x << " " << m_bodies[i]->getLinearVelocity().y << " " << m_bodies[i]->getLinearVelocity().z << "\n";
        }
    }

    for (const std::shared_ptr<RigidBody>& body : m_bodies) body->clearMoveState();

    // Check dynamic body collisions
    std::vector<Constraint> constraints;
    for (const std::pair<std::shared_ptr<RigidBody>, std::shared_ptr<RigidBody>>& couple : couples) {
        auto result = couple.first->intersection(couple.second);
        if (result.colliding()) {
            constraints.emplace_back(couple.first, couple.second, result);
//            std::cout << "Collision: " << couple.first->getName() << " " << couple.second->getName() << "\n";
        }

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

    // Update visual body positions
    for (const std::shared_ptr<Body>& body : m_vis) body->step(delta);

    // Kill floor
    for (uint32_t i = 0; i < m_bodies.size(); i++) {
        if (m_bodies[i]->getType() == RigidBody::Type::DYNAMIC && m_bodies[i]->position().y < -10) {
            m_bodies.erase(m_bodies.begin() + i);
        }
    }
}

void Scene::update()
{
//    for (const std::shared_ptr<Robot>& robot : m_robots) robot->update();

    if (m_colorCollisions) {
        for (const std::shared_ptr<Robot>& robot : m_robots) {
            if (robot->checkMoveState()) {
                colorCollision(robot);
                robot->clearMoveState();
            }
        }
    }

    post();
}

void Scene::stop()
{
    m_running = false;
}

void Scene::setTimeScaling(double scalar)
{
    m_timeScalar = scalar;
}

// Specifies whether to override the color of robot links in case of collisions with scene objects
void Scene::enableCollisionColoring(bool enable)
{
    m_colorCollisions = enable;
}

void Scene::run()
{
    Timer rateTimer;
    while (m_running) {
        if (!m_paused) {
            double delta = rateTimer.getElapsedSeconds();
            try {
                step(delta);
                update();
            } catch (std::exception& e) {
                std::cout << "\033[91m" << e.what() << "\033[0m\n";
                m_paused = true;
                post();
            }
        }

        rateTimer.reset();

        // TODO Adapt sleep time based on target update rate
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
}

void Scene::post()
{
    for (const std::shared_ptr<RenderBuffer>& buffer : m_buffers) {
        buffer->update(this);
    }
}

//void Scene::update(double timestep)
//{
//    m_world->update(timestep);
//}

void Scene::clear(uint8_t level)
{
    uint32_t count = 0;

    m_bodies.clear();
    m_robots.clear();

    m_vis.clear();

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

void Scene::addRenderBuffer(const std::shared_ptr<RenderBuffer>& buffer)
{
    m_buffers.push_back(buffer);
}

void Scene::clearRenderBuffers()
{
    m_buffers.clear();
}

std::shared_ptr<Body> Scene::createVisual(const std::shared_ptr<Mesh>& mesh)
{
    m_vis.push_back(std::make_shared<Body>(mesh));
    return m_vis.back();
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


std::shared_ptr<Robot> Scene::createRobot(const std::shared_ptr<KinematicChain>& kinematics)
{
    auto robot = std::make_shared<Robot>(kinematics);

    robot->prepareLinks();

    for (const auto& link : robot->links()) {
        prepareBody(link);
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
    body->setID(m_total++);
    if (body->getName().empty()) body->setName("BODY" + std::to_string(m_bodies.size()));
    m_bodies.push_back(body);
}

const std::vector<std::shared_ptr<RigidBody>>& Scene::bodies() const
{
    return m_bodies;
}

const std::vector<std::shared_ptr<Body>>& Scene::visualBodies() const
{
    return m_vis;
}

uint32_t Scene::bodyCount() const
{
    return m_bodies.size();
}

bool Scene::isPaused() const
{
    return m_paused;
}

std::tuple<std::shared_ptr<RigidBody>, double> Scene::raycast(const Ray& ray) const
{
    // TODO potential optimization: prepass or sort bodies relative to ray before evaluating
    double tMin = std::numeric_limits<double>::max();
    uint32_t idx = 0;

    for (const std::shared_ptr<RigidBody>& body : m_bodies) {
        auto [hit, t] = body->raycast(ray, tMin);
        if (hit && t < tMin) {
            tMin = t;
            idx = &body - &m_bodies[0];
        }
    }

    if (tMin < std::numeric_limits<double>::max()) {
        return { m_bodies[idx], tMin };
    }

    return { nullptr, 0 };
}


void Scene::print() const
{
    std::cout << "[Scene] running: " << m_running << ", paused: " << m_paused << ", time-scaling: " << m_timeScalar << "\n";

    std::cout << "Robots: \n";
    for (const std::shared_ptr<Robot>& robot : m_robots) std::cout << robot->getName() << " (" << robot << ")\n";

    std::cout << "Bodies: \n";
    for (const std::shared_ptr<RigidBody>& body : m_bodies) std::cout << body->getName() << " [" << body->hull().isValid() << "] (" << body << ")\n";
}