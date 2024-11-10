//
// Created by Cam on 2024-10-21.
//

#include "Scene.h"

#include "fileIO/MeshLoader.h"
#include "geometry/ConvexHull.h"

Scene::Scene()
    : m_world(m_physicsCommon.createPhysicsWorld())
    , m_updateThread()
    , m_paused(false)
{

}

void Scene::start()
{
//    if (!m_updateThread.joinable()) m_updateThread = std::thread(update);
//    auto thread = std::thread(update, meshTransform);

//auto thread = std::thread(this->update);
}

void Scene::pause()
{

}

void Scene::update()
{

}

void Scene::update(float timestep)
{
    m_world->update(timestep);
    sync();
}

void Scene::sync()
{
    for (Body *body : m_bodies) {
        if (body->physicsBody()->getType() != rp3d::BodyType::STATIC && !body->physicsBody()->isSleeping()) {
            body->sync();
        }
    }
}

Body* Scene::createBody(const std::string &filepath, rp3d::BodyType type)
{
    return createBody(MeshLoader::loadAsMeshBody(filepath), type);
}

Body* Scene::createBody(const std::shared_ptr<Mesh>& mesh, rp3d::BodyType type)
{
    m_bodies.emplace_back(new Body(&m_physicsCommon, m_world, mesh));
    m_bodies[m_bodies.size() - 1]->physicsBody()->setType(type);
    return m_bodies[m_bodies.size() - 1];
}