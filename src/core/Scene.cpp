//
// Created by Cam on 2024-10-21.
//

#include "Scene.h"

// Rendering
#include <Qt3DCore/QTransform>

#include "fileIO/MeshLoader.h"
#include "geometry/ConvexHull.h"

Scene::Scene()
    : m_world(m_physicsCommon.createPhysicsWorld())
    , m_root(nullptr)
    , view(nullptr)
    , m_updateThread()
    , m_paused(false)
{

}

Scene::~Scene()
{
    for (std::pair<Body*, RenderEntity*>& link : m_bodies) {
//        delete link.first;
    }
}

void Scene::linkRenderer(Qt3DCore::QEntity *parent, Qt3DExtras::Qt3DWindow *view)
{
    m_root = new Qt3DCore::QEntity(parent);
    this->view = view;

    for (std::pair<Body*, RenderEntity*>& link : m_bodies) {
        if (link.second == nullptr) { // Prepare renders for bodies added before any links
            link.second = prepareRender(link.first);
        } else {
            link.second->setParent(parent);
        }
    }
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
    for (const std::pair<Body*, RenderEntity*>& link : m_bodies) {
        auto physics = link.first->physicsBody();

        if (physics != nullptr && m_root != nullptr && physics->getType() != rp3d::BodyType::STATIC && !physics->isSleeping()) {
            sync(physics, link.second);
        }
    }
}

void Scene::sync(rp3d::RigidBody *physics, RenderEntity *render)
{
    const reactphysics3d::Transform& transform = physics->getTransform();
    const reactphysics3d::Vector3& position = transform.getPosition();
    const reactphysics3d::Quaternion& orientation = transform.getOrientation();

    render->setTranslation({position.x, position.y, position.z});
    render->setRotation(QQuaternion(orientation.w, orientation.x, orientation.y, orientation.z));
}


void Scene::translateBody(uint32_t idx, float w, float x, float y, float z)
{
    if (m_root == nullptr || idx >= m_bodies.size()) return;

    auto physics = m_bodies[idx].first->physicsBody();
    auto render = m_bodies[idx].second;

    if (physics != nullptr) {
        rp3d::Transform transform = physics->getTransform();
        transform.setPosition(transform.getPosition() + rp3d::Vector3{x, y, z});
        physics->setTransform(transform);

        const rp3d::Vector3& position = transform.getPosition();
        render->setTranslation({position.x, position.y, position.z});
    } else{
        Qt3DCore::QTransform *transform = render->transformation();
        transform->setTranslation(transform->translation() + QVector3D{x, y, z});
    }
}
void Scene::rotateBody(uint32_t idx, float w, float x, float y, float z)
{
    if (m_root == nullptr || idx >= m_bodies.size()) return;

    auto physics = m_bodies[idx].first->physicsBody();
    auto render = m_bodies[idx].second;

    if (physics != nullptr) {
        rp3d::Transform transform = physics->getTransform();
        transform.setOrientation(transform.getOrientation() * rp3d::Quaternion(rp3d::Vector3(x, y, z), w));
        physics->setTransform(transform);

        const rp3d::Quaternion& orientation = transform.getOrientation();
        render->setRotation(QQuaternion(orientation.w, orientation.x, orientation.y, orientation.z));
    } else {
        Qt3DCore::QTransform *transform = render->transformation();
        transform->setRotation(transform->rotation() * QQuaternion(w, x, y, z));
    }
}

void Scene::showAll()
{
    if (m_root == nullptr) return;

    for (std::pair<Body*, RenderEntity*>& link : m_bodies) {
        link.second->show();
    }
}
void Scene::hideAll()
{
    if (m_root == nullptr) return;

    for (std::pair<Body*, RenderEntity*>& link : m_bodies) {
        link.second->hide();
    }
}

void Scene::show(uint32_t idx, Model target)
{
    if (m_root == nullptr || idx >= m_bodies.size()) return;

    switch (target) {
        case Model::ALL:
            m_bodies[idx].second->show();
            break;
        case Model::MESH:
            m_bodies[idx].second->show(0);
            break;
        case Model::HULL:
            m_bodies[idx].second->show(1);
            break;
        case Model::BOUNDING_SPHERE:
            break;
    }
}
void Scene::hide(uint32_t idx, Model target)
{
    if (m_root == nullptr || idx >= m_bodies.size()) return;

    switch (target) {
        case Model::ALL:
            m_bodies[idx].second->hide();
            break;
        case Model::MESH:
            m_bodies[idx].second->hide(0);
            break;
        case Model::HULL:
            m_bodies[idx].second->hide(1);
            break;
        case Model::BOUNDING_SPHERE:
            break;
    }
}

void Scene::createBody(const std::string &filepath, rp3d::BodyType type)
{
    createBody(MeshLoader::loadAsMeshBody(filepath), type);
}

void Scene::createBody(const std::shared_ptr<Mesh>& mesh, rp3d::BodyType type)
{
    auto body = new Body(&m_physicsCommon, m_world, mesh);
    body->physicsBody()->setType(type);
    prepareBody(body);
}

void Scene::prepareBody(Body *body)
{
    if (m_root == nullptr) {
        m_bodies.emplace_back(body, nullptr);
        return;
    }

    m_bodies.emplace_back(body, prepareRender(body));
}

RenderEntity* Scene::prepareRender(Body *body)
{
    auto render = new RenderEntity(m_root, view);
    render->add(body->mesh());

    render->add(std::make_shared<Mesh>(body->hull()));
//    if (body->) render->add(std::make_shared<Mesh>(m_hull)); TODO conditional

    render->generate();

    render->hide(1); // Hide convex hull by default

    return render;
}

uint32_t Scene::bodyCount()
{
    return m_bodies.size();
}