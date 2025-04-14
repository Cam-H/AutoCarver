//
// Created by Cam on 2024-10-21.
//

#include "Scene.h"

// Rendering
#include <Qt3DCore/QTransform>

#include "fileIO/MeshHandler.h"
#include "geometry/ConvexHull.h"

Scene::Scene()
    : m_world(m_physicsCommon.createPhysicsWorld())
    , m_updateThread(nullptr)
    , m_running(false)
    , m_paused(false)
{

}

Scene::~Scene()
{
//    for (const SceneEntity& entity : m_entities) {
////        delete link.first;
//    } // TODO deletion, develop functions to clear only specific classes of bodies

    m_physicsCommon.destroyPhysicsWorld(m_world);
}


bool Scene::serialize(const std::string& filename)
{
    return Serializable::serialize(filename);
}
bool Scene::serialize(std::ofstream& file)
{

    Serializer::writeUint(file, m_bodies.size());
    for (const std::shared_ptr<Body>& body : m_bodies) {
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

    std::cout << "->BC: " << bodyCount << "\n";
    for (uint32_t i = 0; i < bodyCount; i++) {
        auto body = std::make_shared<Body>((std::shared_ptr<Mesh>)nullptr);
        if (!body->deserialize(file)) return false;

        m_bodies.push_back(body);
    }

    std::cout << "dddSerialization complete\n";


    // TODO serialize robots

    return true;
}

//void Scene::linkRenderer(Qt3DCore::QEntity *parent, Qt3DExtras::Qt3DWindow *view)
//{
//    m_root = new Qt3DCore::QEntity(parent);
//    this->view = view;
//
//    for (SceneEntity& entity : m_entities) {
//        if (entity.render == nullptr) { // Prepare renders for bodies added before any links
//            entity.render = prepareRender(entity.body);
//        } else {
//            entity.render->setParent(parent);
//        }
//    }
//}

void Scene::start()
{
//    if (!m_updateThread.joinable()) m_updateThread = std::thread(update);
//    auto thread = std::thread(update, meshTransform);

//auto thread = std::thread(this->update);
    m_running = true;
    m_paused = false;
    m_updateThread = std::make_unique<std::thread>(&Scene::run, this);
//    std::make_shared<std::thread>(update());
//    std::thread th(update());
//std::thread(update);
}

void Scene::pause()
{
    m_paused = !m_paused;
}

void Scene::stop()
{
    m_running = false;
}

void Scene::run()
{
    while (m_running) {
        if (!m_paused) update();

        std::this_thread::sleep_for(std::chrono::nanoseconds(100000));
    }
}

void Scene::update()
{
    for (auto robot : m_robots) robot->update();

}

//void Scene::update(float timestep)
//{
//    m_world->update(timestep);
////    sync();
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

std::shared_ptr<Body> Scene::createBody(const std::string &filepath, rp3d::BodyType type)
{
    return createBody(MeshHandler::loadAsMeshBody(filepath), type);
}

std::shared_ptr<Body> Scene::createBody(const std::shared_ptr<Mesh>& mesh, rp3d::BodyType type)
{
    if (mesh == nullptr) return nullptr;

    auto body = std::make_shared<Body>(mesh);
//    body->physicsBody()->setType(type);

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

void Scene::prepareBody(const std::shared_ptr<Body>& body, uint8_t level)
{
//    if (m_root == nullptr) {
//        m_entities.push_back({body, nullptr, level});
//        return;
//    }
//
//    m_entities.push_back({body, prepareRender(body), level});
    m_bodies.push_back(body);
}

//RenderEntity* Scene::prepareRender(Body *body)
//{
//    auto render = new RenderEntity(m_root, view);
//    render->add(body->mesh());
//
//    render->add(std::make_shared<Mesh>(body->hull()));
////    if (body->) render->add(std::make_shared<Mesh>(m_hull)); TODO conditional
//
//    render->generate();
//
//    render->hide(1); // Hide convex hull by default
//
//    return render;
//}

const std::vector<std::shared_ptr<Body>>& Scene::bodies()
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