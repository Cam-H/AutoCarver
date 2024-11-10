//
// Created by Cam on 2024-09-20.
//

#include "Body.h"

#include <utility>

#include <unordered_map>

#include "../core/Timer.h"

Body::Body(const std::shared_ptr<Mesh> &mesh)
    : m_mesh(mesh)
    , m_hull(mesh->vertices(), mesh->vertexCount())
    , m_hullOK(true)
    , m_render(nullptr)
    , m_physEnabled(false)
    , s_phys(nullptr)
    , m_physBody(nullptr)
    , m_isManifold(false)
    , m_area(0)
    , m_volume(0)
    , m_tesselationOK(false)
    , m_isManifoldOK(false)
    , m_areaOK(false)
    , m_volumeOK(false)
{

}

Body::Body(rp3d::PhysicsCommon *phys, rp3d::PhysicsWorld *world, const std::shared_ptr<Mesh> &mesh)
    : m_mesh(mesh)
    , m_hull(mesh->vertices(), mesh->vertexCount())
    , m_hullOK(true)
    , m_render(nullptr)
    , m_physEnabled(true)
    , s_phys(phys)
    , m_physBody(world->createRigidBody(rp3d::Transform::identity()))
    , m_isManifold(false)
    , m_area(0)
    , m_volume(0)
    , m_tesselationOK(false)
    , m_isManifoldOK(false)
    , m_areaOK(false)
    , m_volumeOK(false)
{

    prepareColliders();

    std::cout << "~~~~~~~~~~~~~~~~~~~~~~VERTEX COUNT: " << m_mesh->vertexCount() << " vs " << m_hull.vertexCount() << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
}

void Body::setRenderer(Qt3DCore::QEntity *parent, Qt3DExtras::Qt3DWindow *view)
{
    m_render = new RenderEntity(parent, view);
    m_render->add(m_mesh);

    if (m_hullOK) m_render->add(std::make_shared<Mesh>(m_hull));
    std::cout << "mss\n";
    updateRenderer();
}

void Body::updateRenderer()
{
    m_render->generate();
    hide(Model::HULL);
}

RenderEntity *Body::getRenderEntity()
{
    return m_render;
}

void Body::show(Model model)
{
    switch (model) {
        case Model::ALL:
            m_render->show(0);
            m_render->show(1);
            break;
        case Model::MESH:
            m_render->show(0);
            break;
        case Model::HULL:
            m_render->show(1);
            break;
        case Model::BOUNDING_SPHERE:
            break;
    }
}
void Body::hide(Model model)
{
    switch (model) {
        case Model::ALL:
            m_render->hide(0);
            m_render->hide(1);
            break;
        case Model::MESH:
            m_render->hide(0);
            break;
        case Model::HULL:
            m_render->hide(1);
            break;
        case Model::BOUNDING_SPHERE:
            break;
    }
}

void Body::translate(float x, float y, float z)
{
    rp3d::Transform transform = m_physBody->getTransform();
    transform.setPosition(transform.getPosition() + rp3d::Vector3{x, y, z});
    m_physBody->setTransform(transform);

    const rp3d::Vector3& position = transform.getPosition();
    m_render->setTranslation({position.x, position.y, position.z});
}

void Body::rotate(float w, float x, float y, float z)
{
    if (m_physBody != nullptr) {
        rp3d::Transform transform = m_physBody->getTransform();
        transform.setOrientation(transform.getOrientation() * rp3d::Quaternion(rp3d::Vector3(x, y, z), w));
        m_physBody->setTransform(transform);

        const rp3d::Quaternion& orientation = transform.getOrientation();
        m_render->setRotation(QQuaternion(orientation.w, orientation.x, orientation.y, orientation.z));
    } else {
        Qt3DCore::QTransform *transform = m_render->transformation();
        transform->setRotation(transform->rotation() * QQuaternion(w, x, y, z));
    }
}

void Body::sync()
{
    const reactphysics3d::Transform& transform = m_physBody->getTransform();
    const reactphysics3d::Vector3& position = transform.getPosition();
    const reactphysics3d::Quaternion& orientation = transform.getOrientation();

    m_render->setTranslation({position.x, position.y, position.z});
    m_render->setRotation(QQuaternion(orientation.w, orientation.x, orientation.y, orientation.z));
}

void Body::prepareColliders()
{
    rp3d::VertexArray vertexArray(m_hull.vertices(), 3 * sizeof(float), m_hull.vertexCount(), rp3d::VertexArray::DataType::VERTEX_FLOAT_TYPE);

    std::vector<rp3d::Message> messages;
    rp3d::ConvexMesh *convexMesh = s_phys->createConvexMesh(vertexArray, messages);
    rp3d::ConvexMeshShape *convexMeshShape = s_phys->createConvexMeshShape(convexMesh, rp3d::Vector3(1, 1, 1));

    rp3d::Transform transform = rp3d::Transform::identity();

//    rp3d::Collider *collider =
    m_physBody->addCollider(convexMeshShape, transform);

}

rp3d::RigidBody *Body::physicsBody()
{
    return m_physBody;
}

bool Body::isManifold()
{
    if (!m_isManifoldOK) evaluateManifold();
    return m_isManifold;
}

float Body::area()
{
    if (!m_areaOK) calculateArea();
    return m_area;
}

float Body::volume()
{
    if (!m_volumeOK) calculateVolume();
    return m_volume;
}

//Mesh &Body::mesh()
//{
//    return m_mesh;
//}
//
//Mesh *Body::hullMesh()
//{
//    if (m_hullMesh == nullptr) m_hullMesh = new Mesh(hull());
//
//    return m_hullMesh;
//}

const ConvexHull &Body::hull()
{
    if (!m_hullOK) {
//        m_hull(m_mesh->vertices(), m_mesh->vertexCount());
//        m_hull = ConvexHull(m_mesh->vertices(), m_mesh->vertexCount());
        std::cout << "inaa\n";
        m_hullOK = true;
    }

    std::cout << "xx\n";

    return m_hull;
}

const ConvexHull &Body::hull() const
{
    return m_hull;
}
void Body::evaluateManifold()
{
    //TODO

    m_isManifoldOK = true;
}
void Body::calculateArea()
{
    //TODO

    m_areaOK = true;
}
void Body::calculateVolume()
{
    //TODO

    m_volumeOK = true;
};