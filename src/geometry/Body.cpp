//
// Created by Cam on 2024-09-20.
//

#include "Body.h"

#include <utility>

#include <unordered_map>

#include "../core/Timer.h"

Body::Body(const std::shared_ptr<Mesh> &mesh)
    : m_mesh(mesh)
    , m_hull(mesh->vertices())
    , m_hullMesh(nullptr)
    , m_hullOK(true)
    , m_physEnabled(false)
    , phys(nullptr)
    , world(nullptr)
    , m_physBody(nullptr)
    , m_isManifold(false)
    , m_area(0)
    , m_volume(0)
    , m_isManifoldOK(false)
    , m_areaOK(false)
    , m_volumeOK(false)
{

}

Body::Body(rp3d::PhysicsCommon *phys, rp3d::PhysicsWorld *world, const std::shared_ptr<Mesh> &mesh)
    : m_mesh(mesh)
    , m_hull(mesh->vertices())
    , m_hullMesh(nullptr)
    , m_hullOK(true)
    , m_physEnabled(true)
    , phys(phys)
    , world(world)
    , m_physBody(world->createRigidBody(rp3d::Transform::identity()))
    , m_isManifold(false)
    , m_area(0)
    , m_volume(0)
    , m_isManifoldOK(false)
    , m_areaOK(false)
    , m_volumeOK(false)
{

    prepareColliders();
}

Body::~Body()
{
    //TODO delete body
    std::cout << "Body deletion\n";
    if (m_physBody != nullptr) world->destroyRigidBody(m_physBody);
//    phys->destroyConvexMesh()
}

void Body::prepareColliders()
{
//    rp3d::VertexArray vertexArray(m_hull.vertices().vertices(), 3 * sizeof(float), m_hull.vertexCount(), rp3d::VertexArray::DataType::VERTEX_FLOAT_TYPE);
//
//    std::vector<rp3d::Message> messages;
//    rp3d::ConvexMesh *convexMesh = phys->createConvexMesh(vertexArray, messages);
//    rp3d::ConvexMeshShape *convexMeshShape = phys->createConvexMeshShape(convexMesh, rp3d::Vector3(1, 1, 1));
//
//    rp3d::Transform transform = rp3d::Transform::identity();
//
////    rp3d::Collider *collider =
//    m_physBody->addCollider(convexMeshShape, transform);

}

rp3d::RigidBody *Body::physicsBody()
{
    return m_physBody;
}

void Body::prepareHullMesh()
{
    if (m_hullMesh == nullptr) m_hullMesh = std::make_shared<Mesh>(m_hull);
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

const std::shared_ptr<Mesh>& Body::mesh()
{
    return m_mesh;
}

const ConvexHull& Body::hull()
{
    if (!m_hullOK) {
//        m_hull(m_mesh->vertices(), m_mesh->vertexCount());
//        m_hull = ConvexHull(m_mesh->vertices(), m_mesh->vertexCount());
        m_hullOK = true;
    }

    return m_hull;
}

const ConvexHull& Body::hull() const
{
    return m_hull;
}

const std::shared_ptr<Mesh>& Body::hullMesh()
{
    return m_hullMesh;
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