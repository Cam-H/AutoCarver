//
// Created by Cam on 2024-09-20.
//

#include "Body.h"

#include <utility>

#include <unordered_map>
#include <glm/gtc/matrix_transform.hpp>
//#include <glm/gtc/matrix_inverse.hpp>

#include "../core/Timer.h"
#include "EPA.h"

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
    , m_transform(1.0f)
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
    , m_transform(1.0f)
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

void Body::setMesh(const std::shared_ptr<Mesh>& mesh, bool recalculateHull) {
    if (mesh == nullptr) return;

    m_mesh = mesh;

    if (recalculateHull) updateHull();
}

void Body::setPosition(const glm::vec3& position)
{
    m_transform[3][0] = position.x;
    m_transform[3][1] = position.y;
    m_transform[3][2] = position.z;

}

void Body::translate(const glm::vec3& translation)
{
    m_transform = glm::translate(m_transform, translation);

}
void Body::rotate(const glm::vec3& axis, float theta)
{
    m_transform = glm::rotate(m_transform, theta, axis);
}

void Body::globalTranslate(const glm::vec3& translation)
{
    m_transform = glm::translate(glm::mat4(1.0f), translation) * m_transform;
}
void Body::globalRotate(const glm::vec3& axis, float theta)
{
    m_transform = glm::rotate(glm::mat4(1.0f), theta, axis) * m_transform;
}

void Body::transform(const glm::mat4x4& transform)
{
    m_transform = m_transform * transform;
}

void Body::setTransform(glm::mat4x4 transform)
{
    m_transform = transform;
}

const glm::mat4x4& Body::getTransform()
{
    return m_transform;
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

bool Body::collides(const std::shared_ptr<Body>& body)
{
    if (!m_hullOK || !body->m_hullOK) return false;

    glm::mat4 relative = glm::inverse(m_transform) * body->m_transform;

    std::pair<uint32_t, uint32_t> nearest = cachedCollision(body);

    Simplex simplex = m_hull.gjkIntersection(body->m_hull, relative, nearest);

    cacheCollision(body, simplex[0].idx);

    return simplex.colliding();
}

bool Body::collision(const std::shared_ptr<Body>& body, glm::vec3& offset)
{
    if (!m_hullOK || !body->m_hullOK) return false;


    EPA epa = collision(body);
    offset = epa.colliding() ? epa.overlap() : epa.offset();

    return epa.colliding();
}

EPA Body::collision(const std::shared_ptr<Body>& body)
{
    if (!m_hullOK || !body->m_hullOK) return {};

    glm::mat4 relative = glm::inverse(m_transform) * body->m_transform;

    std::pair<uint32_t, uint32_t> nearest = cachedCollision(body);

    EPA epa = m_hull.epaIntersection(body->m_hull, m_transform, relative, nearest);
    cacheCollision(body, epa.nearest());

    return epa;
}

void Body::cacheCollision(const std::shared_ptr<Body>& body, const std::pair<uint32_t, uint32_t>& start)
{

}

std::pair<uint32_t, uint32_t> Body::cachedCollision(const std::shared_ptr<Body>& body)
{
    // TODO caching last index
    return { std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max() };
}

void Body::updateHull()
{
    m_hull = ConvexHull(m_mesh->vertices());

    // Create new mesh for rendering the hull, if one is already in use (replacement)
    if (m_hullMesh != nullptr) m_hullMesh = std::make_shared<Mesh>(m_hull);
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