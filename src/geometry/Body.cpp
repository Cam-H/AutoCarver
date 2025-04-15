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
#include "MeshBuilder.h"

Body::Body(const std::shared_ptr<Mesh> &mesh)
    : m_mesh(mesh)
    , m_hullMesh(nullptr)
    , m_hullOK(false)
    , m_isManifold(false)
    , m_area(0)
    , m_volume(0)
    , m_isManifoldOK(false)
    , m_areaOK(false)
    , m_volumeOK(false)
    , m_transform(1.0f)
    , m_colliderVisualsEnable(false)
{

    updateColliders();
}

Body::Body(const std::string& filename)
    : m_mesh(nullptr)
    , m_hullMesh(nullptr)
    , m_hullOK(false)
    , m_isManifold(false)
    , m_area(0)
    , m_volume(0)
    , m_isManifoldOK(false)
    , m_areaOK(false)
    , m_volumeOK(false)
    , m_transform(1.0f)
    , m_colliderVisualsEnable(false)
{
    Serializable::deserialize(filename);

    updateColliders();
}

bool Body::serialize(const std::string& filename)
{
    return Serializable::serialize(filename);
}
bool Body::serialize(std::ofstream& file)
{
    if (!m_mesh->serialize(file)) return false;

    Serializer::writeTransform(file, m_transform);

    return true;
}

bool Body::deserialize(const std::string& filename)
{
    return Serializable::deserialize(filename);
}
bool Body::deserialize(std::ifstream& file)
{
    m_mesh = std::make_shared<Mesh>(file);// TODO develop method to share meshes

    if (m_mesh != nullptr && m_mesh->vertexCount() > 0 && m_mesh->faceCount() > 0) {

        updateColliders();

        setTransform(Serializer::readTransform(file));

        return true;
    }

    return false;
}

void Body::setMesh(const std::shared_ptr<Mesh>& mesh, bool doColliderUpdate) {
    if (mesh == nullptr) return;

    m_mesh = mesh;

    if (doColliderUpdate) updateColliders();
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

void Body::updateColliders()
{
    m_hullOK = false;

    if (m_mesh == nullptr) return;

    m_hull = ConvexHull(m_mesh->vertices());
    m_hullOK = m_hull.vertexCount() >= 4;

    m_boundingSphere = Sphere::enclose(m_hull.vertices());

    // Create new mesh for colliders, if one is already in use (replacement)
    if (m_colliderVisualsEnable) {
        prepareHullVisual();
        prepareSphereVisual();
    }
}

void Body::prepareColliderVisuals()
{
    if (!m_colliderVisualsEnable) {
        prepareHullVisual();
        prepareSphereVisual();
    }

    m_colliderVisualsEnable = true;
}

void Body::prepareHullVisual()
{
    m_hullMesh = std::make_shared<Mesh>(m_hull);
}
void Body::prepareSphereVisual()
{
    m_sphereMesh = MeshBuilder::icosphere(m_boundingSphere.radius);
    m_sphereMesh->translate(m_boundingSphere.center.x, m_boundingSphere.center.y, m_boundingSphere.center.z);
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

const ConvexHull& Body::hull() const
{
    return m_hull;
}

const Sphere& Body::boundingSphere() const
{
    return m_boundingSphere;
}

const std::shared_ptr<Mesh>& Body::hullMesh()
{
    return m_hullMesh;
}

const std::shared_ptr<Mesh>& Body::bSphereMesh()
{
    return m_sphereMesh;
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