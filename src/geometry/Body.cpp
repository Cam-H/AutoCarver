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

Body::Body(const std::string& filename)
        : Body((const std::shared_ptr<Mesh>&)nullptr)
{
    Serializable::deserialize(filename);

    updateColliders();
}

Body::Body(const std::shared_ptr<Mesh>& mesh)
    : m_mesh(mesh)
    , m_hullMesh(nullptr)
    , m_hullOK(false)
    , m_layer(1)
    , m_mask(1)
    , m_isManifold(false)
    , m_area(0)
    , m_volume(0)
    , m_isManifoldOK(false)
    , m_areaOK(false)
    , m_volumeOK(false)
    , m_colliderVisualsEnable(false)
    , Transformable()
{

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

void Body::setLayer(uint32_t layer)
{
    m_layer = layer;
}
void Body::setMask(uint32_t mask)
{
    m_mask = mask;
}

uint32_t Body::layer() const
{
    return m_layer;
}
uint32_t Body::mask() const
{
    return m_mask;
}

bool Body::scan(const std::shared_ptr<Body>& body) const
{
    return ((m_mask & body->m_layer) > 0);
}

bool Body::boundaryCollision(const std::shared_ptr<Body>& body)
{
    return scan(body) && m_boundingSphere.intersects(body->m_boundingSphere);
}

bool Body::collides(const std::shared_ptr<Body>& body)
{
    if (!m_hullOK || !body->m_hullOK || !boundaryCollision(body)) return false;

    glm::mat4 relative = glm::inverse(m_transform) * body->m_transform;

    std::pair<uint32_t, uint32_t> nearest = cachedCollision(body);

    Simplex simplex = m_hull.gjkIntersection(body->m_hull, relative, nearest);

    cacheCollision(body, simplex[0].idx);

    return simplex.colliding();
}

bool Body::collision(const std::shared_ptr<Body>& body, glm::vec3& offset)
{
    if (!m_hullOK || !body->m_hullOK || !boundaryCollision(body)) return false;

    EPA epa = collision(body);
    offset = epa.colliding() ? epa.overlap() : epa.offset();

    return epa.colliding();
}

EPA Body::collision(const std::shared_ptr<Body>& body)
{
    if (!m_hullOK || !body->m_hullOK || !boundaryCollision(body)) return {};

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