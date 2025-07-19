//
// Created by Cam on 2024-09-20.
//

#include "RigidBody.h"

#include <utility>

#include <unordered_map>
#include <gtc/matrix_transform.hpp>
//#include <glm/gtc/matrix_inverse.hpp>

#include "core/Timer.h"
#include "geometry/EPA.h"
#include "geometry/MeshBuilder.h"
#include "geometry/Collision.h"

RigidBody::RigidBody(const std::string& filename)
        : RigidBody((const std::shared_ptr<Mesh>&)nullptr)
{
    Serializable::deserialize(filename);

    updateColliders();
}

RigidBody::RigidBody(const std::shared_ptr<Mesh>& mesh)
    : m_type(Type::STATIC)
    , m_mesh(mesh)
    , m_hullMesh(nullptr)
    , m_hullOK(false)
    , m_layer(1)
    , m_mask(1)
    , m_isManifold(false)
    , m_area(0)
    , m_volume(0)
    , m_density(1000.0f)
    , m_mass(0)
    , m_inertiaTensor(0.0f)
    , m_linearVelocity()
    , m_angularVelocity()
    , m_isManifoldOK(false)
    , m_areaOK(false)
    , m_volumeOK(false)
    , m_massOK(false)
    , m_inertiaTensorOK(false)
    , m_colliderVisualsEnable(false)
    , Transformable()
{

    updateColliders();
}

RigidBody::RigidBody(const ConvexHull& hull)
        : RigidBody(std::make_shared<Mesh>(hull))
{

}

bool RigidBody::serialize(const std::string& filename)
{
    return Serializable::serialize(filename);
}
bool RigidBody::serialize(std::ofstream& file)
{
    if (!m_mesh->serialize(file)) return false;

    Serializer::writeTransform(file, m_transform);

    return true;
}

bool RigidBody::deserialize(const std::string& filename)
{
    return Serializable::deserialize(filename);
}
bool RigidBody::deserialize(std::ifstream& file)
{
    m_mesh = std::make_shared<Mesh>(file);// TODO develop method to share meshes

    if (m_mesh != nullptr && m_mesh->vertexCount() > 0 && m_mesh->faceCount() > 0) {

        updateColliders();

        setTransform(Serializer::readTransform(file));

        return true;
    }

    return false;
}

void RigidBody::setType(Type type)
{
    if (m_type != type) m_massOK = m_inertiaTensorOK = false;
    m_type = type;
}

void RigidBody::setMesh(const std::shared_ptr<Mesh>& mesh, bool doColliderUpdate) {
    if (mesh == nullptr) return;

    m_mesh = mesh;

    if (doColliderUpdate) updateColliders();
}

void RigidBody::setLayer(uint32_t layer)
{
    m_layer = layer;
}
void RigidBody::setMask(uint32_t mask)
{
    m_mask = mask;
}

void RigidBody::setDensity(float density)
{
    m_density = density;
}

uint32_t RigidBody::layer() const
{
    return m_layer;
}
uint32_t RigidBody::mask() const
{
    return m_mask;
}

bool RigidBody::scan(const std::shared_ptr<RigidBody>& body) const
{
    return ((m_mask & body->m_layer) > 0);
}

bool RigidBody::boundaryCollision(const std::shared_ptr<RigidBody>& body)
{
    return scan(body) && Collision::test(m_boundingSphere, body->m_boundingSphere);
}

bool RigidBody::collides(const std::shared_ptr<RigidBody>& body)
{
    if (!m_hullOK || !body->m_hullOK || !boundaryCollision(body)) return false;

    glm::mat4 relative = glm::inverse(m_transform) * body->m_transform;

    std::pair<uint32_t, uint32_t> nearest = cachedCollision(body);

    Simplex simplex = Collision::gjk(m_hull, body->m_hull, relative, nearest);

    cacheCollision(body, simplex[0].idx);

    return simplex.colliding();
}

bool RigidBody::collision(const std::shared_ptr<RigidBody>& body, glm::vec3& offset)
{
    if (!m_hullOK || !body->m_hullOK || !boundaryCollision(body)) return false;

    EPA epa = collision(body);
    offset = epa.colliding() ? epa.overlap() : epa.offset();

    return epa.colliding();
}

EPA RigidBody::collision(const std::shared_ptr<RigidBody>& body)
{
    if (!m_hullOK || !body->m_hullOK || !boundaryCollision(body)) return {};

    glm::mat4 relative = glm::inverse(m_transform) * body->m_transform;

    std::pair<uint32_t, uint32_t> nearest = cachedCollision(body);

    EPA epa = m_hull.epaIntersection(body->m_hull, m_transform, relative, nearest);
    cacheCollision(body, epa.nearest());

    return epa;
}

void RigidBody::cacheCollision(const std::shared_ptr<RigidBody>& body, const std::pair<uint32_t, uint32_t>& start)
{

}

std::pair<uint32_t, uint32_t> RigidBody::cachedCollision(const std::shared_ptr<RigidBody>& body)
{
    // TODO caching last index
    return { std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max() };
}

void RigidBody::step(float delta)
{
    if (m_type != Type::STATIC) {
        globalTranslate(m_linearVelocity * delta);
        rotate(m_angularVelocity * delta);
    }

    // Must happen after translation so there is time for constraints to act
    if (m_type == Type::DYNAMIC) m_linearVelocity += delta * glm::vec3{0, -9.81, 0};

}

void RigidBody::zero()
{
    if (m_mesh == nullptr) return;

    glm::vec3 centroid = m_mesh->centroid();
    m_mesh->translate(-centroid);
    globalTranslate(centroid);// TODO confirm

    m_inertiaTensorOK = false;
}


void RigidBody::updateColliders()
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

void RigidBody::prepareColliderVisuals()
{
    if (!m_colliderVisualsEnable) {
        prepareHullVisual();
        prepareSphereVisual();
    }

    m_colliderVisualsEnable = true;
}

void RigidBody::prepareHullVisual()
{
    m_hullMesh = std::make_shared<Mesh>(m_hull);
}
void RigidBody::prepareSphereVisual()
{
    m_sphereMesh = MeshBuilder::icosphere(m_boundingSphere.radius);
    m_sphereMesh->translate(m_boundingSphere.center);
}

RigidBody::Type RigidBody::getType() const
{
    return m_type;
}

bool RigidBody::isManifold()
{
    if (!m_isManifoldOK) evaluateManifold();
    return m_isManifold;
}

float RigidBody::area()
{
    if (!m_areaOK) calculateArea();
    return m_area;
}

float RigidBody::density() const
{
    return m_density;
}

float RigidBody::volume()
{
    if (!m_volumeOK) calculateVolume();
    return m_volume;
}

float RigidBody::mass()
{
    if (!m_massOK) calculateMass();
    return m_mass;
}

glm::vec3 RigidBody::centroid()
{
    return m_mesh != nullptr ? m_mesh->centroid() : glm::vec3{};
}

glm::mat3x3 RigidBody::inertiaTensor()
{
    if (!m_inertiaTensorOK) calculateInertiaTensor();
    return m_inertiaTensor;
}

void RigidBody::setLinearVelocity(glm::vec3 velocity)
{
    m_linearVelocity = velocity;
}

const glm::vec3& RigidBody::getLinearVelocity() const
{
    return m_linearVelocity;
}

void RigidBody::setAngularVelocity(glm::vec3 velocity)
{
    m_angularVelocity = velocity;
}
const glm::vec3& RigidBody::getAngularVelocity() const
{
    return m_angularVelocity;
}

const std::shared_ptr<Mesh>& RigidBody::mesh()
{
    return m_mesh;
}

const ConvexHull& RigidBody::hull() const
{
    return m_hull;
}

const Sphere& RigidBody::boundingSphere() const
{
    return m_boundingSphere;
}

const std::shared_ptr<Mesh>& RigidBody::hullMesh()
{
    return m_hullMesh;
}

const std::shared_ptr<Mesh>& RigidBody::bSphereMesh()
{
    return m_sphereMesh;
}

void RigidBody::evaluateManifold()
{
    //TODO

    m_isManifoldOK = true;
}
void RigidBody::calculateArea()
{
    //TODO

    m_areaOK = true;
}
void RigidBody::calculateVolume()
{
    m_volume = (m_mesh != nullptr) ? m_mesh->volume() : 0;
    m_volumeOK = true;
};

void RigidBody::calculateMass()
{
    m_mass = volume() * m_density;
    m_massOK = true;
}

void RigidBody::calculateInertiaTensor()
{
    m_inertiaTensor = (m_mesh != nullptr) ? glm::inverse(m_mesh->inertiaTensor() * m_density) : 0;
    m_inertiaTensorOK = true;
}