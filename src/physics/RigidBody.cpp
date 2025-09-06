//
// Created by Cam on 2024-09-20.
//

#include "RigidBody.h"

#include <utility>

#include <unordered_map>
#include <gtc/matrix_transform.hpp>
//#include <glm/gtc/matrix_inverse.hpp>

#include "core/Timer.h"
#include "geometry/collision/EPA.h"
#include "geometry/MeshBuilder.h"
#include "geometry/collision/Collision.h"

const double HOLD_THRESHOLD = 1e-6;

RigidBody::RigidBody()
    : m_type(Type::STATIC)
    , m_hullMesh(nullptr)
    , m_hullOK(false)
    , m_mask(0xFFFFFFFF)
    , m_layer(0xFFFFFFFF)
    , m_isManifold(false)
    , m_area(0)
    , m_volume(0)
    , m_density(1000.0f)
    , m_mass(0)
    , m_inertiaTensor(0.0f)
    , m_isManifoldOK(false)
    , m_areaOK(false)
    , m_volumeOK(false)
    , m_massOK(false)
    , m_inertiaTensorOK(false)
    , m_colliderVisualsEnable(false)
    , Body()
{

}

RigidBody::RigidBody(const std::string& filename)
    : RigidBody()
{
    Serializable::load(filename);

    updateColliders();
}

RigidBody::RigidBody(const std::shared_ptr<Mesh>& mesh)
    : RigidBody()
{
    setMesh(mesh, true);
}

RigidBody::RigidBody(const ConvexHull& hull)
    : RigidBody()
{
    m_mesh = std::make_shared<Mesh>(hull);
    m_hull = hull;
    m_hullOK = hull.isValid();
    m_boundingSphere = Sphere::enclose(m_hull.vertices());
}

bool RigidBody::serialize(std::ofstream& file) const
{
    return Body::serialize(file);
}

bool RigidBody::deserialize(std::ifstream& file)
{
    std::cout << "RBDS\n";

    if (Body::deserialize(file)) {
        std::cout << m_mesh << " RBDS0\n";

        std::cout << m_mesh->vertexCount() << "\n";
        updateColliders();
        std::cout << "RBDS1\n";

        return true;
    }
    std::cout << "RBDSNF\n";

    return false;
}

void RigidBody::moved()
{
    Body::moved();
}

void RigidBody::setType(Type type)
{
    if (m_type != type) {
        if (type == RigidBody::Type::DYNAMIC) { // Perform operations to prepare dynamic bodies for physics
            zero();

            // Calculate properties if they are out of date (Or were not prepared yet)
            mass();
            inertiaTensor();
        }
    }
    m_type = type;
}

void RigidBody::setMesh(const std::shared_ptr<Mesh>& mesh, bool doColliderUpdate) {
    if (mesh == nullptr) return;

    m_mesh = mesh;

    if (doColliderUpdate) updateColliders();
}

void RigidBody::setHull(const ConvexHull& hull)
{
    m_hull = hull;
    prepareColliders();
}


void RigidBody::setMask(uint32_t mask)
{
    m_mask = mask;
}
void RigidBody::setLayer(uint32_t layer)
{
    m_layer = layer;
}

void RigidBody::disableCollisions()
{
    m_mask = m_layer = 0;
}

void RigidBody::resetMask()
{
    m_mask = 0xFFFFFFFF;
}
void RigidBody::resetLayer()
{
    m_layer = 0xFFFFFFFF;
}

void RigidBody::setDensity(double density)
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
    if (body.get() == this || !m_hullOK || !body->m_hullOK || !boundaryCollision(body)) return false;

    glm::dmat4 relative = glm::inverse(m_transform) * body->m_transform;

    std::pair<uint32_t, uint32_t> nearest = cachedCollision(body);

    try {
        Simplex simplex = Collision::gjk(m_hull, body->m_hull, relative, nearest);

        cacheCollision(body, simplex[0].idx);
        return simplex.colliding();
    } catch (std::exception& e) {
        m_hull.print();
        body->m_hull.print();
        std::cout << "\033[93m[RigidBody] Failed to test collision [" << m_name + " ~ " << body->m_name << "]\n" << e.what() << "\033[0m\n";
        return false;
    }
}

bool RigidBody::collision(const std::shared_ptr<RigidBody>& body, glm::dvec3& offset)
{
    EPA epa = collision(body);
    offset = epa.colliding() ? epa.overlap() : epa.offset();

    return epa.colliding();
}

EPA RigidBody::collision(const std::shared_ptr<RigidBody>& body)
{
    if (!m_hullOK || !body->m_hullOK || !boundaryCollision(body)) return {};

    glm::dmat4 relative = glm::inverse(m_transform) * body->m_transform;

    std::pair<uint32_t, uint32_t> nearest = cachedCollision(body);

    EPA epa = Collision::intersection(m_hull, body->m_hull, relative, nearest);
    epa.setWorldTransform(m_transform);

    cacheCollision(body, epa.nearest());

    return epa;
}

void RigidBody::cacheCollision(const std::shared_ptr<RigidBody>& body, const std::pair<uint32_t, uint32_t>& start)
{

}

std::pair<uint32_t, uint32_t> RigidBody::cachedCollision(const std::shared_ptr<RigidBody>& body)
{
    // TODO caching last index
//    return { std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max() };
    return { 0, 0 };

}

void RigidBody::step(double delta)
{
    if (m_type != Type::STATIC) {
        glm::dvec3 acceleration = m_linearVelocity + m_angularVelocity;
//        std::cout << "ACCL: " << glm::dot(acceleration, acceleration) << "\n";
        if (glm::dot(acceleration, acceleration) > HOLD_THRESHOLD) {
            Body::step(delta);
        }
    }

    // Must happen after translation so there is time for constraints to act
    if (m_type == Type::DYNAMIC) m_linearVelocity += delta * glm::dvec3{0, -9.81, 0};

}

void RigidBody::zero()
{
    if (m_mesh == nullptr) return;

    glm::dvec3 centroid = m_mesh->centroid();

    m_mesh->print();

    m_mesh->translate(-centroid);
    m_hull = ConvexHull(m_mesh->vertices()); // TODO could translate
    translate(centroid);

    m_mesh->print();

    m_inertiaTensorOK = false;
}


void RigidBody::updateColliders()
{
    m_hullOK = false;

    if (m_mesh == nullptr) return;
    m_hull = ConvexHull(m_mesh->vertices());
    prepareColliders();
}

void RigidBody::prepareColliders()
{
    m_hullOK = m_hull.vertexCount() >= 4;

    if (m_hullOK) m_boundingSphere = Sphere::enclose(m_hull.vertices());
    else m_boundingSphere = Sphere::enclose(m_mesh->vertices());

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

double RigidBody::area()
{
    if (!m_areaOK) calculateArea();
    return m_area;
}

double RigidBody::density() const
{
    return m_density;
}

double RigidBody::volume()
{
    if (!m_volumeOK) calculateVolume();
    return m_volume;
}

double RigidBody::mass()
{
    if (!m_massOK) calculateMass();
    return m_mass;
}

glm::dvec3 RigidBody::centroid()
{
    return m_mesh != nullptr ? m_mesh->centroid() : glm::dvec3{};
}

// Returns the inverse of the inertia tensor for this body
glm::dmat3 RigidBody::inertiaTensor()
{
    if (!m_inertiaTensorOK) calculateInertiaTensor();
    return m_inertiaTensor;
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
    if (m_mesh != nullptr) {
//        m_inertiaTensor = m_mesh->inertiaTensor() * m_density;
        m_inertiaTensor = glm::inverse(m_mesh->inertiaTensor() * m_density);
        m_inertiaTensorOK = true;
    }
}

std::tuple<bool, double> RigidBody::raycast(Ray ray, double tLim)
{
    auto [hit, t, idx] = pickFace(ray, tLim);
    return { hit, t };
}

std::tuple<bool, double, uint32_t> RigidBody::pickFace(Ray ray, double tLim)
{
    if (m_mesh == nullptr) return { false, 0, 0 };

    // Transform ray relative to the body
    ray = glm::inverse(m_transform) * ray;

    // Initial check against bounding sphere
    {
        auto [hit, t] = Collision::raycast(m_boundingSphere, ray);
        if (!hit || t > tLim) return { false, 0, 0 };
    }

    return m_mesh->pickFace(ray);
}

void RigidBody::print() const
{
    auto pos = position();

    std::cout << "[RigidBody] " << m_name << ", position: (" << pos.x << ", " << pos.y << ", " << pos.z << "), layer: "
              << m_layer << ", mask: " << m_mask << "\nmass: " << m_mass << ", volume: " << m_volume << ", density: " << m_density << "\n";
}