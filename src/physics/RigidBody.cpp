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

#include "CompositeBody.h"

const double HOLD_THRESHOLD = 1e-6;

RigidBody::RigidBody()
    : m_type(Type::STATIC)
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
    , Body()
{

}

RigidBody::RigidBody(const std::string& filename)
    : RigidBody()
{
    Serializable::load(filename);
}

RigidBody::RigidBody(const std::shared_ptr<Mesh>& mesh)
    : RigidBody()
{
    setMesh(mesh);
}

RigidBody::RigidBody(const ConvexHull& hull)
    : RigidBody()
{
    m_hull = hull;
    m_mesh = std::make_shared<Mesh>(m_hull);
}

bool RigidBody::serialize(std::ofstream& file) const
{
    if (Body::serialize(file)) {
        m_hull.serialize(file);

        return true;
    }

    return false;
}

bool RigidBody::deserialize(std::ifstream& file)
{
    if (Body::deserialize(file)) {
        m_hull = ConvexHull(file);

        return true;
    }

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

void RigidBody::setMesh(const std::shared_ptr<Mesh>& mesh) {
    if (mesh == nullptr) return;

    m_mesh = mesh;
    updateHull();
}

void RigidBody::setHull(const ConvexHull& hull)
{
    m_hull = hull;
    m_hull.evaluate();
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

bool RigidBody::boundsTest(const std::shared_ptr<RigidBody>& body)
{
    return scan(body)
        && m_hull.isValid() && body->m_hull.isValid()
        && Collision::test(bounds(), body->bounds());
}

//bool RigidBody::test(const std::shared_ptr<CompositeBody>& body)
//{
//    std::cout << "zza\n";
//
//    if (test(std::static_pointer_cast<RigidBody>(body))) {
//        std::cout << "Hit\n";
//    }
//
//    std::cout << "===\n";
//
//    return false;
//}

bool RigidBody::test(const std::shared_ptr<RigidBody>& body)
{
//    if (std::dynamic_pointer_cast<CompositeBody>(body)) std::cout << "zzz\n";
    if (boundsTest(body)) {
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
        }
    }

    return false;
}

std::tuple<bool, glm::dvec3> RigidBody::delta(const std::shared_ptr<RigidBody>& body)
{
    EPA epa = intersection(body);

    return {
        epa.colliding(),
        epa.delta()
    };
}

EPA RigidBody::intersection(const std::shared_ptr<RigidBody>& body)
{
    if (boundsTest(body)) {
        glm::dmat4 relative = glm::inverse(m_transform) * body->m_transform;

        std::pair<uint32_t, uint32_t> nearest = cachedCollision(body);

        EPA epa = Collision::intersection(m_hull, body->m_hull, relative, nearest);
        epa.setWorldTransform(m_transform);

        cacheCollision(body, epa.nearest());
        return epa;
    }

    return {};
}

bool RigidBody::precheck(uint32_t hullID0, uint32_t hullID1, const glm::dmat4& relative) const
{
    return true;
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

    recenter(m_mesh->centroid());

    m_inertiaTensorOK = false;
}

void RigidBody::recenter(const glm::dvec3& offset)
{
    m_mesh->translate(-offset);
    m_hull.translate(-offset);
    translate(offset);
}


void RigidBody::updateHull()
{
    if (m_mesh == nullptr) return;
    setHull(ConvexHull(m_mesh));
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

Sphere RigidBody::bounds() const
{
    auto bounds = m_hull.bounds();
//    bounds.center += position();
    return bounds;
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
        auto [hit, t] = Collision::raycast(bounds(), ray);
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