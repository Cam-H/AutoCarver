//
// Created by Cam on 2024-09-20.
//

#ifndef AUTOCARVER_RIGIDBODY_H
#define AUTOCARVER_RIGIDBODY_H

#include <fstream>

// Mesh manipulation

#include <vector>
#include "glm.hpp"

#include "fileIO/Serializable.h"

#include "geometry/Mesh.h"
#include "geometry/ConvexHull.h"
#include "geometry/shape/Sphere.h"

#include "geometry/Transformable.h"


class RigidBody : public Serializable, public Transformable {
public:

    enum class Type {
        STATIC = 0, KINEMATIC, DYNAMIC
    };

    explicit RigidBody(const std::string& filename);
    explicit RigidBody(const std::shared_ptr<Mesh>& mesh);
    explicit RigidBody(const ConvexHull& hull);

    bool serialize(const std::string& filename) override;
    bool serialize(std::ofstream& file) override;

    bool deserialize(const std::string& filename) override;
    bool deserialize(std::ifstream& file) override;

    void setType(Type type);

    void setMesh(const std::shared_ptr<Mesh>& mesh, bool doColliderUpdate = false);

    void setLayer(uint32_t layer);
    void setMask(uint32_t mask);

    void setDensity(float density);

    void step(float delta);

    void zero();

    void updateColliders();
    void prepareColliderVisuals();

    Type getType() const;

    bool isManifold();
    float area();

    float volume();
    float density() const;
    float mass();
    glm::vec3 centroid();
    glm::mat3x3 inertiaTensor();

    void setLinearVelocity(glm::vec3 velocity);
    [[nodiscard]] const glm::vec3& getLinearVelocity() const;
    void setAngularVelocity(glm::vec3 velocity);
    [[nodiscard]] const glm::vec3& getAngularVelocity() const;

    const std::shared_ptr<Mesh>& mesh();

    const ConvexHull &hull() const;
    const Sphere& boundingSphere() const;

    const std::shared_ptr<Mesh>& hullMesh();
    const std::shared_ptr<Mesh>& bSphereMesh();


    [[nodiscard]] uint32_t layer() const;
    [[nodiscard]] uint32_t mask() const;
    [[nodiscard]] bool scan(const std::shared_ptr<RigidBody>& body) const;

    bool boundaryCollision(const std::shared_ptr<RigidBody>& body);

    bool collides(const std::shared_ptr<RigidBody>& body);
    bool collision(const std::shared_ptr<RigidBody>& body, glm::vec3& offset);

    EPA collision(const std::shared_ptr<RigidBody>& body);

private:

    void prepareHullVisual();
    void prepareSphereVisual();


    void cacheCollision(const std::shared_ptr<RigidBody>& body, const std::pair<uint32_t, uint32_t>& start);
    std::pair<uint32_t, uint32_t> cachedCollision(const std::shared_ptr<RigidBody>& body);

    void evaluateManifold();
    void calculateArea();
    void calculateVolume();
    void calculateMass();
    void calculateInertiaTensor();

protected:

    Type m_type;

    std::shared_ptr<Mesh> m_mesh;

    ConvexHull m_hull;
    bool m_hullOK;

    Sphere m_boundingSphere;

    uint32_t m_layer;
    uint32_t m_mask;

    std::shared_ptr<Mesh> m_hullMesh;
    std::shared_ptr<Mesh> m_sphereMesh;
    bool m_colliderVisualsEnable;


    bool m_isManifold;
    float m_area;
    float m_volume;
    float m_density;
    float m_mass;
    glm::mat3x3 m_inertiaTensor;

    bool m_isManifoldOK;
    bool m_areaOK;
    bool m_volumeOK;
    bool m_massOK;
    bool m_inertiaTensorOK;

    glm::vec3 m_linearVelocity;
    glm::vec3 m_angularVelocity;


private:



};


#endif //AUTOCARVER_RIGIDBODY_H
