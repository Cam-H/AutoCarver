//
// Created by Cam on 2024-09-20.
//

#ifndef AUTOCARVER_RIGIDBODY_H
#define AUTOCARVER_RIGIDBODY_H

#include <fstream>

// Mesh manipulation

#include <vector>
#include "glm.hpp"

#include "physics/Body.h"

#include "geometry/Mesh.h"
#include "geometry/primitives/ConvexHull.h"
#include "geometry/primitives/Sphere.h"
#include "geometry/primitives/Ray.h"

#include "geometry/collision/EPA.h"


class RigidBody : public Body {
public:

    enum class Type {
        STATIC = 0, KINEMATIC, DYNAMIC
    };

    explicit RigidBody(const std::string& filename);
    explicit RigidBody(const std::shared_ptr<Mesh>& mesh);
    explicit RigidBody(const ConvexHull& hull);

    bool serialize(std::ofstream& file) const override;
    bool deserialize(std::ifstream& file) override;

    void moved() override;

    void setType(Type type);

    void setMesh(const std::shared_ptr<Mesh>& mesh, bool doColliderUpdate = false);
    void setHull(const ConvexHull& hull);

    void setMask(uint32_t mask);
    void setLayer(uint32_t layer);

    void disableCollisions();

    void resetMask();
    void resetLayer();

    void setDensity(double density);

    void step(double delta) override;

    void zero();

    void updateColliders();
    void prepareColliderVisuals();

    [[nodiscard]] Type getType() const;

    bool isManifold();
    double area();

    double volume();
    double density() const;
    double mass();
    glm::dvec3 centroid();
    glm::dmat3 inertiaTensor();

    const ConvexHull &hull() const;
    const Sphere& boundingSphere() const;

    const std::shared_ptr<Mesh>& hullMesh();
    const std::shared_ptr<Mesh>& bSphereMesh();


    [[nodiscard]] uint32_t layer() const;
    [[nodiscard]] uint32_t mask() const;
    [[nodiscard]] bool scan(const std::shared_ptr<RigidBody>& body) const;

    bool boundaryCollision(const std::shared_ptr<RigidBody>& body);

    bool collides(const std::shared_ptr<RigidBody>& body);
    bool collision(const std::shared_ptr<RigidBody>& body, glm::dvec3& offset);

    EPA collision(const std::shared_ptr<RigidBody>& body);

    virtual std::tuple<bool, double> raycast(Ray ray, double tLim);
    virtual std::tuple<bool, double, uint32_t> pickFace(Ray ray, double tLim);

    void print() const;

protected:

    RigidBody();

private:

    void prepareColliders();


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

    ConvexHull m_hull;
    bool m_hullOK;

    Sphere m_boundingSphere;

    uint32_t m_mask;
    uint32_t m_layer;

    std::shared_ptr<Mesh> m_hullMesh;
    std::shared_ptr<Mesh> m_sphereMesh;
    bool m_colliderVisualsEnable;

    bool m_isManifold;
    double m_area;
    double m_volume;
    double m_density;
    double m_mass;
    glm::dmat3 m_inertiaTensor;

    bool m_isManifoldOK;
    bool m_areaOK;
    bool m_volumeOK;
    bool m_massOK;
    bool m_inertiaTensorOK;

private:



};


#endif //AUTOCARVER_RIGIDBODY_H
