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

class CompositeBody;

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

    void setMesh(const std::shared_ptr<Mesh>& mesh);
    void setHull(const ConvexHull& hull);

    void setMask(uint32_t mask);
    void setLayer(uint32_t layer);

    void ignore(const std::shared_ptr<RigidBody>& body);

    void disableCollisions();

    void resetMask();
    void resetLayer();

    void setDensity(double density);

    void step(double delta) override;

    void zero();
    virtual void recenter(const glm::dvec3& offset);

    void updateHull();

    [[nodiscard]] Type getType() const;

    bool isManifold();
    double area();

    double volume();
    double density() const;
    double mass();
    glm::dvec3 centroid();
    glm::dmat3 inertiaTensor();

    [[nodiscard]] const ConvexHull& hull() const;
    [[nodiscard]] Sphere bounds() const;

    const std::vector<ConvexHull>& components() const;

    [[nodiscard]] uint32_t layer() const;
    [[nodiscard]] uint32_t mask() const;
    [[nodiscard]] inline bool scan(const std::shared_ptr<RigidBody>& body) const { return body.get() != this && ((m_mask & body->m_layer) > 0); }

    virtual bool boundsTest(const std::shared_ptr<RigidBody>& body);

//    virtual bool test(const std::shared_ptr<CompositeBody>& body);
    virtual bool test(const std::shared_ptr<RigidBody>& body);

    std::tuple<bool, glm::dvec3> delta(const std::shared_ptr<RigidBody>& body);
    EPA intersection(const std::shared_ptr<RigidBody>& body);

    virtual std::tuple<bool, double> raycast(Ray ray, double tLim);
    virtual std::tuple<bool, double, uint32_t> pickFace(Ray ray, double tLim);

    void print() const;

protected:

    RigidBody();

//    bool testHulls(const std::vector<ConvexHull>& hulls, const glm::dmat4& relative);

    virtual bool precheck(uint32_t hullID0, uint32_t hullID1, const glm::dmat4& relative) const;

private:

    void cacheCollision(const std::shared_ptr<RigidBody>& body, const std::pair<uint32_t, uint32_t>& start);
    std::pair<uint32_t, uint32_t> cachedCollision(const std::shared_ptr<RigidBody>& body);

    void evaluateManifold();
    void calculateArea();
    void calculateVolume();
    void calculateMass();
    void calculateInertiaTensor();

protected:

    Type m_type;

    // A list of hulls composing the rigid body [The first hull is the convex hull of the entire body]
    ConvexHull m_hull;

    uint32_t m_mask;
    uint32_t m_layer;

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
