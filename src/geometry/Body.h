//
// Created by Cam on 2024-09-20.
//

#ifndef AUTOCARVER_BODY_H
#define AUTOCARVER_BODY_H

// Physics simulation
#include <reactphysics3d/reactphysics3d.h>

// Mesh manipulation

#include <vector>

#include "Mesh.h"
#include "ConvexHull.h"

class Body {
public:

    explicit Body(const std::shared_ptr<Mesh> &mesh);
    explicit Body(rp3d::PhysicsCommon *phys, rp3d::PhysicsWorld *world, const std::shared_ptr<Mesh>& mesh);

    ~Body();

    bool isManifold();
    float area();
    float volume();

    const std::shared_ptr<Mesh>& mesh();

    const ConvexHull &hull();
    const ConvexHull &hull() const;

    const std::shared_ptr<Mesh>& hullMesh();

    rp3d::RigidBody *physicsBody();

private:

    void prepareColliders();

    void evaluateManifold();
    void calculateArea();
    void calculateVolume();

protected:

    std::shared_ptr<Mesh> m_mesh;

    ConvexHull m_hull;
    std::shared_ptr<Mesh> m_hullMesh;

    bool m_hullOK;


    // Physics
    bool m_physEnabled;
    rp3d::PhysicsCommon *phys;
    rp3d::PhysicsWorld *world;

    rp3d::RigidBody *m_physBody;
    std::vector<rp3d::Collider*> m_colliders;

    bool m_isManifold;
    float m_area;
    float m_volume;

    bool m_isManifoldOK;
    bool m_areaOK;
    bool m_volumeOK;

private:



};


#endif //AUTOCARVER_BODY_H
