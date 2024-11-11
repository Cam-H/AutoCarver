//
// Created by Cam on 2024-09-20.
//

#ifndef AUTOCARVER_BODY_H
#define AUTOCARVER_BODY_H

// Rendering
#include <Qt3DCore/QTransform>

// Physics simulation
#include <reactphysics3d/reactphysics3d.h>

// Mesh manipulation


#include <vector>

#include "Mesh.h"
#include "ConvexHull.h"

#include "renderer/RenderEntity.h"

class Body {
public:

    enum class Model {
        ALL = 0, MESH, HULL, BOUNDING_SPHERE
    };

    explicit Body(const std::shared_ptr<Mesh> &mesh);
    explicit Body(rp3d::PhysicsCommon *phys, rp3d::PhysicsWorld *world, const std::shared_ptr<Mesh>& mesh);

    virtual void setRenderer(Qt3DCore::QEntity *parent, Qt3DExtras::Qt3DWindow *view);
    void updateRenderer();

    void show(Model model = Model::ALL);
    void hide(Model model = Model::ALL);

    RenderEntity *getRenderEntity();

    bool isManifold();
    float area();
    float volume();

//    Mesh &mesh();
//    Mesh *hullMesh();

    const ConvexHull &hull();
    const ConvexHull &hull() const;

    rp3d::RigidBody *physicsBody();

    void translate(float x, float y, float z);
    void rotate(float w, float x, float y, float z);

    void sync();

private:

    void prepareColliders();

    void evaluateManifold();
    void calculateArea();
    void calculateVolume();

protected:

    //    std::vector<Surface> m_surfaces;
//
//    Tesselation m_tesselation;

    std::shared_ptr<Mesh> m_mesh;

    ConvexHull m_hull;
    bool m_hullOK;

    RenderEntity *m_render;

    //

    // Physics
    bool m_physEnabled;
    rp3d::PhysicsCommon *s_phys;

    rp3d::RigidBody *m_physBody;
    std::vector<rp3d::Collider*> m_colliders;

    bool m_isManifold;
    float m_area;
    float m_volume;

    bool m_tesselationOK;
    bool m_isManifoldOK;
    bool m_areaOK;
    bool m_volumeOK;

private:



};


#endif //AUTOCARVER_BODY_H
