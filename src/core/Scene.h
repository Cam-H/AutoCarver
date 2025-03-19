//
// Created by Cam on 2024-10-21.
//

#ifndef AUTOCARVER_SCENE_H
#define AUTOCARVER_SCENE_H

#include <Qt3DCore/QEntity>
#include <Qt3DExtras/Qt3DWindow>

#include <reactphysics3d/reactphysics3d.h>

#include <vector>
#include <thread>

#include "geometry/Body.h"
#include "renderer/RenderEntity.h"


class Scene {
public:

    enum class Model {
        ALL = 0, MESH, HULL, BOUNDING_SPHERE
    };

    Scene();
    ~Scene();

    void start();
    void pause();

//    void linkRenderer(Qt3DCore::QEntity *parent, Qt3DExtras::Qt3DWindow *view);

//    void showAll();
//    void hideAll();
//
//    void show(uint32_t idx, Model target = Model::ALL);
//    void hide(uint32_t idx, Model target = Model::ALL);

    void translateBody(uint32_t idx, float w, float x, float y, float z);
    void rotateBody(uint32_t idx, float w, float x, float y, float z);

    void update(float timestep);

    void clear(uint8_t level = 0);

    void createBody(const std::string &filepath, rp3d::BodyType type = rp3d::BodyType::STATIC);
    void createBody(const std::shared_ptr<Mesh>& mesh, rp3d::BodyType type = rp3d::BodyType::STATIC);

    const std::vector<Body*>& bodies();
    uint32_t bodyCount();

//    std::vector<const std::shared_ptr<Mesh>&> meshes();

protected:
    void prepareBody(Body *body, uint8_t level = 0);
//    RenderEntity* prepareRender(Body *body);

private:
    void update();


//    void sync();
//    static void sync(rp3d::RigidBody *physics, RenderEntity *render);

protected:

//    struct SceneEntity {
//        Body* body;
//        RenderEntity* render;
//        uint8_t level;
//    };

    rp3d::PhysicsCommon m_physicsCommon;
    rp3d::PhysicsWorld *m_world;

//    Qt3DCore::QEntity *m_root;
//    Qt3DExtras::Qt3DWindow *view;

    std::vector<Body*> m_bodies;
//    std::vector<SceneEntity> m_entities;

    std::thread m_updateThread;
    bool m_paused;
//    std::vector<Qt3DRender::QMesh*> m_meshes;
//    std::vector<rp3d::RigidBody*> m_physBodies;

};


#endif //AUTOCARVER_SCENE_H
