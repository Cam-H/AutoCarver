//
// Created by Cam on 2024-10-21.
//

#ifndef AUTOCARVER_SCENE_H
#define AUTOCARVER_SCENE_H

#include <reactphysics3d/reactphysics3d.h>

#include <vector>
#include <thread>

#include "geometry/Body.h"

class Scene {
public:

    Scene();

    void start();
    void pause();

    void update(float timestep);

    Body* createBody(const std::string &filepath, rp3d::BodyType type = rp3d::BodyType::STATIC);
    Body* createBody(const std::shared_ptr<Mesh>& mesh, rp3d::BodyType type = rp3d::BodyType::STATIC);

private:
    void update();

    void sync();
private:

    rp3d::PhysicsCommon m_physicsCommon;
    rp3d::PhysicsWorld *m_world;

    std::vector<Body*> m_bodies;

    std::thread m_updateThread;
    bool m_paused;
//    std::vector<Qt3DRender::QMesh*> m_meshes;
//    std::vector<rp3d::RigidBody*> m_physBodies;

};


#endif //AUTOCARVER_SCENE_H
