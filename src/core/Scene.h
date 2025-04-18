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
#include <fstream>

#include "fileIO/Serializable.h"
#include "geometry/Body.h"
#include "renderer/RenderEntity.h"
#include "robot/Robot.h"


class Scene : public Serializable {
public:

    enum class Model {
        ALL = 0, MESH, HULL, BOUNDING_SPHERE, AABB
    };

    Scene();
    ~Scene();

    bool serialize(const std::string& filename) override;
    bool serialize(std::ofstream& file) override;

    bool deserialize(const std::string& filename) override;
    bool deserialize(std::ifstream& file) override;

    void start();
    void pause();
    void stop();

    void clear(uint8_t level = 0);

    std::shared_ptr<Body> createBody(const std::string &filepath, rp3d::BodyType type = rp3d::BodyType::STATIC);
    std::shared_ptr<Body> createBody(const std::shared_ptr<Mesh>& mesh, rp3d::BodyType type = rp3d::BodyType::STATIC);

    std::shared_ptr<Robot> createRobot(KinematicChain* kinematics);

    const std::vector<std::shared_ptr<Body>>& bodies();
    uint32_t bodyCount();

//    std::vector<const std::shared_ptr<Mesh>&> meshes();

protected:
    void prepareBody(const std::shared_ptr<Body>& body, uint8_t level = 0);

private:
    void run();
    void update();

protected:

    rp3d::PhysicsCommon m_physicsCommon;
    rp3d::PhysicsWorld *m_world;

    std::vector<std::shared_ptr<Body>> m_bodies;
    std::vector<std::shared_ptr<Robot>> m_robots;

    std::unique_ptr<std::thread> m_updateThread;
    bool m_running;
    bool m_paused;

};


#endif //AUTOCARVER_SCENE_H
