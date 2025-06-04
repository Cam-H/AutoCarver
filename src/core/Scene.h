//
// Created by Cam on 2024-10-21.
//

#ifndef AUTOCARVER_SCENE_H
#define AUTOCARVER_SCENE_H

#include <Qt3DCore/QEntity>
#include <Qt3DExtras/Qt3DWindow>

#include <vector>
#include <thread>
#include <fstream>

#include "fileIO/Serializable.h"
#include "geometry/RigidBody.h"
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
    void step(float delta);
    void stop();

    void connect(void(*function)());

    void clear(uint8_t level = 0);

    std::shared_ptr<RigidBody> createBody(const std::string &filepath, RigidBody::Type type = RigidBody::Type::STATIC);
    std::shared_ptr<RigidBody> createBody(const std::shared_ptr<Mesh>& mesh, RigidBody::Type type = RigidBody::Type::STATIC);
    std::shared_ptr<RigidBody> createBody(const ConvexHull& hull, RigidBody::Type type = RigidBody::Type::STATIC);

    std::shared_ptr<Robot> createRobot(KinematicChain* kinematics);

    const std::vector<std::shared_ptr<RigidBody>>& bodies();
    uint32_t bodyCount();

//    std::vector<const std::shared_ptr<Mesh>&> meshes();

protected:
    void prepareBody(const std::shared_ptr<RigidBody>& body, uint8_t level = 0);

private:
    void run();
    void update();

protected:



    std::vector<std::shared_ptr<RigidBody>> m_bodies;
    std::vector<std::shared_ptr<Robot>> m_robots;

    std::unique_ptr<std::thread> m_updateThread;
    bool m_running;
    bool m_paused;

    std::vector<void(*)()> callbacks;

};


#endif //AUTOCARVER_SCENE_H
