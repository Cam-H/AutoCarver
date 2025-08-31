//
// Created by Cam on 2024-10-21.
//

#ifndef AUTOCARVER_SCENE_H
#define AUTOCARVER_SCENE_H

#include <vector>
#include <thread>
#include <fstream>

#include "fileIO/Serializable.h"
#include "physics/RigidBody.h"
#include "robot/Robot.h"
#include "geometry/primitives/Ray.h"

class RenderBuffer;

class Scene : public Serializable {
public:

    enum class Model {
        ALL = 0, MESH, HULL, BOUNDING_SPHERE, AABB, AXES
    };

    Scene();
    ~Scene();

    bool serialize(std::ofstream& file) const override;
    bool deserialize(std::ifstream& file) override;

    void start();
    void pause();
    virtual void step(double delta);
    void update();

    void stop();

    void setTimeScaling(double scalar);

    void enableCollisionColoring(bool enable);

    void clear(uint8_t level = 0);

    void addRenderBuffer(const std::shared_ptr<RenderBuffer>& buffer);
    void clearRenderBuffers();

    std::shared_ptr<Body> createVisual(const std::shared_ptr<Mesh>& mesh);

    std::shared_ptr<RigidBody> createBody(const std::string &filepath, RigidBody::Type type = RigidBody::Type::STATIC);
    std::shared_ptr<RigidBody> createBody(const std::shared_ptr<Mesh>& mesh, RigidBody::Type type = RigidBody::Type::STATIC);
    std::shared_ptr<RigidBody> createBody(const ConvexHull& hull, RigidBody::Type type = RigidBody::Type::STATIC);

    std::shared_ptr<Robot> createRobot(const std::shared_ptr<KinematicChain>& kinematics);

    [[nodiscard]] const std::vector<std::shared_ptr<RigidBody>>& bodies() const;
    [[nodiscard]] const std::vector<std::shared_ptr<Body>>& visualBodies() const;
    [[nodiscard]] uint32_t bodyCount() const;

//    std::vector<const std::shared_ptr<Mesh>&> meshes();
    void prepareBody(const std::shared_ptr<RigidBody>& body, uint8_t level = 0);

    [[nodiscard]] bool isPaused() const;

    [[nodiscard]] std::tuple<std::shared_ptr<RigidBody>, double> raycast(const Ray& ray) const;

    [[nodiscard]] bool test(const std::shared_ptr<Robot>& robot) const;
    [[nodiscard]] bool test(const std::shared_ptr<RigidBody>& body) const;

    void print() const;

protected:

    void post();

private:
    void run();

    void colorCollision(const std::shared_ptr<Robot>& robot);

protected:

    uint32_t m_total;

    std::vector<std::shared_ptr<RigidBody>> m_bodies;
    std::vector<std::shared_ptr<Robot>> m_robots;

    std::vector<std::shared_ptr<Body>> m_vis;

    std::unique_ptr<std::thread> m_updateThread;
    bool m_running;
    bool m_paused;

    double m_timeScalar;

    bool m_colorCollisions;

    std::vector<std::shared_ptr<RenderBuffer>> m_buffers;

};


#endif //AUTOCARVER_SCENE_H
