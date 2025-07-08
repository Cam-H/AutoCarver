//
// Created by Cam on 2024-11-12.
//

#ifndef AUTOCARVER_SCULPTPROCESS_H
#define AUTOCARVER_SCULPTPROCESS_H

#include "Scene.h"

#include <vector>

#include "geometry/VertexArray.h"
#include "geometry/Mesh.h"
#include "core/Sculpture.h"

class Profile;

class SculptProcess : public Scene {
public:

    struct Configuration {
        float materialWidth = 1.0f;
        float materialHeight = 2.0f;
    };

    explicit SculptProcess(const std::shared_ptr<Mesh>& model);
    ~SculptProcess();

    void enableConvexTrim(bool enable);
    void enableProcessCut(bool enable);

    void plan();

    void proceed();
    void skip();

    void step(float delta) override;

    void setSculptingRobot(const std::shared_ptr<Robot>& robot);
    void setContinuous(bool enable);

    bool simulationComplete() const;
    bool simulationIdle() const;
    bool simulationActive() const;

//    std::shared_ptr<Mesh> sculpture();

private:

    struct Action {
        Action (const std::shared_ptr<Trajectory>& trajectory, const std::shared_ptr<Robot>& robot)
                : trajectory(trajectory)
                , target(robot)
                , started(false)
                , trigger(false) {}


        std::shared_ptr<Trajectory> trajectory;
        std::shared_ptr<Robot> target;
        bool started;
        bool trigger;
    };

    void planConvexTrim();
    void planOutlineRefinement(float stepDg);
    void planOutlineRefinement(Profile& profile);

    void planFeatureRefinement();

    void planTurntableAlignment(const glm::vec3& axis);
    void planTurntableAlignment(float theta);

    void planRoboticSection(const glm::vec3& a, const glm::vec3& b);
    void planRoboticSection(const std::vector<glm::vec3>& border);

    void nextAction();

    static uint32_t identifySculpture(const std::vector<std::shared_ptr<Mesh>>& fragments);

    Configuration m_config;

    std::shared_ptr<Mesh> model;
    std::shared_ptr<Mesh> m_reference;
    std::shared_ptr<Sculpture> m_sculpture;

    std::shared_ptr<Robot> m_turntable;
    std::shared_ptr<Robot> m_robot;

    std::vector<Action> m_actions;

    bool m_planned;
    bool m_convexTrimEnable;
    bool m_processCutEnable;

    uint32_t m_step;
    bool m_continuous;

};


#endif //AUTOCARVER_SCULPTPROCESS_H
