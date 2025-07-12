//
// Created by Cam on 2024-11-12.
//

#include "SculptProcess.h"

#include <glm.hpp>

#include "Sculpture.h"
#include "geometry/poly/Profile.h"
#include "renderer/EdgeDetect.h"
#include "renderer/RenderCapture.h"

#include "robot/planning/Trajectory.h"
#include "geometry/MeshBuilder.h"

SculptProcess::SculptProcess(const std::shared_ptr<Mesh>& model)
    : Scene()
    , model(model)
    , m_sculpture(nullptr)
    , m_step(0)
    , m_planned(false)
    , m_convexTrimEnable(true)
    , m_processCutEnable(true)
    , m_continuous(false)
{
    model->normalize();

    m_reference = std::make_shared<Mesh>(*model);
    m_reference->center();

    m_sculpture = std::make_shared<Sculpture>(model, m_config.materialWidth, m_config.materialHeight);

    prepareBody(m_sculpture, 1);
    prepareBody(m_sculpture->model(), 1);

    auto *chain = new KinematicChain;
    chain->addJoint(Joint(Joint::Type::REVOLUTE, { 0.5f, 0.0f, 0.0f, 0.0f }, 0));
    m_turntable = createRobot(chain);
    m_turntable->setEOAT(m_sculpture);

}

SculptProcess::~SculptProcess()
{
    std::cout << "SP destroy\n";
}

void SculptProcess::enableConvexTrim(bool enable)
{
    m_convexTrimEnable = enable;
}
void SculptProcess::enableProcessCut(bool enable)
{
    m_processCutEnable = enable;
}

void SculptProcess::plan()
{
    if (m_planned) {
        std::cout << "\033[33mWarning! The process has already been planned\033[0m\n";
        return;
    } else if (m_robot == nullptr) {
        std::cout << "\033[33mWarning! No sculpting robot has been provided\033[0m\n";
        return;
    }

    // Ready the sculpture by carving the convex hull of the model
    if (m_convexTrimEnable) planConvexTrim();
    else m_sculpture->restoreAsHull();

    // Repeatedly refined the sculpture by cutting to the outline of the model at discrete orientations
    planOutlineRefinement(90.0f);

    // Capture model features with fine carving
    planFeatureRefinement();


    m_planned = true;
    m_step = 0;
}

void SculptProcess::proceed()
{
    if (!m_planned) plan();

    // Block procession before previous step is complete
    if (simulationIdle()) {
        std::cout << "PROC| Next step: " << (m_step + 1) << " / " << m_actions.size() << " -> " << m_actions[m_step].target->getWaypoint() << "\n";

        nextAction();
    }
}

void SculptProcess::skip()
{
    if (!m_planned) plan();

    if (simulationComplete()) return;

    std::cout << "SKIP| Next step: " << (m_step + 1) << " / " << m_actions.size() << " -> " << m_actions[m_step].target->getWaypoint() << "\n";

    // Capture intermediary motions (In case turntable moves) while looking for trigger to indicate a stop
    while (m_step < m_actions.size() && !m_actions[m_step].trigger) {
        m_actions[m_step].target->moveTo( m_actions[m_step].trajectory->end());
        m_step++;
    }

    // Jump robot to endpoint and apply process to the sculpture
    if (m_step < m_actions.size()) {
        m_actions[m_step].target->moveTo( m_actions[m_step].trajectory->end());
        m_sculpture->section();
        m_step++;
    }
}

void SculptProcess::step(float delta)
{
    Scene::step(delta);

    if (simulationIdle()) { // Handle transition between simulation actions
        if (m_actions[m_step].started) {
            if (m_actions[m_step].trigger) m_sculpture->section();
            m_step++;
        } else if (m_continuous) nextAction();
    }
}

void SculptProcess::setSculptingRobot(const std::shared_ptr<Robot>& robot){
    m_robot = robot;
}

void SculptProcess::setContinuous(bool enable)
{
    m_continuous = enable;
}

bool SculptProcess::simulationComplete() const
{
    return m_step >= m_actions.size();
}

bool SculptProcess::simulationIdle() const
{
    return !simulationComplete() && !m_actions[m_step].target->inTransit();
}

bool SculptProcess::simulationActive() const
{
    return !simulationComplete() && (m_actions[m_step].target->inTransit() || m_continuous);
}

void SculptProcess::planConvexTrim()
{
    struct Operation {
        glm::vec3 origin;
        glm::vec3 normal;
    };

    std::vector<Operation> steps;

    ConvexHull hull = ConvexHull(model);

    for (uint32_t i = 0; i < hull.facetCount(); i++) {
        uint32_t idx = hull.faces()[i][0];
        steps.push_back({hull.vertices()[idx], -hull.facetNormal(i)});
    }

    // Plan initial cuts, beginning from the top and moving towards the base
    std::sort(steps.begin(), steps.end(), [](const Operation& a, const Operation& b){
        return glm::dot(a.normal, {0, 1, 0}) < glm::dot(b.normal, {0, 1, 0});
    });

    glm::vec3 p = m_sculpture->position();
    std::cout << p.x << " " << p.y << " " << p.z << "~~~~|\n";
    p = m_sculpture->up();
    std::cout << p.x << " " << p.y << " " << p.z << "~~~~|\n";

    // Process all the cuts preemptively
    for (const Operation& step : steps) {

        // TODO Verify length / access - Get min/max
        auto border = hull.intersection(step.origin, step.normal);
        if (border.size() < 3) continue;
//        if (border.size() < 3) throw std::runtime_error("[SculptProcess] Failed to find intersection for section");

//        glm::vec3 normal = Triangle::normal(border[0], border[1], border[2]);

        // Generate trajectory for robotic carving
        planTurntableAlignment(step.normal);
//        planRoboticSection(min, max);

        // Record mesh manipulation operation for later application during carving process
        m_sculpture->recordSection(step.origin, step.normal);

        // Prepare for next step
        hull = hull.fragment(step.origin, step.normal);
    }
}

void SculptProcess::planOutlineRefinement(float stepDg)
{
    uint32_t steps = std::floor(180.0f / stepDg);
    float angle = 0;

    // Prepare a 3D render / edge detection system to find the profile of the model
    auto* detector = new EdgeDetect(m_reference);

    detector->setSize(1000);
    detector->setEpsilon(120.0f);

    detector->capture()->camera().setViewingAngle(0, 0);
    detector->capture()->focus();

//    detector->capture()->capture();
//    detector->capture()->grabFramebuffer().save(QString("SPOutlineRefinementPrecheck.png"));

    glm::vec3 offset = -model->boundedOffset();
    float scalar = m_sculpture->scalar();
    std::cout << "OFFSET: " << offset.x << " " << offset.y << " " << offset.z << " " << scalar << "\n";


    for (uint32_t i = 0; i < steps; i++) {

        // Render and determine the profile of the result
        detector->update();

        // Save the detection result for debugging purposes
        std::string filename = "SPOutlineRefinement" + std::to_string(angle) + "dg.png";
        detector->sink().save(QString(filename.c_str()));

        // Move the camera for the next image (in dg)
        detector->capture()->camera().rotate(stepDg);
        detector->capture()->focus();
        angle += stepDg;

        // Use the turntable to align the profile with the carving robot
        planTurntableAlignment(m_sculpture->rotation() - angle * (float)M_PI / 180.0f + M_PI / 2);

        std::cout << angle << " " << (angle * (float)M_PI / 180.0f) << " " << m_sculpture->rotation() << " RR\n";

        // Use the profile to prepare appropriate sculpting instructions for the robots
        auto profile = detector->profile();
        profile.setRefinementMethod(Profile::RefinementMethod::DELAUNEY);
//        profile.rotateAbout({ 0, 1, 0 }, -m_sculpture->rotation());
//        profile.centerScale(scalar);
        profile.scale(scalar);
        profile.translate(offset);
//        profile.centerScale(scalar);
//        profile.scale(scalar);
        planOutlineRefinement(profile);
        break;
    }
}

void SculptProcess::planOutlineRefinement(Profile& profile)
{
    auto mesh = MeshBuilder::extrude(profile.projected3D(), profile.normal(), 2.0f);
    mesh->translate(-profile.normal());
    mesh->setFaceColor({0, 0, 1});
    createBody(mesh);

//    profile.correctWinding();

    uint32_t count = 0;
    while (!profile.complete() && count < 10) {

        auto indices = profile.refine();
        auto border = profile.projected3D(indices);

        planRoboticSection(border);

        m_sculpture->recordSection(border, profile.normal());

//        std::cout << "Profile: " << border.size() << ": \n";
//        for (auto& v : border) std::cout << v.x << " " << v.y << " " << v.z << "\n";
        createBody(MeshBuilder::extrude(border, profile.normal(), 2.0f));
        count++;
    }

    std::cout << "Refinement iterations: " << count << ":\n";
    std::cout << m_sculpture->position().x << " " << m_sculpture->position().y << " " << m_sculpture->position().z << "\n";

}

void SculptProcess::planFeatureRefinement()
{
    //TODO
    // Refine features based on layers?
    // Decompose model into patches and handle separately?
}

void SculptProcess::planTurntableAlignment(const glm::vec3& axis)
{
    glm::vec3 up = { 0, 1, 0 }, proj = axis - up * glm::dot(axis, up);
    float theta = atan2f(proj.x, -proj.z);

//    std::cout << "TEST========================\n"
//        << atan2f(1, 0) << " " << atan2f(0, 1) << " " << atan2f(-1, 0) << " " << atan2f(0, -1) << " "
//        << atan2f(1, 1) << " " << atan2f(-1, 1) << " " << atan2f(-1, -1) << " " << atan2f(1, -1) << "\n";

//    std::cout << proj.x << " " << proj.y << " " << proj.z << " | " << theta << " Theta\n";

    planTurntableAlignment(theta);
}
void SculptProcess::planTurntableAlignment(float theta)
{
    std::vector<Waypoint> waypoints = {
            { std::vector<float>{ theta }, 1, false}
    };

    auto trajectory = std::make_shared<Trajectory>(waypoints, TrajectorySolverType::CUBIC);
    trajectory->setMaxVelocity(M_PI / 2);

    m_actions.emplace_back(trajectory, m_turntable);
}

void SculptProcess::planRoboticSection(const glm::vec3& a, const glm::vec3& b)
{
//    glm::vec3 delta = m_sculpture->position() - m_robot->position(), uDel = glm::normalize(delta);
//    std::cout << "DEL: " << delta.x << " " << delta.y << " " << delta.z <<"\n";
//    glm::vec3 origin = border[0] - uDel * glm::dot(uDel, border[0]);
//
//    glm::vec4 calc = glm::vec4{ origin.x, origin.y, origin.z, 1 } * m_robot->getTransform();
//    std::vector<Waypoint> waypoints = {
//            m_robot->inverse({ calc.x, calc.y, calc.z }, { 0, -M_PI / 2, 0 })
//    };
//
//    auto trajectory = std::make_shared<Trajectory>(waypoints, TrajectorySolverType::CUBIC);
//    trajectory->setMaxVelocity(M_PI / 2);
//
//    m_actions.emplace_back(trajectory, m_robot);
//    m_actions[m_actions.size() - 1].trigger = true;
}

void SculptProcess::planRoboticSection(const std::vector<glm::vec3>& border)
{
    std::vector<Waypoint> waypoints = {
            m_robot->getWaypoint(),
            m_robot->getWaypoint()
    };

    auto trajectory = std::make_shared<Trajectory>(waypoints, TrajectorySolverType::CUBIC);
    trajectory->setMaxVelocity(M_PI / 2);

    m_actions.emplace_back(trajectory, m_robot);
    m_actions[m_actions.size() - 1].trigger = true;
}

void SculptProcess::nextAction()
{
    auto& action = m_actions[m_step];

    // Insert deferred start waypoints
    if (action.trajectory->waypointCount() < 2)
        action.trajectory->insertWaypoint(0, action.target->getWaypoint());

    // Begin motion of the robot
    action.target->traverse(action.trajectory);
    action.started = true;
}

uint32_t SculptProcess::identifySculpture(const std::vector<std::shared_ptr<Mesh>>& fragments)
{
    uint32_t idx = 0;
    float volume = fragments[0]->volume();

    for (uint32_t i = 1; i < fragments.size(); i++) {
        float temp = fragments[i]->volume();

        if (volume < temp) {
            volume = temp;
            idx = i;
        }
    }

    return idx;
}