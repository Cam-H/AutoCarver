//
// Created by Cam on 2024-11-12.
//

#include "SculptProcess.h"

#include <glm.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/string_cast.hpp>
#include <gtx/quaternion.hpp>

#include "Sculpture.h"
#include "geometry/poly/Profile.h"
#include "renderer/EdgeDetect.h"
#include "renderer/RenderCapture.h"

#include "robot/planning/Trajectory.h"
#include "geometry/MeshBuilder.h"
#include "geometry/collision/Collision.h"

SculptProcess::SculptProcess(const std::shared_ptr<Mesh>& model)
    : Scene()
    , model(model)
    , m_sculpture(nullptr)
    , m_step(0)
    , m_planned(false)
    , m_convexTrimEnable(true)
    , m_processCutEnable(true)
    , m_continuous(false)
    , m_fwd(1, 0, 0)
    , m_lastestRobotCommand()
{
    model->center();
    model->normalize();

    m_sculpture = std::make_shared<Sculpture>(model, m_config.materialWidth, m_config.materialHeight);


    prepareBody(m_sculpture, 1);
    prepareBody(m_sculpture->model(), 1);

    auto chain = std::make_shared<KinematicChain>();
    chain->addJoint(Joint(Joint::Type::REVOLUTE, { 0.5f, 0.0f, 0.0f, 0.0f }, 0));

    m_turntable = createRobot(chain);
    m_turntable->setEOAT(m_sculpture);

    m_lastestTableCommand = m_turntable->getWaypoint();
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
//    planOutlineRefinement(90.0f);

    // Capture model features with fine carving
//    planFeatureRefinement();

    m_planned = true;
    m_step = 0;
}

void SculptProcess::proceed()
{
    if (!m_planned) plan();

    // Block procession before previous step is complete
    if (simulationIdle()) {

        nextAction();
    } else if (simulationComplete()) std::cout << "Simulation complete!\n";
}

void SculptProcess::skip()
{
    if (!m_planned) plan();

    if (simulationComplete()) {
        std::cout << "Simulation complete!\n";
        return;
    }


    std::cout << "SKIP| Next step: " << (m_step + 1) << " / " << m_actions.size() << " -> " << m_actions[m_step].target->getWaypoint() << "\n";

    // Capture intermediary motions (In case turntable moves) while looking for trigger to indicate a stop
    while (m_step < m_actions.size() && !m_actions[m_step].trigger) {
        m_actions[m_step].target->moveTo( m_actions[m_step].trajectory->end());
        m_step++;
    }

    // Jump robot to endpoint and apply process to the sculpture
    if (m_step < m_actions.size()) {
        m_actions[m_step].target->moveTo( m_actions[m_step].trajectory->end());
        m_sculpture->applySection();
        m_step++;
    }
}

void SculptProcess::step(double delta)
{
    Scene::step(delta);

    if (simulationIdle()) { // Handle transition between simulation actions
        if (m_actions[m_step].started) {
            if (m_actions[m_step].trigger) m_sculpture->applySection();
            m_step++;
        } else if (m_continuous) nextAction();
    }
}

void SculptProcess::setSculptingRobot(const std::shared_ptr<Robot>& robot){
    m_robot = robot;

    if (m_robot != nullptr) {
        m_fwd = glm::normalize(m_turntable->position() - m_robot->position());
        m_lastestRobotCommand = m_robot->getWaypoint();
    }
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

    std::vector<Plane> steps;

    ConvexHull hull = ConvexHull(model);

    for (uint32_t i = 0; i < hull.facetCount(); i++) steps.emplace_back(hull.facePlane(i));

    // Plan initial cuts, beginning from the top and moving towards the base
    std::sort(steps.begin(), steps.end(), [](const Plane& a, const Plane& b){
        return glm::dot(a.normal, {0, 1, 0}) > glm::dot(b.normal, {0, 1, 0});
    });

    glm::dvec3 p = m_sculpture->position();
    std::cout << p.x << " " << p.y << " " << p.z << "~~~~|\n";
    p = m_sculpture->up();
    std::cout << p.x << " " << p.y << " " << p.z << "~~~~|\n";

    std::cout << "PCT " << steps.size() << "\n";

    // Process all the cuts preemptively
    for (Plane& step : steps) {
        step.normal = -step.normal; // Invert direction so normals point into the body (Fragment method takes above plane)

        auto border = Collision::intersection(hull, step);
        std::cout << "Step: " << (&step - &steps[0]) << " " << border.size() << "\n";
        if (border.size() < 3) continue;
//        if (border.size() < 3) throw std::runtime_error("[SculptProcess] Failed to find intersection for section");

//        glm::ddvec3 normal = Triangle::normal(border[0], border[1], border[2]);


        // Generate trajectory for robotic carving
        planTurntableAlignment(step.normal);

        auto normal = step.normal;
        VertexArray::rotate(border, { 0, 1, 0 }, -m_lastestTableCommand.values[0]);
        step.rotate({ 0, 1, 0 }, -m_lastestTableCommand.values[0]);

//        planPlanarSection(step, border);

//        std::cout << "qs\n";
        // Record mesh manipulation operation for later application during carving process
        m_sculpture->queueSection(step.origin, -normal);

//        std::cout << "hf\n";

        // Prepare for next step
//        hull = Collision::fragment(hull, step);
//        std::cout << "rd\n";
//        if (m_actions.size() > 4) break;
    }
}

void SculptProcess::planOutlineRefinement(double stepDg)
{
    uint32_t steps = std::floor(180.0f / stepDg);
    double angle = 0;

    // Prepare a 3D render / edge detection system to find the profile of the model
    auto* detector = new EdgeDetect(model);

    detector->setSize(1000);
    detector->setEpsilon(120.0f);

    detector->capture()->camera().setViewingAngle(0, 0);
    detector->capture()->focus();

//    detector->capture()->capture();
//    detector->capture()->grabFramebuffer().save(QString("SPOutlineRefinementPrecheck.png"));

    glm::dvec3 offset = -model->boundedOffset();
    double scalar = m_sculpture->scalar();
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
        planTurntableAlignment(m_sculpture->rotation() - angle * (double)M_PI / 180.0f + M_PI / 2);

        std::cout << angle << " " << (angle * (double)M_PI / 180.0f) << " " << m_sculpture->rotation() << " RR\n";

        // Use the profile to prepare appropriate sculpting instructions for the robots
        auto profile = detector->profile();
        profile.setRefinementMethod(Profile::RefinementMethod::DELAUNEY);
        profile.rotateAbout({ 0, 1, 0 }, -m_sculpture->rotation());
//        profile.centerScale(scalar);
        profile.scale(scalar);
        profile.translate(offset);
//        profile.centerScale(scalar);
//        profile.scale(scalar);
        planOutlineRefinement(profile);
    }
}

void SculptProcess::planOutlineRefinement(Profile& profile)
{
//    auto mesh = MeshBuilder::extrude(profile.projected3D(), profile.normal(), 2.0f);
//    mesh->translate(-profile.normal());
//    mesh->setFaceColor({0, 0, 1});
//    createBody(mesh);

//    profile.correctWinding();

    uint32_t count = 0;
    while (!profile.complete() && count < 3000) {

        bool external = profile.isNextExternal();
        auto indices = profile.refine();
        std::cout << "IDX: ";
        for (auto i : indices) std::cout << i << " ";
        std::cout << "\n";
        auto border = profile.projected3D(indices);
        for(const glm::dvec3& v : border) std::cout << v.x << " " << v.y << " " << v.z << "\n";

        planRoboticSection(border);

        if (border.size() == 3) {
            m_sculpture->queueSection(border[0], border[1], border[2], profile.normal(), external);
        }

//        std::cout << "Profile: " << border.size() << ": \n";
//        for (auto& v : border) std::cout << v.x << " " << v.y << " " << v.z << "\n";
//        createBody(MeshBuilder::extrude(border, profile.normal(), 2.0f));
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

void SculptProcess::planTurntableAlignment(const glm::dvec3& axis)
{
    glm::dvec3 up = { 0, 1, 0 }, proj = axis - up * glm::dot(axis, up);
    double theta = atan2f(proj.x, -proj.z);
    planTurntableAlignment(theta);
}
void SculptProcess::planTurntableAlignment(double theta)
{
    std::vector<Waypoint> waypoints = {
            m_lastestTableCommand,
            { std::vector<double>{ theta }, 1, false}
    };

    m_lastestTableCommand = waypoints.back();

    auto trajectory = std::make_shared<Trajectory>(waypoints, TrajectorySolverType::CUBIC);
    trajectory->setMaxVelocity(M_PI / 2);

    m_actions.emplace_back(trajectory, m_turntable);
    m_actions[m_actions.size() - 1].trigger = true;
}

//void SculptProcess::planRoboticSection(const glm::dvec3& a, const glm::dvec3& b)
//{
//    glm::dvec3 delta = m_sculpture->position() - m_robot->position(), uDel = glm::normalize(delta);
//    std::cout << "DEL: " << delta.x << " " << delta.y << " " << delta.z <<"\n";
//    glm::dvec3 origin = border[0] - uDel * glm::dot(uDel, border[0]);
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
//}

// Sectioning step wherein the robot moves to remove all material above the specified plane
void SculptProcess::planPlanarSection(const Plane& plane, const std::vector<glm::dvec3>& border)
{
    Axis3D axes(m_fwd, plane.normal);
    uint32_t minIndex, maxIndex;
    VertexArray::extremes(border, plane.normal, minIndex, maxIndex);
    auto low = border[minIndex], high = border[maxIndex];

    // Bring the endpoints in plane with the normal m_fwd
    double lowOff = glm::dot(m_fwd, low), highOff = glm::dot(m_fwd, high);
//    if (lowOff < highOff) high += (lowOff - highOff) * m_fwd;
//    else                  low  -= (lowOff - highOff) * m_fwd;


    plane.print();
    axes.print();
    std::cout << "Split: " << low.x << " " << low.y << " " << low.z << "\n";
    std::cout << "Split: " << high.x << " " << high.y << " " << high.z << "\n";

    glm::dvec3 startPos = { 0, 0, 0 };

    std::cout << "M: " << glm::to_string(axes.toTransform()) << "\n================\n";
    const Waypoint& start = m_robot->inverse(low, axes);
    const Waypoint& end = m_robot->inverse(high, axes);

    if (start.values.empty() || end.values.empty()) {
        std::cout << "[SculptProcess] Can not perform cut. The robot does not reach\n";
        return;
    }

    std::vector<Waypoint> waypoints = {
            m_lastestRobotCommand,
            start,
            end
    };

    m_lastestRobotCommand = start;//todo end

    auto trajectory = std::make_shared<Trajectory>(waypoints, TrajectorySolverType::CUBIC);
    trajectory->setMaxVelocity(M_PI / 2);

    m_actions.emplace_back(trajectory, m_robot);
    m_actions[m_actions.size() - 1].trigger = true;
}

void SculptProcess::planRoboticSection(const std::vector<glm::dvec3>& border)
{
    std::vector<Waypoint> waypoints = {
            m_lastestRobotCommand,
            m_robot->getWaypoint()
    };

//    m_lastestCommandWaypoint = LAST POSITION

    auto trajectory = std::make_shared<Trajectory>(waypoints, TrajectorySolverType::CUBIC);
    trajectory->setMaxVelocity(M_PI / 2);

    m_actions.emplace_back(trajectory, m_robot);
    m_actions[m_actions.size() - 1].trigger = true;
}

void SculptProcess::nextAction()
{
    std::cout << "Step " << m_step << " / " << m_actions.size() << "\n";
    auto& action = m_actions[m_step];

    // Begin motion of the robot
    action.target->traverse(action.trajectory);
    action.started = true;
}

uint32_t SculptProcess::identifySculpture(const std::vector<std::shared_ptr<Mesh>>& fragments)
{
    uint32_t idx = 0;
    double volume = fragments[0]->volume();

    for (uint32_t i = 1; i < fragments.size(); i++) {
        double temp = fragments[i]->volume();

        if (volume < temp) {
            volume = temp;
            idx = i;
        }
    }

    return idx;
}