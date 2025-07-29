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

#include "robot/Pose.h"
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
        return glm::dot(a.normal, UP) > glm::dot(b.normal, UP);
    });

    glm::dvec3 p = m_sculpture->position();
    std::cout << p.x << " " << p.y << " " << p.z << "~~~~|\n";
    p = m_sculpture->up();
    std::cout << p.x << " " << p.y << " " << p.z << "~~~~|\n";

    std::cout << "PCT " << steps.size() << "\n";

    // Process all the cuts preemptively
    hull = m_sculpture->hull();
    for (Plane& step : steps) {
        std::cout << "Step [" << (&step - &steps[0]) << "]:\n";
        hull = planConvexTrim(hull, step);
        if (m_actions.size() > 10) break;
//        break;
    }
}

ConvexHull SculptProcess::planConvexTrim(const ConvexHull& hull, const Plane& plane)
{
    //    static std::array<double, 7> attemptOffsets = { 0, -M_PI / 32, M_PI / 32, -M_PI / 16, M_PI / 16, -M_PI / 8, M_PI / 8 };
    static std::array<double, 1> attemptOffsets = { 0 };

    auto border = Collision::intersection(hull, plane);
    if (border.size() < 3) return hull;
//        if (border.size() < 3) throw std::runtime_error("[SculptProcess] Failed to find intersection for section");

//        glm::ddvec3 normal = Triangle::normal(border[0], border[1], border[2]);

    std::shared_ptr<Trajectory> trajectory;
    double baseRotation = plane.axialRotation(UP), rotation;

    // Try developing a trajectory that will allow the robot to complete the planar section
    for (double offset : attemptOffsets) {
        rotation = baseRotation + offset;

        trajectory = preparePlanarTrajectory(plane.rotated(UP, -rotation), VertexArray::rotated(border, UP, -rotation));
        if (trajectory != nullptr) break;
    }

    if (trajectory == nullptr) {
        std::cout << "Failed to develop an appropriate trajectory\n";
        return hull;
    }

    // If a trajectory was found queue it
    planTurntableAlignment(rotation);
    planRoboticSection(trajectory);

    // Record mesh manipulation operation for later application during carving process
    m_sculpture->queueSection(plane.origin, -plane.normal);

    std::cout << "hf\n";

    // Prepare for next step
    std::cout << "rd\n";
//    return Collision::fragment(hull, plane);
    return hull;
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

void SculptProcess::planTurntableAlignment(double theta)
{
//    std::vector<Waypoint> waypoints = {
//            m_lastestTableCommand,
//            { std::vector<double>{ theta }, 1, false}
//    };
//
//    m_lastestTableCommand = waypoints.back();
//
//    auto trajectory = std::make_shared<Trajectory>(waypoints, Interpolator::SolverType::CUBIC);
//    trajectory->setMaxVelocity(M_PI / 2);
//
//    m_actions.emplace_back(trajectory, m_turntable);
}

void SculptProcess::planRoboticSection(const std::shared_ptr<Trajectory>& trajectory)
{

    m_lastestRobotCommand = trajectory->end();

    m_actions.emplace_back(trajectory, m_robot);
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
std::shared_ptr<Trajectory> SculptProcess::preparePlanarTrajectory(const Plane& plane, const std::vector<glm::dvec3>& border)
{
    // Create a co-ordinate system, x->cut direction, y->normal to cut plane, z->in fwd direction
    Axis3D axes(m_fwd, plane.normal);
    axes.rotateY();

    uint32_t minIndex, maxIndex;
    VertexArray::extremes(border, plane.normal, minIndex, maxIndex);
    auto low = border[minIndex], high = border[maxIndex];

    // Bring the endpoints in plane with the normal m_fwd
    double lowOff = glm::dot(m_fwd, low), highOff = glm::dot(m_fwd, high);
    if (lowOff < highOff) high += (lowOff - highOff) * m_fwd;
    else                  low  -= (lowOff - highOff) * m_fwd;


    plane.print();
    axes.print();
    std::cout << "Split: " << low.x << " " << low.y << " " << low.z << "\n";
    std::cout << "Split: " << high.x << " " << high.y << " " << high.z << "\n";

//    glm::dvec3 startPos = { 0, 1, 0 };
//    low = startPos, high = low + axes.xAxis;

    std::cout << "M: " << glm::to_string(axes.toTransform()) << "\n================\n";
    const Waypoint& start = m_robot->inverse(Pose(low, axes));
    const Waypoint& end = m_robot->inverse(Pose(high, axes));

    // Skip if parts of this section are unreachable
    if (start.values.empty() || end.values.empty()) return nullptr;

//    std::vector<Waypoint> waypoints = {
//            m_lastestRobotCommand,
//            start,
//            end
//    };
//
//    auto trajectory = std::make_shared<Trajectory>(waypoints, TrajectorySolverType::CUBIC);
//    trajectory->setMaxVelocity(M_PI / 2);
//
//    return trajectory;
    return nullptr;
}

void SculptProcess::planRoboticSection(const std::vector<glm::dvec3>& border)
{
    std::vector<Waypoint> waypoints = {
            m_lastestRobotCommand,
            m_robot->getWaypoint()
    };

//    m_lastestCommandWaypoint = LAST POSITION

//    auto trajectory = std::make_shared<Trajectory>(waypoints, TrajectorySolverType::CUBIC);
//    trajectory->setMaxVelocity(M_PI / 2);
//
//    m_actions.emplace_back(trajectory, m_robot);
//    m_actions[m_actions.size() - 1].trigger = true;
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