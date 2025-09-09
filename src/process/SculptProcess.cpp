//
// Created by Cam on 2024-11-12.
//

#include "SculptProcess.h"

#include <glm.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include "gtx/string_cast.hpp"
#include "gtx/quaternion.hpp"

#include "core/Sculpture.h"
#include "core/Debris.h"

#include "robot/trajectory/Trajectory.h"
#include "robot/trajectory/SimpleTrajectory.h"
#include "robot/trajectory/CompositeTrajectory.h"
#include "core/Timer.h"

SculptProcess::SculptProcess(const std::shared_ptr<Mesh>& model)
    : Scene()
    , model(model)
    , m_sculpture(nullptr)
    , m_step(0)
    , m_planned(false)
    , m_ttOffset(0, 0, 0)
    , m_planner()
    , m_smoothed(false)
    , m_duration(0)
    , m_time(0)
// TODO when adapting for different models these need to be changed
{

    // Create dummies to provide space for the sculpture before it is built
    m_bodies.push_back(nullptr);
    m_vis.push_back(nullptr);
    m_total += 2;

    prepareRobot();
    prepareTurntable();

    setTarget(model);
}

void SculptProcess::setTarget(const std::shared_ptr<Mesh>& mesh)
{
    model = mesh;
    model->center();

    if (m_sculpture != nullptr) reset();

    auto sculpture = std::make_shared<Sculpture>(model, getConfiguration().materialWidth, getConfiguration().materialHeight);
    sculpture->setName("SCULPTURE");

    readySculpture(sculpture);
}

void SculptProcess::loadSculpture(const std::string& filename)
{
    readySculpture(std::make_shared<Sculpture>(filename));
}

void SculptProcess::readySculpture(const std::shared_ptr<Sculpture>& sculpture)
{
    if (m_sculpture != sculpture) {
        m_sculpture = sculpture;

        m_sculpture->setID(0);
        m_sculpture->model()->setID(1);

        m_bodies[0] = m_sculpture;
        m_vis[0] = m_sculpture->model();

        attachSculpture();
        update();
    }
}

// TODO eventually load from file / make editable
void SculptProcess::prepareRobot()
{
    glm::dvec3 hOffset = -2.0 * getConfiguration().forward;

    auto table = createBody(getConfiguration().tableMesh);
    table->setName("Robot-Table");
    table->translate(hOffset);

    m_robot = createRobot(getConfiguration().kinematics);
    m_robot->setName("SCULPTOR");

    m_robot->moveTo(getConfiguration().robotHome);

    m_robot->translate(hOffset + glm::dvec3(0, 1, 0));
    m_robot->rotate({ 0, 1, 0 }, M_PI);
    m_robot->setLinkMesh(0, MeshHandler::loadAsMeshBody(R"(..\res\meshes\RobotBase.obj)"));

    m_robot->setLinkMesh(6, MeshHandler::loadAsMeshBody(R"(..\res\meshes\BladeAttachment.obj)"));

    table->ignore(m_robot->links()[1]);

    auto eoatMesh = MeshHandler::loadAsMeshBody("../res/meshes/Blade.obj");
    auto eoat = createBody(eoatMesh);
    eoat->setName("BLADE");
    m_robot->setEOAT(eoat, false);

    m_robot->update();
}

void SculptProcess::prepareTurntable()
{
    auto table = createBody(getConfiguration().tableMesh);
    table->setName("TT-Table");

    auto chain = std::make_shared<KinematicChain>();
    chain->addJoint(Joint(Joint::Type::REVOLUTE, { 0.0f, 0.0f, 0.0f, 0.0f }, 0));

    m_turntable = createRobot(chain);
    m_turntable->translate(UP * getConfiguration().tableMesh->ySpan());
    m_turntable->setName("TURNTABLE");

    auto ttBaseMesh = MeshHandler::loadAsMeshBody("../res/meshes/TurntableBase.obj");
    m_turntable->setLinkMesh(0, ttBaseMesh);

    auto ttj0Mesh = MeshHandler::loadAsMeshBody("../res/meshes/TurntableJ0.obj");
    m_turntable->setLinkMesh(1, ttj0Mesh);

    double near, far;
    glm::dvec3 axis = m_turntable->links().back()->getRotation() * -UP;
    ttj0Mesh->extents(axis, near, far);
    m_ttOffset = UP * far;
}

void SculptProcess::attachSculpture()
{
    m_turntable->setEOAT(m_sculpture, false);
    m_turntable->translateEOAT(m_ttOffset);

    getConfiguration().setCenter(m_sculpture->position());

    auto neutral = m_sculpture->position() + 1.3 * m_sculpture->height() * UP;
    auto neutralPose = Pose(neutral, Axis3D(getConfiguration().forward));
    neutralPose.localTranslate({ 0, 1.0 * m_sculpture->width(), -0.5 * getConfiguration().bladeLength });
    getConfiguration().robotNeutral = m_robot->inverse(neutralPose);
    assert(getConfiguration().robotNeutral.isValid());
}

SculptProcess::~SculptProcess()
{
    std::cout << "SP destroy\n";
}

void SculptProcess::reset()
{
    bool paused = m_paused;
    if (!paused) pause();

    m_step = 0;
    m_actions.clear();

    // Remove any fragments
    uint32_t count = m_bodies.size();
    for (uint32_t i = 1; i < count; i++) {
        if (m_bodies[i]->getType() == RigidBody::Type::DYNAMIC) {
            std::swap(m_bodies[i], m_bodies[count - 1]);
            count--;
        }
    }

    if (count < m_bodies.size()) m_bodies.erase(m_bodies.begin() + count, m_bodies.end());

    removeDebris();

    if (m_sculpture != nullptr) {
        m_sculpture->restore();
        m_sculpture->reset();
    }

    if (m_turntable != nullptr) {
        m_turntable->stop();
        m_turntable->setJointValue(0, 0);
    }

    if (m_robot != nullptr) {
        m_robot->stop();
        m_robot->moveTo(getConfiguration().robotHome);
    }

    m_planned = m_smoothed = false;

    m_time = m_duration = 0;

    if (!paused) pause();

    update();
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

    ScopedTimer timer("[SculptProcess] planning");

    if (m_planner.plan(m_sculpture, m_robot, m_turntable)) {
        m_planned = true;
        m_step = 0;

//        m_actions = m_planner.actions();

        m_actions = getConfiguration().linkActionEnable ? smooth(m_planner.actions()) : m_planner.actions();
    } else std::cout << "\033[33mWarning! Failed to plan\033[0m\n";

    calculateDuration();
    m_time = 0;
}

void SculptProcess::smooth()
{
    if (!m_smoothed) m_actions = smooth(m_actions);
}

void SculptProcess::calculateDuration()
{
    m_duration = 0;
    for (const Action& action : m_actions) m_duration += action.trajectory->duration();
}

// One-off method useful for testing blind cut operations
void SculptProcess::blind(const Pose& pose, double depth)
{
//    m_actions.clear();
//
//    auto trajectory = std::make_shared<CompositeTrajectory>(6);
//    trajectory->setLimits(m_baseVelocityLimits, m_baseAccelerationLimits);
//
//    Action action(trajectory, m_robot);
//
//    if (planBlindCut(pose, depth, action)) {
//        trajectory->update();
//        for (Cut& cut : action.cuts) {
//            auto [ts, tf] = trajectory->tLimits(cut.index);
//            cut.ts = ts;
//            cut.tf = tf;
//        }
//
//        m_actions.push_back(action);
//
//        m_sculpture->queueSection(Plane(pose.position - m_sculpture->position(), -pose.axes.yAxis));
//
//        m_planned = true;
//        m_step = 0;
//    } else std::cout << "\033[91mFailed to prepare cut operation\n\033[0m";
}

// One-off method useful for testing mill operations
void SculptProcess::mill(const Pose& pose, const glm::dvec3& normal, const glm::dvec3& travel, double depth)
{
//    m_latestRobotCommand = m_robot->getWaypoint();
//    m_actions.clear();
//
//    auto trajectory = std::make_shared<CompositeTrajectory>(6);
//    trajectory->setLimits(m_baseVelocityLimits, m_baseAccelerationLimits);
//
//    Action action(trajectory, m_robot);
//
//    if (planMill(pose, normal, travel, depth, action)) {
//        trajectory->update();
//        for (Cut& cut : action.cuts) {
//            auto [ts, tf] = trajectory->tLimits(cut.index);
//            cut.ts = ts;
//            cut.tf = tf;
//        }
////        commitActions({ action });
//        m_actions.push_back(action);
//
//        double theta = acos(glm::dot(UP, pose.axes.yAxis));
//        std::cout << "THETA: " << 180 * theta / M_PI << "\n";
//        double dy = 0.5 * m_bladeWidth * sin(theta) + 0.5 * m_bladeThickness * cos(theta);
////        length = -(length + runup) + 0.5 * m_bladeWidth
////                 - m_bladeThickness * tan(0.5 * M_PI - acos(glm::dot(UP, axes.yAxis)));
//        m_sculpture->queueSection(Plane(pose.position - m_sculpture->position() - glm::dvec3(0, dy, 0), -normal));
//
//        m_planned = true;
//        m_step = 0;
//    } else std::cout << "\033[91mFailed to prepare mill operation\n\033[0m";
}

void SculptProcess::proceed()
{
    if (!m_planned) plan();

    // Do not proceed before previous step is complete
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

    std::cout << "SKIP| Next step: " << (m_step + 1) << " / Remaining:" << m_actions.size() - 1 << " -> " << m_actions.front().target->getWaypoint() << " " << m_actions[0].trajectory->duration() << "\n";

    // Jump robot to endpoint and apply relevant sections to the sculpture
    if (m_actions[0].started) {
        m_time += m_actions[0].target->transitRemaining();
        m_actions[0].target->finishTransit();
    } else {
        m_time += m_actions[0].trajectory->duration();
        m_actions[0].target->moveTo(m_actions[0].trajectory->end());
        if (!m_actions[0].cuts.empty()) m_sculpture->applySection();
    }

    clearDebris();

    m_actions.pop_front();
    m_step++;

//    if (m_actions.empty()) home();

    update();
}

void SculptProcess::home()
{
    m_robot->moveTo(getConfiguration().robotHome);
    m_turntable->moveTo(Waypoint({ 0 }, true));
}

// TODO identify cause of ~5-10% reduced time estimation as opposed to real-world time
void SculptProcess::step(double delta)
{
    if (simulationComplete()) return;

    if (!m_actions[0].started && getConfiguration().continuous) nextAction();

    double rem = m_actions[0].target->transitRemaining();
//    std::cout << rem << " " << delta << " " << m_actions[0].started << "\n";
    while (m_actions[0].started && rem < delta) {
        m_actions[0].target->finishTransit();
        Scene::step(rem);
        m_time += rem;
        delta -= rem;
        process();

        m_actions.pop_front();
        m_step++;

        if (m_actions.empty()) return;

        if (getConfiguration().continuous) nextAction();
        rem = m_actions[0].target->transitRemaining();
    }

    if (simulationActive()){
        Scene::step(delta);
        m_time += delta;
        process();
    }
}

void SculptProcess::process()
{
    Action& action = m_actions[0];
    if (m_debris != nullptr && !action.cuts.empty() && action.cuts.front().ts < action.trajectory->t()) {
        if (!m_debris->inProcess()) m_debris->beginCut();

        // Make sure to take the final pose before completing a cut
        Pose pose = action.cuts.front().tf <= action.trajectory->t()
                    ? m_robot->getPose(action.trajectory->evaluate(action.cuts.front().tf))
                    : m_robot->getPose();

        glm::dvec3 normal = (1.0 - 2.0 * (glm::dot(action.cuts[0].axis, pose.axes.xAxis) < 0)) * pose.axes.xAxis; // Direction to front of blade
        glm::dvec3 reference = pose.position + normal * 0.5 * getConfiguration().bladeWidth; // Center of front-edge of the blade

        double depth = glm::dot(reference - action.cuts[0].origin, action.cuts[0].axis); // Current depth of cut
        depth = std::max(0.0, depth);

        normal = normal * -glm::quat_cast(m_sculpture->getRotation()); // Convert to local space
        const auto& fragments = m_debris->removeMaterial(normal, depth);
        if (getConfiguration().fragmentReleaseEnable && !fragments.empty()) {
            for (const std::shared_ptr<RigidBody>& fragment : fragments) {
                prepareBody(fragment);
                fragment->mesh()->setFaceColor(m_sculpture->baseColor());
            }
        }

        if (action.cuts.front().tf <= action.trajectory->t()) {
            std::cout << "REMOVAL: " << action.cuts.size() << " " << action.cuts.front().tf << " " << action.trajectory->t() <<"\n";
//                m_debris->print();
            m_debris->completeCut();
            action.cuts.pop_front();
        }
    }
}

void SculptProcess::nextAction()
{
    assert(!m_actions.empty());

    if (getConfiguration().cutSimulationEnable) removeDebris();

    if (getConfiguration().linkActionEnable) link();

    std::cout << "Step " << m_step << " / Remaining: " << m_actions.size() - 1 << "\n";
    Action& action = m_actions.front();

//    std::cout << "AS: " << action.cuts.size() << " " << action.target << " " << action.trajectory->start() << " | " << action.trajectory->end() << "\n";

    // Begin motion of the robot
    action.target->traverse(action.trajectory);
    action.started = true;


    if (!action.cuts.empty()) {

        if (getConfiguration().cutSimulationEnable) {
            m_debris = m_sculpture->applySection();
            if (m_debris == nullptr) throw std::runtime_error("[SculptProcess] Failed to prepare debris");
            m_debris->setName("ACTION" + std::to_string(m_step) + "-DEBRIS");
            m_debris->applyCompositeColors(getConfiguration().debrisColoringEnable);

            auto rotation = glm::quat_cast(m_sculpture->getRotation());

            for (Cut& cut : action.cuts) {
                // Calculate local position / directions
                glm::dvec3 origin = (cut.origin - m_sculpture->position()) * rotation;
                glm::dvec3 normal = cut.normal * -rotation;
                glm::dvec3 axis = cut.axis * -rotation;

                m_debris->queueCut(origin, normal, axis, getConfiguration().bladeThickness, cut.theta);
            }

            m_debris->remesh();

            prepareBody(m_debris);
        } else m_sculpture->applySection();
    }
}



void SculptProcess::removeDebris()
{
    if (m_debris != nullptr) {

        if (m_sculpture != nullptr && !m_debris->components().empty()) {
            for (const ConvexHull& hull : m_debris->components()) {
                std::cout << hull.isValid() << " " << hull.vertexCount() << " NH\n";
                if (hull.isValid()) m_sculpture->add(hull);
            }

            m_sculpture->remesh();
        }

        clearDebris();
    }
}

void SculptProcess::clearDebris()
{
    if (m_debris != nullptr) {
        for (uint32_t i = 0; i < m_bodies.size(); i++) {
            if (m_bodies[i] == m_debris) {
                m_bodies.erase(m_bodies.begin() + i);
                break;
            }
        }

        m_debris = nullptr;
    }
}


void SculptProcess::alignToFace(uint32_t faceIdx)
{
    assert(faceIdx < m_sculpture->mesh()->faceCount());
    assert(m_robot != nullptr);

    auto face = m_sculpture->mesh()->faces()[faceIdx];

    auto rotation = glm::angleAxis(m_turntable->getWaypoint().values[0], UP);

    // Convert face into world-space vertices

    glm::dvec3 normal = rotation * m_sculpture->mesh()->faces().normal(faceIdx);

    std::vector<glm::dvec3> border(m_sculpture->mesh()->faces().faceSizes()[faceIdx]);
    for (uint32_t i = 0; i < border.size(); i++) {
        border[i] = rotation * m_sculpture->mesh()->vertices()[face[i]] + getConfiguration().center;
    }

    Waypoint wp = alignedToFaceWP(border, normal);
    if (wp.isValid()) m_robot->moveTo(wp);
}

bool SculptProcess::planned() const
{
    return m_planned;
}

bool SculptProcess::simulationComplete() const
{
    return m_actions.empty();
}

bool SculptProcess::simulationIdle() const
{
    return !simulationComplete() && !m_actions[0].started;
}

bool SculptProcess::simulationActive() const
{
    return !simulationComplete() && m_actions[0].target->inTransit();
}

double SculptProcess::simulationTime() const
{
    return simulationComplete() ? m_duration : m_time;
}
double SculptProcess::simulationDuration() const
{
    return m_duration;
}

ProcessConfiguration& SculptProcess::getConfiguration()
{
    return m_planner.getConfiguration();
}

const ProcessConfiguration& SculptProcess::getConfiguration() const
{
    return m_planner.getConfiguration();
}

const std::shared_ptr<Sculpture>& SculptProcess::getSculpture() const
{
    return m_sculpture;
}

const std::shared_ptr<Body>& SculptProcess::getModel() const
{
    return m_sculpture->model();
}

const std::shared_ptr<Debris>& SculptProcess::getDebris() const
{
    return m_debris;
}

const std::shared_ptr<Robot>& SculptProcess::getSculptor() const
{
    return m_robot;
}

const std::shared_ptr<Robot>& SculptProcess::getTurntable() const
{
    return m_turntable;
}

const glm::dvec3& SculptProcess::forward() const
{
    return getConfiguration().forward;
}

Waypoint SculptProcess::alignedToFaceWP(const std::vector<glm::dvec3>& border, const glm::dvec3& normal) const
{
    auto axes = Axis3D::faceAligned(normal, getConfiguration().forward, false);

    // Find the better position for access (If possible)
    auto optionA = alignedToFaceWP(axes, border);
    axes.flipXZ();
    auto optionB = alignedToFaceWP(axes, border);

    return m_robot->preferredWaypoint(optionA, optionB);
}

// Returns the necessary joint angles for the robot to reach the given pose.
Waypoint SculptProcess::alignedToFaceWP(const Axis3D& axes, const std::vector<glm::dvec3>& border) const
{

    // Select vertex nearest along the cutting parallel
    uint32_t idx = 0;
    if (VertexArray::extreme(border, -axes.zAxis, idx)) {

        // If multiple vertices are equally close, select the one nearest the robot (only applies to adjacent vertices)
        uint32_t pre = (idx == 0 ? border.size() - 1 : idx - 1), post = (idx + 1) % border.size();

        glm::dvec3 delta = border[pre] - border[idx];
        if (std::abs(glm::dot(delta, axes.zAxis)) < 1e-12 && glm::dot(delta, getConfiguration().forward) < 0) idx = pre;

        delta = border[post] - border[idx];
        if (std::abs(glm::dot(delta, axes.zAxis)) < 1e-12 && glm::dot(delta, getConfiguration().forward) < 0) idx = post;

        return alignedToVertexWP(axes, getConfiguration().poseAdjustedVertex(axes, border[idx]));
    }

    return Waypoint();
}

// Returns the necessary joint angles for the robot to reach the given pose. Draws back from vertex until a valid position is found
Waypoint SculptProcess::alignedToVertexWP(const Axis3D& axes, const glm::dvec3& vertex) const
{
    return alignedToVertexWP(axes, vertex, -axes.zAxis, 0.5);
}

Waypoint SculptProcess::alignedToVertexWP(const Axis3D& axes, const glm::dvec3& vertex, const glm::dvec3& tryAxis, double tryLimit) const
{
    int limit = 10;
    glm::dvec3 step = tryAxis * (tryLimit / limit);

    Pose pose(vertex, axes);
    Waypoint wp = m_robot->inverse(pose);

    while (!wp.isValid() && limit-- > 0) {
        pose.globalTranslate(step);
        wp = m_robot->inverse(pose);
    }

    return wp;
}

// Develops trajectories between disjoint actions so all motions are continuous
std::deque<Action> SculptProcess::smooth(const std::deque<Action>& actions)
{
    if (actions.empty()) return {};

    // Save sculpture state to restore later (Geometry must match changing state for testing)
    auto sculpture = std::make_shared<Sculpture>(*m_sculpture);
    m_sculpture->enableAutoRemesh(false);

    std::deque<Action> smoothed = { actions[0] };
    smoothed[0].target->moveTo(smoothed[0].trajectory->end());

    for (uint32_t i = 1; i < actions.size(); i++) {

        // Make sure to test against the actual state of the sculpture when motions are conducted
        if (!smoothed.back().cuts.empty()) m_sculpture->applySection();

        // Create an intermediary trajectory to connect disjoint ends (If required)
        smoothed.emplace_back(linked(actions[i]));
        if (smoothed.back().trajectory == nullptr) smoothed.pop_back();
        else smoothed.back().target->moveTo(smoothed.back().trajectory->end());

        // Attach the previously planned action and ready for next link
        smoothed.push_back(actions[i]);
        smoothed.back().target->moveTo(smoothed.back().trajectory->end());
    }

    // Revert sculpture state

    readySculpture(sculpture);

    m_smoothed = true;
    return smoothed;
}

// Develops a trajectory, if required, to link the current position to the beginning of the next action
void SculptProcess::link()
{
    if (m_smoothed || m_actions.empty()) return;

    m_actions.emplace_front(linked(m_actions.front()));
    if (m_actions.front().trajectory == nullptr) m_actions.pop_front();
    else {
        m_duration += m_actions.front().trajectory->duration();
    }
}

Action SculptProcess::linked(const Action& action)
{
    if (action.target == m_turntable) { // Move robot away if it would collide with the TT when it moves
        if (getConfiguration().collisionTestingEnable && testTurntableCollision(action.trajectory)) {
            return planTurntableClearance();
        }
    } else if (action.target == m_robot) { // Prepare approach trajectories to connect the last pose and the initial pose for this action
        if (!Waypoint::compare(m_robot->getWaypoint(), action.trajectory->start())) {
            auto linkTrajectory = prepareApproach(action.trajectory->start());
            if (linkTrajectory != nullptr) return { linkTrajectory, m_robot };
            else std::cout << "\033[93mFailed to link actions\033[0m\n";
        }
    }

    return { nullptr, nullptr };
}

bool SculptProcess::testTurntableCollision(const std::shared_ptr<Trajectory>& ttTrajectory)
{
    return ttTrajectory->test(this, m_turntable, 0.05);
}

// Develops an action to move the robot out of the way of the turntable
Action SculptProcess::planTurntableClearance()
{
    // TODO better motions than moving towards the neutral position
    return planApproach(getConfiguration().robotNeutral);
}

Action SculptProcess::planApproach(const Waypoint& destination)
{
    auto trajectory = std::make_shared<SimpleTrajectory>(m_robot->getWaypoint(), destination, getConfiguration().solver);
    trajectory->setLimits(getConfiguration().baseVelocityLimits, getConfiguration().baseAccelerationLimits);

    return { trajectory, m_robot };
}

// Tries to identify a path from the last position to the specified position
// TODO Could improve reliability with an actual path planning algorithm
std::shared_ptr<Trajectory> SculptProcess::prepareApproach(const Waypoint& destination)
{
    const double dt = 0.1;

    // Try the direct path first
    auto directTrajectory = std::make_shared<SimpleTrajectory>(m_robot->getWaypoint(), destination, getConfiguration().solver);
    directTrajectory->setLimits(getConfiguration().baseVelocityLimits, getConfiguration().baseAccelerationLimits);
    if (!getConfiguration().collisionTestingEnable || validateTrajectory(directTrajectory, dt)) return directTrajectory;


    auto pose = m_robot->getPose(destination);
    pose.localTranslate({ 0, 0.5, 0});

    std::vector<Waypoint> testWPs = { m_robot->inverse(pose), getConfiguration().robotNeutral };

    for (const Waypoint& midpoint : testWPs) {
        if (midpoint.isValid()) {
            auto trajectory = std::make_shared<CompositeTrajectory>(m_robot->dof());
            trajectory->setLimits(getConfiguration().baseVelocityLimits, getConfiguration().baseAccelerationLimits);
            trajectory->addTrajectory(std::make_shared<SimpleTrajectory>(m_robot->getWaypoint(), midpoint, getConfiguration().solver));
            trajectory->addTrajectory(std::make_shared<SimpleTrajectory>(midpoint, destination, getConfiguration().solver));
            trajectory->update();

            if (validateTrajectory(trajectory, dt)) return trajectory;
        }
    }

    return nullptr;
}

// Returns true if the pose is both reachable, and no collisions are present
bool SculptProcess::validatePose(const Pose& pose) const
{
    Waypoint wp = m_robot->inverse(pose);
    if (wp.isValid()) {
        m_robot->moveTo(wp);
        return test(m_robot);
    }

    return false;
}

bool SculptProcess::validateTrajectory(const std::shared_ptr<Trajectory>& trajectory, double dt)
{
    return trajectory->isValid() && !trajectory->test(this, m_robot, dt);
}