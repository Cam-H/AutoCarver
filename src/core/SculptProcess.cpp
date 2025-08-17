//
// Created by Cam on 2024-11-12.
//

#include "SculptProcess.h"

#include <glm.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/string_cast.hpp>
#include <gtx/quaternion.hpp>

#include "Sculpture.h"
#include "Debris.h"

#include "geometry/poly/Profile.h"
#include "renderer/EdgeDetect.h"
#include "renderer/RenderCapture.h"

#include "robot/Pose.h"
#include "robot/trajectory/SimpleTrajectory.h"
#include "robot/trajectory/TOPPTrajectory.h"
#include "robot/trajectory/CartesianTrajectory.h"
#include "robot/trajectory/HoldPosition.h"

#include "geometry/MeshBuilder.h"
#include "geometry/collision/Collision.h"
#include "fileIO/MeshHandler.h"

static const std::array<double, 13> ATTEMPT_OFFSETS = { 0,
                                                 -M_PI / 32, M_PI / 32
                                                 -M_PI / 16, M_PI / 16,
                                                 -M_PI / 8, M_PI / 8,
                                                 -M_PI / 4, M_PI / 4,
                                                 -M_PI / 3, M_PI / 3,
                                                 -M_PI / 2, M_PI / 2 };

//static const std::array<double, 1> attemptOffsets = { 0 };


SculptProcess::SculptProcess(const std::shared_ptr<Mesh>& model)
    : Scene()
    , model(model)
    , m_sculpture(nullptr)
    , m_step(0)
    , m_planned(false)
    , m_convexTrimEnable(true)
    , m_silhouetteRefinementEnable(true)
    , m_processCutEnable(true)
    , m_linkActionEnable(true)
    , m_collisionTestingEnable(true)
    , m_fragmentReleaseEnable(true)
    , m_sliceOrder(ConvexSliceOrder::TOP_DOWN)
    , m_actionLimit(0xFFFFFFFF)
    , m_actionLimitEnable(false)
    , m_continuous(false)
    , m_ttOffset(0, 0, 0)
    , m_fwd(1, 0, 0)
    // TODO when adapting for different models these need to be changed
    , m_bladeLength(1.5) // Length of cutting area (Does not cut tip + section blocked by base)
    , m_bladeWidth(0.098)
    , m_bladeThickness(0.011)
    , m_minCutVolume(0.005)
    , m_latestRobotCommand()
    , m_solver(Interpolator::SolverType::QUINTIC)
{
    setTarget(model);

    prepareTurntable();
}

void SculptProcess::setTarget(const std::shared_ptr<Mesh>& mesh)
{
    model = mesh;
    model->center();
    model->normalize();

    bool firstTarget = m_sculpture == nullptr;
    m_sculpture = std::make_shared<Sculpture>(model, m_config.materialWidth, m_config.materialHeight);
    m_sculpture->setName("SCULPTURE");

    if (firstTarget) {
        prepareBody(m_sculpture, 1);
        prepareBody(m_sculpture->model(), 1);
    } else { // Overwrite old sculpture if possible
        m_bodies[0] = m_sculpture;
        m_bodies[1] = m_sculpture->model();

        attachSculpture(); // Although already attached, adjusting transform is required

        reset(); // Erase all prior progress
    }
}

void SculptProcess::prepareTurntable()
{
//    model->transform(glm::inverse(KC_AXIS_TRANSFORM));

    auto ttTableMesh = MeshHandler::loadAsMeshBody("../res/meshes/TurntableTable.obj");
    auto table = createBody(ttTableMesh);
    table->setName("TT-Table");
    table->prepareColliderVisuals();

    auto chain = std::make_shared<KinematicChain>();
    chain->addJoint(Joint(Joint::Type::REVOLUTE, { 0.0f, 0.0f, 0.0f, 0.0f }, 0));

    m_turntable = createRobot(chain);
    m_turntable->translate(UP * ttTableMesh->ySpan());
    m_turntable->setName("TURNTABLE");

    auto ttBaseMesh = MeshHandler::loadAsMeshBody("../res/meshes/TurntableBase.obj");
    m_turntable->setLinkMesh(0, ttBaseMesh);

    auto ttj0Mesh = MeshHandler::loadAsMeshBody("../res/meshes/TurntableJ0.obj");
    m_turntable->setLinkMesh(1, ttj0Mesh);

    double near, far;
    glm::dvec3 axis = m_turntable->links().back()->getRotation() * -UP;
    ttj0Mesh->extents(axis, near, far);
    m_ttOffset = UP * far;

    attachSculpture();
}

void SculptProcess::attachSculpture()
{
    m_turntable->setEOAT(m_sculpture, false);
    m_turntable->translateEOAT(m_ttOffset);
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
    m_cuts.clear();

    // Remove any fragments
    for (int i = m_bodies.size() - 1; i >= 0; i--) {
        if (m_bodies[i]->getType() == RigidBody::Type::DYNAMIC) m_bodies.erase(m_bodies.begin() + i);
    }

    removeDebris();

    m_sculpture->restore();
    m_sculpture->reset();

    m_turntable->stop();
    m_turntable->setJointValue(0, 0);
    m_latestTableCommand = m_turntable->getWaypoint();

    if (m_robot != nullptr) {
        m_robot->stop();
        m_robot->moveTo(m_robotHome);
        m_latestRobotCommand = m_robot->getWaypoint();
    }

    m_planned = false;

    if (!paused) pause();

    update();
}

void SculptProcess::enableConvexTrim(bool enable)
{
    m_convexTrimEnable = enable;
}
void SculptProcess::enableSilhouetteRefinement(bool enable)
{
    m_silhouetteRefinementEnable = enable;
}

void SculptProcess::enableProcessCut(bool enable)
{
    m_processCutEnable = enable;
}

void SculptProcess::enableActionLinking(bool enable)
{
    m_linkActionEnable = enable;
}

void SculptProcess::enableCollisionTesting(bool enable)
{
    m_collisionTestingEnable = enable;
}

void SculptProcess::enableFragmentRelease(bool enable)
{
    m_fragmentReleaseEnable = enable;
}


void SculptProcess::setSlicingOrder(ConvexSliceOrder order)
{
    m_sliceOrder = order;
}
void SculptProcess::setActionLimit(uint32_t limit)
{
    m_actionLimit = limit;
    m_actionLimitEnable = true;
}
void SculptProcess::enableActionLimit(bool enable)
{
    m_actionLimitEnable = enable;
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

    const Waypoint initialRobotCommand = m_latestRobotCommand = m_robot->getWaypoint();
    const Waypoint initialTableCommand = m_latestTableCommand = m_turntable->getWaypoint();

    // Ready the sculpture by carving the convex hull of the model
    if (m_convexTrimEnable) planConvexTrim();
    else m_sculpture->restoreAsHull();

    // Repeatedly refined the sculpture by cutting to the outline of the model at discrete orientations
    if (m_silhouetteRefinementEnable) planOutlineRefinement(90.0);

    // Capture model features with fine carving
//    planFeatureRefinement();

    // Return to the home position
    if (std::abs(m_turntable->getJointValue(0)) > 1e-12) commitActions({ planTurntableAlignment(m_turntable->getJointValue(0), 0) });
    commitActions({ Action(std::make_shared<HoldPosition>(m_robotHome, 0.1), m_robot) });

    // Restore initial configuration
    m_robot->moveTo(initialRobotCommand);
    m_turntable->moveTo(initialTableCommand);

    m_sculpture->restore();
    m_sculpture->setRotation(glm::angleAxis(0.0, UP));

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

    // Jump robot to endpoint and apply process to the sculpture
    if (m_step < m_actions.size()) {
        m_actions[m_step].target->moveTo( m_actions[m_step].trajectory->end());
        if (!m_actions[m_step].cuts.empty()) m_sculpture->applySection();
        m_step++;
    }
}

void SculptProcess::step(double delta)
{
    Scene::step(delta);

    if (m_step < m_actions.size()) {
        Action& action = m_actions[m_step];

        if (simulationIdle()) { // Handle transition between simulation actions
            if (action.started) {
                m_step++;
            } else if (m_continuous) nextAction();
        } else if (m_debris != nullptr && !action.cuts.empty() && action.cuts.front().ts < action.trajectory->t()){
            double depth = glm::dot(m_robot->getPosition() - action.cuts[0].pose.position, action.cuts[0].pose.axes.xAxis);
//            std::cout << "NR: " << depth << " " << 0.5 * m_bladeWidth << " -> ";
            depth = std::max(0.0, depth + 0.5 * m_bladeWidth);

            const auto& fragments = m_debris->removeMaterial(depth);
            if (m_fragmentReleaseEnable && !fragments.empty()) {
                for (const std::shared_ptr<RigidBody>& fragment : fragments) {
                    prepareBody(fragment);
                    fragment->mesh()->setFaceColor(m_sculpture->baseColor());
                }
            }

            if (action.cuts.front().tf <= action.trajectory->t()) action.cuts.pop_front();

            if (action.cuts.empty()) {
                std::cout << "REM: " << m_debris->hulls().size() << "\n";
                if (!m_debris->hulls().empty()) {
                    for (const ConvexHull& hull : m_debris->hulls()) {
                        std::cout << hull.isValid() << " " << hull.vertexCount() << "\n";
                        if (hull.isValid()) m_sculpture->add(hull);
                    }

                    m_sculpture->remesh();
                }

                removeDebris();
            }
        }
    }
}

void SculptProcess::setSculptingRobot(const std::shared_ptr<Robot>& robot){
    m_robot = robot;

    if (m_robot != nullptr) {
        m_robot->setName("SCULPTOR");

        auto delta = m_turntable->position() - m_robot->position();
        m_fwd = glm::normalize(delta - UP * glm::dot(UP, delta));

        m_robotMask = { false, true, true, false, false, false, true, false }; // TODO need update for other robot types

        m_eoatMask = std::vector<bool>(m_robotMask.size(), false);
        m_eoatMask.back() = true;

        m_latestRobotCommand = m_robot->getWaypoint();
        m_baseVelocityLimits = std::vector<double>(m_robot->dof(), M_PI / 2);
        m_baseAccelerationLimits = std::vector<double>(m_robot->dof(), M_PI);
        m_slowVelocityLimits = std::vector<double>(m_robot->dof(), M_PI);//TODO slowdown after testing

        m_robotHome = m_robot->getWaypoint();

        auto neutral = m_sculpture->position() + 1.3 * m_sculpture->height() * UP;
        auto neutralPose = Pose(neutral, Axis3D(m_fwd));
        neutralPose.localTranslate({ 0, 1.0 * m_sculpture->width(), -0.5 * m_bladeLength });
        m_robotNeutral = m_robot->inverse(neutralPose);

        assert(m_robotNeutral.values.size() == m_robot->dof());
    }
}

void SculptProcess::setContinuous(bool enable)
{
    m_continuous = enable;
}


void SculptProcess::alignToFace(uint32_t faceIdx)
{
    assert(faceIdx < m_sculpture->mesh()->faceCount());
    assert(m_robot != nullptr);

    auto face = m_sculpture->mesh()->faces()[faceIdx];

    auto rotation = glm::angleAxis(m_turntable->getWaypoint().values[0], UP);

    // Convert face into world-space vertices

    glm::dvec3 normal = m_sculpture->mesh()->faces().normal(faceIdx);
    toWorldSpace(normal, rotation);

    std::vector<glm::dvec3> border(m_sculpture->mesh()->faces().faceSizes()[faceIdx]);
    for (uint32_t i = 0; i < border.size(); i++) border[i] = m_sculpture->mesh()->vertices()[face[i]];
    toWorldSpace(border, rotation);

    Waypoint wp = alignedToFaceWP(border, normal);
    if (wp.isValid()) m_robot->moveTo(wp);
}

void SculptProcess::toWorldSpace(glm::dvec3& normal, const glm::dquat& rotation)
{
    normal = rotation * normal;
}
void SculptProcess::toWorldSpace(std::vector<glm::dvec3>& border, const glm::dquat& rotation) const
{
    for (glm::dvec3& vertex : border) {
        vertex = rotation * vertex + m_sculpture->position();
    }
}

// Develop axes aligned to a face [by face normal] (x along cut direction, y normal to plane, z along blade)
Axis3D SculptProcess::faceAlignedAxes(const glm::dvec3& normal, bool alignHorizontal)
{
    Axis3D axes(normal);
    axes.rotateX(-M_PI / 2);

    // Bring the zAxis into the horizontal plane
    if (alignHorizontal) {
        double planeAngle = atan2(-glm::dot(axes.zAxis, UP), glm::dot(axes.xAxis, UP));
        axes.rotateY(-planeAngle);
    }

    return axes;
}

Waypoint SculptProcess::alignedToFaceWP(const std::vector<glm::dvec3>& border, const glm::dvec3& normal) const
{
    auto axes = faceAlignedAxes(normal, false);

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
        if (std::abs(glm::dot(delta, axes.zAxis)) < 1e-12 && glm::dot(delta, m_fwd) < 0) idx = pre;

        delta = border[post] - border[idx];
        if (std::abs(glm::dot(delta, axes.zAxis)) < 1e-12 && glm::dot(delta, m_fwd) < 0) idx = post;

        return alignedToVertexWP(axes, poseAdjustedVertex(axes, border[idx]));
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

// Apply robot offsets to vertex (Difference between origin and start of cutting surface)
glm::dvec3 SculptProcess::poseAdjustedVertex(const Axis3D& axes, const glm::dvec3& vertex) const
{
    return vertex - axes.zAxis * 0.5 * m_bladeLength + axes.yAxis * 0.5 * m_bladeThickness;
}

uint32_t SculptProcess::getActionLimit() const
{
    return m_actionLimit;
}
bool SculptProcess::isActionLimitEnabled() const
{
    return m_actionLimitEnable;
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

const std::shared_ptr<Sculpture>& SculptProcess::getSculpture() const
{
    return m_sculpture;
}

const std::shared_ptr<Robot>& SculptProcess::getSculptor() const
{
    return m_robot;
}
const std::shared_ptr<Robot>& SculptProcess::getTurntable() const
{
    return m_turntable;
}

//TODO arrange steps better
std::vector<Plane> SculptProcess::orderConvexTrim(const std::vector<Plane>& cuts)
{
    std::vector<Plane> steps = cuts;

    if (m_sliceOrder == ConvexSliceOrder::TOP_DOWN) { // Plan cuts beginning from the top and moving towards the base
        std::sort(steps.begin(), steps.end(), [](const Plane& a, const Plane& b){
            return glm::dot(a.normal, UP) > glm::dot(b.normal, UP);
        });
    } else { // Plan cuts beginning from the bottom and moving upwards
        std::sort(steps.begin(), steps.end(), [](const Plane& a, const Plane& b){
            return glm::dot(a.normal, UP) < glm::dot(b.normal, UP);
        });
    }

    return steps;
}

void SculptProcess::planConvexTrim()
{
    std::vector<Plane> steps;

    ConvexHull hull = ConvexHull(model);

    for (uint32_t i = 0; i < hull.facetCount(); i++) steps.emplace_back(hull.facePlane(i));

    steps = orderConvexTrim(steps);

    // Process all the cuts preemptively
    hull = m_sculpture->hull();
    for (Plane& step : steps) {

        if (m_actionLimitEnable && m_actions.size() >= m_actionLimit) {
            std::cout << "\033[93m[SculptProcess] Action limit exceeded\033[0m\n";
            break;
        }

        auto fragments = Collision::fragments(hull, step);

        if (m_minCutVolume > 0) fragments.first.evaluate();

        double vol = fragments.first.volume();
        std::cout << "Step [" << (&step - &steps[0]) << ", vol: " << vol << "]:\n";

        // Try performing cut if the action would remove sufficient material
        if ((m_minCutVolume == 0 || fragments.first.volume() > m_minCutVolume)) {
            auto actions = planConvexTrim(hull, step);
            if (!actions.empty()) {
                commitActions(actions);
                hull = fragments.second;
                m_sculpture->setHull(hull); // Important to record so commitActions() checks collisions against the latest hull
            }
        }
    }
}

// Develops trajectories between disjoint actions to motions are continuous before attaching them to the list
void SculptProcess::commitActions(const std::vector<Action>& actions)
{
    if (m_linkActionEnable) {
        for (const Action& action : actions) {

            // Move robot away if it would collide with the TT when it moves
            if (action.target == m_turntable) {
                if (m_collisionTestingEnable) {
                    m_robot->moveTo(m_latestRobotCommand);

                    if (testTurntableCollision(action.trajectory)) {
                        m_actions.push_back(planTurntableClearance());
                    }
                }

                m_latestTableCommand = action.trajectory->end();
            }

            // Prepare approach trajectories to connect the last pose and the initial pose for this action
            if (action.target == m_robot) {
                m_turntable->moveTo(m_latestTableCommand);

                auto linkTrajectory = prepareApproach(action.trajectory->start());
                if (linkTrajectory != nullptr) m_actions.emplace_back(linkTrajectory, m_robot);
                else std::cout << "\033[93mFailed to link actions\033[0m\n";

                m_latestRobotCommand = action.trajectory->end();
            }

            m_actions.push_back(action);
        }
    } else for (const Action& action : actions) m_actions.push_back(action);

//    m_robot->print();
//    m_turntable->print();
//
//    print();
}

bool SculptProcess::testTurntableCollision(const std::shared_ptr<Trajectory>& ttTrajectory)
{
    return ttTrajectory->test(this, m_turntable, 0.05);
}

// Develops an action to move the robot out of the way of the turntable
SculptProcess::Action SculptProcess::planTurntableClearance()
{
    // TODO better motions than moving towards the neutral position
    return planApproach(m_robotNeutral);
}

SculptProcess::Action SculptProcess::planApproach(const Waypoint& destination)
{
    auto trajectory = std::make_shared<SimpleTrajectory>(m_latestRobotCommand, destination, m_solver);
    trajectory->setLimits(m_baseVelocityLimits, m_baseAccelerationLimits);

    m_latestRobotCommand = destination;

    return { trajectory, m_robot };
}

// Tries to identify a path from the last position to the specified position
// TODO Could improve reliability with an actual path planning algorithm
std::shared_ptr<Trajectory> SculptProcess::prepareApproach(const Waypoint& destination)
{
    const double dt = 0.1;

    // Try the direct path first
    auto directTrajectory = std::make_shared<SimpleTrajectory>(m_latestRobotCommand, destination, m_solver);
    directTrajectory->setLimits(m_baseVelocityLimits, m_baseAccelerationLimits);
    if (!m_collisionTestingEnable || validateTrajectory(directTrajectory, dt)) return directTrajectory;


    auto pose = m_robot->getPose(destination);
    pose.localTranslate({ 0, 0.5, 0});

    auto midpoint = m_robot->inverse(pose);
    if (midpoint.isValid()) {
        auto trajectory = std::make_shared<CompositeTrajectory>(m_robot->dof());
        trajectory->setLimits(m_baseVelocityLimits, m_baseAccelerationLimits);
        trajectory->addTrajectory(std::make_shared<SimpleTrajectory>(m_latestRobotCommand, midpoint, m_solver));
        trajectory->addTrajectory(std::make_shared<SimpleTrajectory>(midpoint, destination, m_solver));
        trajectory->update();

        if (validateTrajectory(trajectory, dt)) return trajectory;
    }

    return nullptr;
}

bool SculptProcess::validateTrajectory(const std::shared_ptr<Trajectory>& trajectory, double dt)
{
    return trajectory->isValid() && !trajectory->test(this, m_robot, dt);
}

// Generates trajectory to perform a trim action. Returns true if successful
std::vector<SculptProcess::Action> SculptProcess::planConvexTrim(const ConvexHull& hull, const Plane& plane)
{
    auto border = Collision::intersection(hull, plane);
    if (border.size() < 3) return {};

    std::shared_ptr<CompositeTrajectory> trajectory;
    const double initialRotation = m_turntable->getJointValue(0);

    double baseRotation = Ray::axialRotation(UP, plane.normal), theta;

    // Try developing a trajectory that will allow the robot to complete the planar section
    for (double offset : ATTEMPT_OFFSETS) {
        theta = baseRotation + offset;

        auto rotation = glm::angleAxis(theta, UP);

        glm::dvec3 wsNormal = plane.normal;
        toWorldSpace(wsNormal, rotation);

        std::vector<glm::dvec3> wsBorder = border;
        toWorldSpace(wsBorder, rotation);

        m_turntable->setJointValue(0, theta);

        trajectory = preparePlanarTrajectory(wsBorder, wsNormal);
        if (trajectory != nullptr) break;
    }

    if (trajectory == nullptr) {
        std::cout << "\033[93m[SculptProcess] Failed to develop an appropriate trajectory\033[0m\n";
//        throw std::runtime_error("[SculptProcess] Failed to develop an appropriate trajectory to perform planar cut");
        return {};
    }

    // Record mesh manipulation operation for later application during carving process
    m_sculpture->queueSection(plane.inverted());


    // Don't add new command if the TT is already in position
    if (std::abs(initialRotation - theta) < 1e-12) return { planRoboticSection(trajectory) };

    return { planTurntableAlignment(initialRotation, theta), planRoboticSection(trajectory) };
}

void SculptProcess::planOutlineRefinement(double stepDg)
{
    uint32_t steps = std::floor(180.0 / stepDg);
    const double initialRotation = m_turntable->getJointValue(0), baseRotation = -m_sculpture->rotation();
    double step = 0;

    // Prepare a 3D render / edge detection system to find the profile of the model
    auto* detector = new EdgeDetect(model);

    detector->setSize(1000);
    detector->setEpsilon(120.0f);

    detector->capture()->camera().setViewingAngle(180 / M_PI * baseRotation, 0);
    detector->capture()->focus();

    for (uint32_t i = 0; i < steps; i++) {

        // Render and determine the profile of the result
        detector->update();

        // Use the profile to prepare appropriate sculpting instructions for the robots
        auto profile = detector->profile();
        profile.setRefinementMethod(Profile::RefinementMethod::DELAUNEY);

        m_turntable->setJointValue(0, -baseRotation + M_PI * step / 180);

        auto actions = planOutlineRefinement(profile);
        if (!actions.empty()) {

            // Use the turntable to align the profile with the carving robot
            commitActions({ planTurntableAlignment(initialRotation, m_turntable->getJointValue(0)) });
            commitActions(actions);
        }

        // Save the detection result for debugging purposes
//        std::string filename = "SPOutlineRefinement" + std::to_string(step) + "dg.png";
//        detector->sink().save(QString(filename.c_str()));

        // Move the camera for the next image (in dg)
        detector->capture()->camera().rotate(stepDg);
        detector->capture()->focus();
        step += stepDg;

        break;
    }
}

std::vector<SculptProcess::Action> SculptProcess::planOutlineRefinement(Profile& profile)
{
    auto rotation = glm::angleAxis(m_turntable->getJointValue(0), UP);

    auto normal = profile.normal();
    auto border = profile.projected3D();

    auto wsNormal = normal;
    toWorldSpace(wsNormal, rotation);

    auto wsBorder = border;
    toWorldSpace(wsBorder, rotation);

    auto mesh = MeshBuilder::extrude(wsBorder, -wsNormal, 1.0f);
    mesh->translate(0.5 * wsNormal);
    mesh->setFaceColor({0, 0, 1});
    auto body = createBody(mesh);
    body->disableCollisions();

//    profile.correctWinding();

    std::vector<Action> actions;

    uint32_t count = 0;
    while (!profile.complete() && count < 3000) {

        auto clearance = profile.clearance();
        std::cout << "SS: " << profile.remainingSections() << " " << clearance.first << " " << clearance.second << "\n";

        if (clearance.first == 0 || clearance.second == 0) { // TODO actually operate
            profile.skip();
            continue;
        }

        bool external = profile.isNextExternal();
        auto tri = profile.refine();
        std::cout << "IDX: " << tri.I0 << " " << tri.I1 << " " << tri.I2 << "\n";

        if (tri.I0 != tri.I1) { // Verify triangle is valid
//            glm::dvec3 vn = glm::normalize(glm::cross(wsNormal, wsBorder[indices[0]] - wsBorder[indices[1]]));
//            Pose pose(wsBorder[indices[1]], faceAlignedAxes(vn, true));
//            auto wp = m_robot->inverse(pose);
//            if (wp.isValid()) actions.emplace_back(std::make_shared<HoldPosition>(wp), m_robot);

            auto axes = faceAlignedAxes(glm::normalize(glm::cross(wsNormal, UP)), true);
            auto wp = m_robot->inverse(Pose(wsBorder[tri.I0], axes));
            if (wp.isValid()) actions.emplace_back(std::make_shared<HoldPosition>(wp), m_robot);

            wp = m_robot->inverse(Pose(wsBorder[tri.I1], axes));
            if (wp.isValid()) actions.emplace_back(std::make_shared<HoldPosition>(wp), m_robot);

            wp = m_robot->inverse(Pose(wsBorder[tri.I2], axes));
            if (wp.isValid()) actions.emplace_back(std::make_shared<HoldPosition>(wp), m_robot);
//            m_sculpture->queueSection(border[0], border[1], border[2], profile.normal(), external);
        }

//        std::cout << "Profile: " << border.size() << ": \n";
//        for (auto& v : border) std::cout << v.x << " " << v.y << " " << v.z << "\n";
//        createBody(MeshBuilder::extrude(border, profile.normal(), 2.0f));
        count++;
    }

    std::cout << "Refinement iterations: " << count << ":\n";
    std::cout << m_sculpture->position().x << " " << m_sculpture->position().y << " " << m_sculpture->position().z << "\n";

    return actions;
}

void SculptProcess::planFeatureRefinement()
{
    //TODO
    // Refine features based on layers?
    // Decompose model into patches and handle separately?
}

SculptProcess::Action SculptProcess::planTurntableAlignment(double start, double end)
{
    auto trajectory = std::make_shared<SimpleTrajectory>(
            Waypoint({ start }, false), Waypoint({ end }, false), m_solver);

    trajectory->setLimits({ M_PI / 2}, { M_PI });

    return { trajectory, m_turntable };
}

SculptProcess::Action SculptProcess::planRoboticSection(const std::shared_ptr<CompositeTrajectory>& trajectory)
{
    Action action(trajectory, m_robot);
    action.cuts = m_cuts;
    m_cuts.clear();
    return action;
}

// Sectioning step wherein the robot moves to remove all material above the specified plane
std::shared_ptr<CompositeTrajectory> SculptProcess::preparePlanarTrajectory(const std::vector<glm::dvec3>& border, const glm::dvec3& normal)
{
    auto axes = faceAlignedAxes(normal, true);

    uint32_t minIndex, maxIndex;
    VertexArray::extremes(border, axes.xAxis, minIndex, maxIndex);

    double runup = 0.1, length = glm::dot(border[maxIndex] - border[minIndex], axes.xAxis);

    if (glm::dot(UP, normal) > 0) { // Upwards-facing cut -> Can use through cut
        return prepareThroughCut(Pose(border[minIndex], axes), length + runup, runup + 0.5 * m_bladeWidth, 0.4 * axes.yAxis);
    }

    // Adjust length so it stops just before hitting the turntable
    length = -(length + runup) + 0.5 * m_bladeWidth
            - m_bladeThickness * tan(0.5 * M_PI - acos(glm::dot(UP, axes.yAxis)));

    auto off = glm::normalize(glm::cross(UP, axes.zAxis));

    off *= -(0.4 * m_sculpture->width() + glm::dot(off, length * axes.xAxis)); // Margin + depth (along off) cut

    // Downwards-facing cut -> Through but limited -> Leverage departure to push debris away from the piece
    return prepareThroughCut(Pose(border[maxIndex], axes), length, -runup, axes.localize(off));
}

glm::dvec3 SculptProcess::alignedToBlade(const Axis3D& axes, const glm::dvec3& vertex)
{
    return vertex - axes.zAxis * (0.5 * m_bladeLength + glm::dot(axes.zAxis, vertex - m_sculpture->position()))
                  + axes.yAxis * 0.5 * m_bladeThickness;
}

// startPose - Initial position of the robot before beginning the cut
// depth - The distance to travel along the x-direction of startPose
// off - The direction & distance from which to leave the piece after the cut is complete
std::shared_ptr<CompositeTrajectory> SculptProcess::prepareThroughCut(Pose pose, double depth, double runup, const glm::dvec3& off)
{
    glm::dvec3 origin = pose.position;
    pose.position = alignedToBlade(pose.axes, pose.position);

    // Try to generate a cartesian trajectory between the two points
    pose.localTranslate({ -runup, 0, 0 });
    auto cutTrajectory = std::make_shared<CartesianTrajectory>(m_robot, pose, glm::dvec3(depth, 0, 0));

    cutTrajectory->setLimits(m_slowVelocityLimits, m_baseAccelerationLimits);
    if (!cutTrajectory->isValid()) return nullptr;

    auto trajectory = std::make_shared<CompositeTrajectory>(m_robot->dof());
    trajectory->setLimits(m_baseVelocityLimits, m_baseAccelerationLimits);

    // Pass through the cut
    trajectory->addTrajectory(cutTrajectory);

    // Pause for a bit to allow fragments to move away
    trajectory->addTrajectory(std::make_shared<HoldPosition>(cutTrajectory->end(), 0.5));

    // Move off the piece
    if (glm::dot(off, off) > 1e-6) {
        auto endPose = m_robot->getPose(cutTrajectory->end());

        auto offTrajectory = std::make_shared<CartesianTrajectory>(m_robot, endPose, off);
        if (!offTrajectory->isValid()) return nullptr;
        trajectory->addTrajectory(offTrajectory);
    }

    trajectory->update();

    if (trajectory->isValid()) {
        auto [ts, tf] = trajectory->tLimits(0);
        auto axes = pose.axes;
        if (depth < 0) axes.flipXZ();

        m_cuts.emplace_back(Pose(origin, axes), ts, tf, 0);
        return trajectory;
    }

    return nullptr;
}

std::shared_ptr<CompositeTrajectory> SculptProcess::prepareBlindCut(const Pose& pose, double depth)
{
    std::cout << "BLINDCUT\n";
    glm::dvec3 delta = { -depth + 0.5 * m_bladeWidth, 0, 0 }; // In local coordinates
    auto cutTrajectory = std::make_shared<CartesianTrajectory>(m_robot, pose, delta);
    cutTrajectory->setLimits(m_slowVelocityLimits, m_baseAccelerationLimits);
    if (!cutTrajectory->isValid()) return nullptr;

    auto retractTrajectory = cutTrajectory->reversed(m_robot);
    if (!retractTrajectory->isValid()) return nullptr;

    auto trajectory = std::make_shared<CompositeTrajectory>(m_robot->dof());
    trajectory->setLimits(m_baseVelocityLimits, m_baseAccelerationLimits);

    // Move to initial cut position
    trajectory->addTrajectory(std::make_shared<SimpleTrajectory>(m_latestRobotCommand, cutTrajectory->start(), m_solver));

    // Perform blind cut
    trajectory->addTrajectory(cutTrajectory);
    trajectory->addTrajectory(retractTrajectory);

    // Move to the neutral position
//    trajectory->addTrajectory(std::make_shared<SimpleTrajectory>(trajectory->end(), m_robotNeutral, m_solver));

    trajectory->update();

    if (trajectory->isValid()) return trajectory;
    return nullptr;
}

void SculptProcess::nextAction()
{
    std::cout << "Step " << m_step << " / " << m_actions.size() << "\n";
    auto& action = m_actions[m_step];

    std::cout << "AS: " << action.cuts.size() << " " << action.target << " " << action.trajectory->start() << " | " << action.trajectory->end() << "\n";

    // Begin motion of the robot
    action.target->traverse(action.trajectory);
    action.started = true;

    if (!action.cuts.empty()) {
        removeDebris();

        m_debris = m_sculpture->applySection();
        m_debris->setName("ACTION" + std::to_string(m_step) + "-DEBRIS");

        auto rotation = glm::quat_cast(m_sculpture->getRotation());

        for (Cut& cut : action.cuts) {
            auto localPose = Pose((cut.pose.position - m_sculpture->position()) * rotation, cut.pose.axes);
            localPose.axes.rotate(-rotation);

            m_debris->prepareCut(localPose, m_bladeThickness);
        }

        m_debris->remesh();

        prepareBody(m_debris);
    }
}

void SculptProcess::removeDebris()
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