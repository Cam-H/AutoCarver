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
#include "robot/trajectory/SimpleTrajectory.h"
#include "robot/trajectory/TOPPTrajectory.h"
#include "robot/trajectory/CartesianTrajectory.h"

#include "geometry/MeshBuilder.h"
#include "geometry/collision/Collision.h"
#include "fileIO/MeshHandler.h"

SculptProcess::SculptProcess(const std::shared_ptr<Mesh>& model)
    : Scene()
    , model(model)
    , m_sculpture(nullptr)
    , m_step(0)
    , m_planned(false)
    , m_convexTrimEnable(true)
    , m_processCutEnable(true)
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
    model->center();
    model->normalize();

    m_sculpture = std::make_shared<Sculpture>(model, m_config.materialWidth, m_config.materialHeight);
    m_sculpture->setName("SCULPTURE");

    prepareBody(m_sculpture, 1);
    prepareBody(m_sculpture->model(), 1);

    prepareTurntable();

    m_turntable->setEOAT(m_sculpture, false);
    m_turntable->translateEOAT(m_ttOffset);


    m_latestTableCommand = m_turntable->getWaypoint();
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

//    std::cout << "NF: " << near << " " << far << "\n";
//    auto c = ttj0Mesh->boundedOffset();
//    std::cout << "CC " << c.x << " " << c.y << " " << c.z << "\n";
//    c = ttBaseMesh->boundedOffset();
//    std::cout << "CC2 " << c.x << " " << c.y << " " << c.z << "\n";
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

    // Reset the sculpture
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

    if (m_step < m_actions.size())

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

// Develop axes aligned a face by normal (x along cut direction, y normal to plane, z along blade)
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

void SculptProcess::planConvexTrim()
{
    std::vector<Plane> steps;

    ConvexHull hull = ConvexHull(model);

    for (uint32_t i = 0; i < hull.facetCount(); i++) steps.emplace_back(hull.facePlane(i));

    // Plan initial cuts, beginning from the top and moving towards the base
    std::sort(steps.begin(), steps.end(), [](const Plane& a, const Plane& b){
        return glm::dot(a.normal, UP) > glm::dot(b.normal, UP);
    });

    //TODO arrange steps better

    // Process all the cuts preemptively
    hull = m_sculpture->hull();
    for (Plane& step : steps) {

        auto fragments = Collision::fragments(hull, step);
        m_sculpture->setHull(fragments.second);

        if (m_minCutVolume > 0) fragments.first.evaluate();

        double vol = fragments.first.volume();
        std::cout << "Step [" << (&step - &steps[0]) << ", vol: " << vol << "]:\n";

        // Try performing cut if the action would remove sufficient material
        if ((m_minCutVolume == 0 || fragments.first.volume() > m_minCutVolume)
            && planConvexTrim(hull, step)) hull = fragments.second;

        if (m_actions.size() > 8) break;
//        break;
    }
}

// Generates trajectory to perform a trim action. Returns true if successful
bool SculptProcess::planConvexTrim(const ConvexHull& hull, const Plane& plane)
{
    static std::array<double, 13> attemptOffsets = { 0,
                                                     -M_PI / 32, M_PI / 32
                                                     -M_PI / 16, M_PI / 16,
                                                     -M_PI / 8, M_PI / 8,
                                                     -M_PI / 4, M_PI / 4,
                                                     -M_PI / 3, M_PI / 3,
                                                     -M_PI / 2, M_PI / 2 };

//    static std::array<double, 1> attemptOffsets = { 0 };

    auto border = Collision::intersection(hull, plane);
    if (border.size() < 3) return false;

    std::shared_ptr<Trajectory> trajectory;
    double baseRotation = Ray::axialRotation(UP, plane.normal), theta; // m_latestTableCommand.values[0]

    // Try developing a trajectory that will allow the robot to complete the planar section
    for (double offset : attemptOffsets) {
        theta = baseRotation + offset;

        auto rotation = glm::angleAxis(theta, UP);

        glm::dvec3 wsNormal = plane.normal;
        toWorldSpace(wsNormal, rotation);

        std::vector<glm::dvec3> wsBorder = border;
        toWorldSpace(wsBorder, rotation);

        m_sculpture->setRotation(rotation);

        trajectory = preparePlanarTrajectory(wsBorder, wsNormal);
        if (trajectory != nullptr) break;
    }

    if (trajectory == nullptr) {
        std::cout << "\033[93m[SculptProcess] Failed to develop an appropriate trajectory\033[0m\n";
//        throw std::runtime_error("[SculptProcess] Failed to develop an appropriate trajectory to perform planar cut");
        return false;
    }

    // If a trajectory was found queue it
    planTurntableAlignment(theta);
    planRoboticSection(trajectory);

    // Record mesh manipulation operation for later application during carving process
    m_sculpture->queueSection(plane.origin, -plane.normal);

    return true;
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
    if (std::abs(m_latestTableCommand.values[0] - theta) < 1e-12) return; // Don't add new command if the TT is already in position

    auto trajectory = std::make_shared<SimpleTrajectory>(m_latestTableCommand, Waypoint({ theta}, false), m_solver);
    trajectory->setLimits({ M_PI / 2}, { M_PI });
    m_latestTableCommand = trajectory->end();

    m_actions.emplace_back(trajectory, m_turntable);
}

void SculptProcess::planRoboticSection(const std::shared_ptr<Trajectory>& trajectory)
{
    m_latestRobotCommand = trajectory->end();

    m_actions.emplace_back(trajectory, m_robot);
    m_actions[m_actions.size() - 1].trigger = true;
}

// Sectioning step wherein the robot moves to remove all material above the specified plane
std::shared_ptr<Trajectory> SculptProcess::preparePlanarTrajectory(const std::vector<glm::dvec3>& border, const glm::dvec3& normal)
{
    auto axes = faceAlignedAxes(normal, true);

    uint32_t minIndex, maxIndex;
    VertexArray::extremes(border, axes.xAxis, minIndex, maxIndex);
    auto low = border[minIndex], high = border[maxIndex];

    double runup = 0.1 + m_bladeWidth, length = glm::dot(high - low, axes.xAxis);

    if (glm::dot(UP, normal) > 0) { // Upwards-facing cut -> Can use through cut

        low -= axes.zAxis * (0.5 * m_bladeLength + glm::dot(axes.zAxis, low - m_sculpture->position()));

        auto startPose = Pose(low, axes);
        startPose.localTranslate({ -runup, 0, 0 });

        return prepareThroughCut(startPose, length + 2 * runup, 0.4 * startPose.axes.yAxis);
    }

    high -= axes.zAxis * (0.5 * m_bladeLength + glm::dot(axes.zAxis, high - m_sculpture->position()));

    auto startPose = Pose(high, axes);
    startPose.localTranslate({ runup, 0, 0 });

    // Adjust length so it stops just before hitting the turntable
    length = -(length + runup) + 0.5 * m_bladeWidth; // TODO: account for blade thickness

    auto off = glm::normalize(glm::cross(UP, axes.zAxis));

    off *= -(0.4 * m_sculpture->width() + glm::dot(off, length * axes.xAxis)); // Margin + depth (along off) cut

    // Downwards-facing cut -> Through but limited -> Leverage departure to push debris away from the piece
    return prepareThroughCut(startPose, length, startPose.axes.localize(off));
}

// Tries to identify a path from the last position to the specified position
// TODO Could improve reliability with an actual path planning algorithm
std::shared_ptr<CompositeTrajectory> SculptProcess::prepareApproach(const Waypoint& destination)
{
    const double dt = 0.01;
    double duration = 0;

    auto trajectory = std::make_shared<CompositeTrajectory>(m_robot->dof());
    trajectory->setLimits(m_baseVelocityLimits, m_baseAccelerationLimits);

    // Try direct path
//    auto nextTraj = std::make_shared<SimpleTrajectory>(m_latestRobotCommand, destination, m_solver);
//    if (validateTrajectory(nextTraj, dt)) {
//        trajectory->addTrajectory(nextTraj);
//        std::cout << "A1\n";
//        return trajectory;
//    }

    // Try passing through the neutral pose
    auto nextTraj = std::make_shared<SimpleTrajectory>(m_latestRobotCommand, m_robotNeutral, m_solver);
    if (validateTrajectory(nextTraj, dt)) {
        trajectory->addTrajectory(nextTraj);
//        duration += nextTraj->duration();

        nextTraj = std::make_shared<SimpleTrajectory>(m_robotNeutral, destination, m_solver);
        if (validateTrajectory(nextTraj, dt)) { // dt * (duration + nextTraj->duration()) / nextTraj->duration()
            trajectory->addTrajectory(nextTraj);
//            std::cout << "A2\n";
            return trajectory;
        }
    }

//    std::cout << "REJECT\n";

    return nullptr;
}

bool SculptProcess::validateTrajectory(const std::shared_ptr<Trajectory>& trajectory, double dt)
{
    return trajectory->validate(m_robot, dt) && !trajectory->test(this, m_robot, dt);
}

// startPose - Initial position of the robot before beginning the cut
// depth - The distance to travel along the x-direction of startPose
// off - The direction & distance from which to leave the piece after the cut is complete
std::shared_ptr<Trajectory> SculptProcess::prepareThroughCut(const Pose& startPose, double depth, const glm::dvec3& off)
{
    // Try to generate a cartesian trajectory between the two points
    auto cutTrajectory = std::make_shared<CartesianTrajectory>(m_robot, startPose, glm::dvec3(depth, 0, 0));

    cutTrajectory->setLimits(m_slowVelocityLimits, m_baseAccelerationLimits);
    if (!cutTrajectory->validate(m_robot, 0.01)) return nullptr;

    // Move to initial cut position
    auto trajectory = prepareApproach(cutTrajectory->start());
    if (trajectory == nullptr) return nullptr;

    // Pass through the cut
    trajectory->addTrajectory(cutTrajectory);

    // Move off the piece
    auto endPose = m_robot->getPose(cutTrajectory->end());

    auto offTrajectory = std::make_shared<CartesianTrajectory>(m_robot, endPose, off);
    if (!offTrajectory->validate(m_robot, 0.01)) return nullptr;
    trajectory->addTrajectory(offTrajectory);

    trajectory->update();

    if (trajectory->validate(m_robot, 0.01)) return trajectory;

    return nullptr;
}

std::shared_ptr<Trajectory> SculptProcess::prepareBlindCut(const Pose& pose, double depth)
{
    std::cout << "BLINDCUT\n";
    glm::dvec3 delta = { -depth + 0.5 * m_bladeWidth, 0, 0 }; // In local coordinates
    auto cutTrajectory = std::make_shared<CartesianTrajectory>(m_robot, pose, delta);
    cutTrajectory->setLimits(m_slowVelocityLimits, m_baseAccelerationLimits);
    if (!cutTrajectory->validate(m_robot, 0.01)) return nullptr;

    auto retractTrajectory = cutTrajectory->reversed(m_robot);
    if (!retractTrajectory->validate(m_robot, 0.01)) return nullptr;

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

    if (trajectory->validate(m_robot, 0.01)) return trajectory;
    return nullptr;
}

void SculptProcess::planRoboticSection(const std::vector<glm::dvec3>& border)
{
    std::vector<Waypoint> waypoints = {
            m_latestRobotCommand,
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

    std::cout << "AS: " << action.trigger << " " << action.target << " " << action.trajectory->start() << " | " << action.trajectory->end() << "\n";

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