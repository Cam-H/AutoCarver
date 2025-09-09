//
// Created by cjhat on 2025-09-08.
//

#include "ProcessPlanner.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/string_cast.hpp>
#include <gtx/quaternion.hpp>

#include "core/Scene.h"
#include "core/Sculpture.h"

#include "renderer/EdgeDetect.h"
#include "renderer/RenderCapture.h"

#include "geometry/curves/Interpolator.h"
#include "geometry/MeshBuilder.h"
#include "geometry/collision/Collision.h"

#include "robot/Robot.h"
#include "robot/trajectory/Waypoint.h"
#include "robot/trajectory/CompositeTrajectory.h"
#include "robot/trajectory/SimpleTrajectory.h"
#include "robot/trajectory/TOPPTrajectory.h"
#include "robot/trajectory/CartesianTrajectory.h"
#include "robot/trajectory/HoldPosition.h"

#include "geometry/poly/Profile.h"
#include "geometry/poly/SectionOperation.h"

#include "geometry/primitives/ConvexHull.h"
#include "geometry/primitives/Plane.h"


#include "Sequence.h"

#include "fileIO/MeshHandler.h"

static const std::array<double, 13> ATTEMPT_OFFSETS = { 0,
                                                        -M_PI / 32, M_PI / 32
                                                        -M_PI / 16, M_PI / 16,
                                                        -M_PI / 8, M_PI / 8,
                                                        -M_PI / 4, M_PI / 4,
                                                        -M_PI / 3, M_PI / 3,
                                                        -M_PI / 2, M_PI / 2 };

//static const std::array<double, 1> attemptOffsets = { 0 };

ProcessPlanner::ProcessPlanner()
    : m_config()
{

}

bool ProcessPlanner::plan(const std::shared_ptr<Sculpture>& target, const std::shared_ptr<Robot>& sculptor, const std::shared_ptr<Robot>& tt)
{
    sculpture = target;
    robot = sculptor;
    turntable = tt;

    m_actions.clear();
    m_cuts.clear();

    const Waypoint initialRobotCommand = m_latestRobotCommand = robot->getWaypoint();
    const Waypoint initialTableCommand = turntable->getWaypoint();

    // Ready the sculpture by carving the convex hull of the model
    if (m_config.convexTrimEnable) planConvexTrim();
    else sculpture->restoreAsHull();

    // Repeatedly refined the sculpture by cutting to the outline of the model at discrete orientations
    if (m_config.silhouetteRefinementEnable) planOutlineRefinement(m_config.stepOffset);

    // Capture model features with fine carving
//    planFeatureRefinement();

    // Return to the home position
    if (std::abs(turntable->getJointValue(0)) > 1e-12) m_actions.emplace_back(planTurntableAlignment(turntable->getJointValue(0), 0));
    m_actions.emplace_back(std::make_shared<HoldPosition>(m_config.robotHome, 0.1), robot);


    // Restore initial configuration
    robot->moveTo(initialRobotCommand);
    turntable->moveTo(initialTableCommand);

    if (m_config.convexTrimEnable) sculpture->restore();
    else sculpture->restoreAsHull();

    sculpture->setRotation(glm::angleAxis(0.0, UP));

    sculpture = nullptr;
    robot = turntable = nullptr;

    return true;
}

ProcessConfiguration& ProcessPlanner::getConfiguration()
{
    return m_config;
}

const ProcessConfiguration& ProcessPlanner::getConfiguration() const
{
    return m_config;
}

const std::deque<Action>& ProcessPlanner::actions() const
{
    return m_actions;
}

void ProcessPlanner::planConvexTrim()
{
    std::vector<Plane> steps;

    ConvexHull hull = ConvexHull(sculpture->model()->mesh());

    for (uint32_t i = 0; i < hull.facetCount(); i++) steps.emplace_back(hull.facePlane(i));

    steps = orderConvexTrim(steps);

    // Process all the cuts preemptively
    hull = sculpture->hull();
    for (Plane& step : steps) {
        if (!withinActionLimit()) break;

        auto fragments = Collision::fragments(hull, step);

        if (m_config.minCutVolume > 0) fragments.first.evaluate();

        double vol = fragments.first.volume();
        std::cout << "Step [" << (&step - &steps[0]) << ", vol: " << vol << "]:\n";

        // Try performing cut if the action would remove sufficient material
        if ((m_config.minCutVolume == 0 || fragments.first.volume() > m_config.minCutVolume)) {
            auto actions = planConvexTrim(hull, step);
            if (!actions.empty()) {
                m_actions.insert(m_actions.end(), actions.begin(), actions.end());
                hull = fragments.second;
                sculpture->setHull(hull); // Important to record so commitActions() checks collisions against the latest hull
            }
        }
    }
}

// Generates trajectory to perform a trim action. Returns true if successful
std::vector<Action> ProcessPlanner::planConvexTrim(const ConvexHull& hull, const Plane& plane)
{
    auto border = Collision::intersection(hull, plane);
    if (border.size() < 3) return {};

    std::shared_ptr<CompositeTrajectory> trajectory;
    const double initialRotation = turntable->getJointValue(0);

    double baseRotation = Ray::axialRotation(UP, plane.normal), theta;

    // Try developing a trajectory that will allow the robot to complete the planar section
    for (double offset : ATTEMPT_OFFSETS) {
        theta = baseRotation + offset;

        auto rotation = glm::angleAxis(theta, UP);

        glm::dvec3 wsNormal = rotation * plane.normal;

        std::vector<glm::dvec3> wsBorder = border;
        for (glm::dvec3& vertex : wsBorder) vertex = rotation * vertex + m_config.center;

        turntable->setJointValue(0, theta);

        trajectory = preparePlanarTrajectory(wsBorder, wsNormal);
        if (trajectory != nullptr) break;
    }

    if (trajectory == nullptr) {
        std::cout << "\033[93m[SculptProcess] Failed to develop an appropriate trajectory\033[0m\n";
//        throw std::runtime_error("[SculptProcess] Failed to develop an appropriate trajectory to perform planar cut");
        return {};
    }

    // Record mesh manipulation operation for later application during carving process
    sculpture->queueSection(plane.inverted());


    // Don't add new command if the TT is already in position
    if (std::abs(initialRotation - theta) < 1e-12) return { planRoboticSection(trajectory) };

    return { planTurntableAlignment(initialRotation, theta), planRoboticSection(trajectory) };
}

void ProcessPlanner::planOutlineRefinement(double stepDg)
{
    uint32_t steps = std::floor(180.0 / stepDg);
    const double initialRotation = turntable->getJointValue(0), baseRotation = sculpture->rotation(), offset = M_PI / 6;
    double step = 0;

    // Prepare a 3D render / edge detection system to find the profile of the model
    auto* detector = new EdgeDetect(sculpture->model()->mesh());

//    detector->setSize(1000);
//    detector->setEpsilon(120.0);

    detector->capture()->camera().setViewingAngle(180 / M_PI * -baseRotation, 0);
    detector->capture()->focus();

    // Block collision checks between blade and sculpture (Expected but should be ignored)
    robot->getEOAT()->setMask(robot->getEOAT()->mask() & ~sculpture->layer());

    for (uint32_t i = 0; i < steps; i++) {
        if (!withinActionLimit()) break;

        // Render and determine the profile of the result
        detector->update();

        // Use the profile to prepare appropriate sculpting instructions for the robots
        auto profile = detector->profile();
        profile.setRefinementMethod(Profile::RefinementMethod::DELAUNEY);

//        m_turntable->setJointValue(0, M_PI * step / 180);
        turntable->setJointValue(0, offset + baseRotation + M_PI * step / 180);

        auto actions = planOutlineRefinement(profile);
        if (!actions.empty()) {

            // Use the turntable to align the profile with the carving robot
            m_actions.emplace_back(planTurntableAlignment(initialRotation, turntable->getJointValue(0)));
            m_actions.insert(m_actions.end(), actions.begin(), actions.end());
        }

        // Save the detection result for debugging purposes
        std::string filename = "SPOutlineRefinement" + std::to_string(step) + "dg.png";
        detector->sink().save(QString(filename.c_str()));

        // Move the camera for the next image (in dg)
        detector->capture()->camera().rotate(stepDg);
        detector->capture()->focus();
        step += stepDg;
    }

    // Restore blade-sculpture collision checking
    robot->getEOAT()->setMask(robot->getEOAT()->mask() | sculpture->layer());

}

std::vector<Action> ProcessPlanner::planOutlineRefinement(Profile& profile)
{
    std::vector<Action> actions;

    m_latestRobotCommand = m_config.robotHome;

    while (!profile.complete()) {
        if (!withinActionLimit()) break;

        auto operation = profile.next(m_config.bladeWidth, m_config.bladeThickness);

//        std::cout << "IDX: " << tri.I0 << " " << tri.I1 << " " << tri.I2 << "\n";
        if (operation.valid) { // Confirm the operation is possible before attempting
            actions.emplace_back(planOutlineRefinement(profile, operation));
            if (actions.back().cuts.empty()) { // No cuts were possible - do nothing and do not proceed through tree
                std::cout << "\033[93mFailed to generate trajectory\033[0m\n";
                actions.pop_back();
                profile.skip();
            } else {
                m_latestRobotCommand = actions.back().trajectory->end();
                profile.refine();
            }
        } else profile.skip();
    }

    return actions;
}

Action ProcessPlanner::planOutlineRefinement(const Profile& profile, const SectionOperation& operation)
{
    auto rotation = glm::angleAxis(turntable->getJointValue(0), UP);

    auto trajectory = std::make_shared<CompositeTrajectory>(6);
    trajectory->setLimits(m_config.baseVelocityLimits, m_config.baseAccelerationLimits);

    Action action(trajectory, robot);

    const Pose initialPose = robot->getPose(m_latestRobotCommand);

    Sequence left(&profile, operation.left()), right(&profile, operation.right());

    left.exit = profile.projected3D(operation.leftExit());
    left.transform(rotation, m_config.center);

    right.exit = profile.projected3D(operation.rightExit());
    right.transform(rotation, m_config.center);

    // Perform cuts on both edges, beginning from the sequence nearer to the start position
    if (nearest(left, right)) {
        if (!planSequence(left, action) || !planSequence(right, action))
            return { nullptr, nullptr };
    } else {
        if (!planSequence(right, action) || !planSequence(left, action))
            return { nullptr, nullptr };
    }

    trajectory->update();
    for (Cut& cut : action.cuts) {
        auto [ts, tf] = trajectory->tLimits(cut.index);
        cut.ts = ts;
        cut.tf = tf;
    }

    sculpture->queueSection(profile.projected3D(operation.startVertex(m_config.bladeWidth)),
                              profile.projected3D(operation.splitVertex()),
                              profile.projected3D(operation.endVertex(m_config.bladeWidth)),
                              profile.normal());

    return action;
}

bool ProcessPlanner::nearest(const Sequence& test, const Sequence& comparison)
{
    Pose testPose = test.startPose(m_config.forward), compPose = comparison.startPose(m_config.forward);

    testPose.position = m_config.alignedToBlade(testPose.axes, testPose.position);
    compPose.position = m_config.alignedToBlade(compPose.axes, compPose.position);

    return m_latestRobotCommand.delta(robot->inverse(testPose)) < m_latestRobotCommand.delta(robot->inverse(compPose));
}

bool ProcessPlanner::planTranslation(const glm::dvec3& translation, Action& action)
{
    if (glm::dot(translation, translation) > 1e-6) {
        auto trajectory = std::static_pointer_cast<CompositeTrajectory>(action.trajectory);
        auto pose = action.target->getPose(action.trajectory->end());
        pose.position += translation;

        auto wp = action.target->inverse(pose);
        if (!wp.isValid()) return false;

        trajectory->addTrajectory(std::make_shared<SimpleTrajectory>(trajectory->end(), wp, m_config.solver));
    }

    return true;
}


bool ProcessPlanner::planSequence(const Sequence& sequence, Action& action)
{
    for (const Sequence::Set& set : sequence.sets) {

        auto axes = set.axes(m_config.forward);
        double direction = 1 - 2 * (glm::dot(set.axis, axes.xAxis) < 0);
        auto delta = -0.5 * set.axis * m_config.bladeWidth;
        bool started = false;

        for (const std::pair<glm::dvec3, double>& motion : set.motions) {
            auto pose = Pose(m_config.alignedToBlade(axes, motion.first + delta), axes);

            if (set.isBlind()) {
                if (!planBlindCut(pose, direction * motion.second, action)) return false;
            } else {
                auto cutNormal = glm::normalize(glm::cross(sequence.normal, set.travel));
                if (!planMill(pose, cutNormal, set.travel, motion.second, action)) return false;
            }
        }
    }

    return planTranslation(sequence.exit - sequence.end(), action);
}

bool ProcessPlanner::planBlindCut(const Pose& pose, double depth, Action& action)
{
    auto cutTrajectory = std::make_shared<CartesianTrajectory>(robot, pose, glm::dvec3(depth, 0, 0), 20);
    cutTrajectory->setLimits(m_config.slowVelocityLimits, m_config.baseAccelerationLimits);
    if (!cutTrajectory->isValid()) return false;

    auto retractTrajectory = cutTrajectory->reversed(robot);
    if (!retractTrajectory->isValid()) return false;

    // Action must (is known to) only be called on actions formed with composite trajectories
    auto trajectory = std::static_pointer_cast<CompositeTrajectory>(action.trajectory);

    // Commit motions to action
    trajectory->connectTrajectory(cutTrajectory);
    trajectory->addTrajectory(retractTrajectory);

    auto axes = pose.axes;
    if (depth < 0) axes.flipXZ();

    action.cuts.emplace_back(Pose(pose.position + 0.5 * m_config.bladeWidth * pose.axes.xAxis, axes), 0, 0, 0, trajectory->segmentCount() - 2);

    return true;
}

bool ProcessPlanner::planMill(const Pose& pose, const glm::dvec3& normal, const glm::dvec3& travel, double depth, Action& action)
{
    auto millTrajectory = std::make_shared<CartesianTrajectory>(robot, pose, pose.axes.localize(travel * depth), 20);
    millTrajectory->setLimits(m_config.slowVelocityLimits, m_config.baseAccelerationLimits);
    if (!millTrajectory->isValid()) return false;

    // Action must (is known to) only be called on actions formed with composite trajectories
    auto trajectory = std::static_pointer_cast<CompositeTrajectory>(action.trajectory);
//    trajectory->addTrajectory(std::make_shared<HoldPosition>(millTrajectory->start(), 0.2));

    // Commit motions to action
    trajectory->connectTrajectory(millTrajectory);
    trajectory->addTrajectory(millTrajectory->reversed(robot));

    double direction = 1 - 2 * (glm::dot(pose.axes.xAxis, travel) < 0);
    glm::dvec3 origin = pose.position + 0.5 * m_config.bladeWidth * direction * pose.axes.xAxis;

    double theta = acos(glm::dot(travel, pose.axes.yAxis));
//    std::cout << "PMT: " << depth << " " << m_bladeThickness << " " << teff << " " << teff * direction << " " << direction << " | " << " " << normal.x << " " << normal.y << " " << normal.z << "\n";
//    if (teff < 0) throw std::runtime_error("[SculptProcess] Failed to calculate effective thickness");


    action.cuts.emplace_back(origin, normal, travel, theta, 0, 0, trajectory->segmentCount() - 2);

    return true;
}

void ProcessPlanner::planFeatureRefinement()
{
    //TODO
    // Refine features based on layers?
    // Decompose model into patches and handle separately?
}

Action ProcessPlanner::planTurntableAlignment(double start, double end)
{
    auto trajectory = std::make_shared<SimpleTrajectory>(
            Waypoint({ start }, false), Waypoint({ end }, false), m_config.solver);

    trajectory->setLimits({ M_PI / 2}, { M_PI });

    return { trajectory, turntable };
}

Action ProcessPlanner::planRoboticSection(const std::shared_ptr<CompositeTrajectory>& trajectory)
{
    Action action(trajectory, robot);
    action.cuts = m_cuts;
    m_cuts.clear();
    return action;
}

// Sectioning step wherein the robot moves to remove all material above the specified plane
std::shared_ptr<CompositeTrajectory> ProcessPlanner::preparePlanarTrajectory(const std::vector<glm::dvec3>& border, const glm::dvec3& normal)
{
    auto axes = Axis3D::faceAligned(normal, m_config.forward, true);

    uint32_t minIndex, maxIndex;
    VertexArray::extremes(border, axes.xAxis, minIndex, maxIndex);

    double runup = 0.1, length = glm::dot(border[maxIndex] - border[minIndex], axes.xAxis) + 1e-3; // 1e-3 to go a bit further (So debris is disconnected properly (Avoid vertex touch))

    if (glm::dot(UP, normal) > 0) { // Upwards-facing cut -> Can use through cut
        return prepareThroughCut(Pose(border[minIndex], axes), length + runup, runup + 0.5 * m_config.bladeWidth, 0.4 * axes.yAxis);
    }

    // Adjust length so it stops just before hitting the turntable
    length = -(length + runup) + 0.5 * m_config.bladeWidth
             - m_config.bladeThickness * tan(0.5 * M_PI - acos(glm::dot(UP, axes.yAxis)));

    auto off = glm::normalize(glm::cross(UP, axes.zAxis));

    off *= -(0.4 * m_config.materialWidth + glm::dot(off, length * axes.xAxis)); // Margin + depth (along off) cut

    // Downwards-facing cut -> Through but limited -> Leverage departure to push debris away from the piece
    return prepareThroughCut(Pose(border[maxIndex], axes), length, -runup, axes.localize(off));
}

// startPose - Initial position of the robot before beginning the cut
// depth - The distance to travel along the x-direction of startPose
// off - The direction & distance from which to leave the piece after the cut is complete
std::shared_ptr<CompositeTrajectory> ProcessPlanner::prepareThroughCut(Pose pose, double depth, double runup, const glm::dvec3& off)
{
    pose.position = m_config.alignedToBlade(pose.axes, pose.position);

    // Try to generate a cartesian trajectory between the two points
    pose.localTranslate({ -runup, 0, 0 });
    auto cutTrajectory = std::make_shared<CartesianTrajectory>(robot, pose, glm::dvec3(depth, 0, 0), 20);

    cutTrajectory->setLimits(m_config.slowVelocityLimits, m_config.baseAccelerationLimits);
    if (!cutTrajectory->isValid()) return nullptr;

    auto trajectory = std::make_shared<CompositeTrajectory>(robot->dof());
    trajectory->setLimits(m_config.baseVelocityLimits, m_config.baseAccelerationLimits);

    // Pass through the cut
    trajectory->addTrajectory(cutTrajectory);

    // Pause for a bit to allow fragments to move away
    trajectory->addTrajectory(std::make_shared<HoldPosition>(cutTrajectory->end(), 0.5));

    // Move off the piece
    if (glm::dot(off, off) > 1e-6) {
        auto endPose = robot->getPose(cutTrajectory->end());

        auto offTrajectory = std::make_shared<CartesianTrajectory>(robot, endPose, off, 20);
        if (!offTrajectory->isValid()) return nullptr;
        trajectory->addTrajectory(offTrajectory);
    }

    trajectory->update();

    if (trajectory->isValid()) {
        auto [ts, tf] = trajectory->tLimits(0);
        auto axes = pose.axes;
        if (depth < 0) axes.flipXZ();

        m_cuts.emplace_back(Pose(pose.position, axes), 0, ts, tf, 0);
        return trajectory;
    }

    return nullptr;
}

bool ProcessPlanner::withinActionLimit() const
{
    if (m_config.actionLimitEnable && m_actions.size() >= m_config.actionLimit) {
        std::cout << "\033[93m[SculptProcess] Action limit exceeded\033[0m\n";
        return false;
    }

    return true;
}

//TODO arrange steps better
std::vector<Plane> ProcessPlanner::orderConvexTrim(const std::vector<Plane>& cuts)
{
    std::vector<Plane> steps = cuts;

    if (m_config.sliceOrder == ProcessConfiguration::ConvexSliceOrder::TOP_DOWN) { // Plan cuts beginning from the top and moving towards the base
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