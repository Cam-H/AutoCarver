//
// Created by cjhat on 2025-09-08.
//

#include "ProcessPlanner.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/string_cast.hpp>
#include <gtx/quaternion.hpp>
#include <utility>

#include "core/Scene.h"
#include "core/Sculpture.h"

#include "renderer/EdgeDetect.h"
#include "renderer/RenderCapture.h"

#include "geometry/curves/Interpolator.h"
#include "geometry/MeshBuilder.h"
#include "geometry/collision/Collision3D.h"

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
#include "robot/planning/RRTPlanner.h"
#include "fileIO/SerialList.h"
#include "geometry/poly/ProfileBuilder.h"
#include "core/Timer.h"

static const std::array<double, 13> ATTEMPT_OFFSETS = { 0,
                                                        -M_PI / 32, M_PI / 32
                                                        -M_PI / 16, M_PI / 16,
                                                        -M_PI / 8, M_PI / 8,
                                                        -M_PI / 4, M_PI / 4,
                                                        -M_PI / 3, M_PI / 3,
                                                        -M_PI / 2, M_PI / 2 };

//static const std::array<double, 1> attemptOffsets = { 0 };

ProcessPlanner::ProcessPlanner()
    : ProcessPlanner(ProcessConfiguration())
{

}

ProcessPlanner::ProcessPlanner(ProcessConfiguration  configuration)
    : m_config(std::move(configuration))
{

}

ProcessPlanner::Report ProcessPlanner::plan(const std::shared_ptr<Sculpture>& target, const std::shared_ptr<Robot>& sculptor, const std::shared_ptr<Robot>& tt)
{
    m_report = {};
    m_report.success = true;

    sculpture = target;
    robot = sculptor;
    turntable = tt;

    m_actions.clear();
    m_cuts.clear();

    const Waypoint initialRobotCommand = m_latestRobotCommand = robot->getWaypoint();
    const Waypoint initialTableCommand = turntable->getWaypoint();

    // Ready the sculpture by carving the convex hull of the model
    if (m_config.convexTrimEnable) {
        planConvexTrim();

        if (m_config.convexSkipEnable) complete();
    } else sculpture->restoreAsHull();

    // Repeatedly refined the sculpture by cutting to the outline of the model at discrete orientations
    if (m_config.silhouetteRefinementEnable) {
        planSilhouetteRefinement(0.0);
        planSilhouetteRefinement(M_PI / 2);
    }

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

    return m_report;
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

// Applies all queued operations to the sculpture & moves to endpoints of current action set. Then clears queue -> NOT
void ProcessPlanner::complete()
{
    for (Action& action : m_actions) action.skip = true;
//    sculpture->complete();
//
//    bool sculptorUpdate = false, ttUpdate = false;
//    while (!m_actions.empty() && !(sculptorUpdate || ttUpdate)) {
//        if (!sculptorUpdate && m_actions.back().target == robot) {
//            robot->moveTo(m_actions.back().trajectory->end());
//            sculptorUpdate = true;
//        }
//
//        if (!ttUpdate && m_actions.back().target == turntable) {
//            turntable->moveTo(m_actions.back().trajectory->end());
//            ttUpdate = true;
//        }
//
//        m_actions.pop_back();
//    }
//
////    for(uint32_t)
//
//    m_actions.clear();
}

void ProcessPlanner::planConvexTrim()
{
    m_report.appliedConvexTrim = true;

    const auto& steps = convexTrimOrder();

    for (const std::pair<Plane, std::vector<glm::dvec3>>& step : steps) {
        auto actions = planConvexTrim(step);
        if (!actions.empty()) {
            m_actions.insert(m_actions.end(), actions.begin(), actions.end());
        }

        m_report.ctStepCount += actions.size();
    }

    if (m_config.silhouetteRefinementEnable) {

        uint32_t count = std::floor(360.0 / m_config.stepOffset);
        const double baseRotation = sculpture->rotation() + M_PI * m_config.stepStart / 180, offset = M_PI / 6;

        auto* detector = new EdgeDetect(sculpture->model()->mesh());

        detector->capture()->camera().setViewingAngle(180 / M_PI * -baseRotation, 0);
        detector->capture()->focus();

        for (uint32_t i = 0; i < count; i++) {
            if (!withinActionLimit(0)) break;

            // Render and determine the profile of the result
            detector->update();

            // Use the profile to prepare appropriate sculpting instructions for the robots
            auto profile = detector->profile();
            profile.setRefinementMethod(Profile::RefinementMethod::DELAUNEY);
            profile.setToolDimensions(m_config.bladeWidth, m_config.bladeThickness);
            planSilhouetteTrim(profile);

            // Move the camera for the next image (in dg)
            detector->capture()->camera().rotate(m_config.stepOffset);
            detector->capture()->focus();
        }
    }
}

std::vector<std::pair<Plane, std::vector<glm::dvec3>>> ProcessPlanner::convexTrimOrder() const
{
    ConvexHull target(sculpture->model()->mesh());
    ConvexHull base = sculpture->hull();

    bool mod = m_config.sliceOrder != ProcessConfiguration::ConvexSliceOrder::TOP_DOWN;

    // Divide planes into those above and below the horizontal
    std::array<std::vector<Plane>, 2> sets;
    for (uint32_t i = 0; i < target.facetCount(); i++) {
        const Plane& plane = target.facePlane(i);
        sets[mod ^ (glm::dot(UP, plane.normal) < 0)].push_back(plane);
    }

    // Order planes in the optimal way (In this case based on volume)
    std::vector<std::pair<Plane, std::vector<glm::dvec3>>> steps;
    for (const std::vector<Plane>& set : sets) {
        if (m_config.actionLimitEnable && steps.size() >= m_config.actionLimit) break;

        std::vector<Plane> planes = volumeOptimizedOrder(base, set);
        for (const Plane& plane : planes) {
            steps.emplace_back(plane, Collision3D::intersection(base, plane));
            base = Collision3D::fragment(base, plane.inverted());
        }
    }

    // TODO can further improve by ordering based on relative angle between normals
    // (Will require saving index of last reduction to limit regions to re-sort)

    return steps;
}

// Generates a list of planes for the purposes of convex trimming. The planes are ordered such that each step removes
// the maximum volume of material possible
std::vector<Plane> ProcessPlanner::volumeOptimizedOrder(const ConvexHull& base, const std::vector<Plane>& planes) const
{
    std::vector<ConvexHull> fragments;
    for (const Plane& plane : planes) {
        fragments.emplace_back(Collision3D::fragment(base, plane));
        fragments.back().evaluate();
    }

    std::vector<uint32_t> indices(fragments.size());
    std::iota(indices.begin(), indices.end(), 0);

    std::sort(indices.begin(), indices.end(), [&](uint32_t I0, uint32_t I1) {
        return fragments[I0].volume() > fragments[I1].volume();
    });

    // Ensure that action limit is not exceeded
    if (m_config.actionLimitEnable && indices.size() > m_config.actionLimit) {
        indices.erase(indices.end() - indices.size() + m_config.actionLimit, indices.end());
    }

    for (uint32_t i = 0; i < indices.size(); i++) {

        // Remove overly small fragments when requested
        if (m_config.minCutVolume > 0) {
            for (int j = indices.size() - 1; j > i; j--) {
                if (fragments[indices[j]].volume() < m_config.minCutVolume) indices.pop_back();
                else break;
            }
        }

        if (i < indices.size()) {

            // Adjust remaining fragments by removing contribution from the next step
            for (uint32_t j = i + 1; j < indices.size(); j++) {
                if (!Collision3D::below(fragments[indices[j]], planes[indices[i]].inverted())) {
                    fragments[indices[j]] = Collision3D::fragment(fragments[indices[j]], planes[indices[i]].inverted());
                    fragments[indices[j]].evaluate();
                }
            }

            // Order remaining fragment (index)s from largest to smallest
            std::sort(indices.begin() + i, indices.end(), [&](uint32_t I0, uint32_t I1) {
                return fragments[I0].volume() > fragments[I1].volume();
            });
        }
    }

    std::vector<Plane> steps;

    steps.reserve(indices.size());
    for (uint32_t idx : indices) steps.push_back(planes[idx]);

    return steps;
}

// Generates trajectory to perform a trim action. Returns true if successful
std::vector<Action> ProcessPlanner::planConvexTrim(const std::pair<Plane, std::vector<glm::dvec3>>& step)
{
    std::shared_ptr<CompositeTrajectory> trajectory;
    const double initialRotation = turntable->getJointValue(0);

    double baseRotation = Ray<3>::axialRotation(UP, step.first.normal), theta;

    // Try developing a trajectory that will allow the robot to complete the planar section
    for (double offset : ATTEMPT_OFFSETS) {
        theta = baseRotation + offset;

        auto rotation = glm::angleAxis(theta, UP);

        glm::dvec3 wsNormal = rotation * step.first.normal;

        std::vector<glm::dvec3> wsBorder = step.second;
        for (glm::dvec3& vertex : wsBorder) vertex = rotation * vertex + m_config.center;

        turntable->setJointValue(0, theta);

        auto axes = Axis3D::faceAligned(wsNormal, m_config.forward, true);
        trajectory = preparePlanarTrajectory(wsBorder, axes);
        if (trajectory != nullptr) break;
    }

    if (trajectory == nullptr) {
        std::cout << "\033[93m[SculptProcess] Failed to develop an appropriate trajectory\033[0m\n";
//        throw std::runtime_error("[SculptProcess] Failed to develop an appropriate trajectory to perform planar cut");
        return {};
    }

    // Record mesh manipulation operation for later application during carving process
    sculpture->queueSection(step.first.inverted());


    // Don't add new command if the TT is already in position
    if (std::abs(initialRotation - theta) < 1e-12) return { planRoboticSection(trajectory) };

    return { planTurntableAlignment(initialRotation, theta), planRoboticSection(trajectory) };
}

// Generates profiles at the specified offsets before generating cut instructions
void ProcessPlanner::planSilhouetteRefinement(double stepOffset)
{
    m_report.appliedSilhouetteRefinement = true;

    uint32_t steps = 2;
    const double baseRotation = sculpture->rotation() + stepOffset, offset = M_PI / 6;
    double initialRotation = turntable->getJointValue(0), step = 0;

    // Prepare a 3D render / edge detection system to find the profile of the model
    auto* detector = new EdgeDetect(sculpture->model()->mesh());

//    detector->setSize(1000);
//    detector->setEpsilon(120.0);

    detector->capture()->camera().setViewingAngle(180 / M_PI * -baseRotation, 0);
    detector->capture()->focus();

    // Block collision checks between blade and sculpture (Expected but should be ignored)
    robot->getEOAT()->setMask(robot->getEOAT()->mask() & ~sculpture->layer());

    std::cout << "STEPS: " << steps << " | " << m_config.stepOffset << "\n";
    for (uint32_t i = 0; i < steps; i++) {
        if (!withinActionLimit(0)) break;

        auto toNeutral = std::make_shared<SimpleTrajectory>(robot->getWaypoint(), m_config.robotHoverRight, m_config.solver);
        toNeutral->setLimits(m_config.baseVelocityLimits, m_config.baseAccelerationLimits);
        m_actions.emplace_back(toNeutral, robot);

        // Render and determine the profile of the result
        detector->update();

        // Use the profile to prepare appropriate sculpting instructions for the robots
        auto profile = detector->profile();
        profile.setRefinementMethod(Profile::RefinementMethod::DELAUNEY);
        profile.setToolDimensions(m_config.bladeWidth, m_config.bladeThickness);

        turntable->setJointValue(0, offset + baseRotation + M_PI * step / 180);

        auto actions = planProfile(profile);
        m_report.srOrientations.emplace_back(180 * stepOffset / M_PI + step, actions.size());

        if (!actions.empty()) {
            m_report.srStepCount += actions.size() + 1;

            // Use the turntable to align the profile with the carving robot
            m_actions.emplace_back(planTurntableAlignment(initialRotation, turntable->getJointValue(0)));
            initialRotation = turntable->getJointValue(0);

            m_actions.insert(m_actions.end(), actions.begin(), actions.end());
        }
        else m_report.success = false;

        // Save the detection result for debugging purposes
        std::string filename = "SPOutlineRefinement" + std::to_string(step) + "dg.png";
        detector->sink().save(QString(filename.c_str()));

        // Move the camera for the next image (in dg)
        detector->capture()->camera().rotate(180);
        detector->capture()->focus();
        step += 180;
    }

    // Restore blade-sculpture collision checking
    robot->getEOAT()->setMask(robot->getEOAT()->mask() | sculpture->layer());

}

// Trim the sculpture such that the hull (from profile's perspective) matches the model
// This step is important for later path planning to navigate with the profile
void ProcessPlanner::planSilhouetteTrim(const Profile& profile)
{
    auto toNeutral = std::make_shared<SimpleTrajectory>(robot->getWaypoint(), m_config.robotHoverRight, m_config.solver);
    toNeutral->setLimits(m_config.baseVelocityLimits, m_config.baseAccelerationLimits);
    m_actions.emplace_back(toNeutral, robot);

    auto model = std::make_shared<Sculpture>(*sculpture);
    model->complete();

    auto hull = ConvexHull(model->mesh());
    std::vector<glm::dvec2> border = Polygon::convex(profile.border()).border();

    double volume = hull.volume();

    glm::dvec2 prev = border.back();
    std::vector<std::pair<Plane, std::vector<glm::dvec3>>> steps;
    for (const glm::dvec2& vertex : border) {
        glm::dvec2 ext = glm::normalize(vertex - prev);
        ext = { ext.y, -ext.x };

        Plane plane = { profile.projected3D(prev), profile.projected3D(ext) };
        if (glm::dot(ext, { 1, 0}) > 1e-6 && glm::dot(plane.normal, UP) > -0.95) { // Skip downwards facing edge TODO
            steps.emplace_back(plane, Collision3D::intersection(hull, plane));
            hull = Collision3D::fragment(hull, plane.inverted());

            double newVolume = hull.volume();
            if (steps.back().second.size() < 3) { // Ignore slices with negligible effects
//                std::cout << "V: " << volume - hull.volume() << "\n";
                volume = newVolume;
                steps.pop_back();
            }
        }

        prev = vertex;
    }

    for (const std::pair<Plane, std::vector<glm::dvec3>>& step : steps) {
        if (m_config.actionLimitEnable && steps.size() >= m_config.actionLimit) break;

        auto actions = planConvexTrim(step);
        if (!actions.empty()) {
            m_actions.insert(m_actions.end(), actions.begin(), actions.end());
        }

        m_report.ctStepCount += actions.size(); // In SR but same work as CT
    }
}

// Given a profile (silhouette) of the mesh, prepares sculptor actions to perform required cuts to match
std::vector<Action> ProcessPlanner::planProfile(Profile& profile)
{
    ScopedTimer timer("[ProcessPlanner] profile planning");

    std::vector<Action> actions;
    actions.reserve(profile.remainingSections());

    auto rotation = glm::angleAxis(turntable->getJointValue(0), UP);

    profile.prepare();

    static int iter = 0;
    profile.save("../out/APROF" + std::to_string(iter++) + ".bin");

    profile.print();
    while (!profile.complete()) {
        if (!withinActionLimit(actions.size())) break;

        std::cout << "\033[91mPR" << profile.remainingSections() << " " << profile.nextDepth() << " " << actions.size() << " " << m_actions.size() << "\033[0m\n";

        if (profile.planNext()) {
            if (planSection(profile, actions)) profile.refine();
            else profile.skip();
        } else profile.skip();
    }

    if (!actions.empty()) {
        std::cout << "Outline complete. Returning...\n";
        for (uint32_t i = 0; i < 10; i++) {
            std::cout << "Return attempt " << i << ":\n";
            if (profile.planReturn()) {
                Sequence seq(&profile, profile.returnPath());
                seq.transform(rotation, m_config.center);

                auto trajectory = std::make_shared<CompositeTrajectory>(robot->dof());
                trajectory->setLimits(m_config.baseVelocityLimits, m_config.baseAccelerationLimits);

                Action action(trajectory, robot);
                if (planSequence(seq, action)) {
                    trajectory->update();
                    if (trajectory->isValid()) {
                        actions.emplace_back(action);
                        break;
                    }
                } else std::cout << "\033[93mFailed to prepare return sequence\n\033[0m";
            } else std::cout << "\033[93mFailed to plane return\n\033[0m";
        }

    }

    std::cout << "Profile complete\n";

    return actions;
}

bool ProcessPlanner::planSection(const Profile& profile, std::vector<Action>& actions)
{
    auto rotation = glm::angleAxis(turntable->getJointValue(0), UP);

    auto trajectory = std::make_shared<CompositeTrajectory>(robot->dof());
    trajectory->setLimits(m_config.baseVelocityLimits, m_config.baseAccelerationLimits);

    Action action(trajectory, robot);

    Sequence seq(&profile, profile.nextSequence());
    seq.transform(rotation, m_config.center);

    if (!planSequence(seq, action)) {
        std::cout << "\033[93m[ProcessPlanner] Failed to plan section\033[0m\n";
        return false;
    }

    trajectory->update();
    for (Cut& cut : action.cuts) {
        auto [ts, tf] = trajectory->tLimits(cut.index);
        cut.ts = ts;
        cut.tf = tf;
    }

    Triangle2D section = profile.nextBounds();
    sculpture->queueSection(profile.projected3D(section.a),
                            profile.projected3D(section.b),
                            profile.projected3D(section.c),
                            profile.normal());

    actions.emplace_back(action);
    return true;
}

bool ProcessPlanner::planSequence(const Sequence& sequence, Action& action)
{
    for (const Sequence::Set& set : sequence.sets) {

        // Try to approach set position, if required
        auto ref = Axis3D(glm::dot(m_config.forward, sequence.normal) < 0 ? -sequence.normal : sequence.normal);
        if (!set.approach.empty() && !planApproach(ref, set.approach, action)) return false;

        if (!set.motions.empty()) {

            auto axes = set.axes(m_config.forward);
            double direction = 1 - 2 * (glm::dot(set.axis, axes.xAxis) < 0);

            auto trajectory = std::static_pointer_cast<CompositeTrajectory>(action.trajectory);

            for (const std::pair<glm::dvec3, double>& motion : set.motions) {
                auto pose = Pose(motion.first + m_config.bladeCenterOffset(axes.zAxis, motion.first), axes);
                auto wp = robot->inverse(pose);

                assert(wp.values.size() == robot->dof());
                trajectory->connect(wp);

                if (set.isBlind()) {
                    if (!planBlindCut(pose, direction * motion.second, action)) return false;
                } else {
                    auto cutNormal = glm::normalize(glm::cross(sequence.normal, set.travel));
                    if (!planMill(pose, cutNormal, set.travel, motion.second, action)) return false;
                }
            }
        }
    }
    std::cout << "SEQUENCE successful\n";

    return true;
}

bool ProcessPlanner::planApproach(const Axis3D& axes, const std::vector<std::pair<glm::dvec3, double>>& approach, Action& action)
{
    std::cout << approach.size() << " PS APPROACH\n";

//    std::vector<Waypoint> waypoints;
    for (const std::pair<glm::dvec3, double>& vertex : approach) {

        std::cout << vertex.first.x << " " << vertex.first.y << " " << vertex.first.z << " [" << vertex.second << "]\n";
    }

    auto trajectory = std::static_pointer_cast<CompositeTrajectory>(action.trajectory);

    Pose start = Pose(m_config.alignedToBlade(axes, approach[0].first), axes);
    start.axes.rotateZ(M_PI_2 + approach[0].second);

    for (uint32_t i = 1; i < approach.size(); i++) {
        Pose destination = Pose(m_config.alignedToBlade(axes, approach[i].first), axes);
        destination.axes.rotateZ(M_PI_2 + approach[i].second);

        auto transit = std::make_shared<CartesianTrajectory>(robot, start, destination, 10);
        if (transit->isValid()) {
            trajectory->addTrajectory(transit);
        } else {
            std::cout << "\33[93m[ProcessPlanner] Failed to prepare cartesian link for approach. Fallback to simple\033[0m\n";
            std::cout << robot->inverse(start) << "\n" << robot->inverse(destination) << "\n";
            trajectory->connectTrajectory(std::make_shared<SimpleTrajectory>(robot->inverse(start), robot->inverse(destination), m_config.solver));
        }

        start = destination;
    }

    robot->setJointValue(5, trajectory->end().values[5]);
    std::cout << "APPROACH successful\n";

    return true;
}

Action ProcessPlanner::planPath2D(const Profile& profile, const std::vector<glm::dvec3>& path2D)
{
    std::cout << path2D.size() <<  " PLANPATH2D\n";
    auto rotation = glm::angleAxis(turntable->getJointValue(0), UP);

    auto trajectory = std::make_shared<CompositeTrajectory>(robot->dof());
    trajectory->setLimits(m_config.baseVelocityLimits, m_config.baseAccelerationLimits);

    Action action(trajectory, robot);

    Sequence seq(&profile, path2D);
    seq.transform(rotation, m_config.center);

    if (planSequence(seq, action)) {
        std::cout << "Sequence planned\n";
        trajectory->update();
        if (trajectory->isValid()) return action;
    }

    return { nullptr, nullptr };
}

Action ProcessPlanner::planRetraction(const Waypoint& start)
{
    std::cout << start.values.size() <<  " PLAN RETRACTION\n";

    Pose pose = robot->getPose(start);

    auto trajectory = std::make_shared<CartesianTrajectory>(robot, pose, glm::dvec3(0, 0, -m_config.bladeLength), 10);
    if (trajectory->isValid()) {
        trajectory->setLimits(m_config.baseVelocityLimits, m_config.baseAccelerationLimits);
        return {
                trajectory,
                robot
        };
    }

    return { nullptr, nullptr };
}

bool ProcessPlanner::planTranslation(const glm::dvec3& translation, Action& action)
{
    if (glm::dot(translation, translation) > 1e-6) {
        auto pose = action.target->getPose(action.trajectory->end());
        pose.position += translation;

        auto wp = action.target->inverse(pose);
        if (!wp.isValid()) return false;

        auto trajectory = std::static_pointer_cast<CompositeTrajectory>(action.trajectory);
        trajectory->addTrajectory(std::make_shared<SimpleTrajectory>(trajectory->end(), wp, m_config.solver));
    }

    return true;
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
//    trajectory->addTrajectory(cutTrajectory);
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
    millTrajectory->setLimits(m_config.verySlowVelocityLimits, m_config.baseAccelerationLimits);
    if (!millTrajectory->isValid()) return false;

    // Action must (is known to) only be called on actions formed with composite trajectories
    auto trajectory = std::static_pointer_cast<CompositeTrajectory>(action.trajectory);
//    trajectory->addTrajectory(std::make_shared<HoldPosition>(millTrajectory->start(), 0.2));

    // Commit motions to action
//    trajectory->addTrajectory(millTrajectory);
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
std::shared_ptr<CompositeTrajectory> ProcessPlanner::preparePlanarTrajectory(const std::vector<glm::dvec3>& border, const Axis3D& axes)
{
    bool xFUP = glm::dot(UP, axes.xAxis) > 0, yFUP = glm::dot(UP, axes.yAxis) > 0;
    glm::dvec3 xAxis = glm::dot(UP, axes.xAxis) > 0 ? axes.xAxis : -axes.xAxis;

    uint32_t minIndex, maxIndex;
    VertexArray::extremes(border, xAxis, minIndex, maxIndex);

//    std::cout << m_actions.size() << " V " << m_config.center.x << " " << m_config.center.y << " " << m_config.center.z << " | " << border[minIndex].y << " " << border[maxIndex].y << "\n";

    double runup = 0.1, length = glm::dot(border[maxIndex] - border[minIndex], axes.xAxis) + 1e-3; // 1e-3 to go a bit further (So debris is disconnected properly (Avoid vertex touch))
    double floor = std::min(border[minIndex].y, border[maxIndex].y);

//    std::cout << "PPT: " << xFUP << " " << yFUP << " " << glm::dot(UP, axes.yAxis) << " " << runup << " " << length << " (" << border[minIndex].y << " " << border[maxIndex].y
//    << ") " << m_config.center.y << " | " << (floor - runup - m_config.bladeWidth) << "\n";

    if (yFUP && m_config.center.y < floor - runup - m_config.bladeWidth) {
        return prepareThroughCut(Pose(border[minIndex], axes), length + runup, runup + 0.5 * m_config.bladeWidth, 0.4 * axes.yAxis);
    }

    // Adjust length so it stops just before hitting the turntable TODO don't apply if no risk of TT hit
    length = -(length + runup) + (2.0 * xFUP - 1.0) * 0.5 * m_config.bladeWidth
             - m_config.bladeThickness * tan(0.5 * M_PI - acos(glm::dot(UP, axes.yAxis)));

    auto off = glm::normalize(glm::cross(UP, axes.zAxis));
//    std::cout << "OFF: " << off.x << " " << off.y << " " << off.z << "\n";

    off *= -(0.4 * m_config.materialWidth + glm::dot(off, length * xAxis)); // Margin + depth (along off) cut
    if (glm::dot(off, m_config.center - border[maxIndex]) > 0) off = -off;

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
    if (!cutTrajectory->isValid()) {
        std::cout << "\033[93m[ProcessPlaner] Through cut impossible. Could not perform cut!\033[0m\n";
        return nullptr;
    }

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
        if (!offTrajectory->isValid()) {
            std::cout << "\033[93m[ProcessPlaner] Through cut impossible. Could not move off piece!\033[0m\n";
            return nullptr;
        }
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

    std::cout << "\033[93m[ProcessPlaner] Through cut impossible. Invalid trajectory generated!\033[0m\n";
    return nullptr;
}

bool ProcessPlanner::withinActionLimit(uint32_t buffered) const
{
    if (m_config.actionLimitEnable && m_actions.size() + buffered >= m_config.actionLimit) {
        std::cout << "\033[93m[SculptProcess] Action limit exceeded\033[0m\n";
        return false;
    }

    return true;
}

glm::dvec3 ProcessPlanner::projected2D(const Waypoint& wp, const Profile& profile) const
{
    assert(wp.values.size() == robot->dof());

    auto rotation = glm::angleAxis(turntable->getJointValue(0), UP);

    Pose pose = robot->getPose(wp);
    auto position = profile.project(glm::inverse(rotation) * (pose.position - m_config.bladeThicknessOffset(pose.axes) - m_config.center));
    return {
        position.x,
        position.y,
        0.0
    };
}

void ProcessPlanner::Report::print() const
{
    std::cout << "[Planning Report]\nsuccess: " << success << ", steps: " << (ctStepCount + srStepCount + frStepCount) << "\n";

    if (appliedConvexTrim) {
        std::cout << "[Convex Trim] steps: " << ctStepCount << "\n";
    }

    if (appliedSilhouetteRefinement) {
        std::cout << "[Silhouette Refinement] orientations: " << srOrientations.size() << ", steps: " << srStepCount << "\n";
        for (const std::pair<double, uint32_t>& orientation : srOrientations) {
            std::cout << "[" << orientation.first << " -> " << orientation.second << "]\n";
        }
    }
}