//
// Created by cjhat on 2025-09-04.
//

#include "ProcessConfiguration.h"

#include "robot/ArticulatedWrist.h"

#include "fileIO/MeshHandler.h"

ProcessConfiguration::ProcessConfiguration()
    : materialWidth(1.0)
    , materialHeight(1.3)

    , convexTrimEnable(true)
    , sliceOrder(ConvexSliceOrder::TOP_DOWN)
    , minCutVolume(0.005)

    , silhouetteRefinementEnable(true)
    , stepOffset(90.0)

    , continuous(false)

    , actionLimitEnable(false)
    , actionLimit(0xFFFFFFFF)

    , linkActionEnable(true)
    , collisionTestingEnable(true)
    , cutSimulationEnable(true)
    , fragmentReleaseEnable(true)
    , debrisColoringEnable(true)

    , solver(Interpolator::SolverType::QUINTIC)

    , forward(-1, 0, 0)
    , center()

    , bladeLength(1.5) // Length of cutting area (Does not cut tip + section blocked by base)
    , bladeWidth(0.098)
    , bladeThickness(0.011)

    , kinematics(std::make_shared<ArticulatedWrist>(0.2, 1.2, 1.2, 0.35))
    , robotHome(Waypoint({ 0, 110, 20, 0, -130, 0 }, true).toRad())

    , baseVelocityLimits(std::vector<double>(kinematics->jointCount(), M_PI / 2))
    , baseAccelerationLimits(std::vector<double>(kinematics->jointCount(), M_PI))
    , slowVelocityLimits(std::vector<double>(kinematics->jointCount(), M_PI)) //TODO slowdown after testing

    , tableMesh(MeshHandler::loadAsMeshBody("../res/meshes/TableQuick.obj"))
{

}

bool ProcessConfiguration::serialize(std::ofstream& file) const
{
    Serializer::writeDouble(file, materialWidth);
    Serializer::writeDouble(file, materialHeight);

    Serializer::writeBool(file, convexTrimEnable);
    Serializer::writeDouble(file, minCutVolume);

    Serializer::writeBool(file, silhouetteRefinementEnable);
    Serializer::writeDouble(file, stepOffset);

    Serializer::writeBool(file, continuous);

    Serializer::writeBool(file, actionLimitEnable);
    Serializer::writeUint(file, actionLimit);
    Serializer::writeBool(file, linkActionEnable);
    Serializer::writeBool(file, collisionTestingEnable);
    Serializer::writeBool(file, fragmentReleaseEnable);
    Serializer::writeBool(file, debrisColoringEnable);

    return true;
}

bool ProcessConfiguration::deserialize(std::ifstream& file)
{
    materialWidth = Serializer::readDouble(file);
    materialHeight = Serializer::readDouble(file);

    convexTrimEnable = Serializer::readBool(file);
    minCutVolume = Serializer::readDouble(file);

    silhouetteRefinementEnable = Serializer::readBool(file);
    stepOffset = Serializer::readDouble(file);

    continuous = Serializer::readBool(file);

    actionLimitEnable = Serializer::readBool(file);
    actionLimit = Serializer::readUint(file);

    linkActionEnable = Serializer::readBool(file);
    collisionTestingEnable = Serializer::readBool(file);
    fragmentReleaseEnable = Serializer::readBool(file);
    debrisColoringEnable = Serializer::readBool(file);

    return true;
}


void ProcessConfiguration::enableConvexTrim(bool enable)
{
    convexTrimEnable = enable;
}

void ProcessConfiguration::setSlicingOrder(ConvexSliceOrder order)
{
    sliceOrder = order;
}

void ProcessConfiguration::setMinimumCutVolume(double volume)
{
    minCutVolume = volume;
}


void ProcessConfiguration::enableSilhouetteRefinement(bool enable)
{
    silhouetteRefinementEnable = enable;
}

void ProcessConfiguration::setStepOffset(double degrees)
{
    stepOffset = degrees;
}

void ProcessConfiguration::setContinuous(bool enable)
{
    continuous = enable;
}

void ProcessConfiguration::enableActionLimit(bool enable)
{
    actionLimitEnable = enable;
}
void ProcessConfiguration::setActionLimit(uint32_t limit) {
    actionLimit = limit;
    actionLimitEnable = true;
}

void ProcessConfiguration::enableActionLinking(bool enable)
{
    linkActionEnable = enable;
}
void ProcessConfiguration::enableCollisionTesting(bool enable)
{
    collisionTestingEnable = enable;
}

void ProcessConfiguration::enableCutSimulation(bool enable)
{
    cutSimulationEnable = enable;
}
void ProcessConfiguration::enableFragmentRelease(bool enable)
{
    fragmentReleaseEnable = enable;
}

void ProcessConfiguration::enableDebrisColoring(bool enable)
{
    debrisColoringEnable = enable;
}

void ProcessConfiguration::setCenter(const glm::dvec3& position)
{
    center = position;

//    auto neutral = center + materialHeight * UP;
//    auto neutralPose = Pose(neutral, Axis3D(forward));
//    neutralPose.localTranslate({ 0, materialWidth, -0.5 * bladeLength });
//    robotNeutral = Waypoint(kinematics->invkin(neutralPose), false);
////
//    assert(robotNeutral.isValid());
}

bool ProcessConfiguration::isActionLimitEnabled() const
{
    return actionLimitEnable;
}
uint32_t ProcessConfiguration::getActionLimit() const
{
    return actionLimit;
}