//
// Created by cjhat on 2025-09-04.
//

#ifndef AUTOCARVER_PROCESSCONFIGURATION_H
#define AUTOCARVER_PROCESSCONFIGURATION_H

#include <cstdint>
#include <memory>

class SculptProcess;
class ProcessPlanner;

class KinematicChain;

#include "fileIO/Serializable.h"

#include "geometry/curves/Interpolator.h"
#include "robot/trajectory/Waypoint.h"

#include "robot/Pose.h"

class ProcessConfiguration : public Serializable {
public:

    enum class ConvexSliceOrder {
        TOP_DOWN = 0, BOTTOM_UP
    };

    ProcessConfiguration();

    bool serialize(std::ofstream& file) const override;
    bool deserialize(std::ifstream& file) override;


    void enableConvexTrim(bool enable);
    void setSlicingOrder(ConvexSliceOrder order);
    void setMinimumCutVolume(double volume);

    void enableSilhouetteRefinement(bool enable);
    void setStepOffset(double degrees);

    void setContinuous(bool enable);

    void enableActionLimit(bool enable);
    void setActionLimit(uint32_t limit);


    void enableActionLinking(bool enable);
    void enableCollisionTesting(bool enable);

    void enableCutSimulation(bool enable);
    void enableFragmentRelease(bool enable);

    void enableDebrisColoring(bool enable);

    void setCenter(const glm::dvec3& position);

    [[nodiscard]] bool isActionLimitEnabled() const;
    [[nodiscard]] uint32_t getActionLimit() const;


    // Blade-aligned position equivalent to vertex
    [[nodiscard]] inline glm::dvec3 alignedToBlade(const Axis3D& axes, const glm::dvec3& vertex) const { return vertex + bladeOffset(axes, vertex); }

    // Offset to align the blade
    [[nodiscard]] inline glm::dvec3 bladeOffset(const Axis3D& axes, const glm::dvec3& vertex) const { return bladeCenterOffset(axes.zAxis, vertex) + bladeThicknessOffset(axes); }

    // Offset to center the blade's z-axis
    [[nodiscard]] inline glm::dvec3 bladeCenterOffset(const glm::dvec3& zAxis, const glm::dvec3& vertex) const { return -zAxis * (0.5 * bladeLength + glm::dot(zAxis, vertex - center)); }

    // Offset to account for blade thickness
    [[nodiscard]] inline glm::dvec3 bladeThicknessOffset(const Axis3D& axes) const { return 0.5 * bladeThickness * axes.yAxis; }

    // Apply robot offsets to vertex (Difference between origin and start of cutting surface)
    [[nodiscard]] glm::dvec3 poseAdjustedVertex(const Axis3D& axes, const glm::dvec3& vertex) const { return vertex - axes.zAxis * 0.5 * bladeLength + axes.yAxis * 0.5 * bladeThickness; }


    friend class SculptProcess;
    friend class ProcessPlanner;

private:


    // SCULPTING PARAMETERS

    double materialWidth; // Initial block width (Assuming square base)
    double materialHeight; // Initial block height (Maximum allowable)


    // CONVEX REFINEMENT PARAMETERS

    bool convexTrimEnable; // Convex trim subprocess enable

    ConvexSliceOrder sliceOrder; // Specify priority when ordering cuts
    double minCutVolume; // Minimum volume for a cut


    // SILHOUETTE REFINEMENT PARAMETERS

    bool silhouetteRefinementEnable; // Silhouette subprocess enable

    double stepOffset; // Degree offset per step


    // GENERIC PROCESS PARAMETERS

    bool continuous; // Move between steps automatically

    bool actionLimitEnable; // Enables action limiting
    uint32_t actionLimit; // Number of actions prepared, if enabled

    bool linkActionEnable; // Connect disjoint trajectory
    bool collisionTestingEnable; // Consider collisions when connecting disjoint trajectories

    bool cutSimulationEnable; // Simulate carving as the robot travels
    bool fragmentReleaseEnable; // Generate dynamic fragments when disconnected from the sculpture

    bool debrisColoringEnable;

    Interpolator::SolverType solver;

    glm::dvec3 forward; // Horizontal direction from robot to turntable
    glm::dvec3 center; // Origin of the sculpture, centered on the surface of the turntable

    double bladeLength, bladeWidth, bladeThickness;

    std::shared_ptr<KinematicChain> kinematics;

    Waypoint robotHome;
    Waypoint robotNeutral;

    std::vector<double> baseVelocityLimits, baseAccelerationLimits;
    std::vector<double> slowVelocityLimits; // TODO use cartesian speed limit (Needs further trajectory development)

//    Waypoint latestTableCommand;
//    Waypoint latestRobotCommand;

};


#endif //AUTOCARVER_PROCESSCONFIGURATION_H
