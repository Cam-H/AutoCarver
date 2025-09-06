//
// Created by cjhat on 2025-09-04.
//

#ifndef AUTOCARVER_PROCESSCONFIGURATION_H
#define AUTOCARVER_PROCESSCONFIGURATION_H

class SculptProcess;

#include <cstdint>

#include "fileIO/Serializable.h"

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


    [[nodiscard]] bool isActionLimitEnabled() const;
    [[nodiscard]] uint32_t getActionLimit() const;

    friend class SculptProcess;

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

};


#endif //AUTOCARVER_PROCESSCONFIGURATION_H
