//
// Created by cjhat on 2025-09-17.
//

#ifndef AUTOCARVER_CONFIGURATION_H
#define AUTOCARVER_CONFIGURATION_H

#include "fileIO/Serializable.h"

#include "geometry/poly/Polygon.h"

#include <QRandomGenerator>

class Configuration : public Serializable {
public:

    Configuration();

    bool serialize(std::ofstream& file) const override;
    bool deserialize(std::ifstream& file) override;

    void randomize(uint32_t count);

private:

    double random(double range);

public:

    double width;
    double height;

    glm::dvec3 start;
    glm::dvec3 goal;

    std::vector<Polygon> polygons;

private:

    QRandomGenerator rng;

};


#endif //AUTOCARVER_CONFIGURATION_H
