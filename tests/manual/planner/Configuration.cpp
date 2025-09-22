//
// Created by cjhat on 2025-09-17.
//

#include "Configuration.h"



Configuration::Configuration()
    : width(1.0)
    , height(1.0)
    , start()
    , goal(50.0, 20.0, M_PI / 3)
{

}

void Configuration::randomize(uint32_t count)
{
    polygons.clear();

    for (uint32_t i = 0; i < count; i++) {

        uint32_t vertexCount = rng.global()->bounded(3, 12);

        // Generate a cloud of count vertices within a cube around the origin
        glm::dvec2 offset = { random(50.0), random(50.0) };
        std::vector<glm::dvec2> border(vertexCount, offset);

        for (glm::dvec2& vertex : border) {
            vertex.x += random(20.0);
            vertex.y += random(20.0);
        }

        polygons.emplace_back(border);
        polygons.back().convexify();
    }
}

double Configuration::random(double range)
{
    return rng.global()->bounded(range) - 0.5 * range;
}

bool Configuration::serialize(std::ofstream& file) const
{
    Serializer::writeDouble(file, width);
    Serializer::writeDouble(file, height);

    Serializer::writeDVec3(file, start);
    Serializer::writeDVec3(file, goal);

    Serializer::writeUint(file, polygons.size());
    for (const Polygon& poly : polygons) {
        if (!poly.serialize(file)) return false;
    }

    return true;
}

bool Configuration::deserialize(std::ifstream& file)
{

    width = Serializer::readDouble(file);
    height = Serializer::readDouble(file);

    start = Serializer::readDVec3(file);
    goal = Serializer::readDVec3(file);

    polygons.clear();

    uint32_t count = Serializer::readUint(file);
    for (uint32_t i = 0; i < count; i++) {
        polygons.emplace_back(file);
    }

    return true;
}