//
// Created by Cam on 2024-11-10.
//

#ifndef AUTOCARVER_SCULPTURE_H
#define AUTOCARVER_SCULPTURE_H

#include <vector>

#include "geometry/Mesh.h"
#include "geometry/Body.h"

class Sculpture : public Body {
public:

    Sculpture(const std::shared_ptr<Mesh> &model, float width = 2.0f, float height = 6.0f);

    void scaleToFit(float width, float maxHeight);

    void update();

    const std::shared_ptr<Mesh>& sculpture();

    [[nodiscard]] float width() const;
    [[nodiscard]] float height() const;

    [[nodiscard]] float initialVolume() const;
    [[nodiscard]] float currentVolume() const;
    [[nodiscard]] float finalVolume() const;

    [[nodiscard]] float bulkUsageRatio() const;
    [[nodiscard]] float remainderRatio() const;

private:

private:

    std::shared_ptr<Mesh> model;

//    std::vector<Body*> m_fragments;

    float m_width;
    float m_height;

};


#endif //AUTOCARVER_SCULPTURE_H
