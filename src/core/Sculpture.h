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

    virtual void setRenderer(Qt3DCore::QEntity *parent, Qt3DExtras::Qt3DWindow *view) override;

    void scaleToFit(float width, float maxHeight);

    void update();

    const std::shared_ptr<Mesh>& sculpture();

    [[nodiscard]] float initialVolume() const;
    [[nodiscard]] float currentVolume() const;
    [[nodiscard]] float finalVolume() const;

    [[nodiscard]] float bulkUsageRatio() const;
    [[nodiscard]] float remainderRatio() const;

private:

private:

    std::shared_ptr<Mesh> m_sculpture;

    std::vector<Body*> m_fragments;

    float m_width;
    float m_height;

};


#endif //AUTOCARVER_SCULPTURE_H
