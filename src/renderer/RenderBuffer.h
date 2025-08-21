//
// Created by cjhat on 2025-08-21.
//

#ifndef AUTOCARVER_RENDERBUFFER_H
#define AUTOCARVER_RENDERBUFFER_H

#include <array>
#include <mutex>
#include <memory>

#include <glm.hpp>

class Scene;
class Mesh;

#include "geometry/primitives/Sphere.h"

class RenderBuffer {
public:

    RenderBuffer();

    struct Item {
        uint32_t ID;

        glm::dmat4 transform;
        std::shared_ptr<Mesh> mesh;

        std::shared_ptr<Mesh> hull;
        Sphere bounds;
    };

    void update(const Scene* scene);

    const std::vector<Item>& latest();

private:

    uint32_t nextToUpdate();

private:

    std::array<std::vector<Item>, 3> m_buffer;

    std::mutex m_mutex;

    uint32_t m_latest; // Most recently updated buffer
    uint32_t m_active; // Buffer currently in use by renderer

};


#endif //AUTOCARVER_RENDERBUFFER_H
