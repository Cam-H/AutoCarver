//
// Created by Cam on 2024-11-12.
//

#ifndef AUTOCARVER_SCULPTPROCESS_H
#define AUTOCARVER_SCULPTPROCESS_H

#include "mcut/mcut.h"

#include "Scene.h"

#include "geometry/VertexArray.h"
#include "geometry/Mesh.h"
#include "core/Sculpture.h"


class SculptProcess : public Scene {
public:

    struct Configuration {
        float materialWidth = 1.0f;
        float materialHeight = 2.0f;
    };

    // TODO develop for more than just plane slices
    struct Operation {
        vec3f origin;
        vec3f normal;
    };

    explicit SculptProcess(const std::shared_ptr<Mesh>& model);
    ~SculptProcess();

    void next();

private:

    void plan();

    void section(const vec3f& normal, float offset);
    void remove();

    void process();
    std::shared_ptr<Mesh> prepare(const McConnectedComponent& cc);

    void attachFragments(const std::vector<std::shared_ptr<Mesh>>& fragments);
    static uint32_t identifySculpture(const std::vector<std::shared_ptr<Mesh>>& fragments);

    Configuration m_config;

    std::shared_ptr<Mesh> model;
    Sculpture *m_sculpture;

    std::vector<Operation> m_steps;
    uint32_t m_step;

    McContext context;

};


#endif //AUTOCARVER_SCULPTPROCESS_H
