//
// Created by Cam on 2024-11-12.
//

#ifndef AUTOCARVER_SCULPTPROCESS_H
#define AUTOCARVER_SCULPTPROCESS_H

#define LICENSE_NOTICE_ACKNOWLEDGED 1
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

    struct Result {
        std::shared_ptr<Mesh> sculpture;
        std::shared_ptr<Mesh> cut;

        std::vector<std::shared_ptr<Mesh>> debris;
    };

    explicit SculptProcess(const std::shared_ptr<Mesh>& model);
    ~SculptProcess();

    void skipConvexTrim(bool enable);
    void processCut(bool enable);

    void plan();

    void next();

    std::shared_ptr<Mesh> sculpture();

private:


    void planConvexTrim();
    void planOutlineRefinement(float stepDg);
    void planFeatureRefinement();

    void section(const std::shared_ptr<Mesh>& mesh, const glm::vec3& origin, const glm::vec3& normal);
    void remove(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Mesh>& cut);

    void process();

    bool isFragment(const McConnectedComponent& cc);
    bool isPatch(const McConnectedComponent& cc);
    bool isFragmentBelow(const McConnectedComponent& cc);

    std::shared_ptr<Mesh> prepare(const McConnectedComponent& cc);

    void prepareSculptureStyles();

    void activate(const Result& result);
    static uint32_t identifySculpture(const std::vector<std::shared_ptr<Mesh>>& fragments);

    Configuration m_config;

    std::shared_ptr<Mesh> model;
    std::shared_ptr<Sculpture> m_sculpture;

    bool m_planned;
    bool m_convexTrimEnable;
    bool m_processCutEnable;

    uint32_t m_step;

    std::vector<Result> m_results;

    McContext context;

};


#endif //AUTOCARVER_SCULPTPROCESS_H
