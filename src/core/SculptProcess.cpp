//
// Created by Cam on 2024-11-12.
//

#include "SculptProcess.h"

#include <glm/glm.hpp>

#include "Sculpture.h"
#include "fileIO/MeshHandler.h"
#include "geometry/MeshBuilder.h"

SculptProcess::SculptProcess(const std::shared_ptr<Mesh>& model)
    : Scene()
    , model(model)
    , m_sculpture(nullptr)
    , m_step(0)
    , m_planned(false)
    , m_convexTrimEnable(true)
    , m_processCutEnable(true)
{

    m_sculpture = std::make_shared<Sculpture>(model, m_config.materialWidth, m_config.materialHeight);

    prepareBody(std::make_shared<RigidBody>((model)), 1);
    m_bodies[0]->prepareColliderVisuals();

    prepareBody(m_sculpture, 1);

//    for (uint32_t i = 1; i < m_results.size(); i++) {
//        MeshHandler::exportMesh(m_results[i].sculpture, "..\\out\\S" + std::to_string(i) + "Sculpture.obj");
//        MeshHandler::exportMesh(m_results[i].cut, "..\\out\\S" + std::to_string(i) + "Cut.obj");
//
//        for (uint32_t j = 0; j < m_results[i].debris.size(); j++) {
//            MeshHandler::exportMesh(m_results[i].debris[j], "..\\out\\S" + std::to_string(i) + "F" + std::to_string(j) + ".obj");
//        }
//    }
}

SculptProcess::~SculptProcess()
{
    std::cout << "SP destroy\n";
}

void SculptProcess::skipConvexTrim(bool enable)
{
    m_convexTrimEnable = !enable;
}
void SculptProcess::processCut(bool enable)
{
    m_processCutEnable = enable;
}

void SculptProcess::plan()
{
    if (m_planned) {
        std::cout << "\033[33mWarning! The process has already been planned\033[0m\n";
        return;
    }

    // Ready the sculpture by carving the convex hull of the model
    planConvexTrim();

    // Repeatedly refined the sculpture by cutting to the outline of the model at discrete orientations
    planOutlineRefinement(5);

    // Capture model features with fine carving
    planFeatureRefinement();


    // Apply styles to meshes for clarity
    prepareSculptureStyles();

    m_planned = true;
    m_step = 1;
}

void SculptProcess::next()
{
    if (!m_planned) plan();

    if (m_step >= m_results.size()) return;

    std::cout << "Step (" << m_step + 1 << " / " << m_results.size() << "): ";


    activate(m_results[m_step]);

//    if (m_step > 10 || true) {
//        static float offset = 0;
////        auto out = std::make_shared<Mesh>(ConvexHull(m_sculpture->mesh()->vertices()));
//        auto out = std::make_shared<Mesh>(m_sculpture->mesh()->vertices(), m_sculpture->mesh()->faces());
//
//        out->translate(0, 0, offset += 4);
//        MeshHandler::exportMesh(out, "..\\out\\step" + std::to_string(m_step) + ".obj");
//
////        auto temp = MeshBuilder::cleaned(out);
////        temp->translate(2, 0, 0);
////        MeshHandler::exportMesh(temp, "..\\out\\step" + std::to_string(m_step) + "C.obj");
//
//        auto hull = ConvexHull(m_sculpture->mesh()->vertices());
//        auto hm = std::make_shared<Mesh>(hull);
//        hm->translate(2, 0, offset);
//        MeshHandler::exportMesh(hm, "..\\out\\step" + std::to_string(m_step) + "C.obj");
//
//        m_sculpture->overrwrite(std::make_shared<Mesh>(hull));
//
//    }

//    m_sculpture->overrwrite(plane);
//    m_entities[0].render->replace(0, plane);

    m_step++;
}

std::shared_ptr<Mesh> SculptProcess::sculpture()
{
    return m_bodies[0]->mesh();
}

void SculptProcess::planConvexTrim()
{
    struct Operation {
        glm::vec3 origin;
        glm::vec3 normal;
    };

    std::vector<Operation> steps;

    const ConvexHull& hull = m_bodies[0]->hull();

    for (uint32_t i = 0; i < hull.facetCount(); i++) {
        uint32_t idx = hull.faces()[i][0];
        steps.push_back({hull.vertices()[idx], hull.facetNormal(i)});
    }

    // Plan initial cuts, beginning from the top and moving towards the base
    std::sort(steps.begin(), steps.end(), [](const Operation& a, const Operation& b){
        return glm::dot(a.normal, {0, 1, 0}) > glm::dot(b.normal, {0, 1, 0});
    });

    m_results.push_back(Result{m_sculpture->mesh(), nullptr, {}});

    if (m_convexTrimEnable) {

        ConvexHull hull = m_sculpture->hull();

        // Process all the cuts preemptively
        for (uint32_t i = 0; i < steps.size(); i++) {
            auto fragments = hull.fragments(steps[i].origin, -steps[i].normal);
            hull = fragments.first;

            auto mesh = std::make_shared<Mesh>(fragments.first);
            mesh->setBaseColor({1, 1, 1});

            auto debris = std::make_shared<Mesh>(fragments.second);
            m_results.push_back(Result{ mesh, nullptr, { debris } });


        }
    } else {
        m_results.push_back(Result{std::make_shared<Mesh>(hull), nullptr, {}});
    }
}
void SculptProcess::planOutlineRefinement(float stepDg)
{
    //TODO
    float tpi = 1 / (2 * M_PI);
    std::vector<glm::vec3> axes(std::ceil(360.0f / stepDg));
    for (uint32_t i = 0; i < axes.size(); i++) {
        axes[i] = { cosf(i * tpi), 0, sinf(i * tpi) };
    }

//    model->calculateAdjacencies();
    for (const glm::vec3& axis : axes) {
        std::vector<uint32_t> outline = model->outline(axis);
        std::cout << "Updating mesh!\n";
//        m_entities[0].render->replace(0, model);// TODO remove (Temporary force model update)
//        m_entities[0].render->generate();// TODO remove (Temporary style override for testing)

        std::cout << "~~~~~~~~~~~~~~~~~\n";
        break;
    }
}
void SculptProcess::planFeatureRefinement()
{
    //TODO
    // Refine features based on layers?
    // Decompose model into patches and handle separately?
}

void SculptProcess::section(const std::shared_ptr<Mesh>& mesh, const glm::vec3& origin, const glm::vec3& normal)
{
    std::cout << "\033[31m***********Sculpt Process - Section**********\033[0m\n";

    static float offset = 0;

    auto plane = MeshBuilder::plane(2 * m_config.materialWidth * m_config.materialHeight, origin, normal);

//    plane->translate(0, 0, offset);
//    MeshHandler::exportMesh(plane, "..\\out\\step" + std::to_string(m_step) + "p.obj");
//    plane->translate(0, 0, -offset);
//    offset += 4;

}

// Prepare styling for the sculptures
void SculptProcess::prepareSculptureStyles()
{
    uint32_t i = 0;
    for (Result& result : m_results) {
        result.sculpture->setBaseColor({0, 0, 0.6f});


        if (result.cut != nullptr) {
            std::cout << "R" << i++ << " " << result.cut->faceCount() << "\n";
            if (result.cut->faceCount() > 1) {
                result.cut->print();
            }
            std::vector<uint32_t> faces = result.sculpture->sharedFaces(result.cut);
            for (uint32_t face : faces) result.sculpture->setFaceColor(face, {1, 0, 0});
        }
    }
}

void SculptProcess::activate(const Result& result)
{
    m_sculpture->overrwrite(result.sculpture);
//
    std::cout << result.debris.size() << " " << result.sculpture->vertexCount() << " AF\n";
//    if (!m_entities.empty()) m_entities[1].render->replace(0, result.sculpture); // Update render
    //TODO

    // Activate physics on precalculated fragments
    for (uint32_t i = 0; i < result.debris.size(); i++) {

//        prepareBody(new RigidBody(&m_physicsCommon, m_world, fragments[i]));
////        prepareBody(new RigidBody(fragments[i]));
    }
}

uint32_t SculptProcess::identifySculpture(const std::vector<std::shared_ptr<Mesh>>& fragments)
{
    uint32_t idx = 0;
    float volume = fragments[0]->volume();

    for (uint32_t i = 1; i < fragments.size(); i++) {
        float temp = fragments[i]->volume();

        if (volume < temp) {
            volume = temp;
            idx = i;
        }
    }

    return idx;
}