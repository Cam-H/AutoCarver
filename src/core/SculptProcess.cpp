//
// Created by Cam on 2024-11-12.
//

#include "SculptProcess.h"

#include "Sculpture.h"
#include "fileIO/MeshHandler.h"
#include "geometry/MeshBuilder.h"

#define my_assert(cond)                             \
    if (!(cond)) {                                  \
        fprintf(stderr, "MCUT error: %s\n", #cond); \
    }
//std::abort();                               \

extern MCAPI_ATTR void MCAPI_CALL mcDebugOutput(McDebugSource source,
                                                McDebugType type,
                                                unsigned int id,
                                                McDebugSeverity severity,
                                                size_t length,
                                                const char* message,
                                                const void* userParam)
{
    printf("---------------\n");
    printf("Debug message ( %d ): %s ", id, message);

    switch (source) {
        case MC_DEBUG_SOURCE_API:
            printf("Source: API");
            break;
        case MC_DEBUG_SOURCE_KERNEL:
            printf("Source: Kernel");
            break;
            break;
    }

    printf("\n");

    switch (type) {
        case MC_DEBUG_TYPE_ERROR:
            printf("Type: Error");
            break;
        case MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
            printf("Type: Deprecated Behaviour");
            break;
        case MC_DEBUG_TYPE_OTHER:
            printf("Type: Other");
            break;
    }

    printf("\n");

    switch (severity) {
        case MC_DEBUG_SEVERITY_HIGH:
            printf("Severity: high");
            break;
        case MC_DEBUG_SEVERITY_MEDIUM:
            printf("Severity: medium");
            break;
        case MC_DEBUG_SEVERITY_LOW:
            printf("Severity: low");
            break;
        case MC_DEBUG_SEVERITY_NOTIFICATION:
            printf("Severity: notification");
            break;
    }

    printf("\n\n");
}

SculptProcess::SculptProcess(const std::shared_ptr<Mesh>& model)
    : Scene()
    , model(model)
    , m_sculpture(nullptr)
    , m_step(0)
    , context(MC_NULL_HANDLE)
    , m_planned(false)
    , m_convexTrimEnable(true)
    , m_processCutEnable(true)
{

    m_sculpture = new Sculpture(model, m_config.materialWidth, m_config.materialHeight);

    prepareBody(new Body(model), 1);
    prepareBody(m_sculpture, 1);



    std::cout << "Size test " << sizeof(vec3f) << "\n";

    // Create the context
    McResult status = mcCreateContext(&context, MC_DEBUG);

    uint64_t numBytes = 0;
    McFlags contextFlags;

    McResult err = mcGetInfo(context, MC_CONTEXT_FLAGS, 0, nullptr, &numBytes);

    if(err)
    {
        // ...
    }

    err = mcGetInfo(context, MC_CONTEXT_FLAGS, numBytes, &contextFlags, nullptr);

    if(err)
    {
        // ...
    }

    if (contextFlags & MC_DEBUG) {
        mcDebugMessageCallback(context, mcDebugOutput, nullptr);
        mcDebugMessageControl(
                context,
                MC_DEBUG_SOURCE_ALL,
                MC_DEBUG_TYPE_ERROR,
                MC_DEBUG_SEVERITY_ALL,
                true);
    }
    my_assert (status == MC_NO_ERROR);

//    static int count = 0;
//    MeshHandler::exportMesh(std::make_shared<Mesh>(VertexArray{ccVertices.data(), ccVertexCount},
//                                                   FaceArray{ccFaceIndices.data(), ccFaceSizes.data(), (uint32_t)ccFaceSizes.size()}.triangulated()), "..\\out\\f" + std::to_string(count++) + ".obj");


    for (uint32_t i = 1; i < m_results.size(); i++) {
        MeshHandler::exportMesh(m_results[i].sculpture, "..\\out\\S" + std::to_string(i) + "Sculpture.obj");
        MeshHandler::exportMesh(m_results[i].cut, "..\\out\\S" + std::to_string(i) + "Cut.obj");

        for (uint32_t j = 0; j < m_results[i].debris.size(); j++) {
            MeshHandler::exportMesh(m_results[i].debris[j], "..\\out\\S" + std::to_string(i) + "F" + std::to_string(j) + ".obj");
        }
    }
}

SculptProcess::~SculptProcess()
{
    std::cout << "SP destroy\n";

    // Free context memory
    McResult status = mcReleaseContext(context);
    my_assert (status == MC_NO_ERROR);
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
        vec3f origin;
        vec3f normal;
    };

    std::vector<Operation> steps;

    const ConvexHull& hull = m_bodies[0]->hull();

    for (uint32_t i = 0; i < hull.facetCount(); i++) {
        uint32_t idx = hull.faces()[i][0];
        steps.push_back({hull.vertices()[idx], hull.facetNormal(i)});
    }

    // Plan initial cuts, beginning from the top and moving towards the base
    std::sort(steps.begin(), steps.end(), [](const Operation& a, const Operation& b){
        return a.normal.dot({0, 1, 0}) > b.normal.dot({0, 1, 0});
    });

    m_results.push_back(Result{m_sculpture->mesh(), nullptr, {}});

    if (m_convexTrimEnable) {

        // Process all the cuts preemptively
        for (uint32_t i = 0; i < steps.size(); i++) {
            section(m_results[i].sculpture, steps[i].origin, steps[i].normal);

            if (m_results[i + 1].sculpture == nullptr || m_results[i + 1].debris.empty()) {
                m_results.erase(m_results.end() - 1);
                break;
            }

            // Initial cuts will definitely be convex so results may be simplified easily
            m_results[i + 1].sculpture = std::make_shared<Mesh>(ConvexHull(m_results[i + 1].sculpture->vertices()));
            for (uint32_t j = 0; j < m_results[i + 1].debris.size(); j++) {
                m_results[i + 1].debris[j] = std::make_shared<Mesh>(ConvexHull(m_results[i + 1].debris[j]->vertices()));
            }

            // Fix cut mesh (Merging coplanar faces & removing orphaned vertices) for subsequent processing
            if (m_processCutEnable) {
                m_results[i + 1].cut = MeshBuilder::cleaned(m_results[i + 1].cut);
//                if (m_results[i + 1].cut->faceCount() > 1) std::abort();
            }


//        if (i > 50) break;
        }
    } else {
        m_results.push_back(Result{std::make_shared<Mesh>(hull), nullptr, {}});
    }
}
void SculptProcess::planOutlineRefinement(float stepDg)
{
    //TODO
    float tpi = 1 / (2 * M_PI);
    std::vector<vec3f> axes(std::ceil(360.0f / stepDg));
    for (uint32_t i = 0; i < axes.size(); i++) {
        axes[i] = { cosf(i * tpi), 0, sinf(i * tpi) };
    }

//    model->calculateAdjacencies();
    for (const vec3f& axis : axes) {
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

void SculptProcess::section(const std::shared_ptr<Mesh>& mesh, const vec3f& origin, const vec3f& normal)
{
    std::cout << "\033[31m***********Sculpt Process - Section**********\033[0m\n";

    static float offset = 0;

    auto plane = MeshBuilder::plane(2 * m_config.materialWidth * m_config.materialHeight, origin, normal);

//    plane->translate(0, 0, offset);
//    MeshHandler::exportMesh(plane, "..\\out\\step" + std::to_string(m_step) + "p.obj");
//    plane->translate(0, 0, -offset);
//    offset += 4;

    remove(mesh, plane);
}

void SculptProcess::remove(const std::shared_ptr<Mesh>& source, const std::shared_ptr<Mesh>& cut)
{
    McResult status = mcDispatch(
            context,
            MC_DISPATCH_VERTEX_ARRAY_FLOAT,
            source->vertices().data(),
            source->faces().faces(),
            source->faces().faceSizes(),
            source->vertexCount(),
            source->faceCount(),
            cut->vertices().data(),
            cut->faces().faces(),
            cut->faces().faceSizes(),
            cut->vertexCount(),
            cut->faceCount());
    my_assert (status == MC_NO_ERROR);

    process();
}

void SculptProcess::process()
{
    // Query the number of available connected components after the cut
    McUint32 connectedComponentCount;
    std::vector<McConnectedComponent> connectedComponents;

    McResult status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, 0, NULL, &connectedComponentCount);
    my_assert (status == MC_NO_ERROR);

    if (connectedComponentCount == 0) {
        std::cout << "no connected components found\n";
//        exit(EXIT_FAILURE);
    }

    connectedComponents.resize(connectedComponentCount);

    status = mcGetConnectedComponents(context, MC_CONNECTED_COMPONENT_TYPE_ALL, (McUint32)connectedComponents.size(), connectedComponents.data(), NULL);
    my_assert (status == MC_NO_ERROR);

    std::cout << "\033[31mPreparing Results...\033[0m\n";

    Result& result = m_results.emplace_back();

    //  Query the data of each connected component
    for (int i = 0; i < (int)connectedComponents.size(); ++i) {
        McConnectedComponent cc = connectedComponents[i];

        if (isFragment(cc)) {
            if (isFragmentBelow(cc)) {
                if (result.sculpture != nullptr) std::cout << i <<  "\033[33mFragment  WARN WARN\033[0m\n";
                result.sculpture = prepare(cc);
            } else {
                result.debris.push_back(prepare(cc));
            }
        } else if (isPatch(cc)) {
            if (result.cut != nullptr) std::cout << i <<  "\033[33mPatch  WARN WARN\033[0m\n";
            result.cut = prepare(cc);
        }
    }

    if (!result.debris.empty()) {

        // Double-check to make sure that the core was properly identified and try fix if not
        if (result.sculpture == nullptr) {
            std::cout << "\033[33mWarning! Issue identifying the core of the sculpture [step " << (m_results.size() - 1) << "]. The selection may be incorrect\033[0m\n";

            // Instead select based on volume - The largest fragment should be the core
            uint32_t coreIdx = identifySculpture(result.debris);
            result.sculpture = result.debris[coreIdx];
            result.debris.erase(result.debris.begin() + coreIdx);
        }

    } else {
        std::cout << "\033[33mError! No fragments remain! [step " << (m_results.size() - 1) << "]. Can not proceed\033[0m\n";
    }

    // Free individual component memory
    status = mcReleaseConnectedComponents(context, 0, NULL);
    my_assert (status == MC_NO_ERROR);
}

bool SculptProcess::isFragment(const McConnectedComponent& cc)
{
    auto type = (McConnectedComponentType)0;
    auto patchLocation = (McPatchLocation)0;
    auto seal = (McFragmentSealType)0;

    McResult status = mcGetConnectedComponentData(context,
                                                  cc,
                                                  MC_CONNECTED_COMPONENT_DATA_TYPE,
                                                  sizeof(McConnectedComponentType),
                                                  &type,
                                                  NULL);
    my_assert (status == MC_NO_ERROR);

    if (type != MC_CONNECTED_COMPONENT_TYPE_FRAGMENT) return false;

    status = mcGetConnectedComponentData(context,
                                         cc,
                                         MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION,
                                         sizeof(McPatchLocation),
                                         &patchLocation,
                                         NULL);
    my_assert(status == MC_NO_ERROR);

    status = mcGetConnectedComponentData(context,
                                                  cc,
                                                  MC_CONNECTED_COMPONENT_DATA_FRAGMENT_SEAL_TYPE,
                                                  sizeof(McFragmentSealType),
                                                  &seal,
                                                  NULL);
    my_assert(status == MC_NO_ERROR);

    return patchLocation == MC_PATCH_LOCATION_INSIDE && seal == MC_FRAGMENT_SEAL_TYPE_COMPLETE;
}

bool SculptProcess::isPatch(const McConnectedComponent& cc)
{
    auto type = (McConnectedComponentType)0;
    auto patchLocation = (McPatchLocation)0;

    McResult status = mcGetConnectedComponentData(context,
                                                  cc,
                                                  MC_CONNECTED_COMPONENT_DATA_TYPE,
                                                  sizeof(McConnectedComponentType),
                                                  &type,
                                                  NULL);
    my_assert (status == MC_NO_ERROR);

    if (type != MC_CONNECTED_COMPONENT_TYPE_PATCH) return false;

    status = mcGetConnectedComponentData(context,
                                         cc,
                                         MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION,
                                         sizeof(McPatchLocation),
                                         &patchLocation,
                                         NULL);
    my_assert(status == MC_NO_ERROR);

    return patchLocation == MC_PATCH_LOCATION_INSIDE;
}

bool SculptProcess::isFragmentBelow(const McConnectedComponent& cc)
{
    auto fragmentLocation = (McFragmentLocation)0;

    McResult status = mcGetConnectedComponentData(context,
                                                 cc,
                                                 MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION,
                                                 sizeof(McFragmentLocation),
                                                 &fragmentLocation,
                                                 NULL);
    my_assert(status == MC_NO_ERROR);

    return fragmentLocation == MC_FRAGMENT_LOCATION_BELOW;
}

std::shared_ptr<Mesh> SculptProcess::prepare(const McConnectedComponent& cc)
{

    // Vertices

    McSize numBytes = 0;
    McResult status = mcGetConnectedComponentData(context,
                                                  cc,
                                                  MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT,
                                                  0,
                                                  NULL,
                                                  &numBytes);
    my_assert (status == MC_NO_ERROR);

    auto ccVertexCount = (McUint32)(numBytes / (sizeof(McFloat) * 3ull));
    std::vector<McFloat> ccVertices(ccVertexCount * 3u);

    status = mcGetConnectedComponentData(context,
                                         cc,
                                         MC_CONNECTED_COMPONENT_DATA_VERTEX_FLOAT,
                                         numBytes,
                                         (McVoid*)ccVertices.data(),
                                         NULL);
    my_assert (status == MC_NO_ERROR);


    // Faces

    numBytes = 0;

    status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, 0, NULL, &numBytes);
    my_assert (status == MC_NO_ERROR);

    std::vector<McUint32> ccFaceIndices;
    ccFaceIndices.resize(numBytes / sizeof(McUint32));

    status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE, numBytes, (McVoid*)ccFaceIndices.data(), NULL);
    my_assert (status == MC_NO_ERROR);


    // Face sizes (vertices per face)

    numBytes = 0;

    status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, 0, NULL, &numBytes);
    my_assert (status == MC_NO_ERROR);

    std::vector<McUint32> ccFaceSizes;
    ccFaceSizes.resize(numBytes / sizeof(McUint32));

    status = mcGetConnectedComponentData(context, cc, MC_CONNECTED_COMPONENT_DATA_FACE_SIZE, numBytes, (McVoid*)ccFaceSizes.data(), NULL);
    my_assert (status == MC_NO_ERROR);

    return std::make_shared<Mesh>(
            VertexArray{ccVertices.data(), ccVertexCount},
            FaceArray{ccFaceIndices.data(), ccFaceSizes.data(), (uint32_t)ccFaceSizes.size()}
    );
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

//        prepareBody(new Body(&m_physicsCommon, m_world, fragments[i]));
////        prepareBody(new Body(fragments[i]));
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